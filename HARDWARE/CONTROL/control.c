#include "control.h"
#include "string.h"

/* 机械中值。
 *
 * 这个值相当于“小车真正能站稳时的目标角度”。
 * 理想情况下车身竖直时就是平衡点，但实车通常会因为：
 * 1. 重心安装偏移
 * 2. 轮胎、车架或电池位置不完全对称
 * 3. 传感器安装角度存在轻微误差
 *
 * 导致“几何上看起来竖直”和“控制上最稳定”的角度并不完全一致。
 * 因此这里保留一个可人工微调的中值，让直立环围绕这个角度工作。
 *
 * 调参经验：
 * 1. 如果小车静止时总是持续前倾找平衡，可适当增大或减小该值做补偿
 * 2. 这个值只负责修正平衡点，不负责解决抖动、响应慢或超调问题
 * 3. 抖动和响应主要仍然靠直立环、速度环和转向环参数处理
 */
float Med_Angle = 0.5;

/* 高层控制目标。
 *
 * 这里不是最终 PWM，而是“控制器希望小车去做什么”：
 * 1. Target_Speed 表示前后运动目标
 * 2. Turn_Speed   表示左右转向目标
 *
 * 无论目标来自哪一种上层行为：
 * 1. K230 视觉避障
 * 2. 六路有效巡线
 * 3. 原始手动控制兜底
 *
 * 最终都统一进入后面的三环控制：
 * 1. 速度环决定前后趋势
 * 2. 直立环负责保持站稳
 * 3. 转向环负责左右差速
 *
 * 这样做的好处是：
 * 1. 不同功能模块不直接抢电机 PWM
 * 2. 各种行为切换时更平滑
 * 3. 控制结构更清晰，后续加功能时不容易互相打架
 */
float Target_Speed = 0;
float Turn_Speed = 0;

/* 三个核心控制环参数。
 *
 * 一、直立环：
 * 1. Vertical_Kp 根据“当前角度偏离平衡点多少”给出纠正力度
 * 2. Vertical_Kd 根据“当前倒下去的角速度有多快”增加阻尼
 * 3. 它是平衡车最核心的一环，决定能不能站住
 *
 * 二、速度环：
 * 1. Velocity_Kp 根据速度误差做即时修正
 * 2. Velocity_Ki 对误差做积分，用来消除长期偏差
 * 3. 它不直接让车“冲出去”，而是改变直立环的目标角度趋势
 *
 * 三、转向环：
 * 1. Turn_Kp 根据外部给定的转向目标控制左右差速
 * 2. Turn_Kd 根据陀螺仪 Z 轴角速度抑制转向摆动
 * 3. 它既服务于手动左右转，也服务于 K230 避障和巡线模式
 *
 * 注意：
 * 1. 这几个参数之间是耦合的，不能孤立地只看一个数值
 * 2. 直立环没调稳之前，速度环和转向环的任何优化意义都不大
 */
float Vertical_Kp = -500;
float Vertical_Kd = -1.92f;
float Velocity_Kp = -0.44f;
float Velocity_Ki = -0.0022f;
float Turn_Kd = 0.6f;
float Turn_Kp = 20;

/* 高层速度和转向目标的统一限幅。
 *
 * 这里限制的是上层给出的“目标量”，不是最终 PWM。
 * 作用是防止：
 * 1. 手动控制连续累加后目标速度无限增长
 * 2. 视觉避障或巡线在异常情况下给出过激目标
 * 3. 后级控制环在不合理目标下被迫进入饱和区
 *
 * SPEED_Y：
 * 前后速度目标限幅
 *
 * SPEED_Z：
 * 左右转向目标限幅
 */
#define SPEED_Y 40
#define SPEED_Z 100

/* K230 视觉避障参数。
 *
 * K230 通过 USART3 向 STM32 发送颜色识别结果帧：
 *   $C,color_id,pos,size#
 *
 * 当前策略里，K230 不直接改电机 PWM，而是只修改：
 * 1. Target_Speed
 * 2. Turn_Speed
 *
 * 然后仍然交给底层三环控制处理，这样能保留平衡车原本的稳定性。
 *
 * 各参数含义：
 * 1. K230_AVOID_BASE_SPEED
 *    发现障碍物时的基础前进速度，通常会比正常巡线更慢
 * 2. K230_AVOID_TURN_STRONG
 *    障碍明确在左或右时使用的较强转向量
 * 3. K230_AVOID_TURN_MID
 *    障碍在正前方时使用的中等转向量
 */
#define K230_AVOID_BASE_SPEED 8
#define K230_AVOID_TURN_STRONG 45
#define K230_AVOID_TURN_MID 30

/* 目标面积阈值。
 *
 * K230 识别到的目标如果面积太小，通常意味着：
 * 1. 目标很远
 * 2. 只是噪点
 * 3. 或者出现误检
 *
 * 这类目标不值得触发避障，否则小车会过于敏感，频繁误动作。
 */
#define K230_AVOID_MIN_SIZE 200

/* 三环输出。
 *
 * 这几个变量主要用于：
 * 1. 存放每一环的计算结果
 * 2. 方便调试时观察控制链路
 * 3. 最终合成电机输出
 */
int Vertical_out, Velocity_out, Turn_out;

int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_left, int encoder_right);
int Turn(int gyro_Z, int RC);

/* 判断 K230 视觉避障是否当前生效。
 *
 * 返回 1 代表当前视觉结果足以接管高层行为；
 * 返回 0 代表应该交回巡线或原始兜底控制。
 *
 * 判定条件包括：
 * 1. 帧必须是活跃的，不能是超时失效的旧数据
 * 2. 颜色 ID 必须在当前允许的目标集合内
 * 3. 目标面积必须超过最小阈值
 */
static u8 K230_AvoidanceActive(void)
{
	K230_ColorFrame_t frame = K230_GetColorFrame();

	/* active=0 表示：
	 * 1. K230 明确上报“当前没有目标”
	 * 2. 或者这份视觉数据已经超时失效
	 */
	if(frame.active == 0)
	{
		return 0;
	}

	/* 当前只处理红、绿、蓝三类目标。
	 * 如果后续协议扩展了更多类别，这里可以再增加映射规则。
	 */
	if(frame.color_id < 1 || frame.color_id > 3)
	{
		return 0;
	}

	/* 面积太小视为噪点或远处目标，不触发避障。 */
	if(frame.size < K230_AVOID_MIN_SIZE)
	{
		return 0;
	}

	return 1;
}

/* 根据 K230 识别结果生成高层避障目标。
 *
 * 这个函数只负责决定“怎么绕”，不直接碰电机。
 * 规则非常直接：
 * 1. 目标在左边，小车向右绕
 * 2. 目标在右边，小车向左绕
 * 3. 目标在中间，优先减速，再偏向右转
 *
 * 注意：
 * 这里是高层行为策略，不是底层稳定控制。
 * 平衡、速度平滑和实际差速仍由后面的控制环来完成。
 */
static void K230_ApplyAvoidance(void)
{
	K230_ColorFrame_t frame = K230_GetColorFrame();

	Target_Speed = K230_AVOID_BASE_SPEED;

	if(frame.pos == 'L')
	{
		/* 障碍在左边。
		 * 约定负值表示右转，因此这里给一个较强的右转量。
		 */
		Turn_Speed = -K230_AVOID_TURN_STRONG;
	}
	else if(frame.pos == 'R')
	{
		/* 障碍在右边。
		 * 正值表示左转，因此这里给一个较强的左转量。
		 */
		Turn_Speed = K230_AVOID_TURN_STRONG;
	}
	else
	{
		/* 障碍在中间最危险。
		 * 先进一步降低速度，再优先向右转避让。
		 */
		Target_Speed = K230_AVOID_BASE_SPEED / 2;
		Turn_Speed = -K230_AVOID_TURN_MID;
	}
}

/* PB5 外部中断服务函数。
 *
 * 这是整个平衡车主控制链路的核心入口之一。
 * 每次中断触发时，会依次完成：
 * 1. 采集当前传感器与编码器数据
 * 2. 更新视觉结果保活状态
 * 3. 决定当前由哪个高层行为接管
 * 4. 计算速度环、直立环、转向环输出
 * 5. 合成左右电机 PWM
 * 6. 执行倾倒保护
 *
 * 行为优先级固定为：
 * 1. K230 视觉避障
 * 2. 六路有效巡线
 * 3. 原始手动控制兜底
 */
void EXTI9_5_IRQHandler(void)
{
	int PWM_out;

	if(EXTI_GetITStatus(EXTI_Line5) != 0)
	{
		if(PBin(5) == 0)
		{
			EXTI_ClearITPendingBit(EXTI_Line5);

			/* 1. 采集编码器、姿态角、角速度和加速度数据。 */
			Encoder_Left = -Read_Speed(2);
			Encoder_Right = Read_Speed(4);

			mpu_dmp_get_data(&Roll, &Pitch, &Yaw);
			MPU_Get_Gyroscope(&gyroy, &gyrox, &gyroz);
			MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

			/* 2. 视觉结果保活。
			 * 如果 K230 长时间没有发送新帧，这里会自动让旧结果失效，
			 * 避免小车一直沿用过期的视觉判断。
			 */
			K230_ColorFrameHeartbeat();

			/* 3. 高层行为仲裁。 */
			if(K230_AvoidanceActive())
			{
				K230_ApplyAvoidance();
			}
			else if(Tracking_IsActive())
			{
				/* 没有颜色障碍时，由巡线模块接管前进目标。 */
				Tracking_SetSpeed();
				Turn_Speed = 0;
			}
			else
			{
				/* 原始手动控制兜底。
				 * 当前蓝牙逻辑基本停用，这里主要保留历史兼容行为。
				 */
				if((Fore == 0) && (Back == 0))
				{
					Target_Speed = 0;
				}
				if(Fore == 1)
				{
					Target_Speed++;
				}
				if(Back == 1)
				{
					Target_Speed++;
				}
				Target_Speed = Target_Speed > SPEED_Y ? SPEED_Y : (Target_Speed < (-SPEED_Y) ? (-SPEED_Y) : Target_Speed);
			}

			/* 4. 转向目标处理。
			 * 视觉避障和巡线模式下，转向由对应模块负责；
			 * 只有都未生效时，才回退到原始左右控制。
			 */
			if(K230_AvoidanceActive())
			{
				/* Turn_Speed 已在 K230_ApplyAvoidance() 中设置。 */
			}
			else if(Tracking_IsActive())
			{
				Turn_Speed = 0;
			}
			else if((Left == 0) && (Right == 0))
			{
				Turn_Speed = 0;
			}

			if(!K230_AvoidanceActive())
			{
				if(Left == 1)
				{
					Turn_Speed += 30;
				}
				if(Right == 1)
				{
					Turn_Speed -= 30;
				}
			}

			/* 所有来源的转向目标统一限幅，防止过激。 */
			Turn_Speed = Turn_Speed > SPEED_Z ? SPEED_Z : (Turn_Speed < (-SPEED_Z) ? (-SPEED_Z) : Turn_Speed);

			/* 5. 转向环阻尼约束。
			 * 视觉避障或巡线接管时，不叠加原始手动转向的约束策略。
			 */
			if(K230_AvoidanceActive())
			{
				Turn_Kd = 0;
			}
			else if(Tracking_IsActive())
			{
				Turn_Kd = 0;
			}
			else if((Left == 0) && (Right == 0))
			{
				Turn_Kd = 0.6f;
			}
			else if((Left == 1) || (Right == 1))
			{
				Turn_Kd = 0;
			}

			/* 6. 三环控制计算。 */
			Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
			Vertical_out = Vertical(Velocity_out + Med_Angle, Pitch, gyroy);

			if(K230_AvoidanceActive())
			{
				/* 视觉避障模式：根据目标位置做差速绕行。 */
				Turn_out = Turn(gyroz, Turn_Speed);
			}
			else if(Tracking_IsActive())
			{
				/* 巡线模式：根据六路有效数字量/偏差结果输出转向。 */
				Turn_out = Tracking_TurnPD(gyroz);
			}
			else
			{
				Turn_out = Turn(gyroz, Turn_Speed);
			}

			/* 7. 合成左右电机输出。
			 * 直立输出负责“站稳”
			 * 转向输出负责“左右差速”
			 */
			PWM_out = Vertical_out;
			MOTO1 = PWM_out - Turn_out;
			MOTO2 = PWM_out + Turn_out;
			Limit(&MOTO1, &MOTO2);
			Load(MOTO1, MOTO2);

			/* 8. 倾倒保护。 */
			Stop(&Med_Angle, &Pitch);
		}
	}
}

/* 直立环 PD。
 *
 * 输入：
 * 1. Med    目标平衡角
 * 2. Angle  当前俯仰角
 * 3. gyro_Y 当前俯仰角速度
 *
 * 输出：
 * 直立环修正量
 *
 * 物理意义：
 * 1. 角度偏差项负责把车拉回平衡点
 * 2. 角速度项负责抑制摆动和过冲
 */
int Vertical(float Med, float Angle, float gyro_Y)
{
	int PWM_out;

	PWM_out = Vertical_Kp * (Angle - Med) + Vertical_Kd * (gyro_Y - 0);
	return PWM_out;
}

/* 速度环 PI。
 *
 * 输入：
 * 1. Target         目标速度
 * 2. encoder_left   左编码器速度
 * 3. encoder_right  右编码器速度
 *
 * 输出：
 * 速度修正量
 *
 * 工作过程：
 * 1. 计算当前速度误差
 * 2. 用简单一阶低通抑制编码器噪声
 * 3. 对误差做积分，形成位移/趋势补偿
 * 4. 最终输出给直立环作为前后倾趋势
 */
int Velocity(int Target, int encoder_left, int encoder_right)
{
	static int Encoder_S, EnC_Err_Lowout_last, PWM_out, Encoder_Err, EnC_Err_Lowout;
	float a = 0.7f;

	Encoder_Err = ((encoder_left + encoder_right) - Target);
	EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;

	Encoder_S += EnC_Err_Lowout;
	Encoder_S = Encoder_S > 20000 ? 20000 : (Encoder_S < (-20000) ? (-20000) : Encoder_S);

	if(stop == 1)
	{
		Encoder_S = 0;
		stop = 0;
	}

	PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
	return PWM_out;
}

/* 转向环。
 *
 * 输入：
 * 1. gyro_Z 当前 Z 轴角速度
 * 2. RC     外部给定的目标转向量
 *
 * 输出：
 * 转向修正量
 *
 * 说明：
 * 1. gyro_Z 项抑制车体自身转向摆动
 * 2. RC 项来自上层目标
 * 3. RC 既可能来自原始手动控制，也可能来自 K230 或巡线模块
 */
int Turn(int gyro_Z, int RC)
{
	int PWM_out;

	PWM_out = Turn_Kd * gyro_Z + Turn_Kp * RC;
	return PWM_out;
}

/* 在线修改平衡核心 PID。
 *
 * 当前支持的名称：
 * 1. VKP -> Vertical_Kp   直立环比例
 * 2. VKD -> Vertical_Kd   直立环微分
 * 3. SKP -> Velocity_Kp   速度环比例
 * 4. SKI -> Velocity_Ki   速度环积分
 * 5. RKP -> Turn_Kp       转向环比例
 * 6. RKD -> Turn_Kd       转向环微分/阻尼
 *
 * 返回值：
 * 1 表示修改成功
 * 0 表示名称不匹配
 */
int Control_SetPidByName(const char *name, float value)
{
	if(strcmp(name, "VKP") == 0)
	{
		Vertical_Kp = value;
		return 1;
	}
	if(strcmp(name, "VKD") == 0)
	{
		Vertical_Kd = value;
		return 1;
	}
	if(strcmp(name, "SKP") == 0)
	{
		Velocity_Kp = value;
		return 1;
	}
	if(strcmp(name, "SKI") == 0)
	{
		Velocity_Ki = value;
		return 1;
	}
	if(strcmp(name, "RKP") == 0)
	{
		Turn_Kp = value;
		return 1;
	}
	if(strcmp(name, "RKD") == 0)
	{
		Turn_Kd = value;
		return 1;
	}
	return 0;
}

/* 读取当前三环 PID，用于串口回显或上位机查询。 */
void Control_GetPidValues(float *vkp, float *vkd, float *skp, float *ski, float *rkp, float *rkd)
{
	if(vkp != 0)
	{
		*vkp = Vertical_Kp;
	}
	if(vkd != 0)
	{
		*vkd = Vertical_Kd;
	}
	if(skp != 0)
	{
		*skp = Velocity_Kp;
	}
	if(ski != 0)
	{
		*ski = Velocity_Ki;
	}
	if(rkp != 0)
	{
		*rkp = Turn_Kp;
	}
	if(rkd != 0)
	{
		*rkd = Turn_Kd;
	}
}
