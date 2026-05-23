#include "control.h"
#include "string.h"
#include "usart3.h"

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
 * 当前上层行为保留 K210 色块避障和 K210 视觉巡线。
 * 八路传感器巡线不再参与控制。
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
 * 3. 当前主要服务于 K210 视觉巡线和色块避障模式
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
 * 1. 视觉巡线在异常情况下给出过激目标
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

/* K210 视觉巡线参数。
 *
 * K210 通过 USART3 向 STM32 发送巡线结果帧：
 *   $L,error,angle,confidence,flags#
 *
 * BASE_SPEED 沿用当前工程电机方向；MIN_CONFIDENCE 用于过滤低可信结果；
 * ERROR_KP 负责当前横向纠偏，ANGLE_KD 用路线趋势提前处理弯道。
 */
#define K210_LINE_BASE_SPEED -5
#define K210_LINE_MIN_CONFIDENCE 10
#define K210_LINE_TURN_LIMIT 100
#define K210_LINE_ERROR_KP 0.32f
#define K210_LINE_ANGLE_KD 0.18f

/* K210 颜色避障参数。
 *
 * K210 通过 USART3 向 STM32 发送颜色识别结果帧：
 *   $C,color_id,pos,size#
 */
#define K210_AVOID_BASE_SPEED -10
#define K210_AVOID_TURN_STRONG 45
#define K210_AVOID_TURN_MID 30
#define K210_AVOID_MIN_SIZE 200

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

static u8 K210_AvoidanceActive(void)
{
	K210_ColorFrame_t frame = K210_GetColorFrame();

	if(K210_ColorFrameAvailable() == 0)
	{
		return 0;
	}

	if(frame.color_id < 1 || frame.color_id > 3)
	{
		return 0;
	}

	if(frame.size < K210_AVOID_MIN_SIZE)
	{
		return 0;
	}

	return 1;
}

static void K210_ApplyAvoidance(void)
{
	K210_ColorFrame_t frame = K210_GetColorFrame();

	Target_Speed = K210_AVOID_BASE_SPEED;

	if(frame.pos == 'L')
	{
		Turn_Speed = -K210_AVOID_TURN_STRONG;
	}
	else if(frame.pos == 'R')
	{
		Turn_Speed = K210_AVOID_TURN_STRONG;
	}
	else
	{
		Target_Speed = K210_AVOID_BASE_SPEED / 2;
		Turn_Speed = -K210_AVOID_TURN_MID;
	}
}

static u8 K210_LineTrackingActive(void)
{
	K210_LineFrame_t frame = K210_GetLineFrame();

	if(K210_LineFrameAvailable() == 0)
	{
		return 0;
	}

	if(frame.lost)
	{
		return 0;
	}

	if(frame.confidence < K210_LINE_MIN_CONFIDENCE)
	{
		return 0;
	}

	return 1;
}

static void K210_ApplyLineTracking(void)
{
	Target_Speed = K210_LINE_BASE_SPEED;
	Turn_Speed = 0;
}

static int K210_LineTurnPD(void)
{
	float turn;
	K210_LineFrame_t frame = K210_GetLineFrame();

	turn = frame.error * K210_LINE_ERROR_KP + frame.angle * K210_LINE_ANGLE_KD;

	if(turn > K210_LINE_TURN_LIMIT)
	{
		turn = K210_LINE_TURN_LIMIT;
	}
	else if(turn < -K210_LINE_TURN_LIMIT)
	{
		turn = -K210_LINE_TURN_LIMIT;
	}

	return (int)turn;
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
 * 当前只允许 K210 色块避障和 K210 视觉巡线接管。
 * 两者都无效时，停止前进，只保持平衡。
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

			/* 2. K210 视觉结果保活。 */
			K210_ColorFrameHeartbeat();
			K210_LineFrameHeartbeat();

			/* 3. 高层行为仲裁。 */
			if(K210_AvoidanceActive())
			{
				K210_ApplyAvoidance();
			}
			else if(K210_LineTrackingActive())
			{
				K210_ApplyLineTracking();
			}
			else
			{
				/* K210 无有效视觉结果时，只保持平衡，不继续前进。 */
				Target_Speed = 0;
				Turn_Speed = 0;
			}

			/* 4. 转向目标统一限幅，防止过激。 */
			Turn_Speed = Turn_Speed > SPEED_Z ? SPEED_Z : (Turn_Speed < (-SPEED_Z) ? (-SPEED_Z) : Turn_Speed);

			/* 5. 转向环阻尼约束。 */
			if(K210_AvoidanceActive())
			{
				Turn_Kd = 0;
			}
			else if(K210_LineTrackingActive())
			{
				Turn_Kd = 0;
			}
			else
			{
				Turn_Kd = 0.6f;
			}

			/* 6. 三环控制计算。 */
			Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
			Vertical_out = Vertical(Velocity_out + Med_Angle, Pitch, gyroy);

			if(K210_AvoidanceActive())
			{
				/* 视觉避障模式：根据目标位置做差速绕行。 */
				Turn_out = Turn(gyroz, Turn_Speed);
			}
			else if(K210_LineTrackingActive())
			{
				Turn_out = K210_LineTurnPD();
			}
			else
			{
				Turn_out = Turn(gyroz, 0);
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
 * 3. RC 当前只作为 K210 无效时的零转向兜底
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
