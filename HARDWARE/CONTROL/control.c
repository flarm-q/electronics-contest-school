#include "control.h"

/* 机械中值。
 * 如果车体存在静态前倾或后仰，需要通过这里微调，让平衡点更准确。
 */
float Med_Angle = 0;

/* 高层控制目标：
 * Target_Speed 控制前后速度
 * Turn_Speed   控制左右转向
 * 两者不会直接驱动电机，而是交给后面的闭环控制统一处理。
 */
float Target_Speed = 0;
float Turn_Speed = 0;

/* 三个控制环参数：
 * 1. 直立环：根据角度和角速度保持车体直立
 * 2. 速度环：根据编码器速度控制前后运动
 * 3. 转向环：根据陀螺仪 Z 轴和目标转向量控制左右差速
 */
float Vertical_Kp = -430;
float Vertical_Kd = -1.92f;
float Velocity_Kp = -0.44f;
float Velocity_Ki = -0.0022f;
float Turn_Kd = 0.6f;
float Turn_Kp = 20;

extern int length;

/* 原始速度和转向限幅。 */
#define SPEED_Y 40
#define SPEED_Z 100

/* K230 视觉避障参数。
 * K230 通过 USART3 发送颜色识别结果帧：$C,color_id,pos,size#
 * STM32 收到后，不直接改 PWM，而是只改高层速度和转向目标，
 * 最终仍然复用平衡车原来的速度环、直立环和转向环。
 */
#define K230_AVOID_BASE_SPEED 8
#define K230_AVOID_TURN_STRONG 45
#define K230_AVOID_TURN_MID 30

/* 目标面积阈值。
 * 太小的色块通常是远处目标、噪点或误检，不足以触发避障。
 */
#define K230_AVOID_MIN_SIZE 200

int Vertical_out, Velocity_out, Turn_out;

int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_left, int encoder_right);
int Turn(int gyro_Z, int RC);

static u8 K230_AvoidanceActive(void)
{
	K230_ColorFrame_t frame = K230_GetColorFrame();

	/* active=0 表示：
	 * 1. K230 明确上报无目标
	 * 2. 或者视觉数据已经超时失效
	 */
	if(frame.active == 0)
	{
		return 0;
	}

	/* 当前只接收红、绿、蓝三类目标。 */
	if(frame.color_id < 1 || frame.color_id > 3)
	{
		return 0;
	}

	/* 面积过小不触发避障。 */
	if(frame.size < K230_AVOID_MIN_SIZE)
	{
		return 0;
	}

	return 1;
}

static void K230_ApplyAvoidance(void)
{
	K230_ColorFrame_t frame = K230_GetColorFrame();

	/* 视觉避障策略：
	 * 1. 只要检测到红、绿、蓝目标，就统一视为前方障碍物
	 * 2. 目标在左边，小车向右绕开
	 * 3. 目标在右边，小车向左绕开
	 * 4. 目标在中间时最危险，先减速，再优先右转
	 *
	 * 注意：
	 * 这里改的是 Target_Speed 和 Turn_Speed，
	 * 不是直接操作电机 PWM，因此动作会更平滑。
	 */
	Target_Speed = K230_AVOID_BASE_SPEED;

	if(frame.pos == 'L')
	{
		/* 障碍在左，负值右转。 */
		Turn_Speed = -K230_AVOID_TURN_STRONG;
	}
	else if(frame.pos == 'R')
	{
		/* 障碍在右，正值左转。 */
		Turn_Speed = K230_AVOID_TURN_STRONG;
	}
	else
	{
		/* 障碍在中间，优先降速并右转。 */
		Target_Speed = K230_AVOID_BASE_SPEED / 2;
		Turn_Speed = -K230_AVOID_TURN_MID;
	}
}

void EXTI9_5_IRQHandler(void)
{
	int PWM_out;

	if(EXTI_GetITStatus(EXTI_Line5) != 0)
	{
		if(PBin(5) == 0)
		{
			EXTI_ClearITPendingBit(EXTI_Line5);

			/* 1. 采集编码器、角度、角速度和加速度。 */
			Encoder_Left = -Read_Speed(2);
			Encoder_Right = Read_Speed(4);

			mpu_dmp_get_data(&Roll, &Pitch, &Yaw);
			MPU_Get_Gyroscope(&gyroy, &gyrox, &gyroz);
			MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

			/* 2. 视觉结果保活。
			 * 如果 K230 长时间不再发送新帧，这里会让旧视觉结果自动失效，
			 * 避免小车一直沿用上一次识别状态。
			 */
			K230_ColorFrameHeartbeat();

			/* 3. 高层行为优先级：
			 * 视觉避障 > 八路巡线 > 原手动控制
			 */
			if(K230_AvoidanceActive())
			{
				K230_ApplyAvoidance();
			}
			else if(Tracking_IsActive())
			{
				/* 没有颜色障碍时，巡线模块接管前进。 */
				Tracking_SetSpeed();
				Turn_Speed = 0;
			}
			else
			{
				/* 原手动控制兜底逻辑。
				 * 当前 USART3 蓝牙控制已停用，这里通常会回到零速度。
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
			 * 视觉避障和巡线模式下，转向由对应模块接管；
			 * 只有都未生效时，才走原始左右方向控制。
			 */
			if(K230_AvoidanceActive())
			{
				/* Turn_Speed 已在 K230_ApplyAvoidance() 中赋值。 */
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

			/* 对所有高层给出的转向目标统一限幅。 */
			Turn_Speed = Turn_Speed > SPEED_Z ? SPEED_Z : (Turn_Speed < (-SPEED_Z) ? (-SPEED_Z) : Turn_Speed);

			/* 5. 转向约束。
			 * 视觉避障或巡线模式下，不叠加原始手动约束。
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

			/* 6. 闭环控制计算。
			 * 速度环决定前后趋势
			 * 直立环保证车身站稳
			 * 转向环决定左右差速
			 */
			Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
			Vertical_out = Vertical(Velocity_out + Med_Angle, Pitch, gyroy);

			if(K230_AvoidanceActive())
			{
				/* 视觉避障模式：根据颜色目标位置做差速绕行。 */
				Turn_out = Turn(gyroz, Turn_Speed);
			}
			else if(Tracking_IsActive())
			{
				/* 巡线模式：根据八路红外偏差生成转向。 */
				Turn_out = Tracking_TurnPD(gyroz);
			}
			else
			{
				Turn_out = Turn(gyroz, Turn_Speed);
			}

			/* 7. 合成左右电机输出并加载。 */
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

int Vertical(float Med, float Angle, float gyro_Y)
{
	int PWM_out;

	/* 直立环 PD：
	 * 角度偏差负责把车拉回平衡点，
	 * 角速度项负责抑制摆动和过冲。
	 */
	PWM_out = Vertical_Kp * (Angle - Med) + Vertical_Kd * (gyro_Y - 0);
	return PWM_out;
}

int Velocity(int Target, int encoder_left, int encoder_right)
{
	static int Encoder_S, EnC_Err_Lowout_last, PWM_out, Encoder_Err, EnC_Err_Lowout;
	float a = 0.7f;

	/* 速度环 PI：
	 * 先计算当前速度误差，再做简单低通滤波，
	 * 之后积分得到位移趋势，最后输出速度修正量。
	 */
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

int Turn(int gyro_Z, int RC)
{
	int PWM_out;

	/* 转向环：
	 * gyro_Z 项抑制车体自身转向摆动，
	 * RC 项表示外部给定的目标转向量。
	 * 这里的 RC 既可以来自原始手动控制，也可以来自 K230 视觉避障。
	 */
	PWM_out = Turn_Kd * gyro_Z + Turn_Kp * RC;
	return PWM_out;
}
