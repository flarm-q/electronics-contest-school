#include "stm32f10x.h"
#include "sys.h"
#include "oled.h"
#include "usart.h"
#include "tracking.h"

#define TRACKING_START_DELAY_MS 10000
#define USART1_PRINT_PERIOD_MS 100

float Pitch,Roll,Yaw;                    // 姿态角
short gyrox,gyroy,gyroz;                 // 陀螺仪角速度
short aacx,aacy,aacz;                    // 加速度数据
int Encoder_Left,Encoder_Right;          // 左右编码器速度

int PWM_MAX=7200,PWM_MIN=-7200;          // PWM限幅
int MOTO1,MOTO2;                         // 电机输出量

extern int Vertical_out,Velocity_out,Turn_out;

int main(void)
{
	delay_init();
	NVIC_Config();
	uart1_init(9600);

	uart3_init(9600); // 串口3波特率9600
	Tracking_Usart2_Init(115200); // 巡线模块使用USART2：PA2(TX)、PA3(RX)，超声波测距已关闭以避免引脚冲突

	OLED_Init();
	OLED_Clear();

	MPU_Init();
	mpu_dmp_init();
	MPU6050_EXTI_Init();

	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	Motor_Init();
	PWM_Init_TIM1(0,7199);

	OLED_ShowString(0,1,"jiao du:",12);
	OLED_ShowString(0,2,"track:",12);
	OLED_ShowString(0,3,"sd:",12);

	/* 上电后先保持直立，等待姿态稳定10秒，再开启巡线模块数据上报。 */
	delay_ms(TRACKING_START_DELAY_MS);
	Tracking_SendControlData(0,1,0); // 请求巡线模块持续发送模拟量数据帧：$A,...#

	while(1)
	{
		OLED_Float(1,70,Pitch,1);
		OLED_Num3(8,2,Tracking_GetError()); // 显示当前巡线偏差，负值偏左，正值偏右
		OLED_Num3(5,3,(int)((Encoder_Left+Encoder_Right)*2.38));

		printf("Pitch=%.2f Roll=%.2f Yaw=%.2f Track=%d Left=%d Right=%d A=[%u,%u,%u,%u,%u,%u,%u,%u]\r\n",
		       Pitch, Roll, Yaw, Tracking_GetError(), Encoder_Left, Encoder_Right,
		       Tracking_Analog_Data[0], Tracking_Analog_Data[1], Tracking_Analog_Data[2], Tracking_Analog_Data[3],
		       Tracking_Analog_Data[4], Tracking_Analog_Data[5], Tracking_Analog_Data[6], Tracking_Analog_Data[7]);
		delay_ms(USART1_PRINT_PERIOD_MS);
	}
}
