#include "stm32f10x.h"
#include "sys.h"
#include "oled.h"
#include "usart.h"
#include "tracking.h"

#define TRACKING_START_DELAY_MS 10000
#define MAIN_LOOP_DELAY_MS 5
#define USART1_PRINT_PERIOD_MS 300

float Pitch,Roll,Yaw;
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;
int Encoder_Left,Encoder_Right;

int PWM_MAX=7200,PWM_MIN=-7200;
int MOTO1,MOTO2;
extern int Vertical_out,Velocity_out,Turn_out;

int main(void)
{
	u16 print_elapsed_ms = 0;

	delay_init();
	NVIC_Config();
	uart1_init(9600);

	uart3_init(9600);
	Tracking_Usart2_Init(115200);

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

	delay_ms(TRACKING_START_DELAY_MS);
	Tracking_SendControlData(0,0,1);

	while(1)
	{
		Tracking_ProcessRx();
		OLED_Float(1,70,Pitch,1);
		OLED_Num3(8,2,Tracking_GetError());
		OLED_Num3(5,3,(int)((Encoder_Left+Encoder_Right)*2.38));

		if(print_elapsed_ms >= USART1_PRINT_PERIOD_MS)
		{
			print_elapsed_ms = 0;
			printf("Pitch=%.2f Roll=%.2f Yaw=%.2f\nTrackError=%d EncoderLeft=%d EncoderRight=%d SpeedDisplay=%.2f\nDigital=[%u,%u,%u,%u,%u,%u,%u,%u]\r\n",
			       Pitch,
			       Roll,
			       Yaw,
			       Tracking_GetError(),
			       Encoder_Left,
			       Encoder_Right,
			       (Encoder_Left + Encoder_Right) * 2.38f,
			       Tracking_IR_Data[0], Tracking_IR_Data[1], Tracking_IR_Data[2], Tracking_IR_Data[3],
			       Tracking_IR_Data[4], Tracking_IR_Data[5], Tracking_IR_Data[6], Tracking_IR_Data[7]);
		}

		delay_ms(MAIN_LOOP_DELAY_MS);
		print_elapsed_ms += MAIN_LOOP_DELAY_MS;
	}
}
