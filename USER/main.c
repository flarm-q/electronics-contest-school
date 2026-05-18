#include "stm32f10x.h"
#include "sys.h" 
#include "oled.h"
#define TRACKING_START_DELAY_MS 8000
#define TRIG PAout(3) // 超声波触发引脚输出
#define ECHO PAin(2)  // 超声波回响引脚输入
int overcount=0;      // 记录定时器溢出次数
int length;

float Pitch,Roll,Yaw;                        // 姿态角
short gyrox,gyroy,gyroz;                // 陀螺仪角速度
short aacx,aacy,aacz;                        // 加速度数据
int Encoder_Left,Encoder_Right;    // 左右编码器速度

int PWM_MAX=7200,PWM_MIN=-7200;    // PWM限幅
int MOTO1,MOTO2;                                // 电机输出量

extern int Vertical_out,Velocity_out,Turn_out;
void TIM3_Int_Init()
{
		GPIO_InitTypeDef GPIO_InitStruct;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//		NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 使能TIM3时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
		
        GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;// 配置为推挽输出
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
	
        // TIM3基础计时初始化
        TIM_TimeBaseStructure.TIM_Period = 999; // 自动重装值 ARR
        TIM_TimeBaseStructure.TIM_Prescaler =7199; // 预分频系数 PSC
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

//        TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );// 使能TIM3更新中断
//        // 中断优先级配置
//		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
			
		TIM_Cmd(TIM3, DISABLE);
}
// 超声波测距函数
int Senor_Using() // 返回整型距离值
{
		unsigned int sum=0;
		unsigned int tim;
		unsigned int i=0;
		unsigned int length;
		u16 cnt_i=0;
        while(i!=3)        // 连续测量3次后取平均值
		{
            TRIG=1;          // 拉高触发信号
            delay_us(20);    // 保持高电平20us
            TRIG=0;          // 等待回响信号
			cnt_i=0;
			while(ECHO==0){cnt_i++;delay_us(20);if(cnt_i>2000)
            {TRIG=1;          // 超时后重新触发一次
            delay_us(20);    // 保持触发脉冲宽度
            TRIG=0;cnt_i=0;          }} // 检测到回响后开始计时
			TIM_Cmd(TIM3,ENABLE);
			
            i+=1;                     // 完成一次测量后计数加1
			
            while(ECHO==1);    // 等待回响信号结束
            TIM_Cmd(TIM3,DISABLE);    // 关闭定时器
			
            tim=TIM_GetCounter(TIM3);         // 读取TIM3当前计数值
            length=(tim*100)/58.0; // 根据回响时间换算距离
			if(length>300)length=300;
			sum=length+sum;
            TIM3->CNT=0; // 清零TIM3计数器
            overcount=0;                                                                // 清零溢出次数
			delay_ms(100);
		}
		length=sum/3;
        return length; // 返回平均距离
}

int main(void)	
{
	delay_init();
	NVIC_Config();
	uart1_init(115200);	

	uart3_init(9600);// 串口3波特率9600  
	Tracking_Usart2_Init(115200);// 巡线模块使用USART2：PA2(TX)、PA3(RX)，超声波测距已关闭以避免引脚冲突

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
    /* 上电后先保持直立，等待姿态稳定3秒，再开启巡线模块数据上报。 */
	delay_ms(TRACKING_START_DELAY_MS);
    Tracking_SendControlData(0,0,1);// 请求巡线模块持续发送数字量数据帧：$D,x1:0,...,x8:0#
  while(1)	
	{
		
		OLED_Float(1,70,Pitch,1);
        OLED_Num3(8,2,Tracking_GetError());// 显示当前巡线偏差，负值偏左，正值偏右
		OLED_Num3(5,3,(int)((Encoder_Left+Encoder_Right)*2.38));		
	} 	
}

//void TIM3_IRQHandler(void)
//{
//        if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) // 检查是否发生TIM3更新中断
//		{
//            TIM_ClearITPendingBit(TIM3, TIM_IT_Update );   // 清除更新中断标志
//			overcount++;
//		}
//}
