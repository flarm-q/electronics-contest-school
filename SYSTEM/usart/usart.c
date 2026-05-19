#include "usart.h"	  
#include "control.h"
#include "string.h"
#include "stdlib.h"



//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
u8 USART_RX_BUF[64];     //接收缓冲,最大64个字节.
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
u8 USART_RX_STA=0;       //接收状态标记

static void USART1_PrintPidValues(void)
{
	float vkp, vkd, skp, ski, rkp, rkd;
	float tkp, tki, tkd;

	Control_GetPidValues(&vkp, &vkd, &skp, &ski, &rkp, &rkd);
	Tracking_GetPidValues(&tkp, &tki, &tkd);
	printf("PID Vertical_Kp=%.4f Vertical_Kd=%.4f Velocity_Kp=%.4f Velocity_Ki=%.6f Turn_Kp=%.4f Turn_Kd=%.4f Tracking_Kp=%.4f Tracking_Ki=%.4f Tracking_Kd=%.4f\r\n",
	       vkp, vkd, skp, ski, rkp, rkd, tkp, tki, tkd);
}

void USART1_ProcessCommand(void)
{
	char line[64];
	char cmd[16];
	char name[16];
	float value;
	u8 len;
	u8 i;

	if((USART_RX_STA & 0x80) == 0)
	{
		return;
	}

	len = USART_RX_STA & 0x3F;
	if(len >= sizeof(line))
	{
		len = sizeof(line) - 1;
	}

	for(i = 0; i < len; i++)
	{
		line[i] = (char)USART_RX_BUF[i];
	}
	line[len] = '\0';
	USART_RX_STA = 0;
	memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));

	if((strcmp(line, "GET PID") == 0) || (strcmp(line, "PID?") == 0))
	{
		USART1_PrintPidValues();
		return;
	}

	if(strcmp(line, "HELP") == 0)
	{
		printf("CMD: SET VKP/VKD/SKP/SKI/RKP/RKD/TKP/TKI/TKD val | GET PID\r\n");
		return;
	}

	if(sscanf(line, "%15s %15s %f", cmd, name, &value) == 3)
	{
		if(strcmp(cmd, "SET") == 0)
		{
			if(Control_SetPidByName(name, value) || Tracking_SetPidByName(name, value))
			{
				printf("OK %s=%.6f\r\n", name, value);
				USART1_PrintPidValues();
			}
			else
			{
				printf("ERR Unknown PID name: %s\r\n", name);
			}
			return;
		}
	}

	printf("ERR Invalid command. Type HELP\r\n");
}

void uart1_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART1, ENABLE);                    //使能串口 
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x80)==0)//接收未完成
			{
			if(USART_RX_STA&0x40)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x80;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x40;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3F]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>63)USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
} 
