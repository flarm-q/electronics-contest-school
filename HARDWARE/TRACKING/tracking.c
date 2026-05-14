#include "tracking.h"
#include "string.h"

/* 串口帧最大长度。参考协议最长帧约 45 字节，保留余量防止异常数据越界。 */
#define TRACKING_PACKAGE_SIZE 100

/* 巡线时给速度环的目标速度。数值越大越快，调试时建议先小后大。 */
#define TRACKING_SPEED 15

/*
 * 巡线转向参数，移植自参考工程 app_tracking.c。
 * KP 决定压线纠偏力度，KI 用于很小的长期偏差补偿，KD 使用陀螺仪 Z 轴抑制转向震荡。
 */
#define TRACKING_TURN_KP 270
#define TRACKING_TURN_KI 0.01f
#define TRACKING_TURN_KD 0.15f

/* control.c 中的速度目标，巡线模式下由 Tracking_SetSpeed() 接管。 */
extern float Target_Speed;

/* 8 路数字量缓存：0 表示该路检测到黑线，1 表示未检测到黑线。 */
volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];
volatile u8 Tracking_New_Package_Flag = 0;

/* 串口接收状态缓存，只在 USART2 中断和解析函数内部使用。 */
static u8 tracking_rx_buff[TRACKING_PACKAGE_SIZE];
static u8 tracking_new_package[TRACKING_PACKAGE_SIZE];

/* 收到有效 $D...# 数据帧后置 1，作为控制环是否启用巡线的开关。 */
static volatile u8 tracking_active = 0;

/* 当前巡线偏差，负数代表线偏左，正数代表线偏右。 */
static int tracking_error = 0;

static void Tracking_SendU8(u8 ch)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ch);
}

static void Tracking_SendArrayU8(u8 *buffer, u16 length)
{
	while(length--)
	{
		Tracking_SendU8(*buffer++);
	}
}

static void Tracking_ParseDigitalData(void)
{
	u8 i;

	/* 只处理数字量帧。模拟量帧 "$A,..." 当前不参与平衡车巡线控制。 */
	if(tracking_new_package[1] != 'D')
	{
		return;
	}

	/*
	 * 协议示例：$D,x1:0,x2:0,x3:1,x4:1,x5:1,x6:1,x7:0,x8:0#
	 * 数字字符位置固定为 6 + i * 5，直接减 '0' 转成 0/1。
	 */
	for(i = 0; i < TRACKING_IR_NUM; i++)
	{
		Tracking_IR_Data[i] = tracking_new_package[6 + i * 5] - '0';
	}

	tracking_active = 1;
	Tracking_New_Package_Flag = 1;
	memset(tracking_new_package, 0, TRACKING_PACKAGE_SIZE);
}

void Tracking_Usart2_Init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* USART2 使用 PA2/PA3。超声波测距已关闭，PA2/PA3 不再作为 TRIG/ECHO。 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* PA2: USART2_TX，向八路巡线模块发送配置命令。 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA3: USART2_RX，接收八路巡线模块数据。 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void Tracking_SendControlData(u8 adjust, u8 analogData, u8 digitalData)
{
	u8 send_buf[8] = "$0,0,0#";

	/*
	 * 控制命令格式移植自参考工程：
	 *   adjust=1     请求模块校准
	 *   analogData=1 请求模拟量数据
	 *   digitalData=1 请求数字量数据
	 * 当前只需要数字量，所以 main.c 中发送 Tracking_SendControlData(0,0,1)。
	 */
	send_buf[1] = adjust ? '1' : '0';
	send_buf[3] = analogData ? '1' : '0';
	send_buf[5] = digitalData ? '1' : '0';

	Tracking_SendArrayU8(send_buf, strlen((char *)send_buf));
}

void Tracking_DealUsart(u8 rxtemp)
{
	static u8 start = 0;
	static u8 step = 0;

	/* '$' 是帧头。重新遇到帧头时直接从新帧开始，丢弃上一包残留数据。 */
	if(rxtemp == '$')
	{
		start = 1;
		step = 0;
		tracking_rx_buff[step++] = rxtemp;
		return;
	}

	if(start == 0)
	{
		return;
	}

	tracking_rx_buff[step++] = rxtemp;
	if(rxtemp == '#')
	{
		/* '#' 是帧尾。完整帧到达后立即解析，控制中断可直接读取最新状态。 */
		start = 0;
		step = 0;
		memcpy(tracking_new_package, tracking_rx_buff, TRACKING_PACKAGE_SIZE);
		memset(tracking_rx_buff, 0, TRACKING_PACKAGE_SIZE);
		Tracking_ParseDigitalData();
		return;
	}

	if(step >= TRACKING_PACKAGE_SIZE)
	{
		/* 异常长数据说明帧损坏，复位接收状态，等待下一次 '$'。 */
		start = 0;
		step = 0;
		memset(tracking_rx_buff, 0, TRACKING_PACKAGE_SIZE);
	}
}

u8 Tracking_IsActive(void)
{
	return tracking_active;
}

void Tracking_SetSpeed(void)
{
	Target_Speed = TRACKING_SPEED;
}

int Tracking_GetError(void)
{
	/*
	 * x1 在最左侧，x8 在最右侧。
	 * 参考工程定义：0 表示压线，1 表示未压线。
	 * 输出偏差范围大致为 -5~5，绝对值越大表示偏离中心越严重。
	 */
	u8 x1 = Tracking_IR_Data[0];
	u8 x2 = Tracking_IR_Data[1];
	u8 x3 = Tracking_IR_Data[2];
	u8 x4 = Tracking_IR_Data[3];
	u8 x5 = Tracking_IR_Data[4];
	u8 x6 = Tracking_IR_Data[5];
	u8 x7 = Tracking_IR_Data[6];
	u8 x8 = Tracking_IR_Data[7];

	if(x1 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x8 == 0) tracking_error = 0;
	else if((x1 == 0 || x2 == 0) && x8 == 1) tracking_error = -5;
	else if((x7 == 0 || x8 == 0) && x1 == 1) tracking_error = 5;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -1;
	else if(x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -2;
	else if(x1 == 1 && x2 == 1 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -2;
	else if(x1 == 1 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -3;
	else if(x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -3;
	else if(x1 == 0 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -4;
	else if(x1 == 0 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = -4;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1) tracking_error = 1;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1) tracking_error = 2;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 1 && x8 == 1) tracking_error = 2;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 1) tracking_error = 3;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 1) tracking_error = 3;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 0) tracking_error = 4;
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 0) tracking_error = 4;
	else if(x1 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x8 == 1) tracking_error = 0;

	return tracking_error;
}

int Tracking_TurnPD(int gyro_z)
{
	static float integral = 0;
	float err = Tracking_GetError();

	/*
	 * 转向输出最终在 control.c 中做差速：
	 *   MOTO1 = PWM_out - Turn_out
	 *   MOTO2 = PWM_out + Turn_out
	 * 如果实车方向相反，优先调整这里的 TRACKING_TURN_KP 符号。
	 */
	integral += err;
	return (int)(err * TRACKING_TURN_KP + integral * TRACKING_TURN_KI + gyro_z * TRACKING_TURN_KD);
}

void USART2_IRQHandler(void)
{
	u8 rx_temp;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		/* 中断只做字节接收和轻量解析，避免阻塞姿态控制中断。 */
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		rx_temp = USART_ReceiveData(USART2);
		Tracking_DealUsart(rx_temp);
	}
}
