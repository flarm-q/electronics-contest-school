#include "tracking.h"
#include "string.h"

/* 串口帧最大长度。参考协议最长帧约 45 字节，保留余量防止异常数据越界。 */
#define TRACKING_PACKAGE_SIZE 100

/* 巡线时给速度环的目标速度。数值越大越快，调试时建议先小后大。 */
#define TRACKING_SPEED -8

/* 模拟量巡线的加权中心参数。 */
#define TRACKING_CENTER_SCALE 100	// 后调
#define TRACKING_ANALOG_THRESHOLD 80  // 先调

/*
 * 巡线转向参数，移植自参考工程 app_tracking.c。
 * KP 决定压线纠偏力度，KI 用于很小的长期偏差补偿，KD 使用陀螺仪 Z 轴抑制转向震荡。
 */
#define TRACKING_TURN_KP 400
#define TRACKING_TURN_KI 0.0f
#define TRACKING_TURN_KD 0.10f

/* control.c 中的速度目标，巡线模式下由 Tracking_SetSpeed() 接管。 */
extern float Target_Speed;

/* 8 路数字量缓存：0 表示该路检测到黑线，1 表示未检测到黑线。 */
volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];
volatile u16 Tracking_Analog_Data[TRACKING_IR_NUM];
volatile u8 Tracking_New_Package_Flag = 0;

/* 串口接收状态缓存，只在 USART2 中断和解析函数内部使用。 */
static u8 tracking_rx_buff[TRACKING_PACKAGE_SIZE];
static u8 tracking_new_package[TRACKING_PACKAGE_SIZE];

/* 收到有效 $D...# 数据帧后置 1，作为控制环是否启用巡线的开关。 */
static volatile u8 tracking_active = 0;

/* 当前巡线偏差，负数代表线偏左，正数代表线偏右。 */
static int tracking_error = 0;
static const int tracking_weights[TRACKING_IR_NUM] = {-350, -250, -150, -50, 50, 150, 250, 350};

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

static u16 Tracking_ParseU16(const char *text)
{
	u16 value = 0;

	while((*text >= '0') && (*text <= '9'))
	{
		value = (u16)(value * 10 + (u16)(*text - '0'));
		text++;
	}
	return value;
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

static void Tracking_ParseAnalogData(void)
{
	const char *cursor = (const char *)tracking_new_package;
	u8 index = 0;

	if(tracking_new_package[1] != 'A')
	{
		return;
	}

	while((*cursor != '\0') && (index < TRACKING_IR_NUM))
	{
		if((*cursor >= '0') && (*cursor <= '9'))
		{
			Tracking_Analog_Data[index++] = Tracking_ParseU16(cursor);
			while((*cursor >= '0') && (*cursor <= '9'))
			{
				cursor++;
			}
			continue;
		}
		cursor++;
	}

	if(index < TRACKING_IR_NUM)
	{
		return;
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
	 * 当前巡线优先使用模拟量，所以 main.c 中发送 Tracking_SendControlData(0,1,0)。
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
		if(tracking_new_package[1] == 'A')
		{
			Tracking_ParseAnalogData();
		}
		else
		{
			Tracking_ParseDigitalData();
		}
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
	u8 i;
	u32 weighted_sum = 0;
	u32 strength_sum = 0;
	u16 min_value = Tracking_Analog_Data[0];
	u16 max_value = Tracking_Analog_Data[0];

	for(i = 0; i < TRACKING_IR_NUM; i++)
	{
		if(Tracking_Analog_Data[i] < min_value)
		{
			min_value = Tracking_Analog_Data[i];
		}
		if(Tracking_Analog_Data[i] > max_value)
		{
			max_value = Tracking_Analog_Data[i];
		}
	}

	for(i = 0; i < TRACKING_IR_NUM; i++)
	{
		u16 strength = (u16)(max_value - Tracking_Analog_Data[i]);

		if(strength <= TRACKING_ANALOG_THRESHOLD)
		{
			strength = 0;
		}
		else
		{
			strength = (u16)(strength - TRACKING_ANALOG_THRESHOLD);
		}

		strength_sum += strength;
		weighted_sum += (u32)(strength * (u16)(tracking_weights[i] + 400));
	}

	if(strength_sum > 0)
	{
		int weighted_center = (int)(weighted_sum / strength_sum) - 400;
		tracking_error = weighted_center / TRACKING_CENTER_SCALE;
	}

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
