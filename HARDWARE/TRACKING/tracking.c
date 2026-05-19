#include "tracking.h"
#include "string.h"

#define TRACKING_PACKAGE_SIZE 100
#define TRACKING_SPEED -8
#define TRACKING_TURN_KP 400
#define TRACKING_TURN_KI 0.0f
#define TRACKING_TURN_KD 0.10f

extern float Target_Speed;

volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];
volatile u16 Tracking_Analog_Data[TRACKING_IR_NUM];
volatile u8 Tracking_New_Package_Flag = 0;

static u8 tracking_rx_buff[TRACKING_PACKAGE_SIZE];
static u8 tracking_new_package[TRACKING_PACKAGE_SIZE];
static volatile u8 tracking_irq_fifo[TRACKING_PACKAGE_SIZE];
static volatile u16 tracking_irq_fifo_head = 0;
static volatile u16 tracking_irq_fifo_tail = 0;

static volatile u8 tracking_active = 0;
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
	const char *cursor = (const char *)tracking_new_package;
	u8 index = 0;

	if(tracking_new_package[1] != 'D')
	{
		return;
	}

	while((*cursor != '\0') && (index < TRACKING_IR_NUM))
	{
		if((*cursor == ':') && ((cursor[1] == '0') || (cursor[1] == '1')))
		{
			Tracking_IR_Data[index++] = (u8)(cursor[1] - '0');
			cursor += 2;
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

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

	send_buf[1] = adjust ? '1' : '0';
	send_buf[3] = analogData ? '1' : '0';
	send_buf[5] = digitalData ? '1' : '0';

	Tracking_SendArrayU8(send_buf, strlen((char *)send_buf));
}

void Tracking_DealUsart(u8 rxtemp)
{
	static u8 start = 0;
	static u8 step = 0;

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
		start = 0;
		step = 0;
		memset(tracking_rx_buff, 0, TRACKING_PACKAGE_SIZE);
	}
}

void Tracking_ProcessRx(void)
{
	u8 budget = 16;
	u8 rx_temp;
	u16 tail;

	while(budget--)
	{
		tail = tracking_irq_fifo_tail;
		if(tail == tracking_irq_fifo_head)
		{
			break;
		}

		rx_temp = tracking_irq_fifo[tail];
		tail++;
		if(tail >= TRACKING_PACKAGE_SIZE)
		{
			tail = 0;
		}
		tracking_irq_fifo_tail = tail;
		Tracking_DealUsart(rx_temp);
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
	int weighted_sum = 0;
	u8 hit_count = 0;

	for(i = 0; i < TRACKING_IR_NUM; i++)
	{
		/* 数字量协议里 0 表示检测到黑线，1 表示未检测到黑线。 */
		if(Tracking_IR_Data[i] == 0)
		{
			weighted_sum += tracking_weights[i];
			hit_count++;
		}
	}

	if(hit_count > 0)
	{
		tracking_error = weighted_sum / hit_count;
	}

	return tracking_error;
}

int Tracking_TurnPD(int gyro_z)
{
	static float integral = 0;
	float err = Tracking_GetError();

	integral += err;
	return (int)(err * TRACKING_TURN_KP + integral * TRACKING_TURN_KI + gyro_z * TRACKING_TURN_KD);
}

void USART2_IRQHandler(void)
{
	u8 rx_temp;
	u16 next_head;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		rx_temp = USART_ReceiveData(USART2);

		next_head = tracking_irq_fifo_head + 1;
		if(next_head >= TRACKING_PACKAGE_SIZE)
		{
			next_head = 0;
		}

		if(next_head != tracking_irq_fifo_tail)
		{
			tracking_irq_fifo[tracking_irq_fifo_head] = rx_temp;
			tracking_irq_fifo_head = next_head;
		}
	}
}
