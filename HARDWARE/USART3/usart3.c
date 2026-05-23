#include "usart3.h"

/* USART3 接收缓冲区最大长度。
 * 当前 K210 巡线帧很短，例如 "$L,-23,5,86,1#"，
 * 48 字节已经足够，也能限制异常数据导致的越界风险。
 */
#define K210_USART3_FRAME_MAX_LEN 48

/* 巡线/颜色结果超时计数。
 * 该值不是毫秒，而是控制周期计数。
 */
#define K210_COLOR_FRAME_TIMEOUT_TICKS 40
#define K210_LINE_FRAME_TIMEOUT_TICKS 40

/* 兼容保留的原蓝牙方向变量。 */
u8 Fore, Back, Left, Right;

/* 最近一次收到并成功解析的 K210 颜色识别结果。 */
static volatile K210_ColorFrame_t g_k210_color = {0, 'N', 0, 0};

/* 最近一次收到并成功解析的 K210 巡线结果。 */
static volatile K210_LineFrame_t g_k210_line = {0, 0, 0, 0x02, 0, 1};

/* 巡线/颜色结果保活倒计时。 */
static volatile u8 g_k210_color_frame_counter = 0;
static volatile u8 g_k210_line_frame_counter = 0;

/* USART3 原始字节接收缓冲区。 */
static u8 g_k210_rx_buf[K210_USART3_FRAME_MAX_LEN];

static void K210_ResetRxState(u8 *start, u8 *index)
{
	*start = 0;
	*index = 0;
	memset(g_k210_rx_buf, 0, sizeof(g_k210_rx_buf));
}

static void K210_ParseColorFrame(void)
{
	int color_id;
	char pos;
	int size;
	int matched;
	K210_ColorFrame_t frame;

	matched = sscanf((char *)g_k210_rx_buf, "$C,%d,%c,%d#", &color_id, &pos, &size);
	if(matched != 3)
	{
		return;
	}

	if(color_id < 0)
	{
		color_id = 0;
	}
	else if(color_id > 255)
	{
		color_id = 255;
	}

	if(size < 0)
	{
		size = 0;
	}
	else if(size > 65535)
	{
		size = 65535;
	}

	frame.color_id = (u8)color_id;
	frame.pos = pos;
	frame.size = (u16)size;
	frame.active = (frame.color_id != 0) ? 1 : 0;

	g_k210_color = frame;
	g_k210_color_frame_counter = K210_COLOR_FRAME_TIMEOUT_TICKS;

	Fore = 0;
	Back = 0;
	Left = 0;
	Right = 0;
}

static void K210_ParseLineFrame(void)
{
	int error;
	int angle;
	int confidence;
	int flags;
	int matched;
	K210_LineFrame_t frame;

	matched = sscanf((char *)g_k210_rx_buf, "$L,%d,%d,%d,%d#", &error, &angle, &confidence, &flags);
	if(matched != 4)
	{
		return;
	}

	if(error > 32767)
	{
		error = 32767;
	}
	else if(error < -32768)
	{
		error = -32768;
	}

	if(angle > 32767)
	{
		angle = 32767;
	}
	else if(angle < -32768)
	{
		angle = -32768;
	}

	if(confidence > 100)
	{
		confidence = 100;
	}
	else if(confidence < 0)
	{
		confidence = 0;
	}

	if(flags > 255)
	{
		flags = 255;
	}
	else if(flags < 0)
	{
		flags = 0;
	}

	frame.error = (s16)error;
	frame.angle = (s16)angle;
	frame.confidence = (u8)confidence;
	frame.flags = (u8)flags;
	frame.lost = (frame.flags & 0x02) ? 1 : 0;
	frame.active = ((frame.flags & 0x01) && (frame.lost == 0)) ? 1 : 0;

	g_k210_line = frame;
	g_k210_line_frame_counter = K210_LINE_FRAME_TIMEOUT_TICKS;

	Fore = 0;
	Back = 0;
	Left = 0;
	Right = 0;
}

void uart3_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* PB10 -> USART3_TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB11 -> USART3_RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

u8 K210_ColorFrameAvailable(void)
{
	return g_k210_color.active;
}

K210_ColorFrame_t K210_GetColorFrame(void)
{
	return *(K210_ColorFrame_t *)&g_k210_color;
}

void K210_ColorFrameHeartbeat(void)
{
	if(g_k210_color_frame_counter > 0)
	{
		g_k210_color_frame_counter--;
		if(g_k210_color_frame_counter == 0)
		{
			g_k210_color.color_id = 0;
			g_k210_color.pos = 'N';
			g_k210_color.size = 0;
			g_k210_color.active = 0;
		}
	}
}

u8 K210_LineFrameAvailable(void)
{
	return g_k210_line.active;
}

K210_LineFrame_t K210_GetLineFrame(void)
{
	return *(K210_LineFrame_t *)&g_k210_line;
}

void K210_LineFrameHeartbeat(void)
{
	if(g_k210_line_frame_counter > 0)
	{
		g_k210_line_frame_counter--;
		if(g_k210_line_frame_counter == 0)
		{
			g_k210_line.error = 0;
			g_k210_line.angle = 0;
			g_k210_line.confidence = 0;
			g_k210_line.flags = 0x02;
			g_k210_line.active = 0;
			g_k210_line.lost = 1;
		}
	}
}

void USART3_IRQHandler(void)
{
	static u8 start = 0;
	static u8 index = 0;
	u8 rx_temp;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		rx_temp = (u8)USART_ReceiveData(USART3);

		if(rx_temp == '$')
		{
			start = 1;
			index = 0;
			memset(g_k210_rx_buf, 0, sizeof(g_k210_rx_buf));
			g_k210_rx_buf[index++] = rx_temp;
			return;
		}

		if(start == 0)
		{
			return;
		}

		if(index >= K210_USART3_FRAME_MAX_LEN - 1)
		{
			K210_ResetRxState(&start, &index);
			return;
		}

		g_k210_rx_buf[index++] = rx_temp;

		if(rx_temp == '#')
		{
			g_k210_rx_buf[index] = '\0';
			if(g_k210_rx_buf[1] == 'C')
			{
				K210_ParseColorFrame();
			}
			else if(g_k210_rx_buf[1] == 'L')
			{
				K210_ParseLineFrame();
			}
			K210_ResetRxState(&start, &index);
		}
	}
}

void USART3_Send_Data(char data)
{
	USART_SendData(USART3, data);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
}

void USART3_Send_String(char *String)
{
	u16 len, j;

	len = strlen(String);
	for(j = 0; j < len; j++)
	{
		USART3_Send_Data(*String++);
	}
}
