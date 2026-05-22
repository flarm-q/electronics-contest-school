#include "usart3.h"

/* USART3 接收缓冲区最大长度。
 * 当前颜色协议帧很短，例如 "$C,1,L,356#"，32 字节已经足够，
 * 同时也能限制异常数据导致的越界风险。
 */
#define K230_USART3_FRAME_MAX_LEN 64

/* 视觉帧超时计数。
 * 该值不是“毫秒”，而是“控制周期计数”。
 * 每次 EXTI9_5_IRQHandler() 进入都会调用一次 K230_ColorFrameHeartbeat() 递减。
 * 如果长时间没有收到新的有效颜色帧，则自动把视觉目标清空，防止旧数据一直控制小车。
 */
#define K230_FRAME_TIMEOUT_TICKS 40

/* 兼容保留的原蓝牙方向变量。
 * 当前已经不再由 USART3 蓝牙协议更新，但工程其他模块仍通过 extern 引用它们，
 * 因此这里保留定义，并在收到 K230 视觉帧时强制清零，避免旧逻辑残留干扰。
 */
u8 Fore, Back, Left, Right;

/* 最近一次收到并成功解析的 K230 颜色识别结果。 */
static volatile K230_ColorFrame_t g_k230_color = {0, 'N', 0, 0};

/* 最近一次收到并成功解析的 K230 视觉巡线结果。默认 lost，避免上电误接管。 */
static volatile K230_LineFrame_t g_k230_line = {0, 0, 0, 0x02, 0, 1};

/* 视觉结果“保活”倒计时。
 * 每收到一帧新的合法数据就重装为 K230_FRAME_TIMEOUT_TICKS。
 */
static volatile u8 g_k230_frame_counter = 0;

/* 巡线帧保活倒计时，与颜色帧分开，避免色块帧给丢失的巡线帧续命。 */
static volatile u8 g_k230_line_counter = 0;

/* USART3 原始字节接收缓冲区。 */
static u8 g_k230_rx_buf[K230_USART3_FRAME_MAX_LEN];

static void K230_ResetRxState(u8 *start, u8 *index)
{
	/* start 表示当前是否已经等到帧头 '$'。
	 * index 表示当前已经写入多少个字节。
	 * 每次一帧收完或检测到异常时，都把接收状态清零，等待下一帧重新开始。
	 */
	*start = 0;
	*index = 0;
	memset(g_k230_rx_buf, 0, sizeof(g_k230_rx_buf));
}

static u16 K230_ParseU16(const char *text)
{
	u16 value = 0;

	/* 把 ASCII 数字串转成无符号整数。
	 * 这里只解析连续数字，到非数字字符就停止。
	 * 对当前协议中的 size 字段已经足够。
	 */
	while((*text >= '0') && (*text <= '9'))
	{
		value = value * 10 + (u16)(*text - '0');
		text++;
	}
	return value;
}

static void K230_ParseColorFrame(void)
{
	char color_text[4] = {0};
	char pos_text[4] = {0};
	char size_text[8] = {0};
	int matched;
	K230_ColorFrame_t frame;

	/* 协议格式：
	 *   $C,color_id,pos,size#
	 * 用 sscanf 把三个字段拆出来。
	 *
	 * 例如：
	 *   "$C,1,L,356#"
	 * 解析后：
	 *   color_text = "1"
	 *   pos_text   = "L"
	 *   size_text  = "356"
	 */
	matched = sscanf((char *)g_k230_rx_buf, "$C,%3[^,],%3[^,],%7[^#]#", color_text, pos_text, size_text);
	if(matched != 3)
	{
		/* 不是完整合法帧则直接丢弃，不更新控制状态。 */
		return;
	}

	/* 把字符串字段转换成控制逻辑可直接使用的数值/字符。 */
	frame.color_id = (u8)atoi(color_text);
	frame.pos = pos_text[0];
	frame.size = K230_ParseU16(size_text);

	/* 只有 color_id 非 0 才视为“识别到了目标”。 */
	frame.active = (frame.color_id != 0) ? 1 : 0;

	/* 更新全局最新视觉结果。 */
	g_k230_color = frame;

	/* 只要收到一次新帧，就把保活计数器重新装载。 */
	g_k230_frame_counter = K230_FRAME_TIMEOUT_TICKS;

	/* 关闭原蓝牙方向控制，避免和 K230 视觉避障并行时互相抢占。
	 * 这里相当于明确声明：USART3 现在只用于 K230 视觉结果输入，不再作为蓝牙遥控入口。
	 */
	Fore = 0;
	Back = 0;
	Left = 0;
	Right = 0;
}

static void K230_ParseLineFrame(void)
{
	char error_text[8] = {0};
	char angle_text[8] = {0};
	char confidence_text[4] = {0};
	char flags_text[4] = {0};
	int matched;
	K230_LineFrame_t frame;

	/* 解析巡线帧：$L,error,angle,confidence,flags#。
	 * STM32 不参与图像处理，只缓存 K230 计算好的控制摘要。
	 */
	matched = sscanf((char *)g_k230_rx_buf, "$L,%7[^,],%7[^,],%3[^,],%3[^#]#",
	                 error_text, angle_text, confidence_text, flags_text);
	if(matched != 4)
	{
		return;
	}

	frame.error = atoi(error_text);
	frame.angle = atoi(angle_text);
	frame.confidence = (u8)atoi(confidence_text);
	frame.flags = (u8)atoi(flags_text);
	frame.active = ((frame.flags & 0x01) != 0) ? 1 : 0;
	frame.lost = ((frame.flags & 0x02) != 0) ? 1 : 0;
	if(frame.confidence > 100)
	{
		frame.confidence = 100;
	}

	g_k230_line = frame;
	g_k230_line_counter = K230_FRAME_TIMEOUT_TICKS;
	Fore = 0;
	Back = 0;
	Left = 0;
	Right = 0;
}

static void K230_ParseFrame(void)
{
	if(g_k230_rx_buf[1] == 'C')
	{
		K230_ParseColorFrame();
	}
	else if(g_k230_rx_buf[1] == 'L')
	{
		K230_ParseLineFrame();
	}
}

void uart3_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* PB10 -> USART3_TX。
	 * 当前主要需求是接收 K230 数据，但依然保留 TX，后续如果要给 K230 回发应答或调试信息会更方便。
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB11 -> USART3_RX。
	 * K230 应把其 UART_TX 接到 STM32 的 PB11(USART3_RX)。
	 */
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

u8 K230_ColorFrameAvailable(void)
{
	/* 对外提供一个简化查询接口。
	 * 目前 control.c 更直接使用 K230_GetColorFrame()，这个函数保留给其他模块扩展。
	 */
	return g_k230_color.active;
}

K230_ColorFrame_t K230_GetColorFrame(void)
{
	/* 返回当前缓存的最近一帧视觉结果。
	 * 由于结构体很小，直接按值返回即可。
	 */
	return *(K230_ColorFrame_t *)&g_k230_color;
}

void K230_ColorFrameHeartbeat(void)
{
	/* 心跳超时机制：
	 * 每次控制周期把计数器减 1。
	 * 如果连续多个控制周期都没有收到新的合法颜色帧，
	 * 则自动认为视觉目标已经消失或串口断开，清除当前视觉避障状态。
	 */
	if(g_k230_frame_counter > 0)
	{
		g_k230_frame_counter--;
		if(g_k230_frame_counter == 0)
		{
			/* 超时后恢复为空目标状态，避免“最后一帧”永久生效。 */
			g_k230_color.color_id = 0;
			g_k230_color.pos = 'N';
			g_k230_color.size = 0;
			g_k230_color.active = 0;
		}
	}
}

void K230_FrameHeartbeat(void)
{
	K230_ColorFrameHeartbeat();
	if(g_k230_line_counter > 0)
	{
		g_k230_line_counter--;
		if(g_k230_line_counter == 0)
		{
			g_k230_line.error = 0;
			g_k230_line.angle = 0;
			g_k230_line.confidence = 0;
			g_k230_line.flags = 0x02;
			g_k230_line.active = 0;
			g_k230_line.lost = 1;
		}
	}
}

u8 K230_LineFrameAvailable(void)
{
	if(g_k230_line.active == 0)
	{
		return 0;
	}
	if(g_k230_line.lost != 0)
	{
		return 0;
	}
	return 1;
}

K230_LineFrame_t K230_GetLineFrame(void)
{
	return *(K230_LineFrame_t *)&g_k230_line;
}

int K230_GetLineError(void)
{
	return g_k230_line.error;
}

void USART3_IRQHandler(void)
{
	/* 简单串口帧状态机：
	 * 1. 遇到 '$' 认为一帧开始
	 * 2. 持续缓存后续字节
	 * 3. 遇到 '#' 认为一帧结束
	 * 4. 对完整帧进行解析
	 */
	static u8 start = 0;
	static u8 index = 0;
	u8 rx_temp;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		rx_temp = (u8)USART_ReceiveData(USART3);

		/* 新帧帧头。无论之前是否收残，都直接从这里重新开始。 */
		if(rx_temp == '$')
		{
			start = 1;
			index = 0;
			memset(g_k230_rx_buf, 0, sizeof(g_k230_rx_buf));
			g_k230_rx_buf[index++] = rx_temp;
			return;
		}

		if(start == 0)
		{
			/* 在遇到 '$' 之前的杂散数据一律忽略。 */
			return;
		}

		if(index >= K230_USART3_FRAME_MAX_LEN - 1)
		{
			/* 接收长度异常，说明这一帧大概率已经损坏，直接丢弃重来。 */
			K230_ResetRxState(&start, &index);
			return;
		}

		g_k230_rx_buf[index++] = rx_temp;

		/* '#' 是一帧结束符。 */
		if(rx_temp == '#')
		{
			g_k230_rx_buf[index] = '\0';
			K230_ParseFrame();
			K230_ResetRxState(&start, &index);
		}
	}
}

void USART3_Send_Data(char data)
{
	/* 仍保留基本发送接口，方便后续串口联调。 */
	USART_SendData(USART3, data);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
}

void USART3_Send_String(char *String)
{
	u16 len, j;

	/* 发送一个 C 字符串，不含额外协议处理。 */
	len = strlen(String);
	for(j = 0; j < len; j++)
	{
		USART3_Send_Data(*String++);
	}
}
