#include "tracking.h"
#include "string.h"

/* 串口组包缓冲区长度。
 *
 * 巡线模块通过 USART2 发送一整帧 ASCII 数据，
 * 例如数字量格式可能类似：
 *   $D:0:1:1:0:...#
 *
 * 这里给足够大的缓存，避免后续协议扩展时不够用。
 */
#define TRACKING_PACKAGE_SIZE 100

/* 巡线模式下的基础前进速度目标。
 *
 * 该值写入 control.c 里的 Target_Speed，
 * 由速度环和直立环共同把它转换成实际运动。
 *
 * 当前是负值，表示按照本工程现有编码器/电机方向定义，
 * 巡线前进应当使用这个符号方向。
 */
#define TRACKING_SPEED -1
#define TRACKING_ERROR_MIDDLE 0
#define TRACKING_INTEGRAL_LIMIT 100.0f

/* 巡线转向 PID 参数。
 *
 * 这里本质上是“巡线偏差 -> 转向输出”的控制器。
 * 1. Kp 根据当前偏差立即修正
 * 2. Ki 用于累计长期偏差
 * 3. Kd 这里直接利用陀螺仪 Z 轴角速度做阻尼项
 */
float Tracking_Turn_Kp = 410;
float Tracking_Turn_Ki = 0.0f;
float Tracking_Turn_Kd = 0.0f;

/* 来自主控制模块的全局速度目标。 */
extern float Target_Speed;

/* 八路巡线数字量数据。
 *
 * 下标 0~7 对应从左到右 8 个探头。
 * 当前协议里：
 * 0 表示检测到黑线
 * 1 表示未检测到黑线
 */
volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];

/* 新数据包到达标志。
 *
 * 目前主流程里主要用于调试和状态同步保留。
 */
volatile u8 Tracking_New_Package_Flag = 0;

/* 串口组包缓冲区。 */
static u8 tracking_rx_buff[TRACKING_PACKAGE_SIZE];

/* 最近一次完整收到的新数据包。 */
static u8 tracking_new_package[TRACKING_PACKAGE_SIZE];

/* USART2 中断接收 FIFO。
 *
 * 中断里只做“快速收字节入队”，
 * 真正解析放到主循环里做，降低中断耗时。
 */
static volatile u8 tracking_irq_fifo[TRACKING_PACKAGE_SIZE];
static volatile u16 tracking_irq_fifo_head = 0;
static volatile u16 tracking_irq_fifo_tail = 0;

/* 巡线模块活跃标志。
 *
 * 只要成功解析过一帧有效数字量数据，就置 1。
 * control.c 会据此判断巡线功能是否可以接管高层行为。
 */
static volatile u8 tracking_active = 0;

/* 当前巡线偏差。 */
static int tracking_error = 0;
static u8 tracking_has_line = 0;

/* 八路探头的权重表。
 *
 * 左侧权重为负，右侧权重为正，中间靠近 0。
 * 当某一路探头检测到黑线时，就把该路权重参与平均。
 * 最终得到的平均值越负，说明黑线越偏左；
 * 越正，说明黑线越偏右。
 */
static const int tracking_weights[TRACKING_IR_NUM] = {-4, -3, -2, -1, 1, 2, 3, 4};
static int Tracking_GetPriorityError(u8 x1, u8 x2, u8 x3, u8 x4, u8 x5, u8 x6, u8 x7, u8 x8);

/* 发送单字节到巡线模块。 */
static void Tracking_SendU8(u8 ch)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ch);
}

/* 连续发送一段字节数据。 */
static void Tracking_SendArrayU8(u8 *buffer, u16 length)
{
	while(length--)
	{
		Tracking_SendU8(*buffer++);
	}
}

/* 解析数字量巡线数据。
 *
 * 只有当数据包的第二个字符是 'D' 时，
 * 才按数字量协议处理。
 *
 * 解析成功后会更新：
 * 1. Tracking_IR_Data[]
 * 2. tracking_active
 * 3. Tracking_New_Package_Flag
 */
static void Tracking_ParseDigitalData(void)
{
	const char *cursor = (const char *)tracking_new_package;
	u8 index = 0;

	if(tracking_new_package[1] != 'D')
	{
		return;
	}

	while((*cursor != '\0') && (*cursor != '#') && (index < TRACKING_IR_NUM))
	{
		if((*cursor == ':') && ((cursor[1] == '0') || (cursor[1] == '1')))
		{
			Tracking_IR_Data[index++] = (u8)(cursor[1] - '0');
			cursor += 2;
			continue;
		}
		cursor++;
	}

	/* 少于 8 路则认为该帧不完整，不更新有效状态。 */
	if(index < TRACKING_IR_NUM)
	{
		return;
	}

	tracking_active = 1;
	Tracking_New_Package_Flag = 1;
	memset(tracking_new_package, 0, TRACKING_PACKAGE_SIZE);
}

/* 初始化 USART2，用于连接八路巡线模块。 */
void Tracking_Usart2_Init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* PA2 -> USART2_TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA3 -> USART2_RX */
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

/* 发送巡线模块控制命令。
 *
 * 当前保留两个开关位：
 * 1. adjust      是否让模块进入参数调整相关模式
 * 2. digitalData 是否开启数字量数据回传
 *
 * 本工程现在只用数字量巡线，因此模拟量相关位固定为 0。
 */
void Tracking_SendControlData(u8 adjust, u8 digitalData)
{
	u8 send_buf[8] = "$0,0,0#";

	send_buf[1] = adjust ? '1' : '0';
	send_buf[3] = '0';
	send_buf[5] = digitalData ? '1' : '0';

	Tracking_SendArrayU8(send_buf, strlen((char *)send_buf));
}

/* 逐字节处理巡线串口数据并完成组包。
 *
 * 协议规则：
 * 1. '$' 表示一帧开始
 * 2. '#' 表示一帧结束
 *
 * 收到完整一帧后立即尝试解析数字量数据。
 */
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
		Tracking_ParseDigitalData();
		return;
	}

	/* 超出缓冲区则放弃当前帧，等待下一帧重新同步。 */
	if(step >= TRACKING_PACKAGE_SIZE)
	{
		start = 0;
		step = 0;
		memset(tracking_rx_buff, 0, TRACKING_PACKAGE_SIZE);
	}
}

/* 在主循环中分批处理 USART2 中断 FIFO。
 *
 * 每次最多处理 16 个字节，避免主循环一次性卡太久，
 * 保持显示、调参和其他逻辑的响应性。
 */
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

/* 返回巡线模块是否已经成功提供过有效数据。 */
u8 Tracking_IsActive(void)
{
	return tracking_active;
}

/* 巡线模式下写入基础前进速度目标。 */
void Tracking_SetSpeed(void)
{
	Target_Speed = TRACKING_SPEED;
}

/* 根据八路数字量状态计算巡线偏差。
 *
 * 处理方法：
 * 1. 找出所有“检测到黑线”的探头
 * 2. 把这些探头的权重求和
 * 3. 再按命中个数做平均
 *
 * 这样比单独只看一位更平滑，
 * 当黑线压到相邻两三个探头时也能得到连续偏差。
 */
int Tracking_GetError(void)
{
	u8 i;
	int weighted_sum = 0;
	u8 hit_count = 0;
	u8 x1 = Tracking_IR_Data[0];
	u8 x2 = Tracking_IR_Data[1];
	u8 x3 = Tracking_IR_Data[2];
	u8 x4 = Tracking_IR_Data[3];
	u8 x5 = Tracking_IR_Data[4];
	u8 x6 = Tracking_IR_Data[5];
	u8 x7 = Tracking_IR_Data[6];
	u8 x8 = Tracking_IR_Data[7];

	tracking_has_line = 0;
	for(i = 0; i < TRACKING_IR_NUM; i++)
	{
		if(Tracking_IR_Data[i] == 0)
		{
			tracking_has_line = 1;
			break;
		}
	}

	if(tracking_has_line == 0)
	{
		return tracking_error;
	}

	tracking_error = Tracking_GetPriorityError(x1, x2, x3, x4, x5, x6, x7, x8);
	if(tracking_error == 99)
	{
		for(i = 0; i < TRACKING_IR_NUM; i++)
		{
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
	}

	return tracking_error;
}

/* 巡线转向 PID/PD 控制器。
 *
 * 输入：
 * 1. 当前巡线偏差
 * 2. 陀螺仪 Z 轴角速度
 *
 * 输出：
 * 巡线模式下的转向控制量
 *
 * 说明：
 * 1. err * Kp      负责根据偏差立即纠偏
 * 2. integral * Ki 负责消除长期累计偏差
 * 3. gyro_z * Kd   负责转向阻尼，抑制摆动
 */
int Tracking_TurnPD(int gyro_z)
{
	static float integral = 0;
	float err = (float)(Tracking_GetError() - TRACKING_ERROR_MIDDLE);

	if(tracking_has_line == 0)
	{
		integral = 0;
		return (int)(gyro_z * Tracking_Turn_Kd);
	}

	integral += err;
	if(integral > TRACKING_INTEGRAL_LIMIT)
	{
		integral = TRACKING_INTEGRAL_LIMIT;
	}
	else if(integral < -TRACKING_INTEGRAL_LIMIT)
	{
		integral = -TRACKING_INTEGRAL_LIMIT;
	}

	/* 当前车体混控方向与巡线误差正负定义相反，这里统一反相输出。 */
	return (int)((err * Tracking_Turn_Kp + integral * Tracking_Turn_Ki + gyro_z * Tracking_Turn_Kd));
}

static int Tracking_GetPriorityError(u8 x1, u8 x2, u8 x3, u8 x4, u8 x5, u8 x6, u8 x7, u8 x8)
{
	if(x1 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x8 == 0)
	{
		return 0;
	}
	if((x1 == 0 || x2 == 0) && x8 == 1)
	{
		return -5;
	}
	if((x7 == 0 || x8 == 0) && x1 == 1)
	{
		return 5;
	}
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
	{
		return -1;
	}
	if((x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) ||
	   (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1))
	{
		return -2;
	}
	if((x1 == 1 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) ||
	   (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1))
	{
		return -3;
	}
	if((x1 == 0 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) ||
	   (x1 == 0 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1))
	{
		return -4;
	}
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)
	{
		return 1;
	}
	if((x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1) ||
	   (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 1 && x8 == 1))
	{
		return 2;
	}
	if((x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 1) ||
	   (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 1))
	{
		return 3;
	}
	if((x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 0) ||
	   (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 0))
	{
		return 4;
	}
	if(x1 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x8 == 1)
	{
		return 0;
	}
	return 99;
}

/* 在线修改巡线 PID 参数。
 *
 * 支持：
 * 1. TKP -> Tracking_Turn_Kp
 * 2. TKI -> Tracking_Turn_Ki
 * 3. TKD -> Tracking_Turn_Kd
 */
int Tracking_SetPidByName(const char *name, float value)
{
	if(strcmp(name, "TKP") == 0)
	{
		Tracking_Turn_Kp = value;
		return 1;
	}
	if(strcmp(name, "TKI") == 0)
	{
		Tracking_Turn_Ki = value;
		return 1;
	}
	if(strcmp(name, "TKD") == 0)
	{
		Tracking_Turn_Kd = value;
		return 1;
	}
	return 0;
}

/* 读取当前巡线 PID 参数，用于串口查询。 */
void Tracking_GetPidValues(float *tkp, float *tki, float *tkd)
{
	if(tkp != 0)
	{
		*tkp = Tracking_Turn_Kp;
	}
	if(tki != 0)
	{
		*tki = Tracking_Turn_Ki;
	}
	if(tkd != 0)
	{
		*tkd = Tracking_Turn_Kd;
	}
}

/* USART2 中断服务函数。
 *
 * 职责非常单一：
 * 1. 读出收到的新字节
 * 2. 写入循环 FIFO
 * 3. 不在中断里做复杂字符串解析
 *
 * 这样能让中断尽量短，减少对主控制回路的干扰。
 */
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

		/* FIFO 满时直接丢弃新字节，优先保证系统不中断失控。 */
		if(next_head != tracking_irq_fifo_tail)
		{
			tracking_irq_fifo[tracking_irq_fifo_head] = rx_temp;
			tracking_irq_fifo_head = next_head;
		}
	}
}
