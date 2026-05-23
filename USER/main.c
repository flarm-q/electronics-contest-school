#include "stm32f10x.h"
#include "sys.h"
#include "oled.h"
#include "usart.h"
#include "usart3.h"

/* K210 巡线接管前的等待时间。
 *
 * 上电后先给整车一点稳定时间，避免：
 * 1. MPU6050 和 DMP 刚初始化完成时姿态还没完全稳定
 * 2. K210 摄像头和串口刚启动时数据还没完全稳定
 * 3. 小车刚通电就立刻进入巡线导致误动作
 */
#define K210_START_DELAY_MS 2000

/* 主循环的节拍延时。
 *
 * 主控制闭环主要在外部中断里跑，
 * main() 里的 while 循环主要负责：
 * 1. 处理串口接收后的非中断逻辑
 * 2. 刷新 OLED 显示
 * 3. 定时打印调试信息
 *
 * 因此这里不需要很快，固定 5ms 足够。
 */
#define MAIN_LOOP_DELAY_MS 5

/* USART1 调试打印周期。
 *
 * 过快打印会占用串口时间，也会影响调试可读性；
 * 过慢则不利于观察实时状态。
 * 这里每 300ms 输出一次比较平衡。
 */
#define USART1_PRINT_PERIOD_MS 300

/* 姿态角。
 *
 * Roll  : 横滚角
 * Pitch : 俯仰角
 * Yaw   : 偏航角
 *
 * 这三个值由 MPU6050 + DMP 解算得到，
 * 其中平衡车最核心使用的是 Pitch。
 */
float Pitch,Roll,Yaw;

/* 陀螺仪原始角速度。
 *
 * gyroy 常用于直立环，
 * gyroz 常用于转向环，
 * gyrox 当前主要作为调试保留。
 */
short gyrox,gyroy,gyroz;

/* 加速度计原始数据。
 *
 * 当前主流程里没有直接参与闭环计算，
 * 主要保留给 MPU/DMP 和后续调试扩展使用。
 */
short aacx,aacy,aacz;

/* 左右编码器速度。
 *
 * 由定时器编码器接口读取，
 * 供速度环计算整车前后运动趋势。
 */
int Encoder_Left,Encoder_Right;

/* 电机 PWM 上下限。
 *
 * 用于底层限幅，防止输出超过驱动允许范围。
 */
int PWM_MAX=7200,PWM_MIN=-7200;

/* 左右电机最终装载值。 */
int MOTO1,MOTO2;

/* 三环输出调试量。
 *
 * 这几个变量在 control.c 中定义，
 * 这里用 extern 引用，方便必要时观察控制效果。
 */
extern int Vertical_out,Velocity_out,Turn_out;

int main(void)
{
	/* 用于主循环里的定时打印累计。 */
	u16 print_elapsed_ms = 0;
	K210_LineFrame_t line_frame;
	K210_ColorFrame_t color_frame;

	/* 1. 基础时基与中断配置。 */
	delay_init();
	NVIC_Config();

	/* 2. 初始化串口。
	 * USART1：上位机调试、在线调参
	 * USART3：K210 视觉巡线
	 */
	uart1_init(9600);
	uart3_init(115200);

	/* 3. 初始化 OLED，用于现场观察关键状态。 */
	OLED_Init();
	OLED_Clear();

	/* 4. 初始化姿态传感器与对应中断。 */
	MPU_Init();
	mpu_dmp_init();
	MPU6050_EXTI_Init();

	/* 5. 初始化编码器、电机和 PWM。 */
	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	Motor_Init();
	PWM_Init_TIM1(0,7199);

	/* 6. OLED 固定标签。
	 * 第一行显示角度
	 * 第二行显示巡线偏差
	 * 第三行显示速度估计值
	 */
	OLED_ShowString(0,1,"jiao du:",12);
	OLED_ShowString(0,2,"vision:",12);
	OLED_ShowString(0,3,"sd:",12);

	/* 7. 等待系统和 K210 视觉数据稳定。 */
	// delay_ms(K210_START_DELAY_MS);

	while(1)
	{
		/* 8. 处理 USART1 的在线调参命令。 */
		USART1_ProcessCommand();

		line_frame = K210_GetLineFrame();
		color_frame = K210_GetColorFrame();

		/* 9. OLED 实时显示核心状态。 */
		OLED_Float(1,70,Pitch,1);
		OLED_Num3(8,2,line_frame.error);
		OLED_Num3(5,3,(int)((Encoder_Left+Encoder_Right)*2.38));

		/* 10. 周期性向上位机打印调试信息，便于串口观察：
		 * 1. 当前姿态角
		 * 2. K210 巡线偏差、趋势和可信度
		 * 3. K210 颜色识别结果
		 * 4. 左右编码器速度
		 */
		if(print_elapsed_ms >= USART1_PRINT_PERIOD_MS)
		{
			print_elapsed_ms = 0;
			printf("VisionError=%d VisionAngle=%d VisionConfidence=%u VisionFlags=0x%02X ColorID=%u ColorPos=%c ColorSize=%u ColorActive=%u EncoderLeft=%d EncoderRight=%d SpeedDisplay=%.2f\r\n",
			       line_frame.error,
			       line_frame.angle,
			       line_frame.confidence,
			       line_frame.flags,
			       color_frame.color_id,
			       color_frame.pos,
			       color_frame.size,
			       color_frame.active,
			       Encoder_Left,
			       Encoder_Right,
			       (Encoder_Left + Encoder_Right) * 2.38f);
		}

		/* 11. 主循环固定节拍。 */
		delay_ms(MAIN_LOOP_DELAY_MS);
		print_elapsed_ms += MAIN_LOOP_DELAY_MS;
	}
}
