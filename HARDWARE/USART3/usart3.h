#ifndef __USART3_H
#define __USART3_H

#include "sys.h"

/* K230 发送给 STM32 的颜色识别结果。
 *
 * 串口协议格式固定为：
 *   $C,color_id,pos,size#
 *
 * 例如：
 *   $C,1,L,356#
 *
 * 各字段含义：
 *   color_id : 颜色编号，0=无目标，1=红色，2=绿色，3=蓝色
 *   pos      : 目标横向位置，L=左，C=中，R=右，N=无目标
 *   size     : 目标面积/像素数量，用于判断目标是否足够大，避免远处噪点误触发避障
 *   active   : STM32 侧派生出的有效标志，1 表示当前帧可参与控制，0 表示无有效目标
 *
 * 这里把 K230 的视觉结果统一缓存成一个结构体，供 control.c 在姿态中断中直接读取。
 */
typedef struct
{
	u8 color_id;
	char pos;
	u16 size;
	u8 active;
} K230_ColorFrame_t;

/* K230 发送给 STM32 的视觉巡线结果。
 * 协议格式：$L,error,angle,confidence,flags#。
 * K230 负责图像处理，STM32 只保存连续偏差、路线趋势和可信度。
 */
typedef struct
{
	int error;          /* 黑线中心相对图像中心的偏差，左负右正。 */
	int angle;          /* 路线方向趋势，左负右正，用于圆角提前转向。 */
	u8 confidence;      /* 识别可信度，低可信度时控制层不接管。 */
	u8 flags;           /* bit0=valid, bit1=lost, bit2=curve, bit3=marker。 */
	u8 active;          /* 由 valid 位派生，便于控制层快速判断。 */
	u8 lost;            /* 由 lost 位派生，避免长期使用旧误差。 */
} K230_LineFrame_t;

void USART3_Send_String(char *String);
void uart3_init(u32 bound);
void USART3_IRQHandler(void);

u8 K230_ColorFrameAvailable(void);
K230_ColorFrame_t K230_GetColorFrame(void);
void K230_ColorFrameHeartbeat(void);

u8 K230_LineFrameAvailable(void);
K230_LineFrame_t K230_GetLineFrame(void);
int K230_GetLineError(void);
void K230_FrameHeartbeat(void);

#endif
