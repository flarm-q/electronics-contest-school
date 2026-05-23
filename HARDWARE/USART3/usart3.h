#ifndef __USART3_H
#define __USART3_H

#include "sys.h"

/* K210 视觉结果帧。
 *
 * 串口协议格式：
 *   $C,color_id,pos,size#
 *   $L,error,angle,confidence,flags#
 *
 * 各字段含义：
 *   color_id   : 颜色编号，1=红，2=绿，3=蓝，0=无目标
 *   pos        : 目标横向位置，L=左，C=中，R=右，N=无目标
 *   size       : 目标面积/像素数量
 *   error      : 黑线中心相对画面中心的偏差，左负右正
 *   angle      : 路线趋势，左负右正
 *   confidence : 可信度，0~100
 *   flags      : bit0=valid, bit1=lost, bit2=curve, bit3=marker
 */
typedef struct
{
	u8 color_id;
	char pos;
	u16 size;
	u8 active;
} K210_ColorFrame_t;

typedef struct
{
	s16 error;
	s16 angle;
	u8 confidence;
	u8 flags;
	u8 active;
	u8 lost;
} K210_LineFrame_t;

void USART3_Send_String(char *String);
void uart3_init(u32 bound);
void USART3_IRQHandler(void);

u8 K210_ColorFrameAvailable(void);
K210_ColorFrame_t K210_GetColorFrame(void);
void K210_ColorFrameHeartbeat(void);

u8 K210_LineFrameAvailable(void);
K210_LineFrame_t K210_GetLineFrame(void);
void K210_LineFrameHeartbeat(void);

#endif
