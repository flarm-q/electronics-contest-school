#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

/*
 * 八路巡线模块使用串口协议接入：
 *   1. PA2  -> USART2_TX，给巡线模块发送配置命令。
 *   2. PA3  -> USART2_RX，接收巡线模块回传的数据帧。
 *   3. 参考工程协议为 "$D,x1:0,x2:0,...,x8:0#"，0/1 表示每一路红外状态。
 *
 * 注意：PA2/PA3 原工程被超声波 TRIG/ECHO 占用。本工程已按需求关闭超声波测距，
 * 因此 PA2/PA3 专用于八路巡线，避免引脚冲突。
 */
#define TRACKING_IR_NUM 8

/* 最新一次解析出的 8 路数字量，数组下标 0~7 对应 x1~x8。 */
extern volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];

/* USART2 收到并成功解析一帧数字量数据后置 1，可用于调试或主循环显示。 */
extern volatile u8 Tracking_New_Package_Flag;

void Tracking_Usart2_Init(u32 baudrate);
void Tracking_SendControlData(u8 adjust, u8 analogData, u8 digitalData);
void Tracking_DealUsart(u8 rxtemp);

/* 收到至少一帧有效巡线数据后返回 1，此后控制环进入巡线模式。 */
u8 Tracking_IsActive(void);

/* 巡线模式下设置固定前进速度，速度值在 tracking.c 的 TRACKING_SPEED 调整。 */
void Tracking_SetSpeed(void);

/* 根据八路红外偏差和 Z 轴陀螺仪生成转向 PWM 修正量。 */
int Tracking_TurnPD(int gyro_z);

/* 返回当前巡线偏差：负值偏左，正值偏右，0 为中间或特殊直行状态。 */
int Tracking_GetError(void);

#endif
