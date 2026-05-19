#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

#define TRACKING_IR_NUM 8

extern volatile u8 Tracking_IR_Data[TRACKING_IR_NUM];
extern volatile u16 Tracking_Analog_Data[TRACKING_IR_NUM];
extern volatile u8 Tracking_New_Package_Flag;

void Tracking_Usart2_Init(u32 baudrate);
void Tracking_SendControlData(u8 adjust, u8 analogData, u8 digitalData);
void Tracking_DealUsart(u8 rxtemp);
void Tracking_ProcessRx(void);

u8 Tracking_IsActive(void);
void Tracking_SetSpeed(void);
int Tracking_TurnPD(int gyro_z);
int Tracking_GetError(void);

#endif
