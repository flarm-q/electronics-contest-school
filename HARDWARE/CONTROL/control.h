#ifndef  _CONTROL_H
#define  _CONTROL_H

#include "sys.h" 




int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z,int RC);
void EXTI9_5_IRQHandler(void);
int Control_SetPidByName(const char *name, float value);
void Control_GetPidValues(float *vkp, float *vkd, float *skp, float *ski, float *rkp, float *rkd);
#endif

