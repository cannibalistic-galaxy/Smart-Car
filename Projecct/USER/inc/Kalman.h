#ifndef _KALMAN_H
#define _KALMAN_H

#include "headfile.h"



extern int16 FILTRATE_Kalman(int16 MsrValue);
extern void Angle_Calculate(void);
extern void Kalman_Filter(float, float);


extern float Car_Angle,Angle_Speed,Angle_acc;

#endif