#ifndef _RADIUS_H
#define _RADIUS_H

#include "headfile.h"


typedef struct {
  
  float speedradius;    //����ת��뾶
  int16  SpeedMin;      //�����ȶ�������ٶ�
  float MaxK;       //�������ת������
  float yawrate;
  
} Carinfotypedef;

float Get_Car_curvature(float);      //���������ڴ˴����ٶ��ںϻ���������ٶ�

extern Carinfotypedef Carinfo;




#endif
