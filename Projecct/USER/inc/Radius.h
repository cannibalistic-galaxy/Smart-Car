#ifndef _RADIUS_H
#define _RADIUS_H

#include "headfile.h"


typedef struct {
  
  float speedradius;    //车体转弯半径
  int16  SpeedMin;      //车体稳定的最低速度
  float MaxK;       //车体最大转弯曲率
  float yawrate;
  
} Carinfotypedef;

float Get_Car_curvature(float);      //期望曲率在此处与速度融合获得期望角速度

extern Carinfotypedef Carinfo;




#endif
