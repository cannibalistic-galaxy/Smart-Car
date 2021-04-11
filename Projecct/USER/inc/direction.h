#ifndef _DIRECTION_H
#define _DIRECTION_H

#define BUZZER_ON    gpio_set(H0,0)
#define BUZZER_OFF  gpio_set(H0,1)
/***************************************
 *变量区
 ***************************************/
extern uint16 second_infer;  //第二次检测
extern char flag_round;
extern char flag_round_out;
extern char podao_dianbo_flag;
extern char circle_Flag,Turn_Flag,BaoMing_Flag,Turn_Flag2,Turn_Right_Flag,Turn_Left_Flag,Go_Out_Circle;
extern float Middle_Err;
extern uint8 MaxPosition;
extern float AD_Error_3;
extern float AD_Error_2;
extern uint16 AD_Middle_Max; //中间电感最大值用来做分母上
extern uint16 AD_More; //超出部分叠加到分母上
extern float Last_AD_Error[3];  //偏差储存数组
extern float AD_Second_Derivative;  //偏差二阶导数
extern float SpeedGivenTemp;
extern char huandaonum;
extern char jiansu_flag;
extern float ADError2,ADSub43;
extern int16 turn_Error,turn_lastError,times;   //切点判断差值
extern float ADError_3[3],AD_Error_3_Max;

/****
 * 7月16日更新定义
 ***/
extern char Turn_Flag_Last; //上次转向判断标志位
extern char TurnChange_1; //转向状态切换
extern float AD_Error_Lead; //圆环引导时使用的偏差
extern float AD_Error_Avg,AD_Error_In,AD_Error_In_Last; //入环偏差
/****/
//////////////////////
/*************************************************
 *函数区
 ************************************************/
extern void AD_Collect(void);
extern void AD_Filter(void);  //均值滤波
extern void AD_Calculate(void);
extern float Slope_Calculate(uint8,uint8,float *);
extern void Push_And_Pull(float *,int,float);
extern void DividePositon(void);


#endif