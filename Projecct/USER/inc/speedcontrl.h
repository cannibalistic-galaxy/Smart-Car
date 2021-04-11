#ifndef _SPEEDCONTRL_H
#define _SPEEDCONTRL_H


#include "headfile.h"

/*****����***********/
#define ADN    6
#define AMP1    ADC0_SE12
#define AMP2    ADC0_SE14
#define AMP3    ADC0_SE13
#define AMP4    ADC0_SE5
#define AMP5    ADC0_SE6
#define AMP6    ADC0_SE7

/************************************
 *������
 ***********************************/
extern float TurnSpeed_Sum;
extern int16 speed_left,speed_right; 
//extern uint16 AD_Value[ADN]; 
extern char flag_round;
extern char flag_error_sign;
extern float M_P;
extern float DerivativeGain;
extern float Delt_Error;
extern float AD_Last_Error;
extern float ProportionalGain;
extern uint8 Calculate_Length;
extern uint16 AD_Value_temp[ADN][3];
extern float CarTurn_temp_value;             //ֱ����ת����Ʒ���
extern int32 TurnControlPWM;                 //���յ�ת�����pwm����
extern float  TurnSpeed;
extern float Car_Turn_Err;                      //ͨ��pid�͵�ǰ�ٶȳ�Ȩֵ�����ת��ƫ��������ƫ�������뵱ǰ�ٶ�Ҳ�йأ������ǵ�ǰ���ֵ�̶���ƫ��
extern int32 SpeedControlPWM;                //�ٶȻ����������pwm
extern float  Speed_P,Speed_I, Turn_P, Turn_D,Turn_I,Turn2P,Turn2D,SpeedGiven,Round_P, Round_D,speedweight,BT,X1,X2,SpeedCut;
extern int32 Left_PWM,Right_PWM; 
extern uint16 AD_MAX[ADN],dis_AD[ADN],AD_Value[ADN],AD_MIN[ADN];    //�����ĵ������;
extern float AD_Error,ADSub43;
extern int16 Speed_now;
extern float Error_Delta;
extern char PerfectStop;
extern float  Ka_Run,Kb_Run,Kc_Run,Ki_Run,Kd_Run,CurrentError_Run,AdjustValue_Run; 
/********************************
 *������
 *******************************/
extern void speed_test(void);
extern void AD_Collect(void);
extern void AD_Filter(void);  //��ֵ�˲�
extern int32 PID_Control_Speed(uint16 Speed_set,int Speed_now);
extern void Speed_Get_Add(void);
extern void Speed_Clear(void);  // �ٶ��ۼ����
extern void speed_ctrl(int SpeedSet);
extern void CarRun(void);             //����ٶȻ�PWM
extern void turn_ctrl(void);    //ת��
extern int32 PID_TurnDiffer(int16 GivenValue,int16 MeasureValue);
extern void all_ctrl(void);
extern void TurnErr_Caculate(void);
extern float PID_Control_Turn(float Err);
extern void AD_Calculate(void);
extern void Motor_Ctrl(int32 Left_Motor,int32 Right_Motor);
//extern void lights_on(void);
//extern void lights_off(void);
extern float Differential_In_Advance(float Y);//΢������
//extern float  DirectionErr[9]={0},Error_Delta_Deceleration;
/*************8��14�����������ٶȻ�������������****************/
extern float Ratio_Encoder_Left;
extern float Ratio_Encoder_Righ;
//1175
extern float g_fRealSpeed;				//��ʵ�ٶ�
extern float g_fLeftRealSpeed;
extern float g_fRighRealSpeed;
extern float g_fExpectSpeed;		//�����ٶ�
extern float g_fSpeedError;				//�ٶ�ƫ��
extern float g_fSpeedErrorTemp[5];
extern float fSpeedErrorInteg;
extern float g_fSpeedControlOut;		//�ٶ����
extern void CalSpeedError(uint16 Speed_set);
extern int32 SpeedControl(uint16 Speed_set);
/**************************************************/
#endif