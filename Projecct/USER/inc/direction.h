#ifndef _DIRECTION_H
#define _DIRECTION_H

#define BUZZER_ON    gpio_set(H0,0)
#define BUZZER_OFF  gpio_set(H0,1)
/***************************************
 *������
 ***************************************/
extern uint16 second_infer;  //�ڶ��μ��
extern char flag_round;
extern char flag_round_out;
extern char podao_dianbo_flag;
extern char circle_Flag,Turn_Flag,BaoMing_Flag,Turn_Flag2,Turn_Right_Flag,Turn_Left_Flag,Go_Out_Circle;
extern float Middle_Err;
extern uint8 MaxPosition;
extern float AD_Error_3;
extern float AD_Error_2;
extern uint16 AD_Middle_Max; //�м������ֵ��������ĸ��
extern uint16 AD_More; //�������ֵ��ӵ���ĸ��
extern float Last_AD_Error[3];  //ƫ�������
extern float AD_Second_Derivative;  //ƫ����׵���
extern float SpeedGivenTemp;
extern char huandaonum;
extern char jiansu_flag;
extern float ADError2,ADSub43;
extern int16 turn_Error,turn_lastError,times;   //�е��жϲ�ֵ
extern float ADError_3[3],AD_Error_3_Max;

/****
 * 7��16�ո��¶���
 ***/
extern char Turn_Flag_Last; //�ϴ�ת���жϱ�־λ
extern char TurnChange_1; //ת��״̬�л�
extern float AD_Error_Lead; //Բ������ʱʹ�õ�ƫ��
extern float AD_Error_Avg,AD_Error_In,AD_Error_In_Last; //�뻷ƫ��
/****/
//////////////////////
/*************************************************
 *������
 ************************************************/
extern void AD_Collect(void);
extern void AD_Filter(void);  //��ֵ�˲�
extern void AD_Calculate(void);
extern float Slope_Calculate(uint8,uint8,float *);
extern void Push_And_Pull(float *,int,float);
extern void DividePositon(void);


#endif