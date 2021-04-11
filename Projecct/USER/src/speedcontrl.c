/*****
*
* @brief�����������ڵ������,���ֵ����
* @author ��gerui��hzy��linxuankun
* @date   2018-6-2
* ��ﵽĿ���ٶ�ʱ�����2
*��ˮƽ1 ��ˮƽ2 �м�3 ��б4 ��б5
********/



#include "headfile.h"
#include "speedcontrl.h"



#define OFF      0
#define ON       1
#define THREEMODE 2
#define abs(x)   (x>0?x:-x)
#define GYRO_SET   48

#define TurnDelta   100

#define DifferentialInAdvance   0 //�Ƿ�����΢������
#define Slope   1  //�Ƿ�������С����
#define balance 1 
#define stop    1

/*******�뾶����*****************/



/************************/
uint16 AD_Value[ADN];    //�����ĵ������
uint16 dis_AD[ADN];      //δ�޷�֮ǰ�ĵ��ԭʼֵ
uint16 AD_Value_temp[ADN][3];   //��ʱ�������
uint16 AD_MAX[ADN];
uint16 AD_MIN[ADN] = {20,20,20,20,20};
int16 speed_left,speed_right;     //��������ֵ
int16 speed_last_L,speed_last_R;
float AD_Error,Delt_Error;
uint8 Calculate_Length;
uint16 speed_out_times;
uint8 stop_flag;
uint8 carcarstop=0;
/*****************************8��14�¼ӵ��ٶȿ��Ʋ���****************************/
//֮ǰ�ٶȻ�Ϊ8��135
float Ratio_Encoder_Left = 207/(5860*0.009);// �����ٶ�=counter*�����ܳ�(mm)/(����תһȦ��Ӧ��������*��������)
float Ratio_Encoder_Righ = 207/(5860*0.009);// �����ٶ�=counter*�����ܳ�(mm)/(����תһȦ��Ӧ��������*��������)
//1175
float g_fRealSpeed = 0;				//��ʵ�ٶ�
float g_fLeftRealSpeed;
float g_fRighRealSpeed;
float g_fExpectSpeed;		//�����ٶ�
float g_fSpeedError;				//�ٶ�ƫ��
float g_fSpeedErrorTemp[5] = {0};
float fSpeedErrorInteg = 0;
float g_fSpeedControlOut = 0;		//�ٶ����
/*********************************************************************************/
/***-------------------------����----------------------------------------------***/
float  Speed_P,Speed_I, Turn_P, Turn_D,Turn_I,Turn2P,Turn2D,SpeedGiven,Round_P, Round_D,speedweight,BT,X1,X2,SpeedCut;
char PerfectStop;
int offset = 0;
float Error_Delta;
int32  Turn_Out_Filter(int32);
float Slope_Calculate(uint8,uint8,float *);
void Push_And_Pull(float *,int,float);
extern int16 FILTRATE_Kalman(int16 MsrValue);
void Motor_Ctrl(int32 Left_Motor,int32 Right_Motor)     //�����PWMתΪ��·��ʽ
{   
   
   uint32 left,right;
   if(Left_Motor>=0)
   { 
     //left=Left_Motor+left_dd_front;      //��������ѹΪ��ֱ�������Է���
     left=Left_Motor;                                      //left_dd_front  ����������
     ftm_pwm_duty(ftm2, ftm_ch5, 0);
     ftm_pwm_duty(ftm2, ftm_ch4, left);
   }
   else if(Left_Motor<0)
   {
     //left=-(Left_Motor+left_dd_back);
     left=-Left_Motor;
     ftm_pwm_duty(ftm2, ftm_ch4, 0);
     ftm_pwm_duty(ftm2, ftm_ch5, left);
     
   }
   
     if(Right_Motor>=0)
   { 
     //right=Right_Motor+right_dd_front;
     right=Right_Motor;
     ftm_pwm_duty(ftm2, ftm_ch3, 0);
     ftm_pwm_duty(ftm2, ftm_ch2, right);
     
   }
   else if(Right_Motor<0)
   {
     //right=-(Right_Motor+right_dd_back);
     right=-Right_Motor;
     ftm_pwm_duty(ftm2, ftm_ch2, 0);
     ftm_pwm_duty(ftm2, ftm_ch3, right);
   }
   
   
}

/*-----------------------�ٶȺ���--------------------------------------------*/

void speed_test(void)   // �ٶȻ�ȡ
{
     speed_right=-ftm_count_get(ftm0)*4;   //����
     ftm_count_clean(ftm0);
     if(!gpio_get(E1))speed_right=-speed_right;
     
   /* speed_right=(speed_right>500?500:speed_right);
     speed_right=(speed_right<-200?-200:speed_right);
     
     speed_right=(speed_right-speed_last_R>40?speed_right+40:speed_right);
     speed_right=(speed_right-speed_last_R<-40?speed_left-40:speed_left);
    */
      speed_right=(int16)(0.8*speed_right+0.2*speed_last_R);        //�����������˲�
       speed_last_R=speed_right;
     
      
     speed_left=ftm_count_get(ftm1)*4;   //����
      ftm_count_clean(ftm1);
     if(!gpio_get(H5))speed_left=-speed_left;
     
    /* speed_left=(speed_left>500?500:speed_left);
     speed_left=(speed_left<-200?-200:speed_left);
     
     speed_left=(speed_left-speed_last_L>40?speed_left+40:speed_left);
     speed_left=(speed_left-speed_last_L<-40?speed_left-40:speed_left);*/
     
     speed_left=(int16)(0.8*speed_left+0.2*speed_last_L);
     speed_last_L=speed_left;
     

}

/***-------------------------�ٶȻ�����----------------------------------------***/
int16 Speed_now;                        //��ǰ���ٶ�
int32 SpeedControlOld=0;                //�ɳ��ٶȿ�����
int32 SpeedControlNew=0;                //�³��ٶȿ�����
int32 SpeedControlDelta=0;              //�ٶ�ƫ��
int32 SpeedControlPWM=0;                //�ٶȻ����������pwm


/********************************�Լ�����Ĳ���*************************************/
float Sample_Count=30,Divider_Prarm_Count=3;//
uint16 Speed_D;//�ܵĲ����в�δ�õ���Ϊ0
/***-----------------------------------------------------------------------------***/


void speed_ctrl(int SpeedSet)    //�ٶ�PID����
{
  
  SpeedControlOld=SpeedControlNew;                //����ǰ�ٶȴ�Ϊ�ϴ��ٶ�
// SpeedAdd_Left/=Sample_Count;                   //�ٶ��ۼ�ֵ��30����Sample_Count=30��
 // SpeedAdd_Right/=Sample_Count;                 //�ٶ��ۼ�ֵ��30
#if 1
  Speed_now=(speed_left+speed_right);    //�������ٶ��ۼ�ֵ��ƽ��,���ﲻ��2�ˣ���֤512�ߵı���������
  
    
  SpeedControlNew=PID_Control_Speed((uint16)SpeedSet,Speed_now);   //PID�����ٶȣ��ú������Ϊ�µ�PID������ٶ����
  SpeedControlDelta=SpeedControlNew-SpeedControlOld;        //���ٶ�ƫ��ٶ�ǰ��ı���
  SpeedControlDelta/=Divider_Prarm_Count;               //�ٶ�ƫ�����5��Divider_Prarm_Count=5��ϸ�ֿ��ƣ���ֹ��
#else
  SpeedControlNew=SpeedControl((uint16)SpeedSet);
#endif
  
}

void CarRun(void)              //����ٶȻ�PWM
{       
    //SpeedControlPWM=0.3*SpeedControlOld+0.7*SpeedControlNew;       //�ٶȷ����ó�
    SpeedControlOld=SpeedControlNew;    //�ٶȿ���������
#if 1
  SpeedControlOld+=SpeedControlDelta;   // ֱ�����ٶȿ���������
#endif
    SpeedControlPWM=SpeedControlOld;       //�ٶȷ���

    /*******�ٶ��޷�*********/
   // SpeedControlPWM=(SpeedControlPWM>6500?6500:SpeedControlPWM);
   // SpeedControlPWM=(SpeedControlPWM<-6500?-6500:SpeedControlPWM);
  
}


/////////////////////////////////�ٶȻ�PID����//////////////////////////////////////////////////
     float  ProportionalGain_Run=30;//110;      //��������ϵ��Kp
     float  IntegralGain_Run=0.02;         //��������ϵ��Ti   0.007
     float  DerivativeGain_Run=0;//13;    //����΢��ϵ��Td
     float  ControlPeriod_Run=0.01;        //��������T
     float  FrontErrorK_1_Run=0.0;          //ǰһ�����ֵe(k-1)
     float  FrontErrorK_2_Run=0.0;          //ǰǰ�����ֵe(k-2)
     float  MaxError_Run=100.0;            //������
     float  MinError_Run=30;               //��С���
     float  AddAdjust_Value_Run=0;        //�ۼƵ�����
     uint8  Integ_FLAG=1;                      //���ַ����־
    float  Ka_Run,Kb_Run,Kc_Run,Ki_Run,Kd_Run,CurrentError_Run,AdjustValue_Run; 
    uint16 speedp_begin;
     float speedcount;    //�ٶ�p����ֵ    
//////////////////////////////////////////////////////////////////////////////////////
int32 PID_Control_Speed(uint16 Speed_set,int Speed_now)    //���뵱ǰƽ���ٶ����趨ƽ���ٶȣ����ؾ���һ���ٶ�PID�����ó��������
{
    
     //speedp_begin=35-Speed_P;
     //speedcount=0.02*speedcount+0.98*speedp_begin;   //�𲽵���  
     //if(runtime>500) ProportionalGain_Run=500;
      ProportionalGain_Run=Speed_P;          //�������㣬Speed_p��I��DΪ�޸Ĳ�������
    //if(runtime>500) IntegralGain_Run=0.035;
     //Ki_Run=((float)(Speed_I))/1000;      //Speed_I������Kp����??����
    
  //  DerivativeGain_Run=((float)(Speed_D))/100;
    
    
     MaxError_Run=5;      //�����ֵ
                                                        //����PIDǰǰ�����ϵ��   
    CurrentError_Run=Speed_set-Speed_now;             //���㵱ǰ�ٶ����ֵ 
    if(CurrentError_Run>=300) Ki_Run = Speed_I/1000;
    if(CurrentError_Run>=250&&CurrentError_Run<300) Ki_Run = Speed_I/2000;
    if(CurrentError_Run>=200&&CurrentError_Run<250) Ki_Run = Speed_I/3000;
    if(CurrentError_Run>=150&&CurrentError_Run<200) Ki_Run = Speed_I/4500;
    if(CurrentError_Run>=100&&CurrentError_Run<150) Ki_Run = Speed_I/5000;
    if(CurrentError_Run>=50&&CurrentError_Run<100) Ki_Run = Speed_I/5500;
    if(CurrentError_Run>=0&&CurrentError_Run<50)  Ki_Run = 0;
    //�ٶ�ƫ���޷�
  // CurrentError_Run=(CurrentError_Run>120?120:CurrentError_Run);
   //CurrentError_Run=(CurrentError_Run<-120?-120:CurrentError_Run);
    
    
    //�ٶȻ����ַ��룬��ֹ������    
    
 //    Ki_Run=(CurrentError_Run>20?0:Ki_Run);
   //  Ki_Run=(CurrentError_Run<-20?0:Ki_Run);
     
    if(abs(CurrentError_Run)<MinError_Run||CurrentError_Run<-10)            //��ֵ��С�������ֵ��ʱ�򣬲���PID��������������СʱƵ������������
      {
          AdjustValue_Run=0.85*AddAdjust_Value_Run;
          FrontErrorK_2_Run=FrontErrorK_1_Run;         //��ǰһ�ε�����ǰǰ��
          FrontErrorK_1_Run=CurrentError_Run;          //����ǰ����ǰһ��
      }
    else         //��������ֵʱ 
      {  
      
        
        Ka_Run =ProportionalGain_Run+Kd_Run+Ki_Run;                //����PID��ǰ���ϵ����Kiԭ����Ҫ���Ի��ַ���ϵ��
        Kb_Run =-ProportionalGain_Run-2*Kd_Run;                //����PIDǰһ�����ϵ��
        Kc_Run =Kd_Run;
        AdjustValue_Run=AddAdjust_Value_Run+CurrentError_Run*Ka_Run+FrontErrorK_1_Run*Kb_Run+FrontErrorK_2_Run*Kc_Run;    //��������ʽPID�㷨�������ֵ
        FrontErrorK_2_Run=FrontErrorK_1_Run;              //��ǰһ�ε�����ǰǰ��
        FrontErrorK_1_Run=CurrentError_Run;                          //����ǰ����ǰһ��
      }
   // AdjustValue_Run=(AdjustValue_Run>6500?6500:AdjustValue_Run);
    AddAdjust_Value_Run=AdjustValue_Run;                      //�ۻ���������ֵ  uk-1=uk
    return  ((int32)AdjustValue_Run);
}
/******************************8��14���¼ӵ��ٶȿ��ƺ���**************************/
/**
 * @file	�����ٶ�ƫ��
 * @note      	����ȫ�ֱ���g_fSpeedError
 * @date	2018.8.14
 */
#if 0
void CalSpeedError(uint16 Speed_set)
{
  g_fExpectSpeed = Speed_set;
  
  g_fLeftRealSpeed = speed_left;
  g_fRighRealSpeed = speed_right;
  g_fRealSpeed = g_fLeftRealSpeed + g_fRighRealSpeed;				//��ʵ�ٶ�
  g_fSpeedError =  g_fExpectSpeed - g_fRealSpeed;
  g_fSpeedErrorTemp[4] = g_fSpeedErrorTemp[3];
  g_fSpeedErrorTemp[3] = g_fSpeedErrorTemp[2];
  g_fSpeedErrorTemp[2] = g_fSpeedErrorTemp[1];
  g_fSpeedErrorTemp[1] = g_fSpeedErrorTemp[0];
  g_fSpeedErrorTemp[0] = g_fSpeedError;

}
/**
 * @file        �ٶȿ���
 * @note      	�ٶ��ݶ�ƽ��
 * @date	2018.8.14
 */
int32 SpeedControl(uint16 Speed_set)
{
  int8 index=1;

  CalSpeedError(Speed_set);	//�����ٶ�ƫ��
  g_fSpeedError = (g_fSpeedError>200?200:g_fSpeedError);//�ٶ�ƫ���޷�
  //���ַ���
  if((g_fSpeedError<=100)&&(g_fSpeedError>=-100)) index=1;
  else  index=0;	
  fSpeedErrorInteg = index * Speed_I * g_fSpeedError * 0.01;
  //�ٶȿ����㷨������ʽPI��
  g_fSpeedControlOut += Speed_P/10*(g_fSpeedErrorTemp[0]-g_fSpeedErrorTemp[1]) + fSpeedErrorInteg;
 // g_fSpeedControlOut = (g_fSpeedControlOut>=6000?6000:g_fSpeedControlOut);
  return ((int32)g_fSpeedControlOut);
	
}
#endif

/*********************************************************************************/
/***--------------------------ת�򻷲���---------------------------------------***/
float CarTurn_temp_value=0;             //ֱ����ת����Ʒ���
int32 TurnControlPWM=0;                 //���յ�ת�����pwm����
float Car_Turn_Err=0;                      //ͨ��pid�͵�ǰ�ٶȳ�Ȩֵ�����ת��ƫ��������ƫ�������뵱ǰ�ٶ�Ҳ�йأ������ǵ�ǰ���ֵ�̶���ƫ��
float M_P;//�����ǳ˵�ϵ��
float speed_err;


//////////////////////////////����PID����////////////////////////////////////////////
     float  ProportionalGain_TD=66;      //��������ϵ��Kp     66 
     float  IntegralGain_TD=0.24;           //��������ϵ��Ti  0.24
     float  DerivativeGain_TD=0.01;        //����΢��ϵ��Td
     float  ControlPeriod_TD=0.01;        //��������T
     float  FrontErrorK_1_TD=0.0;          //ǰһ�����ֵe(k-1)
     float  FrontErrorK_2_TD=0.0;          //ǰǰ�����ֵe(k-2)
     float  AddAdjust_Value_TD=0.0;        //�ۼƵ�����
     float  Ka_TD,Kb_TD,Kc_TD,Ki_TD,Kd_TD,CurrentError_TD,AdjustValue_TD;
     float  FrontError[5] = {0,0,0,0,0};
///////////////////////////////////////////////////////////////////////////////////////
int32 PID_TurnDiffer(int16 GivenValue,int16 MeasureValue)   //�������ƣ�ת������ڻ������������ת����Ʒ������ٶȲ���ֵ
{
    
       ProportionalGain_TD=Turn_P/10;
      IntegralGain_TD=Turn_I/100; 
       DerivativeGain_TD=Turn_D/100;
      // ProportionalGain_TD = ProportionalGain_TD*(1-0.5*(SpeedGiven/(speed_left+speed_right)>1?1:SpeedGiven/(speed_left+speed_right)));
       
         
   
    //Ki_TD=0;      //ȡ����������
    
#if DifferentialInAdvance
    Kd_TD=0;//΢�������ڻ��˴�����D
#endif
 
    
    CurrentError_TD=MeasureValue-GivenValue;      //���㵱ǰ���ֵ���������ٶ�ʵ�ʲ�ֵ��������ٶȲ�ֵ
      
#if 1 //λ��ʽ    
    FrontError[4] = FrontError[3];
    FrontError[3] = FrontError[2];
    FrontError[2] = FrontError[1];
    FrontError[1] = FrontError[0];
    FrontError[0] = CurrentError_TD;
    AdjustValue_TD=FrontError[0]*ProportionalGain_TD + IntegralGain_TD*(FrontError[4]+FrontError[3]
                +FrontError[2]+FrontError[1]+FrontError[0]) + DerivativeGain_TD*(FrontError[0]-FrontError[1]);
    
#else //����ʽ
       Ki_TD=ControlPeriod_TD/IntegralGain_TD;          //�������ϵ��T/Ti
       Kd_TD=DerivativeGain_TD/ControlPeriod_TD;           //����΢��ϵ��Td/T 
       Ka_TD =ProportionalGain_TD+Kd_TD+Ki_TD;                //����PID��ǰ���ϵ��
       Kb_TD =-ProportionalGain_TD-2*Kd_TD;                //����PIDǰһ�����ϵ��
       Kc_TD =Kd_TD;                                    //����PIDǰǰ�����ϵ��
       AdjustValue_TD=AddAdjust_Value_TD+CurrentError_TD*Ka_TD+FrontErrorK_1_TD*Kb_TD+FrontErrorK_2_TD*Kc_TD;        //��������ʽPID�㷨�������ֵ
       FrontErrorK_2_TD=FrontErrorK_1_TD;              //��ǰһ�ε�����ǰǰ��
       FrontErrorK_1_TD=CurrentError_TD;                          //����ǰ����ǰһ��
        AddAdjust_Value_TD=AdjustValue_TD;
#endif
       
    return  ((int32)AdjustValue_TD);  
}

/*******************************�ܵĴ������ƣ�����֮�еĺ����ٶ��㷨��������������������*******************************************/
int32 Left_PWM=0,Right_PWM=0;       //�������뵽�����PWM
uint8 jj;
/***----------------------------���ƫ��-----------------------------***/

float AD_Last_Error;
void all_ctrl(void)   //�������ٶ�PID�ϲ�����
{
  
#if stop
  if((((AD_Value[0]<100&&AD_Value[1]<100&&AD_Value[2]<100&&AD_Value[5]<100)||(gpio_get(H2) == 0))||(gpio_get(I2)==0&&PerfectStop==1))||carcarstop)  //ͣ������
#else
   if((gpio_get(H2) == 0))  //����ͣ���������޳���ͣ����
#endif
  {
    carcarstop=1;
   // Left_PWM=-4200;
  //  Right_PWM=-4200;
    
    SpeedControlPWM=-2000;
   
 //   Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);
    
    //DisableInterrupts; 
  }
 
#if   balance 

    Left_PWM =SpeedControlPWM + TurnControlPWM ;       //ֱ������ת�򻷼��ٶȻ�
    Right_PWM=SpeedControlPWM - TurnControlPWM ;        //ֱ������ת�򻷼��ٶȻ�
    //  Left_PWM =3100 + TurnControlPWM ;       //ֱ������ת�򻷼��ٶȻ�
    //Right_PWM=3100 - TurnControlPWM ;        //ֱ������ת�򻷼��ٶȻ�
    
  

#else  
   if(TurnControlPWM>0)
  {
    Left_PWM =SpeedControlPWM + TurnControlPWM + 2000;       //ֱ������ת�򻷼��ٶȻ�
    Right_PWM=SpeedControlPWM - TurnControlPWM + 2000;        //ֱ������ת�򻷼��ٶȻ�
  }
  else
  {
  Left_PWM =SpeedControlPWM + TurnControlPWM + 2000;       //ֱ������ת�򻷼��ٶȻ�
  Right_PWM=SpeedControlPWM - TurnControlPWM + 2000;         //ֱ������ת�򻷼��ٶȻ�
  
  }
#endif  
  if(Left_PWM>9900) Left_PWM=9900;                        //�޷�
  else if(Left_PWM<-9900) Left_PWM=-9900;
  if(Right_PWM>9900) Right_PWM=9900;
  else if(Right_PWM<-9900) Right_PWM=-9900;
    /********ͣ������********/
/*#if stop
  if((((AD_Value[0]<80&&AD_Value[1]<80&&AD_Value[3]<80&&AD_Value[4]<80)||(gpio_get(H2) == 0))||(gpio_get(I2)==0&&PerfectStop==1))||carcarstop)  //ͣ������
  
#endif
  {
    carcarstop=1;
    Left_PWM=-6000;
    Right_PWM=-6000;
    
   
    Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);
   // SpeedControlPWM=0;
  //  SpeedGiven=0;
    //DisableInterrupts; 
  }*/
  if(carcarstop&&Speed_now<10)
  {
    Left_PWM=0;
    Right_PWM=0;
    Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);
    DisableInterrupts;
  
  
  }
/*  if(((AD_Value[0]<80&&AD_Value[1]<80&&AD_Value[3]<80&&AD_Value[4]<80)||(gpio_get(H2) == 0))&&abs(Speed_now)<50)
  {
    Left_PWM=0;
    Right_PWM=0;
    Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);
   // stop_flag = 1;
    DisableInterrupts; 
  }*/

  
  if(Left_PWM<200&&Left_PWM>-200) Left_PWM=0;                //����������
  
  if(Right_PWM<200&&Right_PWM>-200) Right_PWM=0;
  

  Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);               //�����·���
  
}

float TurnSpeed;
void TurnErr_Caculate(void)           //ת��ƫ����㡣�����PID���ƽ���
{
   Get_Gyro();    //��ȡ��ת�����ٶ�   
   //mpu_gyro_x=FILTRATE_Kalman(mpu_gyro_x)-47;
   TurnSpeed=0.1*(mpu_gyro_z-GYRO_SET);
   TurnSpeed = (float)FILTRATE_Kalman((int16)TurnSpeed);
   
   Carinfo.yawrate=TurnSpeed;     
   
  Car_Turn_Err=PID_Control_Turn(AD_Error);
  
  Car_Turn_Err=Car_Turn_Err*2-0.005*M_P * TurnSpeed;    //�ٶ�ռȨ�ز���speedweight
  
//  Car_Turn_Err=(Car_Turn_Err>1200?1200:Car_Turn_Err);
 // Car_Turn_Err=(Car_Turn_Err<-1200?-1200:Car_Turn_Err);
   
}


/*-------------------------------ת��----------------------------------------------------------------------------*/

int32 TurnPWMLast;
float TurnSpeed_Sum = 0;
/***---------------------------------------------------------------------------***/

void turn_ctrl(void)      //ת��PID���Ʋ���
{

  CarTurn_temp_value=Car_Turn_Err;      //���ĸ���ת����ֵ��ת����Ʒ���  
  //CarTurn_temp_value = 500;
#if     DifferentialInAdvance
 //speed_err = Differential_In_Advance((float)(speed_right-speed_left));//����·΢��
  speed_err = Differential_In_Advance((float)(-0.8*TurnSpeed));
   TurnControlPWM=PID_TurnDiffer(CarTurn_temp_value,speed_err); 
  
#else
  //TurnControlPWM=PID_TurnDiffer((Get_Car_curvature(CarTurn_temp_value))*1,(float)(0.8*TurnSpeed));  //ת���������
   
   TurnControlPWM=PID_TurnDiffer(((int16)Get_Car_curvature(CarTurn_temp_value))*1,(int16)(speed_right-speed_left));  //ת���������
#endif 
   //�������ٱ仯��
 TurnControlPWM=(TurnControlPWM-TurnPWMLast>TurnDelta?TurnControlPWM+TurnDelta:TurnControlPWM);
 TurnControlPWM=((TurnControlPWM-TurnPWMLast<-TurnDelta?TurnControlPWM-TurnDelta:TurnControlPWM));
  //if(TurnControlPWM<0) TurnControlPWM = 0.8*TurnControlPWM;
 // TurnControlPWM=Turn_Out_Filter(TurnControlPWM);
   TurnPWMLast = TurnControlPWM ; 
}

/***----------------------------------����-----------------------***/
float  FrontErrorK_1=0.0;          //ǰһ�����ֵe(k-1)
float  FrontErrorK_2=0.0;          //ǰǰ�����ֵe(k-2)
float  AddAdjust_Value=0.0;        //�ۼƵ�����
//float  Turn2P,Turn2D;
/***-------------------------------------------------------------***/

///////////////////////////////���ֵPID����������//////////////////////////////////////////

    float  ProportionalGain=2.5;          //��������ϵ��Kp  
    
    float  DerivativeGain=0.05;        //����΢��ϵ��Td //hxd
    float  ControlPeriod=0.01;        //��������TΪ10ms
    
    //static float  MinError=0.0;               //��С���
//  static float   MaxIntegral=127.0;          //������ֵ������ֵ�޷�����ֹ���ֱ���
//  static float   MinIntegral=3.0;            //��С����ֵ������ֵ�޷�����ֹ���ֱ���
    float  Ka,Kb,Kc,Ki,Kd,CurrentError,AdjustValue;  
/////////////////////////////ת���⻷/////////////////////////////////////////////
float PID_Control_Turn(float Err)     //����ƫ��������ת�򻷵�ƫ����,���PID���ƺ�����PD����
{     

  //  ProportionalGain=Turn2P/100;          //����Turn����
    
   // DerivativeGain  =Turn2D/1000;
    CurrentError=Err;      //���㵱ǰ���ֵ
    
   // Ki=ControlPeriod/IntegralGain;           //�������ϵ��T/Ti
  //  Ki=0.0;                  //����PD����         
    
    
    Kd=DerivativeGain/ControlPeriod;    //����΢��ϵ��Td/T
    Kd=Kd/10;
#if  1
    //λ��ʽPD�㷨
   // AdjustValue=CurrentError*ProportionalGain+Kd*(CurrentError - FrontErrorK_1); 
    AdjustValue=CurrentError*ProportionalGain+Kd*Error_Delta;
#else
 
    Ka =ProportionalGain+Kd+Ki;                //����PID��ǰ���ϵ��
    Kb =-ProportionalGain-2*Kd;                //����PIDǰһ�����ϵ��
    Kc =Kd;                                    //����PIDǰǰ�����ϵ��
     AdjustValue=AddAdjust_Value+CurrentError*Ka+FrontErrorK_1*Kb+FrontErrorK_2*Kc;        //��������ʽPID�㷨�������ֵ
#endif
       FrontErrorK_2=FrontErrorK_1;              //��ǰһ�ε�����ǰǰ��
       FrontErrorK_1=CurrentError;                          //����ǰ����ǰһ�� 
    AddAdjust_Value=AdjustValue;

    return  AdjustValue;
}

//////////////////////////////////�ڻ�΢������/////////////////////////////
/*****************��speed_right-speed_left����΢������*******************/

/***-----------------ϵ��---------------***/
float gama = 0.5; // ΢�ַŴ�ϵ��
float Ts = 0.01;        //��������Ts
float Td = 0.01; //΢��ʱ�䳣��,���ڻ�΢��ʱ�䳣����ͬ
float c1,c2,c3;//΢������ϵ��
float Yd_Last,Y_Last,Yd;
/***------------------------------------***/

float Differential_In_Advance(float Y)
{
  //Td =Turn_P/Turn_D;
  c1 = gama*Td/(gama*Td+Ts);
  c2 = (Td+Ts)/(gama*Td+Ts);
  c3 = Td/(gama*Td+Ts);
          
  Yd = c1*Yd_Last + c2*Y - c3*Y_Last;
  Yd_Last = Yd;
  Y_Last = Y;
  return Yd;
}


