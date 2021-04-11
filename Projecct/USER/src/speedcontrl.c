/*****
*
* @brief：本程序用于电机控制,电感值处理
* @author ：gerui，hzy，linxuankun
* @date   2018-6-2
* 快达到目标速度时输出除2
*左水平1 右水平2 中间3 左斜4 右斜5
********/



#include "headfile.h"
#include "speedcontrl.h"



#define OFF      0
#define ON       1
#define THREEMODE 2
#define abs(x)   (x>0?x:-x)
#define GYRO_SET   48

#define TurnDelta   100

#define DifferentialInAdvance   0 //是否启用微分先行
#define Slope   1  //是否启用最小二乘
#define balance 1 
#define stop    1

/*******半径参数*****************/



/************************/
uint16 AD_Value[ADN];    //处理后的电感数据
uint16 dis_AD[ADN];      //未限幅之前的电感原始值
uint16 AD_Value_temp[ADN][3];   //临时电感数据
uint16 AD_MAX[ADN];
uint16 AD_MIN[ADN] = {20,20,20,20,20};
int16 speed_left,speed_right;     //编码器数值
int16 speed_last_L,speed_last_R;
float AD_Error,Delt_Error;
uint8 Calculate_Length;
uint16 speed_out_times;
uint8 stop_flag;
uint8 carcarstop=0;
/*****************************8月14新加的速度控制参数****************************/
//之前速度环为8和135
float Ratio_Encoder_Left = 207/(5860*0.009);// 左轮速度=counter*左轮周长(mm)/(左轮转一圈对应的脉冲数*程序周期)
float Ratio_Encoder_Righ = 207/(5860*0.009);// 右轮速度=counter*右轮周长(mm)/(右轮转一圈对应的脉冲数*程序周期)
//1175
float g_fRealSpeed = 0;				//真实速度
float g_fLeftRealSpeed;
float g_fRighRealSpeed;
float g_fExpectSpeed;		//期望速度
float g_fSpeedError;				//速度偏差
float g_fSpeedErrorTemp[5] = {0};
float fSpeedErrorInteg = 0;
float g_fSpeedControlOut = 0;		//速度输出
/*********************************************************************************/
/***-------------------------参数----------------------------------------------***/
float  Speed_P,Speed_I, Turn_P, Turn_D,Turn_I,Turn2P,Turn2D,SpeedGiven,Round_P, Round_D,speedweight,BT,X1,X2,SpeedCut;
char PerfectStop;
int offset = 0;
float Error_Delta;
int32  Turn_Out_Filter(int32);
float Slope_Calculate(uint8,uint8,float *);
void Push_And_Pull(float *,int,float);
extern int16 FILTRATE_Kalman(int16 MsrValue);
void Motor_Ctrl(int32 Left_Motor,int32 Right_Motor)     //将输出PWM转为四路形式
{   
   
   uint32 left,right;
   if(Left_Motor>=0)
   { 
     //left=Left_Motor+left_dd_front;      //加死区电压为了直立环调试方便
     left=Left_Motor;                                      //left_dd_front  死区纠正量
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

/*-----------------------速度函数--------------------------------------------*/

void speed_test(void)   // 速度获取
{
     speed_right=-ftm_count_get(ftm0)*4;   //左轮
     ftm_count_clean(ftm0);
     if(!gpio_get(E1))speed_right=-speed_right;
     
   /* speed_right=(speed_right>500?500:speed_right);
     speed_right=(speed_right<-200?-200:speed_right);
     
     speed_right=(speed_right-speed_last_R>40?speed_right+40:speed_right);
     speed_right=(speed_right-speed_last_R<-40?speed_left-40:speed_left);
    */
      speed_right=(int16)(0.8*speed_right+0.2*speed_last_R);        //编码器互补滤波
       speed_last_R=speed_right;
     
      
     speed_left=ftm_count_get(ftm1)*4;   //右轮
      ftm_count_clean(ftm1);
     if(!gpio_get(H5))speed_left=-speed_left;
     
    /* speed_left=(speed_left>500?500:speed_left);
     speed_left=(speed_left<-200?-200:speed_left);
     
     speed_left=(speed_left-speed_last_L>40?speed_left+40:speed_left);
     speed_left=(speed_left-speed_last_L<-40?speed_left-40:speed_left);*/
     
     speed_left=(int16)(0.8*speed_left+0.2*speed_last_L);
     speed_last_L=speed_left;
     

}

/***-------------------------速度环参数----------------------------------------***/
int16 Speed_now;                        //当前车速度
int32 SpeedControlOld=0;                //旧车速度控制量
int32 SpeedControlNew=0;                //新车速度控制量
int32 SpeedControlDelta=0;              //速度偏差
int32 SpeedControlPWM=0;                //速度环最终算出的pwm


/********************************自己定义的参数*************************************/
float Sample_Count=30,Divider_Prarm_Count=3;//
uint16 Speed_D;//总的参数中并未用到，为0
/***-----------------------------------------------------------------------------***/


void speed_ctrl(int SpeedSet)    //速度PID控制
{
  
  SpeedControlOld=SpeedControlNew;                //将当前速度存为上次速度
// SpeedAdd_Left/=Sample_Count;                   //速度累计值除30，即Sample_Count=30；
 // SpeedAdd_Right/=Sample_Count;                 //速度累计值除30
#if 1
  Speed_now=(speed_left+speed_right);    //左右轮速度累计值求平均,这里不除2了，保证512线的编码器精度
  
    
  SpeedControlNew=PID_Control_Speed((uint16)SpeedSet,Speed_now);   //PID控制速度，该函数输出为新的PID计算后速度输出
  SpeedControlDelta=SpeedControlNew-SpeedControlOld;        //求速度偏差，速度前后改变量
  SpeedControlDelta/=Divider_Prarm_Count;               //速度偏差除以5，Divider_Prarm_Count=5，细分控制，防止震荡
#else
  SpeedControlNew=SpeedControl((uint16)SpeedSet);
#endif
  
}

void CarRun(void)              //输出速度环PWM
{       
    //SpeedControlPWM=0.3*SpeedControlOld+0.7*SpeedControlNew;       //速度分量得出
    SpeedControlOld=SpeedControlNew;    //速度控制量更新
#if 1
  SpeedControlOld+=SpeedControlDelta;   // 直立车速度控制量调整
#endif
    SpeedControlPWM=SpeedControlOld;       //速度分量

    /*******速度限幅*********/
   // SpeedControlPWM=(SpeedControlPWM>6500?6500:SpeedControlPWM);
   // SpeedControlPWM=(SpeedControlPWM<-6500?-6500:SpeedControlPWM);
  
}


/////////////////////////////////速度环PID参数//////////////////////////////////////////////////
     float  ProportionalGain_Run=30;//110;      //给定比例系数Kp
     float  IntegralGain_Run=0.02;         //给定积分系数Ti   0.007
     float  DerivativeGain_Run=0;//13;    //给定微分系数Td
     float  ControlPeriod_Run=0.01;        //控制周期T
     float  FrontErrorK_1_Run=0.0;          //前一次误差值e(k-1)
     float  FrontErrorK_2_Run=0.0;          //前前次误差值e(k-2)
     float  MaxError_Run=100.0;            //最大误差
     float  MinError_Run=30;               //最小误差
     float  AddAdjust_Value_Run=0;        //累计调整量
     uint8  Integ_FLAG=1;                      //积分分离标志
    float  Ka_Run,Kb_Run,Kc_Run,Ki_Run,Kd_Run,CurrentError_Run,AdjustValue_Run; 
    uint16 speedp_begin;
     float speedcount;    //速度p迭代值    
//////////////////////////////////////////////////////////////////////////////////////
int32 PID_Control_Speed(uint16 Speed_set,int Speed_now)    //输入当前平均速度与设定平均速度，返回经过一次速度PID计算后得出的输出量
{
    
     //speedp_begin=35-Speed_P;
     //speedcount=0.02*speedcount+0.98*speedp_begin;   //起步迭代  
     //if(runtime>500) ProportionalGain_Run=500;
      ProportionalGain_Run=Speed_P;          //给定计算，Speed_p、I、D为修改参数出口
    //if(runtime>500) IntegralGain_Run=0.035;
     //Ki_Run=((float)(Speed_I))/1000;      //Speed_I包含了Kp因子??解耦
    
  //  DerivativeGain_Run=((float)(Speed_D))/100;
    
    
     MaxError_Run=5;      //最大误差赋值
                                                        //增量PID前前次误差系数   
    CurrentError_Run=Speed_set-Speed_now;             //计算当前速度误差值 
    if(CurrentError_Run>=300) Ki_Run = Speed_I/1000;
    if(CurrentError_Run>=250&&CurrentError_Run<300) Ki_Run = Speed_I/2000;
    if(CurrentError_Run>=200&&CurrentError_Run<250) Ki_Run = Speed_I/3000;
    if(CurrentError_Run>=150&&CurrentError_Run<200) Ki_Run = Speed_I/4500;
    if(CurrentError_Run>=100&&CurrentError_Run<150) Ki_Run = Speed_I/5000;
    if(CurrentError_Run>=50&&CurrentError_Run<100) Ki_Run = Speed_I/5500;
    if(CurrentError_Run>=0&&CurrentError_Run<50)  Ki_Run = 0;
    //速度偏差限幅
  // CurrentError_Run=(CurrentError_Run>120?120:CurrentError_Run);
   //CurrentError_Run=(CurrentError_Run<-120?-120:CurrentError_Run);
    
    
    //速度环积分分离，防止超调震    
    
 //    Ki_Run=(CurrentError_Run>20?0:Ki_Run);
   //  Ki_Run=(CurrentError_Run<-20?0:Ki_Run);
     
    if(abs(CurrentError_Run)<MinError_Run||CurrentError_Run<-10)            //误差阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
      {
          AdjustValue_Run=0.85*AddAdjust_Value_Run;
          FrontErrorK_2_Run=FrontErrorK_1_Run;         //将前一次的误差赋给前前次
          FrontErrorK_1_Run=CurrentError_Run;          //将当前误差赋给前一次
      }
    else         //误差大于阈值时 
      {  
      
        
        Ka_Run =ProportionalGain_Run+Kd_Run+Ki_Run;                //增量PID当前误差系数，Ki原本需要乘以积分分离系数
        Kb_Run =-ProportionalGain_Run-2*Kd_Run;                //增量PID前一次误差系数
        Kc_Run =Kd_Run;
        AdjustValue_Run=AddAdjust_Value_Run+CurrentError_Run*Ka_Run+FrontErrorK_1_Run*Kb_Run+FrontErrorK_2_Run*Kc_Run;    //按照增量式PID算法计算输出值
        FrontErrorK_2_Run=FrontErrorK_1_Run;              //将前一次的误差赋给前前次
        FrontErrorK_1_Run=CurrentError_Run;                          //将当前误差赋给前一次
      }
   // AdjustValue_Run=(AdjustValue_Run>6500?6500:AdjustValue_Run);
    AddAdjust_Value_Run=AdjustValue_Run;                      //累积调整量赋值  uk-1=uk
    return  ((int32)AdjustValue_Run);
}
/******************************8月14日新加的速度控制函数**************************/
/**
 * @file	计算速度偏差
 * @note      	产生全局变量g_fSpeedError
 * @date	2018.8.14
 */
#if 0
void CalSpeedError(uint16 Speed_set)
{
  g_fExpectSpeed = Speed_set;
  
  g_fLeftRealSpeed = speed_left;
  g_fRighRealSpeed = speed_right;
  g_fRealSpeed = g_fLeftRealSpeed + g_fRighRealSpeed;				//真实速度
  g_fSpeedError =  g_fExpectSpeed - g_fRealSpeed;
  g_fSpeedErrorTemp[4] = g_fSpeedErrorTemp[3];
  g_fSpeedErrorTemp[3] = g_fSpeedErrorTemp[2];
  g_fSpeedErrorTemp[2] = g_fSpeedErrorTemp[1];
  g_fSpeedErrorTemp[1] = g_fSpeedErrorTemp[0];
  g_fSpeedErrorTemp[0] = g_fSpeedError;

}
/**
 * @file        速度控制
 * @note      	速度梯度平滑
 * @date	2018.8.14
 */
int32 SpeedControl(uint16 Speed_set)
{
  int8 index=1;

  CalSpeedError(Speed_set);	//计算速度偏差
  g_fSpeedError = (g_fSpeedError>200?200:g_fSpeedError);//速度偏差限幅
  //积分分离
  if((g_fSpeedError<=100)&&(g_fSpeedError>=-100)) index=1;
  else  index=0;	
  fSpeedErrorInteg = index * Speed_I * g_fSpeedError * 0.01;
  //速度控制算法（增量式PI）
  g_fSpeedControlOut += Speed_P/10*(g_fSpeedErrorTemp[0]-g_fSpeedErrorTemp[1]) + fSpeedErrorInteg;
 // g_fSpeedControlOut = (g_fSpeedControlOut>=6000?6000:g_fSpeedControlOut);
  return ((int32)g_fSpeedControlOut);
	
}
#endif

/*********************************************************************************/
/***--------------------------转向环参数---------------------------------------***/
float CarTurn_temp_value=0;             //直立车转向控制分量
int32 TurnControlPWM=0;                 //最终的转向控制pwm分量
float Car_Turn_Err=0;                      //通过pid和当前速度乘权值算出的转向偏差量，即偏差量是与当前速度也有关，而不是当前电感值固定的偏差
float M_P;//陀螺仪乘的系数
float speed_err;


//////////////////////////////差速PID参数////////////////////////////////////////////
     float  ProportionalGain_TD=66;      //给定比例系数Kp     66 
     float  IntegralGain_TD=0.24;           //给定积分系数Ti  0.24
     float  DerivativeGain_TD=0.01;        //给定微分系数Td
     float  ControlPeriod_TD=0.01;        //控制周期T
     float  FrontErrorK_1_TD=0.0;          //前一次误差值e(k-1)
     float  FrontErrorK_2_TD=0.0;          //前前次误差值e(k-2)
     float  AddAdjust_Value_TD=0.0;        //累计调整量
     float  Ka_TD,Kb_TD,Kc_TD,Ki_TD,Kd_TD,CurrentError_TD,AdjustValue_TD;
     float  FrontError[5] = {0,0,0,0,0};
///////////////////////////////////////////////////////////////////////////////////////
int32 PID_TurnDiffer(int16 GivenValue,int16 MeasureValue)   //串级控制，转向控制内环，输入给定的转向控制分量和速度测量值
{
    
       ProportionalGain_TD=Turn_P/10;
      IntegralGain_TD=Turn_I/100; 
       DerivativeGain_TD=Turn_D/100;
      // ProportionalGain_TD = ProportionalGain_TD*(1-0.5*(SpeedGiven/(speed_left+speed_right)>1?1:SpeedGiven/(speed_left+speed_right)));
       
         
   
    //Ki_TD=0;      //取消积分作用
    
#if DifferentialInAdvance
    Kd_TD=0;//微分先行内环此处不用D
#endif
 
    
    CurrentError_TD=MeasureValue-GivenValue;      //计算当前误差值，即两轮速度实际差值与给定的速度差值
      
#if 1 //位置式    
    FrontError[4] = FrontError[3];
    FrontError[3] = FrontError[2];
    FrontError[2] = FrontError[1];
    FrontError[1] = FrontError[0];
    FrontError[0] = CurrentError_TD;
    AdjustValue_TD=FrontError[0]*ProportionalGain_TD + IntegralGain_TD*(FrontError[4]+FrontError[3]
                +FrontError[2]+FrontError[1]+FrontError[0]) + DerivativeGain_TD*(FrontError[0]-FrontError[1]);
    
#else //增量式
       Ki_TD=ControlPeriod_TD/IntegralGain_TD;          //计算积分系数T/Ti
       Kd_TD=DerivativeGain_TD/ControlPeriod_TD;           //计算微分系数Td/T 
       Ka_TD =ProportionalGain_TD+Kd_TD+Ki_TD;                //增量PID当前误差系数
       Kb_TD =-ProportionalGain_TD-2*Kd_TD;                //增量PID前一次误差系数
       Kc_TD =Kd_TD;                                    //增量PID前前次误差系数
       AdjustValue_TD=AddAdjust_Value_TD+CurrentError_TD*Ka_TD+FrontErrorK_1_TD*Kb_TD+FrontErrorK_2_TD*Kc_TD;        //按照增量式PID算法计算输出值
       FrontErrorK_2_TD=FrontErrorK_1_TD;              //将前一次的误差赋给前前次
       FrontErrorK_1_TD=CurrentError_TD;                          //将当前误差赋给前一次
        AddAdjust_Value_TD=AdjustValue_TD;
#endif
       
    return  ((int32)AdjustValue_TD);  
}

/*******************************总的串联控制，重中之中的核心速度算法！！！！！！！！！！*******************************************/
int32 Left_PWM=0,Right_PWM=0;       //最终输入到电机的PWM
uint8 jj;
/***----------------------------电感偏差-----------------------------***/

float AD_Last_Error;
void all_ctrl(void)   //差速与速度PID合并函数
{
  
#if stop
  if((((AD_Value[0]<100&&AD_Value[1]<100&&AD_Value[2]<100&&AD_Value[5]<100)||(gpio_get(H2) == 0))||(gpio_get(I2)==0&&PerfectStop==1))||carcarstop)  //停车保护
#else
   if((gpio_get(H2) == 0))  //单独停车保护（无出界停车）
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

    Left_PWM =SpeedControlPWM + TurnControlPWM ;       //直立环减转向环加速度环
    Right_PWM=SpeedControlPWM - TurnControlPWM ;        //直立环加转向环加速度环
    //  Left_PWM =3100 + TurnControlPWM ;       //直立环减转向环加速度环
    //Right_PWM=3100 - TurnControlPWM ;        //直立环加转向环加速度环
    
  

#else  
   if(TurnControlPWM>0)
  {
    Left_PWM =SpeedControlPWM + TurnControlPWM + 2000;       //直立环减转向环加速度环
    Right_PWM=SpeedControlPWM - TurnControlPWM + 2000;        //直立环加转向环加速度环
  }
  else
  {
  Left_PWM =SpeedControlPWM + TurnControlPWM + 2000;       //直立环减转向环加速度环
  Right_PWM=SpeedControlPWM - TurnControlPWM + 2000;         //直立环加转向环加速度环
  
  }
#endif  
  if(Left_PWM>9900) Left_PWM=9900;                        //限幅
  else if(Left_PWM<-9900) Left_PWM=-9900;
  if(Right_PWM>9900) Right_PWM=9900;
  else if(Right_PWM<-9900) Right_PWM=-9900;
    /********停车保护********/
/*#if stop
  if((((AD_Value[0]<80&&AD_Value[1]<80&&AD_Value[3]<80&&AD_Value[4]<80)||(gpio_get(H2) == 0))||(gpio_get(I2)==0&&PerfectStop==1))||carcarstop)  //停车保护
  
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

  
  if(Left_PWM<200&&Left_PWM>-200) Left_PWM=0;                //死区处理？？
  
  if(Right_PWM<200&&Right_PWM>-200) Right_PWM=0;
  

  Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);               //变成四路输出
  
}

float TurnSpeed;
void TurnErr_Caculate(void)           //转向偏差计算。即电感PID控制矫正
{
   Get_Gyro();    //读取车转动角速度   
   //mpu_gyro_x=FILTRATE_Kalman(mpu_gyro_x)-47;
   TurnSpeed=0.1*(mpu_gyro_z-GYRO_SET);
   TurnSpeed = (float)FILTRATE_Kalman((int16)TurnSpeed);
   
   Carinfo.yawrate=TurnSpeed;     
   
  Car_Turn_Err=PID_Control_Turn(AD_Error);
  
  Car_Turn_Err=Car_Turn_Err*2-0.005*M_P * TurnSpeed;    //速度占权重参数speedweight
  
//  Car_Turn_Err=(Car_Turn_Err>1200?1200:Car_Turn_Err);
 // Car_Turn_Err=(Car_Turn_Err<-1200?-1200:Car_Turn_Err);
   
}


/*-------------------------------转向环----------------------------------------------------------------------------*/

int32 TurnPWMLast;
float TurnSpeed_Sum = 0;
/***---------------------------------------------------------------------------***/

void turn_ctrl(void)      //转向PID控制部分
{

  CarTurn_temp_value=Car_Turn_Err;      //车的给定转向误差赋值给转向控制分量  
  //CarTurn_temp_value = 500;
#if     DifferentialInAdvance
 //speed_err = Differential_In_Advance((float)(speed_right-speed_left));//副回路微分
  speed_err = Differential_In_Advance((float)(-0.8*TurnSpeed));
   TurnControlPWM=PID_TurnDiffer(CarTurn_temp_value,speed_err); 
  
#else
  //TurnControlPWM=PID_TurnDiffer((Get_Car_curvature(CarTurn_temp_value))*1,(float)(0.8*TurnSpeed));  //转向分量计算
   
   TurnControlPWM=PID_TurnDiffer(((int16)Get_Car_curvature(CarTurn_temp_value))*1,(int16)(speed_right-speed_left));  //转向分量计算
#endif 
   //限制轮速变化率
 TurnControlPWM=(TurnControlPWM-TurnPWMLast>TurnDelta?TurnControlPWM+TurnDelta:TurnControlPWM);
 TurnControlPWM=((TurnControlPWM-TurnPWMLast<-TurnDelta?TurnControlPWM-TurnDelta:TurnControlPWM));
  //if(TurnControlPWM<0) TurnControlPWM = 0.8*TurnControlPWM;
 // TurnControlPWM=Turn_Out_Filter(TurnControlPWM);
   TurnPWMLast = TurnControlPWM ; 
}

/***----------------------------------参数-----------------------***/
float  FrontErrorK_1=0.0;          //前一次误差值e(k-1)
float  FrontErrorK_2=0.0;          //前前次误差值e(k-2)
float  AddAdjust_Value=0.0;        //累计调整量
//float  Turn2P,Turn2D;
/***-------------------------------------------------------------***/

///////////////////////////////电感值PID调节器参数//////////////////////////////////////////

    float  ProportionalGain=2.5;          //给定比例系数Kp  
    
    float  DerivativeGain=0.05;        //给定微分系数Td //hxd
    float  ControlPeriod=0.01;        //控制周期T为10ms
    
    //static float  MinError=0.0;               //最小误差
//  static float   MaxIntegral=127.0;          //最大调整值，调整值限幅，防止积分饱和
//  static float   MinIntegral=3.0;            //最小调整值，调整值限幅，防止积分饱和
    float  Ka,Kb,Kc,Ki,Kd,CurrentError,AdjustValue;  
/////////////////////////////转向外环/////////////////////////////////////////////
float PID_Control_Turn(float Err)     //输入偏差计算给于转向环的偏差量,电感PID控制函数，PD控制
{     

  //  ProportionalGain=Turn2P/100;          //调节Turn即可
    
   // DerivativeGain  =Turn2D/1000;
    CurrentError=Err;      //计算当前误差值
    
   // Ki=ControlPeriod/IntegralGain;           //计算积分系数T/Ti
  //  Ki=0.0;                  //利用PD计算         
    
    
    Kd=DerivativeGain/ControlPeriod;    //计算微分系数Td/T
    Kd=Kd/10;
#if  1
    //位置式PD算法
   // AdjustValue=CurrentError*ProportionalGain+Kd*(CurrentError - FrontErrorK_1); 
    AdjustValue=CurrentError*ProportionalGain+Kd*Error_Delta;
#else
 
    Ka =ProportionalGain+Kd+Ki;                //增量PID当前误差系数
    Kb =-ProportionalGain-2*Kd;                //增量PID前一次误差系数
    Kc =Kd;                                    //增量PID前前次误差系数
     AdjustValue=AddAdjust_Value+CurrentError*Ka+FrontErrorK_1*Kb+FrontErrorK_2*Kc;        //按照增量式PID算法计算输出值
#endif
       FrontErrorK_2=FrontErrorK_1;              //将前一次的误差赋给前前次
       FrontErrorK_1=CurrentError;                          //将当前误差赋给前一次 
    AddAdjust_Value=AdjustValue;

    return  AdjustValue;
}

//////////////////////////////////内环微分先行/////////////////////////////
/*****************对speed_right-speed_left进行微分运算*******************/

/***-----------------系数---------------***/
float gama = 0.5; // 微分放大系数
float Ts = 0.01;        //控制周期Ts
float Td = 0.01; //微分时间常数,和内环微分时间常数相同
float c1,c2,c3;//微分先行系数
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


