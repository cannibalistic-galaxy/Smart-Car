/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		�ж��ļ�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/



#include "isr.h"
#include "headfile.h"


float var[SHANWAI];
extern int16 speed_left,speed_right;
extern uint16 speed_out_times;
extern char circle_Flag;
extern float round_radius;
float SpeedGiven_2;
int speed_control_flag;
extern int Pnn,Dnn;
extern uint8 stop_flag;
float Speed_control,Speed_control_length,Speed_old;
float dingshijiansu,speed_flag,first_flag;

void PIT_CH0_IRQHandler(void)
{
   static uint16 timecnt,timecnt2,timecnt3,timecnt4,timecnt5,onetime,Flag_1S,Round_ErrCunt;
   gpio_set(D7,1);     //ʱ���������
   timecnt++;
   timecnt3++;
   Get_AccData();
   Angle_Calculate();
   
   if(Car_Angle>60||Car_Angle<0) 
   {
     podao_dianbo_flag=1;
    // BUZZER_ON;
   }
  /* else  
   {
     podao_dianbo_flag=0;
      BUZZER_OFF;
   }*/
   if(podao_dianbo_flag==1) timecnt5++;
   if(timecnt5>33) 
   {
     podao_dianbo_flag=0;
     timecnt5=0;
   }
  /* if(podao_dianbo_flag) BUZZER_ON;
   else BUZZER_OFF;*/
   /************************������ʱ***********************/
  if(onetime==0)
  {
    timecnt2++;
   if(timecnt2>999) //����3s��ͣ��
   {
     onetime=1;    
    PerfectStop=1;
   } 
  }
  /*****************************************************/
  /***********************����ģʽ��ʱ*************************/
  if(timecnt3==33) //0.1s   0.1*3=0.3=30c
  {
     if(dingshijiansu==1) 
   {
       speed_flag++;  
     if(speed_flag>=X1&&speed_flag<=X2) 
     {
       first_flag=1; //������������
       //SpeedGiven=SpeedCut;
     }
       else 
       {
         first_flag=0;
         //SpeedGiven=SpeedGivenTemp;
       }
   }
   timecnt3=0;
  }//
  /**************************************************************/
  /**************************ǿ�Ƴ���************************************/
   if(timecnt==333)   //1S
   { 
      if(Turn_Flag2==1)Round_ErrCunt++;    //������Ϻ󣬻������־��ʼ����
      else Round_ErrCunt=0;
      if(Round_ErrCunt>4) Go_Out_Circle=2;    //����4�κ�Ҳ����4����֮�ڻ�û�г�����,ǿ�Ƴ�����������ݴ���    
      timecnt=0;    
   }
   /*************************��������ʱ��*************************/
  if(second_infer==1) 
  {
    timecnt4++;
  }
 if(timecnt4>SetTime+100) 
  {
    timecnt4=0;
    second_infer=0; //1s��������
  }
 /*****************************************************************/
 /*************************������ʱ*********************************/
    if(Go_Out_Circle==1)//��⵽����
    {   
      Flag_1S++;    
    }
   if(Flag_1S>=SetTime)     //��ʱ����mS�󳹵׳�����*3
     {
        Flag_1S=0;
        Go_Out_Circle=2;
     }
   /******************************************************/
  
   
  speed_control_flag++;
  //speed_out_times++;
   //���ú�����
    AD_Filter(); 

    AD_Calculate(); 
    
    DividePositon();
    speed_test();

    TurnErr_Caculate();
    turn_ctrl();
    if(speed_control_flag==3) //9ms����һ���ٶ�      
    {
     speed_ctrl((int)SpeedGiven);   //�����ٶȸ���
     speed_control_flag = 0;
    }
    
    if(Speed_now-Speed_old>0) Speed_control++;
    //if(Speed_now<)
  /*  if(Speed_control>=300) //��ʼֱ������
    {
      Speed_control = 0;      
      Speed_control_length=100;//SpeedGiven = 0.8*SpeedGivenTemp;
    }
    if(Speed_control_length>0)      
    {
      Speed_control_length--;
      SpeedGiven = 0;//SpeedGivenTemp;
      BUZZER_ON;
    }
    if(Speed_control_length==0&&Speed_control==0) 
    {
      SpeedGiven=SpeedGivenTemp;
      BUZZER_OFF;
    }*/
    CarRun();
    
     all_ctrl();
      var[0]=(float)Car_Angle;
      var[1]=(float)Angle_acc;
      var[2]=(float)Angle_Speed;
     /* var[3]=(float)AD_Error_3_Max;
      var[4]=(float)100*huandaonum;
      var[5]=(float)100*Go_Out_Circle;
      var[6]=(float)100*Turn_Flag;
      var[7]=(float)100*Turn_Flag2;*/
      //var[8]=(float)abs(speed_left-speed_right);
      
     Speed_old = Speed_now; //�ٶȸ���
     gpio_set(D7,0);     //ʱ���������
  // gpio_set(D7,0);
    PIT_FLAG_CLR(pit0);   //��ն�ʱ��־λ

}

void PIT_CH1_IRQHandler(void)
{
    PIT_FLAG_CLR(pit1);

    
}

void IRQ_IRQHandler(void)
{
     // CLEAR_IRQ_FLAG;
   
    /* Left_PWM=0;
    Right_PWM=0;
    Motor_Ctrl((int32)Left_PWM,(int32)Right_PWM);*/
 
  stop_flag=1;
    gpio_set(G0,0);
    gpio_set(G1,0);
    gpio_set(G2,0);
    gpio_set(G3,0);
  //  BUZZER_ON;
 //  DisableInterrupts; 
    
    CLEAR_IRQ_FLAG;
    
}


void KBI0_IRQHandler(void)
{
    CLEAN_KBI0_FLAG;
    
}





/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ�� ͨ��0���ж�
void PIT_CH0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



