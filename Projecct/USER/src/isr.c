/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		中断文件
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
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
   gpio_set(D7,1);     //时间测试引脚
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
   /************************发车定时***********************/
  if(onetime==0)
  {
    timecnt2++;
   if(timecnt2>999) //发车3s不停车
   {
     onetime=1;    
    PerfectStop=1;
   } 
  }
  /*****************************************************/
  /***********************减速模式定时*************************/
  if(timecnt3==33) //0.1s   0.1*3=0.3=30c
  {
     if(dingshijiansu==1) 
   {
       speed_flag++;  
     if(speed_flag>=X1&&speed_flag<=X2) 
     {
       first_flag=1; //环岛减速屏蔽
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
  /**************************强制出环************************************/
   if(timecnt==333)   //1S
   { 
      if(Turn_Flag2==1)Round_ErrCunt++;    //进环完毕后，环岛错标志开始计数
      else Round_ErrCunt=0;
      if(Round_ErrCunt>4) Go_Out_Circle=2;    //计数4次后，也就是4秒钟之内还没有出环岛,强制出环岛，提高容错率    
      timecnt=0;    
   }
   /*************************环岛屏蔽时间*************************/
  if(second_infer==1) 
  {
    timecnt4++;
  }
 if(timecnt4>SetTime+100) 
  {
    timecnt4=0;
    second_infer=0; //1s不检测进环
  }
 /*****************************************************************/
 /*************************出环定时*********************************/
    if(Go_Out_Circle==1)//检测到出环
    {   
      Flag_1S++;    
    }
   if(Flag_1S>=SetTime)     //定时多少mS后彻底出环岛*3
     {
        Flag_1S=0;
        Go_Out_Circle=2;
     }
   /******************************************************/
  
   
  speed_control_flag++;
  //speed_out_times++;
   //调用函数。
    AD_Filter(); 

    AD_Calculate(); 
    
    DividePositon();
    speed_test();

    TurnErr_Caculate();
    turn_ctrl();
    if(speed_control_flag==3) //9ms控制一次速度      
    {
     speed_ctrl((int)SpeedGiven);   //输入速度给定
     speed_control_flag = 0;
    }
    
    if(Speed_now-Speed_old>0) Speed_control++;
    //if(Speed_now<)
  /*  if(Speed_control>=300) //开始直道控速
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
      
     Speed_old = Speed_now; //速度更新
     gpio_set(D7,0);     //时间测试引脚
  // gpio_set(D7,0);
    PIT_FLAG_CLR(pit0);   //清空定时标志位

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
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器 通道0得中断
void PIT_CH0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位

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



