/**************************************************
 * 用来存放电感处理和循迹函数
 *小于0右转，大于0左转
 * 0右，3右,斜电感左4，右3,中间电感左2右5,外电感左1右0
 *************************************************/
#include "headfile.h"
   
#define OFF      0
#define ON       1
#define THREEMODE 2
#define abs(x)   (x>0?x:-x)  
#define AI       1
  
#define Slope   1  //是否启用最小二乘
#define fuzzy   1  //是否使用fuzzy   

   
char flag_round=OFF;     //环岛标志位
char flag_round_out = OFF; //出环标志
/***环岛参数***/
char circle_Flag,Turn_Flag,BaoMing_Flag,Turn_Flag2,Turn_Right_Flag,Turn_Left_Flag,Go_Out_Circle;
char Turn_Left_Flag_1,Turn_Left_Flag_2,Turn_Left_Flag_3,Turn_Right_Flag_1,Turn_Right_Flag_2,Turn_Right_Flag_3;
//char Force_In;  //强制进环标志位
/****
 *7月16日更新定义
 ***/
char Turn_Flag_Last; //上次转向判断标志位
char TurnChange_1; //转向状态切换
float AD_Error_Lead; //圆环引导时使用的偏差
/***/
int16 turn_Error,turn_lastError,times;   //切点判断差值
uint16 second_infer;  //第二次检测
char panduanyici;
char huandaonum;
char jiansu_flag;
char podao_dianbo_flag;
float round_radius;      //编码器积分累加半径
float P_Speed;
float ADError2,ADSub43;
float DirectionErr[9]={0},Error_Delta_Deceleration;
float AD_real_Error;
float AD_Error_2,AD_Error_3;//水平电感差值
float ADError_3[3],AD_Error_3_Max;
float SpeedGivenTemp; //速度暂存
float AD_Error_Avg,AD_Error_In,AD_Error_In_Last; //入环偏差
float Last_AD_Error[3];  //偏差储存数组
float AD_Second_Derivative;  //偏差二阶导数
float BeginSpeed = 1600;
uint16 ADSum34;
int16 ADSub34;
uint16 AD_Middle_Max; //中间电感最大值用来做分母上
uint16 AD_More; //超出部分叠加到分母上
float AD_More_Para;//乘的系数
float ProportionalGain_last;
float Angle_Sum_R;
extern float dingshijiansu,first_flag;

void AD_Calculate(void)   //电感值计算处理，得到差值进行差速处理
{ 
  AD_Error_3 = ((dis_AD[4]/100)*(dis_AD[3]/100)*(dis_AD[2]/100)*(dis_AD[5]/100))/(4096-dis_AD[4]+4096-dis_AD[3]+4096-dis_AD[2]+4096-dis_AD[5]); //神奇判断环岛算法
  //************************环岛判断（判断全部使用原始值！）*************************************/
  if(circle_Flag==0&&second_infer==0&&AD_Error_3>=JinZhi) //判断环岛,开始进环
  {  
    huandaonum++; //判断进环岛次数
    switch(roundmode) //屏蔽环岛
    {
    case 0:circle_Flag=0;huandaonum=0;break; //屏蔽所有环岛
    case 1:                                   //第一个环岛
      if(huandaonum==1) {circle_Flag=1;BUZZER_ON;}   
      else {circle_Flag=0;second_infer=1;}
      break;
    case 2:                                   //一二环岛
      if(huandaonum==1||huandaonum==2) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 3:                                  //一二三环岛
      if(huandaonum==1||huandaonum==2||huandaonum==3) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 4:                                  //全部环岛
      {circle_Flag=1;BUZZER_ON;}
      break;
    case 5:                                  //只第二个环岛
      if(huandaonum==2) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 6:                                  //第二三个环岛
      if(huandaonum==2||huandaonum==3) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 7:                                   //二三四个环岛
      if(huandaonum==2||huandaonum==3||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 8:                                  //只第三个环岛
      if(huandaonum==3) {circle_Flag=1;BUZZER_ON;}   
      else {circle_Flag=0;second_infer=1;}
      break;
    case 9:                                   //三四个环岛
      if(huandaonum==3||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 10:                                  //只第四个环岛
      if(huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 11:                                  //第一第三个环岛
      if(huandaonum==1||huandaonum==3) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 12:                                  //134
      if(huandaonum==1||huandaonum==3||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 13:                                   //14
      if(huandaonum==1||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 14:                                    //124
      if(huandaonum==1||huandaonum==2||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 15:                                     //24
      if(huandaonum==2||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    }               
        
       
        //找到切点前判断一次左右：
        //左侧进环
        if(dis_AD[4]<dis_AD[3])Turn_Left_Flag=1;
        //右侧入环岛
        if(dis_AD[4]>dis_AD[3])Turn_Right_Flag=1;       
        if(Turn_Left_Flag==0&&Turn_Right_Flag==0)circle_Flag=0;   //识别有误，重新判断

        
  }
/*斜电感左4，右3,中间电感左2右5,外电感左0右1*/

 
   turn_Error=dis_AD[4]-dis_AD[3];    //利用过零特性检测切点
  //切点寻找
if(((turn_Error>=0&&turn_lastError<=0)||(turn_Error<=0&&turn_lastError>=0))&&circle_Flag==1&&Turn_Flag==0/* && dis_AD[4]>1200 &&dis_AD[3]>1200*/)   //利用过零寻找切点,原始判断方法
{
   Turn_Flag=1;    //找到切点，开始转向     
}
if(AD_Error_3>=AD_Error_3_Max&&Turn_Flag==0&&circle_Flag==1) AD_Error_3_Max = AD_Error_3;  //寻找最大值
if(circle_Flag==1&&Turn_Flag==0&&AD_Error_3<=((float)SaveLife)/100*AD_Error_3_Max) //保命算法
{
  BaoMing_Flag=1;
  Turn_Flag=1;
}
if(circle_Flag==0||Turn_Flag==1) AD_Error_3_Max=0;

/*if(circle_Flag==1&&((zhuan1==3&&huandaonum==1)||(zhuan2==3&&huandaonum==2)||(zhuan3==3&&huandaonum==3)||(zhuan4==4&&huandaonum==4))) 
Turn_Flag=1;*/

   if(Turn_Flag==1&&Turn_Right_Flag==1&&Turn_Flag2==0)//找到切点以后开始引导
  {
     Angle_Sum_R+=(speed_left-speed_right)*0.01;
    if(abs(Angle_Sum_R)>StopLead)
    {    
          Turn_Flag2=1;   //转向后期，已经进入,不需要引导
           BUZZER_OFF;
           Angle_Sum_R=0;
    }

  }
  if(Turn_Flag==1&&Turn_Left_Flag==1&& Turn_Flag2==0)
  {
     Angle_Sum_R+=(speed_left-speed_right)*0.01;
     if(abs(Angle_Sum_R)>StopLead)
     {     
           Turn_Flag2=1;        //说明环岛引导完毕
           BUZZER_OFF;
           Angle_Sum_R=0;
     }
 
  }
  
//******************导向部分结束判断***************************/

if(Go_Out_Circle==0 && (AD_Error_3>=Chuzhi) && Turn_Flag2==1 && Turn_Flag==1)//判断出环
  {
     Go_Out_Circle=1;    //判断到出环岛，开始出环岛计时
     BaoMing_Flag=0;
     BUZZER_ON;      //检测到出环提示一下
  }

  if(Go_Out_Circle==2)   //彻底离开环岛区域
  {
     //Flag_2s=0;
     if(huandaonum==4) huandaonum = 0; 
     second_infer=0;
     round_radius=0;
     Go_Out_Circle=0;
     Turn_Flag2=0;
     Turn_Flag=0;
     BaoMing_Flag=0;
     circle_Flag=0;
     Turn_Right_Flag=0;
     Turn_Left_Flag=0;
     BUZZER_OFF;        //彻底出环
  }
  
/****************************环岛判断结束*******************************************************/

/*******转向偏差计算部分（分区算法）***********/
/*斜电感左4，右3,中间电感左2右5,外电感左1右0*/


/********END*********/

ADError2=(float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1));
//if(Turn_Flag==1) TurnChange_1 = 1;//说明在切换状态
/************环岛二次处理，差值引导部分**************/
if(Turn_Flag==1&&Turn_Flag2==0&&Go_Out_Circle==0)    //找到切点后进圆环切换电感引导，引导阶段
{//在找到切点之后，引导结束之前
        //进一步判断左右环，直接利用斜电感判断即可？
  if(!panduanyici) //判断一次判断左右环岛
  {
      /* if(dis_AD[4]>dis_AD[3])    //过零后左边环岛左边斜电感大
        {
           Turn_Left_Flag=1;
           Turn_Right_Flag=0;
        }
        if(dis_AD[4]<dis_AD[3])
        {
           Turn_Left_Flag=0;
           Turn_Right_Flag=1;
        }*/
    if(turn_Error>=0&&turn_lastError<0) 
      {
           Turn_Left_Flag=1;
           Turn_Right_Flag=0;
       }
    else if(turn_Error<0&&turn_lastError>=0)
      {
           Turn_Left_Flag=0;
           Turn_Right_Flag=1;
      }
  }
        //判断后进行切换电感进进环岛处理，并且用原始值计算，防止限幅去除了有效量
       // 斜电感左4，右3,中间电感左2右5,水平电感左1右0
#if !AI
        if(Turn_Right_Flag==1)
        {                    
          AD_Error = -(float)RoundAng;    //进环固定偏差引导
        }
        if(Turn_Left_Flag==1)
        {
          AD_Error = (float)RoundAng;
        }
#else
    if(huandaonum==1)//
    {
      zhuan = 1;  
      //if(zhuan1!=3)
      //{
        if(zhuan1==0) AD_Error = ((float)huandao1)*Speed_now/RuSpeed;  //左转大于0
        else if(zhuan1==1) AD_Error = -((float)huandao1)*Speed_now/RuSpeed; //右转小于0*/       
        if(BaoMing_Flag==1)
        {
          if(zhuan1==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao1)*Speed_now/RuSpeed;  //左转大于0
          else if(zhuan1==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao1)*Speed_now/RuSpeed; //右转小于0*/
        }
     // }
     // else if(zhuan1==3) AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
        
      /*if(Turn_Left_Flag) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[2]))/(dis_AD[2] + dis_AD[1]+1));;
    //  else if(Turn_Right_Flag) AD_Error = -(float)huandao1; //右转小于0*/     
     // if(zhuan2==2) huandaonum=0;
      
    } //第一次环岛
    
    if(huandaonum==2)//
    {
        zhuan = 2;
        //if(zhuan2!=3)
        //{
          if(zhuan2==0) AD_Error = ((float)huandao2)*Speed_now/RuSpeed;  //左转大于0
          else if(zhuan2==1) AD_Error = -((float)huandao2)*Speed_now/RuSpeed; //右转小于0   */           
          if(BaoMing_Flag==1)
          {
            if(zhuan2==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao2)*Speed_now/RuSpeed;  //左转大于0
            else if(zhuan2==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao2)*Speed_now/RuSpeed; //右转小于0*/
          }
        //}
        //else if(zhuan2==3) AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
       /* if(Turn_Left_Flag) AD_Error = huandao2;
        else if(Turn_Right_Flag) AD_Error = -(float)huandao2; //右转小于0*/

       // if(zhuan3==2) huandaonum=0;     
    } //第二次环岛
    
   // if(huandaonum==3&&zhuan3!=2)//
    if(huandaonum==3)
    {     
        zhuan = 3;       
        //if(zhuan3!=3)
        //{
          if(zhuan3==0) AD_Error = ((float)huandao3)*Speed_now/RuSpeed;  //左转大于0
          else if(zhuan3==1) AD_Error = -((float)huandao3)*Speed_now/RuSpeed; //右转小于0*/        
          if(BaoMing_Flag==1)
          {
            if(zhuan3==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao3)*Speed_now/RuSpeed;  //左转大于0
            else if(zhuan3==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao3)*Speed_now/RuSpeed; //右转小于0*/
          }
        //}
       // else if(zhuan3==3)  AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
      /*  if(Turn_Left_Flag) AD_Error = huandao3;
        else if(Turn_Right_Flag) AD_Error = -(float)huandao3; //右转小于0*/
       
       // if(zhuan4==2) huandaonum = 0;
    } //第三次环岛
    if(huandaonum==4&&zhuan4!=2)
    {
      zhuan = 4; 
      //if(zhuan4!=3)
      //{
        if(zhuan4==0) AD_Error = ((float)huandao4)*Speed_now/RuSpeed;  //左转大于0
        else if(zhuan4==1) AD_Error = -((float)huandao4)*Speed_now/RuSpeed; //右转小于0*/
        if(BaoMing_Flag==1)
          {
            if(zhuan4==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao4)*Speed_now/RuSpeed;  //左转大于0
            else if(zhuan4==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao4)*Speed_now/RuSpeed; //右转小于0*/
          }
     // }
     // else if(zhuan4==3)  AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
     /*  if(Turn_Left_Flag) AD_Error = huandao4;
       else if(Turn_Right_Flag) AD_Error = -(float)huandao4; //右转小于0*/
      huandaonum = 0;
    }//第四次环岛
   //-------------------------------------------------------------------------------------------------------//
#endif
       ProportionalGain=Round_P/100;          //引导进环使用另一套参数
       DerivativeGain=Round_D/1000;
 
       // AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
        //AD_Error=(AD_Error<=-1600?-1600:AD_Error);
          
        panduanyici = 1;
        if(AD_Value[4]<100&&AD_Value[3]<100) //丢线处理
        {
          AD_Error = AD_Last_Error;
        }     
       // AD_Error_In_Last = AD_Error_In;
}
else  ////////////////非引导阶段，分为正常行驶、检测入环未找到切点、引导结束未检测出环、出环未定时结束、完全出环（即正常行驶）
{   
       if(Go_Out_Circle==1) //出环，还未定时结束阶段
       {    
        /* if(!first_flag)
         {
          SpeedGiven = ChuSpeed;
         }*/
         if(ChuNum==0)  //不用给偏差的情况
         {
          // AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+0.1*(float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[1] + AD_Value[0]+1));
           AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
           //AD_Error = (float)(1600*((dis_AD[2] - dis_AD[5]))/(dis_AD[2] + dis_AD[5]+1));
           AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
           AD_Error=(AD_Error<=-1600?-1600:AD_Error);
         }//////////////////////////////////////////
         else if(ChuNum==1) //第一个环岛要给偏差
         {
           if(zhuan==1) //第一个环岛
           {
              if(ChuDir==0) AD_Error = -(float)ChuCha;//左转出环，给负偏差
              if(ChuDir==1) AD_Error =  ChuCha;//右转出环，给正偏差
           }
           else  //其他环岛
           {
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }//////////////////////////////////////////
         else if(ChuNum==2) //第二个环岛出环要给偏差
         {
          if(zhuan==2) //第二个环岛
           {
              if(ChuDir==0) AD_Error = -(float)ChuCha;//左转出环，给负偏差
              if(ChuDir==1) AD_Error =  ChuCha;//右转出环，给正偏差
           }
          else  //其他环岛
           {
           // AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
            AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }///////////////////////////
         else if(ChuNum==3) //第三个环岛出环要给偏差
         {
          if(zhuan==3) //第三个环岛
           {
             if(ChuDir==0) AD_Error = -(float)ChuCha;//左转出环，给负偏差
             if(ChuDir==1) AD_Error =  ChuCha;//右转出环，给正偏差
           }
            else  //其他环岛
           {
           // AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }///////////////////////////////////////////
          else if(ChuNum==4) //其他环岛
          {
            if(zhuan==4) //第四个环岛
            {
             if(ChuDir==0) AD_Error = -(float)ChuCha;//左转出环，给负偏差
             if(ChuDir==1) AD_Error =  ChuCha;//右转出环，给正偏差
            }
            else //其它环岛
           {
            //AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }//////////////////////////////
       
        //AD_Error = (float)(1600*(0.3*(float)(AD_Value[1] - AD_Value[0])+0.7*(float)(AD_Value[3]-AD_Value[4]))/(0.3*(float)(AD_Value[0] + AD_Value[1])+0.7*(float)(AD_Value[3]+AD_Value[4])+1));
         
       }
       else  //非出环阶段
       {
#if 0
     /*    if(circle_Flag==1&&Turn_Flag==0&&Turn_Flag2==0&&Go_Out_Circle==0)//判断到环岛但是还未找到切点，让车身往靠近环岛一侧偏移
         {
           if(huandaonum==1)//
          { 
              if(zhuan1==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1)); //左转大于0
              else if(zhuan1==1) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //右转小于0      
           // AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
           } //第一次环岛
    
           if(huandaonum==2)//
          {
              if(zhuan2==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1));  //左转大于0
              else if(zhuan2==1) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //右转小于0   
            //AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
          } //第二次环岛
    
          if(huandaonum==3)
          {                 
            if(zhuan3==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1));  //左转大于0
            else if(zhuan3==1)  AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //右转小于0
          } //第三次环岛
          if(huandaonum==4&&zhuan4!=2)
          {            
            if(zhuan4==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1)); //左转大于0
            else if(zhuan4==1)  AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //右转小于0
          }//第四次环岛
         }
         else //除了上面两个阶段的其它阶段
         {*/
#endif
         AD_Error = (float)(1600*(0.8*(float)(AD_Value[1] - AD_Value[0])-0.2*(float)(AD_Value[3]-AD_Value[4]))/(0.8*(float)(AD_Value[0] + AD_Value[1])+0.2*(float)(AD_Value[3]+AD_Value[4])+1));   //正常赛道采用限幅处理后的电感巡线
         AD_Error = (AD_Error>1600?1600:AD_Error);
         AD_Error = (AD_Error<-1600?-1600:AD_Error);
        // }
       
       }
          DerivativeGain=Turn2D/1000;
          AD_real_Error = AD_Error;
          
#if fuzzy          //模糊控制
          P_Speed=Speed_now/BeginSpeed;
         // P_Speed = 0;
          Delt_Error = (AD_Error-AD_Last_Error)/100;
         // ProportionalGain = (Turn2P/100)*(1+P_Speed)+Fuzzy(AD_Error/10,Error_Delta/10)/100;
          //DerivativeGain=(Turn2D/1000)*(1+P_Speed/10)+Fuzzy(AD_Error/125,Error_Delta/30)/1000;
          ProportionalGain = (Turn2P/100)*(Fuzzy(AD_Error/125,Error_Delta/30)+1+P_Speed);
          ProportionalGain = 0.7*ProportionalGain + 0.3*ProportionalGain_last;
          ProportionalGain_last = ProportionalGain;
        //  ProportionalGain = ((float)(FILTRATE_Kalman((int16)ProportionalGain)*100))/100;
#else
          ProportionalGain=Turn2P/100;          //不在环岛则使用正常参数
#endif  
      if(AD_Value[0]<100&&AD_Value[1]<100) //丢线处理
       {
         AD_Error = AD_Last_Error;
       }
      AD_Error = AD_Error-(BT/100)*AD_Second_Derivative;
        AD_Error=(AD_Error>=1600?1600:AD_Error);    //偏差限幅
        AD_Error=(AD_Error<=-1600?-1600:AD_Error);
          


}
/*******************全部控速控制*********************************************/
if(jiansu_flag==1&&first_flag==0)//开圆环减速模式时,不在定时减速时间内
{
   if(circle_Flag==1&&Turn_Flag==0&&Turn_Flag2==0&&Go_Out_Circle==0) SpeedGiven=RuSpeed; //进环了但是没有找到切点，此时减速为进环速度
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==0&&Go_Out_Circle==0) SpeedGiven=SpeedGivenTemp;//找到切点以后，引导没有结束
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==1&&Go_Out_Circle==0) SpeedGiven=SpeedGivenTemp;//进环了引导结束但是还未出环恢复原速度
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==1&&Go_Out_Circle==1) SpeedGiven=ChuSpeed; //检测到了出环但是没有完全出环
   else {SpeedGiven=SpeedGivenTemp;BUZZER_OFF;} //正常赛道行驶时速度
}
else if(jiansu_flag==1&&first_flag==1&&podao_dianbo_flag==0)  SpeedGiven=SpeedCut;//开了圆环减速并且在减速时间内
else if(jiansu_flag==0&&first_flag==1&&podao_dianbo_flag==0)  SpeedGiven=SpeedCut;//没开圆环减速并且在减速时间内
else if(jiansu_flag==0&&first_flag==0&&podao_dianbo_flag==0&&circle_Flag==0)  {SpeedGiven=SpeedGivenTemp;BUZZER_OFF;} //没开圆环减速并且不在减速时间内，速度始终为给定
else if(jiansu_flag==0&&first_flag==0&&podao_dianbo_flag==0&&circle_Flag==1)   SpeedGiven=SpeedGivenTemp; //没开圆环减速并且不在减速时间内，速度始终为给定
if(podao_dianbo_flag==1&&circle_Flag==0)     {SpeedGiven=SpeedCut; BUZZER_ON;}

/****************END******************/

  
  
    
  
  
  //AD_Error=AD_Error*abs(AD_Error);

//***************最小二乘法***********************//
  
#if  Slope
          Push_And_Pull(DirectionErr,8,AD_real_Error); //数组更新
          
          if(Calculate_Length<8)
          {
              Calculate_Length++;
          }
          else
          {
            Error_Delta = -10*Slope_Calculate(0,Calculate_Length,DirectionErr);//求得斜率
          }
          //最小斜率限幅
          Error_Delta=(Error_Delta>500?500:Error_Delta);
          Error_Delta=(Error_Delta<-500?-500:Error_Delta);
#endif

  //差值更新系列
   AD_Last_Error = AD_Error;
   turn_lastError=turn_Error;
   Turn_Flag_Last = Turn_Flag; //更新圆环转向标志位
 /********************偏差更新数组***********************************/
   Last_AD_Error[2] =  Last_AD_Error[1];  //last_last
   Last_AD_Error[1] =  Last_AD_Error[0];  //last
   Last_AD_Error[0] = AD_Error; //储存偏差数组 now
   AD_Second_Derivative = (Last_AD_Error[0]-Last_AD_Error[1])-(Last_AD_Error[1]-Last_AD_Error[2]);
   
  //AD_Error = (float)FILTRATE_Kalman((int16)(AD_Error)); //ADERROR卡尔曼滤波
    /*AD_Error=(AD_Error-AD_Last_Error>100?AD_Last_Error+100:AD_Error);
    AD_Error=(AD_Error-AD_Last_Error<-100?AD_Last_Error-100:AD_Error); //偏差变化限制*/
      
}

/*-------------------------------------AD函数-----------------------------*/
void AD_Collect()   //电感值读取
{        
   //空间换时间
        AD_Value_temp[0][0]  = adc_once(AMP1, ADC_12bit); //读取相应结果寄存器值清相应转换完成通道标志ATD0STAT1
        AD_Value_temp[1][0]  = adc_once(AMP2, ADC_12bit);             
        AD_Value_temp[2][0]  = adc_once(AMP3, ADC_12bit);                        
        AD_Value_temp[3][0]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][0]  = adc_once(AMP5, ADC_12bit); // 
        AD_Value_temp[5][0]  = adc_once(AMP6, ADC_12bit);
      
        AD_Value_temp[0][1]  = adc_once(AMP1, ADC_12bit); //读取相应结果寄存器值清相应转换完成通道标志ATD0STAT1
        AD_Value_temp[1][1]  = adc_once(AMP2, ADC_12bit);     
        AD_Value_temp[2][1]  = adc_once(AMP3, ADC_12bit);                        
        AD_Value_temp[3][1]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][1]  = adc_once(AMP5, ADC_12bit); 
        AD_Value_temp[5][1]  = adc_once(AMP6, ADC_12bit);
         
        AD_Value_temp[0][2]  = adc_once(AMP1, ADC_12bit); //读取相应结果寄存器值清相应转换完成通道标志ATD0STAT1
        AD_Value_temp[1][2]  = adc_once(AMP2, ADC_12bit);             
        AD_Value_temp[2][2]  = adc_once(AMP3, ADC_12bit);                       
        AD_Value_temp[3][2]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][2]  = adc_once(AMP5, ADC_12bit);
        AD_Value_temp[5][2]  = adc_once(AMP6, ADC_12bit);        
}


void AD_Filter(void)  //电感采集+均值滤波
{
  uint8 i;
  uint16 sum = 0;
  AD_Collect();
  for(i=0;i<6;i++)
  {
    
      sum += AD_Value_temp[i][0];
      sum += AD_Value_temp[i][1];
      sum += AD_Value_temp[i][2];
    AD_Value[i] =sum/3;
    AD_Value[i] =AD_Value[i]/10*10;    //去除个位最多0.01V，减少震荡
   /*if(i==3||i==4)
   { AD_Value[i]=(uint16)(1300*((float)AD_Value[i]/(float)AD_MAX[i]));}
  //   AD_Value[i]=sqrt(AD_Value[i]);
    sum = 0;
  }*/
    AD_Value[i]=(uint16)((1800*(float)(AD_Value[i]-AD_MIN[i]))/((float)(AD_MAX[i]-AD_MIN[i])));//归一化
    sum = 0;
   /********电感值限幅，去除异常值**********/
    dis_AD[i]=AD_Value[i];    //保留原始值
    AD_Value[i]=(AD_Value[i]>2000?2000:AD_Value[i]);   
  }
}

float Slope_Calculate(uint8 begin,uint8 end,float *p)    //最小二乘法拟合斜率
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

void Push_And_Pull(float *buff,int len,float newdata)  //更新数组
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}

int32  Turn_Out_Filter(int32 turn_out)    //转向控制输出滤波      
{
  int32 Turn_Out_Filtered; 
  static int32 Pre1_Error[10]; 
  Pre1_Error[9]=Pre1_Error[8];
  Pre1_Error[8]=Pre1_Error[7];
  Pre1_Error[7]=Pre1_Error[6];
  Pre1_Error[6]=Pre1_Error[5];
  Pre1_Error[5]=Pre1_Error[4];
  Pre1_Error[4]=Pre1_Error[3];
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]+Pre1_Error[1]+Pre1_Error[2]+Pre1_Error[3]+Pre1_Error[4]+Pre1_Error[5]+Pre1_Error[6]+Pre1_Error[7]+Pre1_Error[8]+Pre1_Error[9];
  Turn_Out_Filtered=Turn_Out_Filtered/10;
  return Turn_Out_Filtered;
}

uint8 MaxPosition;    //最大水平电感编号
float Middle_Err,Last_Middle;   //分区算出的电感偏差
void DividePositon(void)
{
  uint16 ADMAX=0;
  
  //找到最大值电感的序号
  if(dis_AD[0]>ADMAX)
  {
      MaxPosition=0; ADMAX=dis_AD[0];
  }
  
  if(dis_AD[1]>ADMAX)
  {
      MaxPosition=1; ADMAX=dis_AD[1];
  }
  
  if(dis_AD[2]>ADMAX)
  {
      MaxPosition=2; ADMAX=dis_AD[2];
  }
  if(dis_AD[5]>ADMAX)
  {
      MaxPosition=5; ADMAX=dis_AD[5];
  }
  
  if(MaxPosition==2)     //最大电感为中间靠左
  {
  
     float ad_allow = AD_Value[2]*0.1;     
     int16 AD_ZY = AD_Value[1]-AD_Value[5];
     int16 AD_MZ = AD_Value[2]-AD_Value[1];
     int16 AD_MY = AD_Value[2]-AD_Value[5];
     
            if(AD_ZY>0)    //车靠右偏移
          {
            if(AD_MZ>ad_allow)
            {              
              Middle_Err = -(-30 - 30 +30.0 * AD_MZ/AD_MY);
            }
            else 
            {
              
              Middle_Err = -(-30-30);
            }
          }
          else
          {
            if(AD_MY>ad_allow)
            {
              Middle_Err = -(-30+ 30 -30.0*AD_MY/AD_MZ);
            }
            else
            {
              Middle_Err = -(-30+30);
            }
          }
  
  }    
  
  else if(MaxPosition==5)
  {
     
     float ad_allow = AD_Value[5]*0.1;  
     int16 AD_ZY = AD_Value[2]-AD_Value[0];
     int16 AD_MZ = AD_Value[5]-AD_Value[2];
     int16 AD_MY = AD_Value[5]-AD_Value[0];
     
     
      if(AD_ZY>0)//youpian
          {
            if(AD_MZ>ad_allow)
            {
              Middle_Err = -(30 - 30 +30.0 * AD_MZ/AD_MY);
            }
            else 
            {
              Middle_Err = -(30-30);
            }
          }
          else
          {
            if(AD_MY>ad_allow)
            {
              Middle_Err = -(30+ 30 -30.0*AD_MY/AD_MZ);
            }
            else
            {
         
              Middle_Err = -(30+30);
            }
          }
       }
   else if(MaxPosition == 1)
        {
          Middle_Err = 60 +(60.0 *(AD_Value[1]-AD_Value[2])/(AD_Value[1]+AD_Value[2]));
        }
        else if(MaxPosition == 0)
        {
                Middle_Err = -60 +(60.0 *(AD_Value[5]-AD_Value[0])/(AD_Value[5]+AD_Value[0]));
        }
  
  /*
  Middle_Err=(Middle_Err-Last_Middle>10?Last_Middle+10:Middle_Err);
  Middle_Err=(Middle_Err-Last_Middle<-10?Last_Middle-10:Middle_Err);
*/  
  
  Middle_Err=13*Middle_Err;
 Last_Middle=Middle_Err;
  




}