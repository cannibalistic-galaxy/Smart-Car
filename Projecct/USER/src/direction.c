/**************************************************
 * ������ŵ�д����ѭ������
 *С��0��ת������0��ת
 * 0�ң�3��,б�����4����3,�м�����2��5,������1��0
 *************************************************/
#include "headfile.h"
   
#define OFF      0
#define ON       1
#define THREEMODE 2
#define abs(x)   (x>0?x:-x)  
#define AI       1
  
#define Slope   1  //�Ƿ�������С����
#define fuzzy   1  //�Ƿ�ʹ��fuzzy   

   
char flag_round=OFF;     //������־λ
char flag_round_out = OFF; //������־
/***��������***/
char circle_Flag,Turn_Flag,BaoMing_Flag,Turn_Flag2,Turn_Right_Flag,Turn_Left_Flag,Go_Out_Circle;
char Turn_Left_Flag_1,Turn_Left_Flag_2,Turn_Left_Flag_3,Turn_Right_Flag_1,Turn_Right_Flag_2,Turn_Right_Flag_3;
//char Force_In;  //ǿ�ƽ�����־λ
/****
 *7��16�ո��¶���
 ***/
char Turn_Flag_Last; //�ϴ�ת���жϱ�־λ
char TurnChange_1; //ת��״̬�л�
float AD_Error_Lead; //Բ������ʱʹ�õ�ƫ��
/***/
int16 turn_Error,turn_lastError,times;   //�е��жϲ�ֵ
uint16 second_infer;  //�ڶ��μ��
char panduanyici;
char huandaonum;
char jiansu_flag;
char podao_dianbo_flag;
float round_radius;      //�����������ۼӰ뾶
float P_Speed;
float ADError2,ADSub43;
float DirectionErr[9]={0},Error_Delta_Deceleration;
float AD_real_Error;
float AD_Error_2,AD_Error_3;//ˮƽ��в�ֵ
float ADError_3[3],AD_Error_3_Max;
float SpeedGivenTemp; //�ٶ��ݴ�
float AD_Error_Avg,AD_Error_In,AD_Error_In_Last; //�뻷ƫ��
float Last_AD_Error[3];  //ƫ�������
float AD_Second_Derivative;  //ƫ����׵���
float BeginSpeed = 1600;
uint16 ADSum34;
int16 ADSub34;
uint16 AD_Middle_Max; //�м������ֵ��������ĸ��
uint16 AD_More; //�������ֵ��ӵ���ĸ��
float AD_More_Para;//�˵�ϵ��
float ProportionalGain_last;
float Angle_Sum_R;
extern float dingshijiansu,first_flag;

void AD_Calculate(void)   //���ֵ���㴦���õ���ֵ���в��ٴ���
{ 
  AD_Error_3 = ((dis_AD[4]/100)*(dis_AD[3]/100)*(dis_AD[2]/100)*(dis_AD[5]/100))/(4096-dis_AD[4]+4096-dis_AD[3]+4096-dis_AD[2]+4096-dis_AD[5]); //�����жϻ����㷨
  //************************�����жϣ��ж�ȫ��ʹ��ԭʼֵ����*************************************/
  if(circle_Flag==0&&second_infer==0&&AD_Error_3>=JinZhi) //�жϻ���,��ʼ����
  {  
    huandaonum++; //�жϽ���������
    switch(roundmode) //���λ���
    {
    case 0:circle_Flag=0;huandaonum=0;break; //�������л���
    case 1:                                   //��һ������
      if(huandaonum==1) {circle_Flag=1;BUZZER_ON;}   
      else {circle_Flag=0;second_infer=1;}
      break;
    case 2:                                   //һ������
      if(huandaonum==1||huandaonum==2) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 3:                                  //һ��������
      if(huandaonum==1||huandaonum==2||huandaonum==3) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 4:                                  //ȫ������
      {circle_Flag=1;BUZZER_ON;}
      break;
    case 5:                                  //ֻ�ڶ�������
      if(huandaonum==2) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 6:                                  //�ڶ���������
      if(huandaonum==2||huandaonum==3) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 7:                                   //�����ĸ�����
      if(huandaonum==2||huandaonum==3||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 8:                                  //ֻ����������
      if(huandaonum==3) {circle_Flag=1;BUZZER_ON;}   
      else {circle_Flag=0;second_infer=1;}
      break;
    case 9:                                   //���ĸ�����
      if(huandaonum==3||huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 10:                                  //ֻ���ĸ�����
      if(huandaonum==4) {circle_Flag=1;BUZZER_ON;}
      else {circle_Flag=0;second_infer=1;}
      break;
    case 11:                                  //��һ����������
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
        
       
        //�ҵ��е�ǰ�ж�һ�����ң�
        //������
        if(dis_AD[4]<dis_AD[3])Turn_Left_Flag=1;
        //�Ҳ��뻷��
        if(dis_AD[4]>dis_AD[3])Turn_Right_Flag=1;       
        if(Turn_Left_Flag==0&&Turn_Right_Flag==0)circle_Flag=0;   //ʶ�����������ж�

        
  }
/*б�����4����3,�м�����2��5,������0��1*/

 
   turn_Error=dis_AD[4]-dis_AD[3];    //���ù������Լ���е�
  //�е�Ѱ��
if(((turn_Error>=0&&turn_lastError<=0)||(turn_Error<=0&&turn_lastError>=0))&&circle_Flag==1&&Turn_Flag==0/* && dis_AD[4]>1200 &&dis_AD[3]>1200*/)   //���ù���Ѱ���е�,ԭʼ�жϷ���
{
   Turn_Flag=1;    //�ҵ��е㣬��ʼת��     
}
if(AD_Error_3>=AD_Error_3_Max&&Turn_Flag==0&&circle_Flag==1) AD_Error_3_Max = AD_Error_3;  //Ѱ�����ֵ
if(circle_Flag==1&&Turn_Flag==0&&AD_Error_3<=((float)SaveLife)/100*AD_Error_3_Max) //�����㷨
{
  BaoMing_Flag=1;
  Turn_Flag=1;
}
if(circle_Flag==0||Turn_Flag==1) AD_Error_3_Max=0;

/*if(circle_Flag==1&&((zhuan1==3&&huandaonum==1)||(zhuan2==3&&huandaonum==2)||(zhuan3==3&&huandaonum==3)||(zhuan4==4&&huandaonum==4))) 
Turn_Flag=1;*/

   if(Turn_Flag==1&&Turn_Right_Flag==1&&Turn_Flag2==0)//�ҵ��е��Ժ�ʼ����
  {
     Angle_Sum_R+=(speed_left-speed_right)*0.01;
    if(abs(Angle_Sum_R)>StopLead)
    {    
          Turn_Flag2=1;   //ת����ڣ��Ѿ�����,����Ҫ����
           BUZZER_OFF;
           Angle_Sum_R=0;
    }

  }
  if(Turn_Flag==1&&Turn_Left_Flag==1&& Turn_Flag2==0)
  {
     Angle_Sum_R+=(speed_left-speed_right)*0.01;
     if(abs(Angle_Sum_R)>StopLead)
     {     
           Turn_Flag2=1;        //˵�������������
           BUZZER_OFF;
           Angle_Sum_R=0;
     }
 
  }
  
//******************���򲿷ֽ����ж�***************************/

if(Go_Out_Circle==0 && (AD_Error_3>=Chuzhi) && Turn_Flag2==1 && Turn_Flag==1)//�жϳ���
  {
     Go_Out_Circle=1;    //�жϵ�����������ʼ��������ʱ
     BaoMing_Flag=0;
     BUZZER_ON;      //��⵽������ʾһ��
  }

  if(Go_Out_Circle==2)   //�����뿪��������
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
     BUZZER_OFF;        //���׳���
  }
  
/****************************�����жϽ���*******************************************************/

/*******ת��ƫ����㲿�֣������㷨��***********/
/*б�����4����3,�м�����2��5,������1��0*/


/********END*********/

ADError2=(float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1));
//if(Turn_Flag==1) TurnChange_1 = 1;//˵�����л�״̬
/************�������δ�����ֵ��������**************/
if(Turn_Flag==1&&Turn_Flag2==0&&Go_Out_Circle==0)    //�ҵ��е���Բ���л���������������׶�
{//���ҵ��е�֮����������֮ǰ
        //��һ���ж����һ���ֱ������б����жϼ��ɣ�
  if(!panduanyici) //�ж�һ���ж����һ���
  {
      /* if(dis_AD[4]>dis_AD[3])    //�������߻������б��д�
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
        //�жϺ�����л���н�����������������ԭʼֵ���㣬��ֹ�޷�ȥ������Ч��
       // б�����4����3,�м�����2��5,ˮƽ�����1��0
#if !AI
        if(Turn_Right_Flag==1)
        {                    
          AD_Error = -(float)RoundAng;    //�����̶�ƫ������
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
        if(zhuan1==0) AD_Error = ((float)huandao1)*Speed_now/RuSpeed;  //��ת����0
        else if(zhuan1==1) AD_Error = -((float)huandao1)*Speed_now/RuSpeed; //��תС��0*/       
        if(BaoMing_Flag==1)
        {
          if(zhuan1==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao1)*Speed_now/RuSpeed;  //��ת����0
          else if(zhuan1==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao1)*Speed_now/RuSpeed; //��תС��0*/
        }
     // }
     // else if(zhuan1==3) AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
        
      /*if(Turn_Left_Flag) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[2]))/(dis_AD[2] + dis_AD[1]+1));;
    //  else if(Turn_Right_Flag) AD_Error = -(float)huandao1; //��תС��0*/     
     // if(zhuan2==2) huandaonum=0;
      
    } //��һ�λ���
    
    if(huandaonum==2)//
    {
        zhuan = 2;
        //if(zhuan2!=3)
        //{
          if(zhuan2==0) AD_Error = ((float)huandao2)*Speed_now/RuSpeed;  //��ת����0
          else if(zhuan2==1) AD_Error = -((float)huandao2)*Speed_now/RuSpeed; //��תС��0   */           
          if(BaoMing_Flag==1)
          {
            if(zhuan2==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao2)*Speed_now/RuSpeed;  //��ת����0
            else if(zhuan2==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao2)*Speed_now/RuSpeed; //��תС��0*/
          }
        //}
        //else if(zhuan2==3) AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
       /* if(Turn_Left_Flag) AD_Error = huandao2;
        else if(Turn_Right_Flag) AD_Error = -(float)huandao2; //��תС��0*/

       // if(zhuan3==2) huandaonum=0;     
    } //�ڶ��λ���
    
   // if(huandaonum==3&&zhuan3!=2)//
    if(huandaonum==3)
    {     
        zhuan = 3;       
        //if(zhuan3!=3)
        //{
          if(zhuan3==0) AD_Error = ((float)huandao3)*Speed_now/RuSpeed;  //��ת����0
          else if(zhuan3==1) AD_Error = -((float)huandao3)*Speed_now/RuSpeed; //��תС��0*/        
          if(BaoMing_Flag==1)
          {
            if(zhuan3==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao3)*Speed_now/RuSpeed;  //��ת����0
            else if(zhuan3==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao3)*Speed_now/RuSpeed; //��תС��0*/
          }
        //}
       // else if(zhuan3==3)  AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
      /*  if(Turn_Left_Flag) AD_Error = huandao3;
        else if(Turn_Right_Flag) AD_Error = -(float)huandao3; //��תС��0*/
       
       // if(zhuan4==2) huandaonum = 0;
    } //�����λ���
    if(huandaonum==4&&zhuan4!=2)
    {
      zhuan = 4; 
      //if(zhuan4!=3)
      //{
        if(zhuan4==0) AD_Error = ((float)huandao4)*Speed_now/RuSpeed;  //��ת����0
        else if(zhuan4==1) AD_Error = -((float)huandao4)*Speed_now/RuSpeed; //��תС��0*/
        if(BaoMing_Flag==1)
          {
            if(zhuan4==0) AD_Error = ((float)SaveLifePar)/10*((float)huandao4)*Speed_now/RuSpeed;  //��ת����0
            else if(zhuan4==1) AD_Error = -((float)SaveLifePar)/10*((float)huandao4)*Speed_now/RuSpeed; //��תС��0*/
          }
     // }
     // else if(zhuan4==3)  AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
     /*  if(Turn_Left_Flag) AD_Error = huandao4;
       else if(Turn_Right_Flag) AD_Error = -(float)huandao4; //��תС��0*/
      huandaonum = 0;
    }//���Ĵλ���
   //-------------------------------------------------------------------------------------------------------//
#endif
       ProportionalGain=Round_P/100;          //��������ʹ����һ�ײ���
       DerivativeGain=Round_D/1000;
 
       // AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
        //AD_Error=(AD_Error<=-1600?-1600:AD_Error);
          
        panduanyici = 1;
        if(AD_Value[4]<100&&AD_Value[3]<100) //���ߴ���
        {
          AD_Error = AD_Last_Error;
        }     
       // AD_Error_In_Last = AD_Error_In;
}
else  ////////////////�������׶Σ���Ϊ������ʻ������뻷δ�ҵ��е㡢��������δ������������δ��ʱ��������ȫ��������������ʻ��
{   
       if(Go_Out_Circle==1) //��������δ��ʱ�����׶�
       {    
        /* if(!first_flag)
         {
          SpeedGiven = ChuSpeed;
         }*/
         if(ChuNum==0)  //���ø�ƫ������
         {
          // AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+0.1*(float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[1] + AD_Value[0]+1));
           AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
           //AD_Error = (float)(1600*((dis_AD[2] - dis_AD[5]))/(dis_AD[2] + dis_AD[5]+1));
           AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
           AD_Error=(AD_Error<=-1600?-1600:AD_Error);
         }//////////////////////////////////////////
         else if(ChuNum==1) //��һ������Ҫ��ƫ��
         {
           if(zhuan==1) //��һ������
           {
              if(ChuDir==0) AD_Error = -(float)ChuCha;//��ת����������ƫ��
              if(ChuDir==1) AD_Error =  ChuCha;//��ת����������ƫ��
           }
           else  //��������
           {
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }//////////////////////////////////////////
         else if(ChuNum==2) //�ڶ�����������Ҫ��ƫ��
         {
          if(zhuan==2) //�ڶ�������
           {
              if(ChuDir==0) AD_Error = -(float)ChuCha;//��ת����������ƫ��
              if(ChuDir==1) AD_Error =  ChuCha;//��ת����������ƫ��
           }
          else  //��������
           {
           // AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
            AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }///////////////////////////
         else if(ChuNum==3) //��������������Ҫ��ƫ��
         {
          if(zhuan==3) //����������
           {
             if(ChuDir==0) AD_Error = -(float)ChuCha;//��ת����������ƫ��
             if(ChuDir==1) AD_Error =  ChuCha;//��ת����������ƫ��
           }
            else  //��������
           {
           // AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }///////////////////////////////////////////
          else if(ChuNum==4) //��������
          {
            if(zhuan==4) //���ĸ�����
            {
             if(ChuDir==0) AD_Error = -(float)ChuCha;//��ת����������ƫ��
             if(ChuDir==1) AD_Error =  ChuCha;//��ת����������ƫ��
            }
            else //��������
           {
            //AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1));
             AD_Error = (float)(1600*((AD_Value[1] - AD_Value[0]))/(AD_Value[0] + AD_Value[1]+1))+0.6*(float)(1600*((AD_Value[3] - AD_Value[4]))/(AD_Value[3] + AD_Value[4]+1));
            AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
            AD_Error=(AD_Error<=-1600?-1600:AD_Error);
           }
         }//////////////////////////////
       
        //AD_Error = (float)(1600*(0.3*(float)(AD_Value[1] - AD_Value[0])+0.7*(float)(AD_Value[3]-AD_Value[4]))/(0.3*(float)(AD_Value[0] + AD_Value[1])+0.7*(float)(AD_Value[3]+AD_Value[4])+1));
         
       }
       else  //�ǳ����׶�
       {
#if 0
     /*    if(circle_Flag==1&&Turn_Flag==0&&Turn_Flag2==0&&Go_Out_Circle==0)//�жϵ��������ǻ�δ�ҵ��е㣬�ó�������������һ��ƫ��
         {
           if(huandaonum==1)//
          { 
              if(zhuan1==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1)); //��ת����0
              else if(zhuan1==1) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //��תС��0      
           // AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
           } //��һ�λ���
    
           if(huandaonum==2)//
          {
              if(zhuan2==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1));  //��ת����0
              else if(zhuan2==1) AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //��תС��0   
            //AD_Error = (float)(1600*((AD_Value[2] - AD_Value[5]))/(AD_Value[2] + AD_Value[5]+1))+1.6*(float)(1600*((AD_Value[4] - AD_Value[3]))/(AD_Value[4] + AD_Value[3]+1));
          } //�ڶ��λ���
    
          if(huandaonum==3)
          {                 
            if(zhuan3==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1));  //��ת����0
            else if(zhuan3==1)  AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //��תС��0
          } //�����λ���
          if(huandaonum==4&&zhuan4!=2)
          {            
            if(zhuan4==0) AD_Error = (float)(1600*((dis_AD[2] - dis_AD[0]))/(dis_AD[2] + dis_AD[0]+1)); //��ת����0
            else if(zhuan4==1)  AD_Error = (float)(1600*((dis_AD[1] - dis_AD[5]))/(dis_AD[1] + dis_AD[5]+1)); //��תС��0
          }//���Ĵλ���
         }
         else //�������������׶ε������׶�
         {*/
#endif
         AD_Error = (float)(1600*(0.8*(float)(AD_Value[1] - AD_Value[0])-0.2*(float)(AD_Value[3]-AD_Value[4]))/(0.8*(float)(AD_Value[0] + AD_Value[1])+0.2*(float)(AD_Value[3]+AD_Value[4])+1));   //�������������޷������ĵ��Ѳ��
         AD_Error = (AD_Error>1600?1600:AD_Error);
         AD_Error = (AD_Error<-1600?-1600:AD_Error);
        // }
       
       }
          DerivativeGain=Turn2D/1000;
          AD_real_Error = AD_Error;
          
#if fuzzy          //ģ������
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
          ProportionalGain=Turn2P/100;          //���ڻ�����ʹ����������
#endif  
      if(AD_Value[0]<100&&AD_Value[1]<100) //���ߴ���
       {
         AD_Error = AD_Last_Error;
       }
      AD_Error = AD_Error-(BT/100)*AD_Second_Derivative;
        AD_Error=(AD_Error>=1600?1600:AD_Error);    //ƫ���޷�
        AD_Error=(AD_Error<=-1600?-1600:AD_Error);
          


}
/*******************ȫ�����ٿ���*********************************************/
if(jiansu_flag==1&&first_flag==0)//��Բ������ģʽʱ,���ڶ�ʱ����ʱ����
{
   if(circle_Flag==1&&Turn_Flag==0&&Turn_Flag2==0&&Go_Out_Circle==0) SpeedGiven=RuSpeed; //�����˵���û���ҵ��е㣬��ʱ����Ϊ�����ٶ�
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==0&&Go_Out_Circle==0) SpeedGiven=SpeedGivenTemp;//�ҵ��е��Ժ�����û�н���
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==1&&Go_Out_Circle==0) SpeedGiven=SpeedGivenTemp;//�����������������ǻ�δ�����ָ�ԭ�ٶ�
   else if(circle_Flag==1&&Turn_Flag==1&&Turn_Flag2==1&&Go_Out_Circle==1) SpeedGiven=ChuSpeed; //��⵽�˳�������û����ȫ����
   else {SpeedGiven=SpeedGivenTemp;BUZZER_OFF;} //����������ʻʱ�ٶ�
}
else if(jiansu_flag==1&&first_flag==1&&podao_dianbo_flag==0)  SpeedGiven=SpeedCut;//����Բ�����ٲ����ڼ���ʱ����
else if(jiansu_flag==0&&first_flag==1&&podao_dianbo_flag==0)  SpeedGiven=SpeedCut;//û��Բ�����ٲ����ڼ���ʱ����
else if(jiansu_flag==0&&first_flag==0&&podao_dianbo_flag==0&&circle_Flag==0)  {SpeedGiven=SpeedGivenTemp;BUZZER_OFF;} //û��Բ�����ٲ��Ҳ��ڼ���ʱ���ڣ��ٶ�ʼ��Ϊ����
else if(jiansu_flag==0&&first_flag==0&&podao_dianbo_flag==0&&circle_Flag==1)   SpeedGiven=SpeedGivenTemp; //û��Բ�����ٲ��Ҳ��ڼ���ʱ���ڣ��ٶ�ʼ��Ϊ����
if(podao_dianbo_flag==1&&circle_Flag==0)     {SpeedGiven=SpeedCut; BUZZER_ON;}

/****************END******************/

  
  
    
  
  
  //AD_Error=AD_Error*abs(AD_Error);

//***************��С���˷�***********************//
  
#if  Slope
          Push_And_Pull(DirectionErr,8,AD_real_Error); //�������
          
          if(Calculate_Length<8)
          {
              Calculate_Length++;
          }
          else
          {
            Error_Delta = -10*Slope_Calculate(0,Calculate_Length,DirectionErr);//���б��
          }
          //��Сб���޷�
          Error_Delta=(Error_Delta>500?500:Error_Delta);
          Error_Delta=(Error_Delta<-500?-500:Error_Delta);
#endif

  //��ֵ����ϵ��
   AD_Last_Error = AD_Error;
   turn_lastError=turn_Error;
   Turn_Flag_Last = Turn_Flag; //����Բ��ת���־λ
 /********************ƫ���������***********************************/
   Last_AD_Error[2] =  Last_AD_Error[1];  //last_last
   Last_AD_Error[1] =  Last_AD_Error[0];  //last
   Last_AD_Error[0] = AD_Error; //����ƫ������ now
   AD_Second_Derivative = (Last_AD_Error[0]-Last_AD_Error[1])-(Last_AD_Error[1]-Last_AD_Error[2]);
   
  //AD_Error = (float)FILTRATE_Kalman((int16)(AD_Error)); //ADERROR�������˲�
    /*AD_Error=(AD_Error-AD_Last_Error>100?AD_Last_Error+100:AD_Error);
    AD_Error=(AD_Error-AD_Last_Error<-100?AD_Last_Error-100:AD_Error); //ƫ��仯����*/
      
}

/*-------------------------------------AD����-----------------------------*/
void AD_Collect()   //���ֵ��ȡ
{        
   //�ռ任ʱ��
        AD_Value_temp[0][0]  = adc_once(AMP1, ADC_12bit); //��ȡ��Ӧ����Ĵ���ֵ����Ӧת�����ͨ����־ATD0STAT1
        AD_Value_temp[1][0]  = adc_once(AMP2, ADC_12bit);             
        AD_Value_temp[2][0]  = adc_once(AMP3, ADC_12bit);                        
        AD_Value_temp[3][0]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][0]  = adc_once(AMP5, ADC_12bit); // 
        AD_Value_temp[5][0]  = adc_once(AMP6, ADC_12bit);
      
        AD_Value_temp[0][1]  = adc_once(AMP1, ADC_12bit); //��ȡ��Ӧ����Ĵ���ֵ����Ӧת�����ͨ����־ATD0STAT1
        AD_Value_temp[1][1]  = adc_once(AMP2, ADC_12bit);     
        AD_Value_temp[2][1]  = adc_once(AMP3, ADC_12bit);                        
        AD_Value_temp[3][1]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][1]  = adc_once(AMP5, ADC_12bit); 
        AD_Value_temp[5][1]  = adc_once(AMP6, ADC_12bit);
         
        AD_Value_temp[0][2]  = adc_once(AMP1, ADC_12bit); //��ȡ��Ӧ����Ĵ���ֵ����Ӧת�����ͨ����־ATD0STAT1
        AD_Value_temp[1][2]  = adc_once(AMP2, ADC_12bit);             
        AD_Value_temp[2][2]  = adc_once(AMP3, ADC_12bit);                       
        AD_Value_temp[3][2]  = adc_once(AMP4, ADC_12bit);          
        AD_Value_temp[4][2]  = adc_once(AMP5, ADC_12bit);
        AD_Value_temp[5][2]  = adc_once(AMP6, ADC_12bit);        
}


void AD_Filter(void)  //��вɼ�+��ֵ�˲�
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
    AD_Value[i] =AD_Value[i]/10*10;    //ȥ����λ���0.01V��������
   /*if(i==3||i==4)
   { AD_Value[i]=(uint16)(1300*((float)AD_Value[i]/(float)AD_MAX[i]));}
  //   AD_Value[i]=sqrt(AD_Value[i]);
    sum = 0;
  }*/
    AD_Value[i]=(uint16)((1800*(float)(AD_Value[i]-AD_MIN[i]))/((float)(AD_MAX[i]-AD_MIN[i])));//��һ��
    sum = 0;
   /********���ֵ�޷���ȥ���쳣ֵ**********/
    dis_AD[i]=AD_Value[i];    //����ԭʼֵ
    AD_Value[i]=(AD_Value[i]>2000?2000:AD_Value[i]);   
  }
}

float Slope_Calculate(uint8 begin,uint8 end,float *p)    //��С���˷����б��
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
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ�� 
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

void Push_And_Pull(float *buff,int len,float newdata)  //��������
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}

int32  Turn_Out_Filter(int32 turn_out)    //ת���������˲�      
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

uint8 MaxPosition;    //���ˮƽ��б��
float Middle_Err,Last_Middle;   //��������ĵ��ƫ��
void DividePositon(void)
{
  uint16 ADMAX=0;
  
  //�ҵ����ֵ��е����
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
  
  if(MaxPosition==2)     //�����Ϊ�м俿��
  {
  
     float ad_allow = AD_Value[2]*0.1;     
     int16 AD_ZY = AD_Value[1]-AD_Value[5];
     int16 AD_MZ = AD_Value[2]-AD_Value[1];
     int16 AD_MY = AD_Value[2]-AD_Value[5];
     
            if(AD_ZY>0)    //������ƫ��
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