#include "headfile.h"
#include "Radius.h"


Carinfotypedef Carinfo;

//�����Ч�뾶Ϊ0.1725��������ϣ�
float Get_Car_curvature(float Excurvature)      //���������ڴ˴����ٶ��ںϻ���������ٶ�
{
  float ExYawRate;            //ͨ���ٶ���ƫ����Ƶõ��������������ٶ�
  float SpeedGainCur=1;            //�ٶ�����
  Carinfo.SpeedMin=270;            //�趨��С�ȶ��ٶ�Ϊ300��������
  Carinfo.MaxK=1200;      //�����������
  int16 SpeedTrue;
  /**********·���Ż�����************/
   
 /* float th = 500;           // ���ٶȴ��ڸ�ֵʱ, ����, С��ʱ, ���ּ�С���˴��ɸ�
  static float inte = 0;  // ���ٶȻ���ֵ
  float b = 50;           ////ת���45��ʱ���Ӱ뾶
  float rate = 1000;
  float PathMinRadius=0;*/
  /*
    if(Carinfo.yawrate > 0)     //��ת
    {   // ��ת
        inte += 0.01 * (Carinfo.yawrate - th);
        // �������������������
        if (Excurvature < 300)               //�����������ĳ��ֵ�޷�
            inte -= 0.008 * rate;
        // ������
        if (inte > 210) inte = 210;      //�ۼ��޷�
        if (inte < 0) inte = 0;
    }
    else                            //��ת
    {   // ��ת
        inte -= 0.01 * (-Carinfo.yawrate - th);
        // �������������������
        if (Excurvature > -300)
            inte += 0.008 * rate;
        // �����޷�
        if (inte < -210) inte = -210;
        if (inte > 0) inte = 0;
    }
  float inteABS = abs(inte);
  if(inteABS > b)
  {
    PathMinRadius = Carinfo.MaxK -(inteABS - b) * 0.05 ;
  }
  else
  {
    PathMinRadius = Carinfo.MaxK;
  }*/
   
  /***********************************/
 /* PathMinRadius=(PathMinRadius<900?900:PathMinRadius);*/
  
  
  
  SpeedTrue=speed_left+speed_right;
  
  if(SpeedTrue>Carinfo.SpeedMin){
  SpeedGainCur=0.01*(SpeedTrue-Carinfo.SpeedMin)+1;
  SpeedGainCur=(SpeedGainCur>1.7?1.7:SpeedGainCur);
  }
  Excurvature*=SpeedGainCur;     //���������ʽ�������
  
  //���������޷�
/* 
  Excurvature=(Excurvature>PathMinRadius?PathMinRadius:Excurvature);
  Excurvature=(Excurvature<-PathMinRadius?-PathMinRadius:Excurvature);*/
  
  /* 
  Excurvature=(Excurvature>PathMinRadius?PathMinRadius:Excurvature);
  Excurvature=(Excurvature<-PathMinRadius?-PathMinRadius:Excurvature);*/
  

 //����ɽ��ٶ�
  
  SpeedTrue=(SpeedTrue>450?450:SpeedTrue);
    SpeedTrue=(SpeedTrue<200?200:SpeedTrue);


  ExYawRate=SpeedTrue*Excurvature/300;          //������������ٶȣ����з���ջ�
  

  
  
  
  
  return ExYawRate;
  

  /**********·���滮���޷�****************/
  
  
  
     
  
 // int16 devspeed = 0;
 // int16 sumspeed = 0;
  
  //devspeed = speed_left - ;   //��������ʵ�ٶȲ�
  //sumspeed = speed_left + speed_right;   //��������ʵ�ٶȺ�

  
 /* if(sumspeed == 0)CarInfo.curvature = 0;     //�������Ϊ0������ֱ��Ϊ0���൱��ԭ��תȦ�����������Ӧ���������Ϊ�˴���ֱ������
  else CarInfo.curvature = devspeed / (sumspeed * CarInfo.BodyRadius);    //���ʼ���*/

  
  


}