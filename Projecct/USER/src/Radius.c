#include "headfile.h"
#include "Radius.h"


Carinfotypedef Carinfo;

//车体等效半径为0.1725（线性拟合）
float Get_Car_curvature(float Excurvature)      //期望曲率在此处与速度融合获得期望角速度
{
  float ExYawRate;            //通过速度与偏差控制得到真正的期望角速度
  float SpeedGainCur=1;            //速度增益
  Carinfo.SpeedMin=270;            //设定最小稳定速度为300（量化后）
  Carinfo.MaxK=1200;      //限制最大曲率
  int16 SpeedTrue;
  /**********路径优化部分************/
   
 /* float th = 500;           // 角速度大于该值时, 积分, 小于时, 积分减小，此处可改
  static float inte = 0;  // 角速度积分值
  float b = 50;           ////转向大45°时增加半径
  float rate = 1000;
  float PathMinRadius=0;*/
  /*
    if(Carinfo.yawrate > 0)     //左转
    {   // 右转
        inte += 0.01 * (Carinfo.yawrate - th);
        // 期望曲率用于清除积分
        if (Excurvature < 300)               //最大期望曲率某个值限幅
            inte -= 0.008 * rate;
        // 积分限
        if (inte > 210) inte = 210;      //累加限幅
        if (inte < 0) inte = 0;
    }
    else                            //右转
    {   // 左转
        inte -= 0.01 * (-Carinfo.yawrate - th);
        // 期望曲率用于清除积分
        if (Excurvature > -300)
            inte += 0.008 * rate;
        // 积分限幅
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
  Excurvature*=SpeedGainCur;     //对期望曲率进行增益
  
  //期望曲率限幅
/* 
  Excurvature=(Excurvature>PathMinRadius?PathMinRadius:Excurvature);
  Excurvature=(Excurvature<-PathMinRadius?-PathMinRadius:Excurvature);*/
  
  /* 
  Excurvature=(Excurvature>PathMinRadius?PathMinRadius:Excurvature);
  Excurvature=(Excurvature<-PathMinRadius?-PathMinRadius:Excurvature);*/
  

 //折算成角速度
  
  SpeedTrue=(SpeedTrue>450?450:SpeedTrue);
    SpeedTrue=(SpeedTrue<200?200:SpeedTrue);


  ExYawRate=SpeedTrue*Excurvature/300;          //计算出期望角速度，进行方向闭环
  

  
  
  
  
  return ExYawRate;
  

  /**********路径规划与限幅****************/
  
  
  
     
  
 // int16 devspeed = 0;
 // int16 sumspeed = 0;
  
  //devspeed = speed_left - ;   //左右轮真实速度差
  //sumspeed = speed_left + speed_right;   //左右轮真实速度和

  
 /* if(sumspeed == 0)CarInfo.curvature = 0;     //如果车速为0，曲率直接为0，相当于原地转圈的情况，曲率应该是无穷大，为了错误直接置零
  else CarInfo.curvature = devspeed / (sumspeed * CarInfo.BodyRadius);    //曲率计算*/

  
  


}