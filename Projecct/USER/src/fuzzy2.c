#include "fuzzy2.h"

/***********************************************
函数功能：PID参数Kp的计算
************************************************/
float fuzzy_kp(float e, float ec) //e,ec，表示误差，误差变化率 
{
  float Kp_calcu;
  unsigned int num,pe,pec;
  const float   eRule[7]={-1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0};     //误差E的模糊论域 
  const float  ecRule[7]={-1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0}; //误差变化率EC的模糊论域
  float eFuzzy[2]={0.0,0.0};                              //隶属于误差E的隶属程度 
  float ecFuzzy[2]={0.0,0.0};                            //隶属于误差变化率EC的隶属程度 
  const float  kpRule[4]={0, 400, 800, 1200};          //Kp的模糊子集----------------
  float KpFuzzy[4]={0.0,0.0,0.0,0.0};                 //隶属于Kp的隶属程度              |
  const int  KpRule[7][7]=                          //Kp的模糊控制表 -------------------
  {                                              //e        e=0
    3,3,3,3,3,3,3,                    //▲ec  /*                               */                           
    3,3,3,3,3,3,3,                             /*                               */
    2,2,1,1,1,2,2,                          /*                               */
    3,2,1,0,1,2,3,                //▲ec=0 /*                               */
    2,2,1,1,1,2,2,                        /*                               */
    3,3,3,3,3,3,3,                           /*                               */
    3,3,3,3,3,3,3                       /*                               */
  };
/*****误差E隶属函数描述*****/
  if(e<eRule[0])
  {
  eFuzzy[0] =1.0;
  pe = 0;
  }
  else if(eRule[0]<=e && e<eRule[1])
  {
  eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
  pe = 0;
  }
  else if(eRule[1]<=e && e<eRule[2])
  {
  eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
  pe = 1;
  }
  else if(eRule[2]<=e && e<eRule[3])
  {
  eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]); pe = 2;
  }
  else if(eRule[3]<=e && e<eRule[4])///////////////////////////////
  { eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]); pe = 3;//
  }
  else if(eRule[4]<=e && e<eRule[5])//////////////////////////////
  {
  eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]); pe = 4;//
  }
  else if(eRule[5]<=e && e<eRule[6])
  {
  eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]); pe = 5;
  }
  else
  {
  eFuzzy[0] =0.0;
  pe =5;
  }
  eFuzzy[1] =1.0 - eFuzzy[0];
  /*****误差变化率EC隶属函数描述*****/ 
  if(ec<ecRule[0])
  {
  ecFuzzy[0] =1.0;
  pec = 0;
  }
  else if(ecRule[0]<=ec && ec<ecRule[1])
  {
  ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]); pec = 0 ;
  }
  else if(ecRule[1]<=ec && ec<ecRule[2])
  {
  ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]); pec = 1;
  }
  else if(ecRule[2]<=ec && ec<ecRule[3])
  {
  ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]); pec = 2 ;
  }
  else if(ecRule[3]<=ec && ec<ecRule[4])
  { ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
  pec=3;
  }
  else if(ecRule[4]<=ec && ec<ecRule[5])
  { ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
  pec=4;
  }
  else if(ecRule[5]<=ec && ec<ecRule[6])
  { ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
  pec=5;
  }
  else
  {
  ecFuzzy[0] =0.0;
  pec = 5;
  }
  ecFuzzy[1] = 1.0 - ecFuzzy[0];
  /*********查询模糊规则表*********/
  num =KpRule[pe][pec];                 /*根据e,ec查询*/
  KpFuzzy[num] += eFuzzy[0]*ecFuzzy[0];/*相乘？？(保证KpFuzzy数组中的四个数相加等于一，可以验证，用于加权平均法解模糊)*/
  num =KpRule[pe][pec+1];
  KpFuzzy[num] += eFuzzy[0]*ecFuzzy[1]; /*KpFuzzy数组中的数是占kpRule数组中的比重，他们是一一对应的关系*/
  num =KpRule[pe+1][pec];
  KpFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
  num =KpRule[pe+1][pec+1];
  KpFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
  /*********加权平均法解模糊*********/ 
  Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]
  +KpFuzzy[3]*kpRule[3];
  return(Kp_calcu);
}