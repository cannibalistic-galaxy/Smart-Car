#ifndef _AI_H
#define _AI_H

#define AI_Num 20 //20个参数
extern uint16 RoungAI[AI_Num];
extern uint16 huanbiaozhi[AI_Num];
extern int16 AINumber;
extern uint16 AIValue;
extern uint16 zhuan1,huandao1,zhuan2,huandao2,zhuan3,huandao3,zhuan4,huandao4,JinZhi,StopLead,Chuzhi,ChuSpeed,RuSpeed,ChuNum,ChuDir,ChuCha,SetTime,SaveLife,SaveLifePar,LeadPar;  //三个环岛的正反，三个环岛的给偏差
extern uint16 zhuan; //定义检测转向方向
extern uint16 roundmode;

extern void Round_AI();
extern void AI_FLASH_Read();
extern void AI_FLASH_Write();
extern void Round_Mode();
#endif