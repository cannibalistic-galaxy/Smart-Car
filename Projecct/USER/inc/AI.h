#ifndef _AI_H
#define _AI_H

#define AI_Num 20 //20������
extern uint16 RoungAI[AI_Num];
extern uint16 huanbiaozhi[AI_Num];
extern int16 AINumber;
extern uint16 AIValue;
extern uint16 zhuan1,huandao1,zhuan2,huandao2,zhuan3,huandao3,zhuan4,huandao4,JinZhi,StopLead,Chuzhi,ChuSpeed,RuSpeed,ChuNum,ChuDir,ChuCha,SetTime,SaveLife,SaveLifePar,LeadPar;  //�������������������������ĸ�ƫ��
extern uint16 zhuan; //������ת����
extern uint16 roundmode;

extern void Round_AI();
extern void AI_FLASH_Read();
extern void AI_FLASH_Write();
extern void Round_Mode();
#endif