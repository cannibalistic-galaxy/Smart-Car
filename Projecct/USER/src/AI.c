#include "headfile.h"

//#define AI_Num 6
uint16 RoungAI[AI_Num]={0}; //����AI����
uint16 huanbiaozhi[AI_Num]={0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7,0,1,2,3};
uint16 reset[AI_Num] = {0,65,1,40,1,110,0,0,15,50,20,440,440,2,1,50,150,35,25,0};
int16 AINumber=0;
uint16 AIValue=0;
uint16 zhuan1,huandao1,zhuan2,huandao2,zhuan3,huandao3,zhuan4,huandao4,JinZhi,StopLead,Chuzhi,ChuSpeed,RuSpeed,ChuNum,ChuDir,ChuCha,SetTime,SaveLife,SaveLifePar,LeadPar;  //�������������������������ĸ�ƫ��,������ֵ��ֹͣ������������ֵ,������ʼ�ٶȲ��ȫ����·��
uint16 zhuan; //������ת����
uint16 roundmode;
void Round_AI(void)
{
 if(AINumber>=0&&AINumber<8) //������ʾ	
 {
  OLED_P6x8Str(0,0,"zhuan1:");    
  if(zhuan1==0) OLED_P6x8Str(60,0,"left");   
  else if(zhuan1==1) OLED_P6x8Str(60,0,"right");
  else if(zhuan1==2) OLED_P6x8Str(60,0,"none");  //��������
  //else if(zhuan1==3) OLED_P6x8Str(60,0,"yindao");
  //OLED_ShowData((uint16)zhuan1,60,0);//��һ������ת��  
  OLED_P6x8Str(0,1,"huandao1:");
  OLED_ShowData((uint16)huandao1,68,1);//��һ������ƫ��
  
  OLED_P6x8Str(0,2,"zhuan2:");
 // OLED_ShowData((uint16)zhuan2,60,2); //�ڶ�������ת��
  if(zhuan2==0) OLED_P6x8Str(60,2,"left");   
  else if(zhuan2==1) OLED_P6x8Str(60,2,"right");
  else if(zhuan2==2) OLED_P6x8Str(60,2,"none");
  //else if(zhuan2==3) OLED_P6x8Str(60,2,"yindao");
  OLED_P6x8Str(0,3,"huandao2:");    
  OLED_ShowData((uint16)huandao2,68,3);//�ڶ�������ƫ��
  
  OLED_P6x8Str(0,4,"zhuan3:");
 // OLED_ShowData((uint16)zhuan3,60,4); //����������ת��
  if(zhuan3==0) OLED_P6x8Str(60,4,"left");   
  else if(zhuan3==1) OLED_P6x8Str(60,4,"right");
  else if(zhuan3==2) OLED_P6x8Str(60,4,"none");
  //else if(zhuan3==3) OLED_P6x8Str(60,4,"yindao");
  OLED_P6x8Str(0,5,"huandao3:");
  OLED_ShowData((uint16)huandao3,68,5);//����������ƫ��
  
  OLED_P6x8Str(0,6,"zhuan4:");   
  if(zhuan4==0) OLED_P6x8Str(60,6,"left");  
  else if(zhuan4==1) OLED_P6x8Str(60,6,"right");  
  else if(zhuan4==2) OLED_P6x8Str(60,6,"none");  
  //else if(zhuan4==3) OLED_P6x8Str(60,6,"yindao");
  OLED_P6x8Str(0,7,"huandao4:");
  OLED_ShowData((uint16)huandao4,68,7);//����������ƫ��
  //OLED_ShowData((uint16)SetTime,68,6); 
  
  
  //OLED_P6x8Str(0,7," left 0 , right 1 ");  //��ʾ
  OLED_P6x8Str(110,huanbiaozhi[AINumber],"<-");
 }
 else if(AINumber>=7&&AINumber<16)
 {
  OLED_P6x8Str(0,0,"JinZhi:");    
  OLED_ShowData((uint16)JinZhi,70,0); 
  
  OLED_P6x8Str(0,1,"StopLead:");
  OLED_ShowData((uint16)StopLead,70,1);
  
  OLED_P6x8Str(0,2,"Chuzhi:");
  OLED_ShowData((uint16)Chuzhi,70,2); 
  
  OLED_P6x8Str(0,3,"ChuSpeed:");    
  OLED_ShowData((uint16)ChuSpeed,70,3);
  
  OLED_P6x8Str(0,4,"RuSpeed:");
  OLED_ShowData((uint16)RuSpeed,70,4); 
  
  OLED_P6x8Str(0,5,"ChuNum:");
  if(ChuNum==0) OLED_P6x8Str(70,5,"None"); //û�л�����Ҫ��ƫ�����
  else if(ChuNum==1) OLED_P6x8Str(70,5,"huan1");
  else if(ChuNum==2) OLED_P6x8Str(70,5,"huan2");
  else if(ChuNum==3) OLED_P6x8Str(70,5,"huan3");
  else if(ChuNum==4) OLED_P6x8Str(70,5,"huan4");
  //OLED_ShowData((uint16)ChuNum,70,5);
  
  OLED_P6x8Str(0,6,"ChuDir:");
  if(ChuDir==0) OLED_P6x8Str(70,6,"left");
  else OLED_P6x8Str(70,6,"right");
  //OLED_ShowData((uint16)ChuDir,70,6);
  
  OLED_P6x8Str(0,7,"ChuCha:");
  OLED_ShowData((uint16)ChuCha,70,7);
  //OLED_P6x8Str(0,7," left 0 , right 1 ");  //��ʾ
  OLED_P6x8Str(110,huanbiaozhi[AINumber],"<-");
 }
 else
 {
   OLED_P6x8Str(0,0,"SetTime:");    
   OLED_ShowData((uint16)SetTime,70,0); 
   
   OLED_P6x8Str(0,1,"SaveLife:");    
   OLED_ShowData((uint16)SaveLife,70,1); 
   
   OLED_P6x8Str(0,2,"SaveLifePar:");    
   OLED_ShowData((uint16)SaveLifePar,70,2); 
   
   OLED_P6x8Str(0,3,"LeadPar:");    
   OLED_ShowData((uint16)LeadPar,70,3); 
   
   OLED_P6x8Str(110,huanbiaozhi[AINumber],"<-");
 }
   if(key_value == 17) //��������
   {
     AINumber += 8;
     if(AINumber>=19&&AINumber<24) AINumber=19;
     if(AINumber>23) AINumber = 0;     //20������ 
     OLED_CLS();
   }
	  
   if(key_value==4 || key_value==12) //4��12��ʾ����ѡ��        
   {
     if(AINumber>19) AINumber = 0;     //20������
      if(AINumber<0) AINumber = 19;      
      switch (key_value)
      {
        case 4: AINumber++; break; //��һ��
        case 12: AINumber--; break; //��һ��
      }
      if(AINumber>19) AINumber = 0;     //20������
      if(AINumber<0) AINumber = 19;      
      OLED_CLS();
   }
   
   if(key_value<=3&&key_value>=1 || key_value<=11&&key_value>=9) //1��3,9��11
   {
    switch (key_value)
    {
      case 1: RoungAI[AINumber]+=1; OLED_CLS(); break; //+1
      case 9: RoungAI[AINumber]-=1; OLED_CLS(); break; //-1
      case 2: RoungAI[AINumber]+=5; OLED_CLS(); break; //+5
      case 10: RoungAI[AINumber]-=5; OLED_CLS(); break; //-5
      case 3: RoungAI[AINumber]+=10; OLED_CLS(); break; //+10
      case 11: RoungAI[AINumber]-=10; OLED_CLS(); break; //-10
    }  
    if(RoungAI[0]>2&&RoungAI[0]<4) RoungAI[0] = 0;
    else if(RoungAI[0]>4) RoungAI[0] = 2;
    if(RoungAI[2]>2&&RoungAI[2]<4) RoungAI[2] = 0;
    else if(RoungAI[2]>4) RoungAI[2] = 2;
    if(RoungAI[4]>2&&RoungAI[4]<4) RoungAI[4] = 0;
    else if(RoungAI[4]>4) RoungAI[4] = 2;
    if(RoungAI[6]>2&&RoungAI[6]<4) RoungAI[6] = 0;
    else if(RoungAI[6]>4) RoungAI[6] = 2;
    if(RoungAI[13]>4) RoungAI[13] = 0;
    if(RoungAI[14]>1) RoungAI[14] = 0;
    zhuan1 = RoungAI[0];
    huandao1 = RoungAI[1];
    zhuan2 = RoungAI[2];
    huandao2 = RoungAI[3];
    zhuan3 = RoungAI[4];
    huandao3 = RoungAI[5];
    zhuan4 = RoungAI[6];
    huandao4 = RoungAI[7];   
    JinZhi = RoungAI[8];
    StopLead = RoungAI[9];
    Chuzhi = RoungAI[10];
    ChuSpeed = RoungAI[11];
    RuSpeed = RoungAI[12];
    ChuNum = RoungAI[13];
    ChuDir = RoungAI[14];
    ChuCha = RoungAI[15];
    SetTime = RoungAI[16];
    SaveLife = RoungAI[17];
    SaveLifePar = RoungAI[18];
    LeadPar = RoungAI[19];
}
/*if(key_value==18) 
{
  writetoflash();   
  BUZZER_ON;
  pit_delay_ms(pit0,200);
  BUZZER_OFF;
}  //�����ղ���д�뵽flash������һ��ʹ�ã�����������*/
if(key_value == 19)
{
  for(uint8 i=0;i<AI_Num;i++) //��ֹFLASH��
    {
      RoungAI[i] = reset[i];
      BUZZER_ON;
    }
}
}

void Round_Mode(void) //Ϊ2�����Σ���Ϊ2�ǿ���
{
  if(zhuan1==2 && zhuan2==2 && zhuan3==2 && zhuan4==2) roundmode=0;  //ȫ����������          0000   0
  if(zhuan1!=2 && zhuan2==2 && zhuan3==2 && zhuan4==2) roundmode=1;  //ֻ�е�һ������        1000   1
  if(zhuan1!=2 && zhuan2!=2 && zhuan3==2 && zhuan4==2) roundmode=2;  //��һ���͵ڶ�������    1100   2
  if(zhuan1!=2 && zhuan2!=2 && zhuan3!=2 && zhuan4==2) roundmode=3;  //һ����������          1110   3
  if(zhuan1!=2 && zhuan2!=2 && zhuan3!=2 && zhuan4!=2) roundmode=4;  //ȫ������              1111   4
  if(zhuan1==2 && zhuan2!=2 && zhuan3==2 && zhuan4==2) roundmode=5;  //ֻ�еڶ�������        0100   1
  if(zhuan1==2 && zhuan2!=2 && zhuan3!=2 && zhuan4==2) roundmode=6;  //�ڶ����͵���������    0110   2
  if(zhuan1==2 && zhuan2!=2 && zhuan3!=2 && zhuan4!=2) roundmode=7;  //�����ĸ�����          0111   3
  if(zhuan1==2 && zhuan2==2 && zhuan3!=2 && zhuan4==2) roundmode=8;  //ֻ�е���������        0010   1
  if(zhuan1==2 && zhuan2==2 && zhuan3!=2 && zhuan4!=2) roundmode=9;  //���������ĸ�����      0011   2
  if(zhuan1==2 && zhuan2==2 && zhuan3==2 && zhuan4!=2) roundmode=10; //ֻ�е��ĸ�����        0001   1
  if(zhuan1!=2 && zhuan2==2 && zhuan3!=2 && zhuan4==2) roundmode=11; //��һ������������      1010   2
  if(zhuan1!=2 && zhuan2==2 && zhuan3!=2 && zhuan4!=2) roundmode=12; //һ���ĸ�����          1011   3
  if(zhuan1!=2 && zhuan2==2 && zhuan3==2 && zhuan4!=2) roundmode=13; //��һ���ĸ�����        1001   2
  if(zhuan1!=2 && zhuan2!=2 && zhuan3==2 && zhuan4!=2) roundmode=14; //һ���ĸ�����          1101   3
  if(zhuan1==2 && zhuan2!=2 && zhuan3==2 && zhuan4!=2) roundmode=15; //�ڶ����ĸ�����        0101   2
}

void AI_FLASH_Read(void)
{
  RoungAI[0] = flash_read(247,0,uint16);
  RoungAI[1] = flash_read(247,4,uint16);
  RoungAI[2] = flash_read(247,8,uint16);
  RoungAI[3] = flash_read(247,12,uint16);
  RoungAI[4] = flash_read(247,16,uint16);
  RoungAI[5] = flash_read(247,20,uint16);
  RoungAI[6] = flash_read(247,24,uint16);
  RoungAI[7] = flash_read(247,28,uint16);
  RoungAI[8] = flash_read(247,32,uint16);
  RoungAI[9] = flash_read(247,36,uint16);
  RoungAI[10] = flash_read(247,40,uint16);
  RoungAI[11] = flash_read(247,44,uint16);
  RoungAI[12] = flash_read(247,48,uint16);
  RoungAI[13] = flash_read(247,52,uint16);
  RoungAI[14] = flash_read(247,56,uint16);
  RoungAI[15] = flash_read(247,60,uint16);
  RoungAI[16] = flash_read(247,64,uint16);
  RoungAI[17] = flash_read(247,68,uint16);
  RoungAI[18] = flash_read(247,72,uint16);
  RoungAI[19] = flash_read(247,76,uint16);
  
    zhuan1 = RoungAI[0];
    huandao1 = RoungAI[1];
    zhuan2 = RoungAI[2];
    huandao2 = RoungAI[3];
    zhuan3 = RoungAI[4];
    huandao3 = RoungAI[5];
    zhuan4 = RoungAI[6];
    huandao4 = RoungAI[7];   
    JinZhi = RoungAI[8];
    StopLead = RoungAI[9];
    Chuzhi = RoungAI[10];
    ChuSpeed = RoungAI[11];
    RuSpeed = RoungAI[12];
    ChuNum = RoungAI[13];
    ChuDir = RoungAI[14];
    ChuCha = RoungAI[15];
    SetTime = RoungAI[16];
    SaveLife = RoungAI[17];
    SaveLifePar = RoungAI[18];
    LeadPar = RoungAI[19];
}

void AI_FLASH_Write(void)
{
   
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[0],2,0);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[1],2,4); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[2],2,8); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[3],2,12); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[4],2,16); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[5],2,20); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[6],2,24);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[7],2,28); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[8],2,32);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[9],2,36); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[10],2,40); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[11],2,44); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[12],2,48); 
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[13],2,52);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[14],2,56);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[15],2,60);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[16],2,64);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[17],2,68);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[18],2,72);
   FLASH_WriteSector(247,(const uint8 *)&RoungAI[19],2,76);
}