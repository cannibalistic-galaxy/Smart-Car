/*********************************************************************************************************************
 * @data   6-17
 * @brief  �����ٶ�speed_left
 *         �����ٶ�speed_right
 *         �˷Ŵ����ң����ţ�E7,E6,E5,E14,E13,E12
           ��ˮƽ1 ��ˮƽ2 �м�3 ��б4 ��б5
           #define AMP1    ADC0_SE12
           #define AMP2    ADC0_SE14
           #define AMP3    ADC0_SE13
           #define AMP4    ADC0_SE5
           #define AMP5    ADC0_SE6
 *  ������I6��ͣ��D5
   right<left
 ********************************************************************************************************************/

#include "headfile.h"

//UART_Type * uart[3] = UART_BASES;
#define canshu_number 15
//#define debug    //�Ƿ�ʹ������
uint16 canshu[canshu_number]={0};    //��ʼʱ��12��������ֵ���Լ���ֵ,         
uint8 biaozhi[canshu_number]={0,1,2,3,4,5,6,0,1,2,3,4,5,6,7}; //����ǩλ
uint16 fuwei2[canshu_number]= {7,400,110,10,0,20,50,500,230,9,15,24,30,50,0};   //��λ���飬�����涨Ϊһ�����ڸ�λ
uint16 fuwei3[canshu_number] = {7,480,190,10,0,16,52,500,200,9,15,24,33,44,200};//��������
uint16 fuwei[canshu_number] = {8,150,130,19,0,18,35,500,159,9,19,9,30,40,0};//������
int shuaxin;                      //ˢ��
uint8 show_flag_1,show_flag_2,show_flag_3,show_flag_4; //���뿪�ش��ϵ�������Ϊ1��2��3��4
uint16 ADC_Value_temp[6][3];
uint8 key,key_oled_mode;         //��ֵ���ͼ�ֵץȡ,�Լ�����oled��ʾѡ��
uint8 DataScope_SendMode;
uint8 next_page;
extern float dingshijiansu;
/*----------------------------������------------------------------------------*/
extern float  Speed_P,Speed_I, Turn_P, Turn_D,Turn_I,Turn2P,Turn2D,SpeedGiven,Round_P,Round_D,speedweight,BT,X1,X2,SpeedCut;
extern float var[SHANWAI];

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/


void System_Init(void);  //��ʼ��
void readfromflash(void);     //��flash��ȡ����
void writetoflash(void);    //����д��flash
void ad_writetoflash(void);
void ad_readfromflash(void);

int main(void)
{
    get_clk();              //��ȡʱ��Ƶ�� ����ִ��
    System_Init();
    boma_init();   //���뿪�س�ʼ��  
    OLED_Init();
    IIC_init();  
    M_IIC_init(); 
    //�жϳ�ʼ�� 
    InitMPU6050();    //�����ǳ�ʼ�� 
    uart_init(uart1,115200);   //���ڳ�ʼ��    
    //FLASH��ʼ��
    FLASH_Init();     //flash������ʼ��   
    readfromflash();    //��flash�е����ݶ�ȡ��  
    pit_init_ms(pit0,3); //��ʼ��pit0 ��������Ϊ50ms	
    //set_irq_priority(IRQ_IRQn,0);        //�ⲿ�жϵ����ȼ����
    set_irq_priority(PIT_CH0_IRQn,1);	//����pit0���ȼ�
        
    gpio_init(C7,GPI,0);
    port_pull(C7);    //A1����������
    boma_mode();  //�ɼ����뿪������
    show_flag_1 = gpio_get(A0);
    show_flag_2 = gpio_get(C7);
    show_flag_3 = gpio_get(C6);
    show_flag_4 = gpio_get(I3);
        
    while(!key_oled_mode)    //��ȷ��������֮ǰ������ʾ�Լ����κ���
    {     
        shuaxin++;
        if(shuaxin==800) shuaxin = 0;
        if(shuaxin==1)  OLED_CLS();
        key=gpio_get(A1);
        if(!key)
        {
          key_value=simiic_read_reg(0x70,0x01,IIC);       
          while(!(gpio_get(A1)));    //�ȴ��������ͷ�
        }       
        if(key_value==20)key_oled_mode=1;
        if(key_value==17) OLED_CLS();
        if(key_value==18) 
        {
          writetoflash();   
          BUZZER_ON;
          pit_delay_ms(pit0,200);
          BUZZER_OFF;
        }  //�����ղ���д�뵽flash������һ��ʹ�ã�����������
       
    /********************************ģʽ��***************************************************/
    
       if(gpio_get(sw1)==0&&show_flag_2)//���޸Ĳ�������ʾ���               
       {   //out4=B1  out5=F6  out6=F5  out3=B2  =F4
        uint8 j;
        for(j=0;j<3;j++)
        {
         ADC_Value_temp[0][j]=adc_once(ADC0_SE12,ADC_12bit);   //��ʱADC���ݶ�ȡ�����ڹ۲�guiyihua /*��ˮƽ*/
         ADC_Value_temp[1][j]=adc_once(ADC0_SE14,ADC_12bit);
         ADC_Value_temp[2][j]=adc_once(ADC0_SE13,ADC_12bit);
         ADC_Value_temp[3][j]=adc_once(ADC0_SE5,ADC_12bit);
         ADC_Value_temp[4][j]=adc_once(ADC0_SE6,ADC_12bit); // /*��б*/
         ADC_Value_temp[5][j]=adc_once(ADC0_SE7,ADC_12bit);
        }
        for(j=0;j<6;j++)
        {
          
        ADC_Value_temp[j][0]+= ADC_Value_temp[j][1]+ ADC_Value_temp[j][2];
        ADC_Value_temp[j][0]= ADC_Value_temp[j][0]/3;
        AD_MAX[j]=( ADC_Value_temp[j][0]>AD_MAX[j]? ADC_Value_temp[j][0]:AD_MAX[j]);
        AD_MIN[j] =(ADC_Value_temp[j][0]<AD_MIN[j]? ADC_Value_temp[j][0]:AD_MIN[j]);
        }
              
         OLEDInADNormalization(); 
         if(key_value == 17) //��������
          {
            OLED_CLS();
          }
	  if(key_value == 19)
          {
            ad_writetoflash();
            BUZZER_ON;
            pit_delay_ms(pit0,200);
            BUZZER_OFF;
          }
        }
    /******************************ģʽһ***************************************************/   
       
       if(gpio_get(sw1)==1&&show_flag_2) //��������
        {
         
          if(key_number>=0&&key_number<7)
		  {			  

              OLED_P6x8Str(0,0,"Speed_P:");    //������ʾ
              OLED_ShowData((uint16)Speed_P,55,0);
              OLED_P6x8Str(0,1,"Speed_I:");
              OLED_ShowData((uint16)Speed_I,55,1);
              OLED_P6x8Str(0,2,"Turn_P:");
              OLED_ShowData((uint16)Turn_P,55,2);
              OLED_P6x8Str(0,3,"Turn_D:");
              OLED_ShowData((uint16)Turn_D,55,3); 
              OLED_P6x8Str(0,4,"Turn_I:");
              OLED_ShowData((uint16)Turn_I,55,4);
              OLED_P6x8Str(0,5,"Turn2P:");
              OLED_ShowData((uint16)Turn2P,55,5);
              OLED_P6x8Str(0,6,"Turn2D:");
              OLED_ShowData((uint16)Turn2D,55,6);
              OLED_P6x8Str(110,biaozhi[key_number],"<-");
              //OLED_CLS();
		  }
		  else if(key_number>=7&&key_number<15)     //
		  { // OLED_CLS();                  
              OLED_P6x8Str(0,0,"SpeedGiven:");    //������ʾ�ڶ�ҳ
              OLED_ShowData((uint16)SpeedGiven,64,0);
              OLED_P6x8Str(0,1,"Round_P:");
              OLED_ShowData((uint16)Round_P,64,1);
              OLED_P6x8Str(0,2,"Round_D:");
              OLED_ShowData((uint16)Round_D,64,2);
              OLED_P6x8Str(0,3,"M_P:");
              OLED_ShowData((uint16)M_P,64,3); 
              OLED_P6x8Str(0,4,"BT:");              
              OLED_ShowData((uint16)BT,64,4); 
              OLED_P6x8Str(0,5,"X1:");              
              OLED_ShowData((uint16)X1,68,5); 
              OLED_P6x8Str(0,6,"X2:");              
              OLED_ShowData((uint16)X2,68,6); 
              OLED_P6x8Str(0,7,"SpeedCut:");              
              OLED_ShowData((uint16)SpeedCut,68,7); 
              OLED_P6x8Str(110,biaozhi[key_number],"<-");
		  }
             /* else
              {
                  OLED_P6x8Str(0,0,"X3:");    //������ʾ�ڶ�ҳ
                  OLED_ShowData((uint16)X3,64,0);
                  OLED_P6x8Str(0,1,"X4:");
                  OLED_ShowData((uint16)X4,64,1);
                  OLED_P6x8Str(110,biaozhi[key_number],"<-");
              }*/
	  
	  if(key_value == 17) //��������
          {
            key_number += 7;
            if(key_number>14) key_number=0;
            OLED_CLS();
          }
	  
         if(key_value==4 || key_value==12) //4��12��ʾ����ѡ��
        
         {
          switch (key_value)
          {
            case 4: key_number++; break; //��һ��
            case 12: key_number--; break; //��һ��
          }
          if(key_number>14) key_number = 0;     //ʮ�������
          if(key_number<0)  key_number = 14;
          OLED_CLS();
         }
              
         if(key_value<=3&&key_value>=1 || key_value<=11&&key_value>=9) //1��3,9��11
          {
            switch (key_value)
            {
              case 1: canshu[key_number]+=1; break; //+1
              case 9: canshu[key_number]-=1; break; //-1
              case 2: canshu[key_number]+=5; break; //+5
              case 10: canshu[key_number]-=5; break; //-5
              case 3: canshu[key_number]+=10; break; //+10
              case 11: canshu[key_number]-=10; break; //-10
            }  
            Speed_P=canshu[0];    //������ֵ
            Speed_I=canshu[1]; 
            Turn_P=canshu[2]; 
            Turn_D=canshu[3];
            Turn_I=canshu[4];
            Turn2P=canshu[5];
            Turn2D=canshu[6];   
            SpeedGiven=canshu[7];
            
            Round_P=canshu[8];
            Round_D=canshu[9];
            M_P=canshu[10];
            BT=canshu[11];
            X1=canshu[12];
            X2=canshu[13];
            SpeedCut=canshu[14];
            
          }
          if(key_value == 19)
          {
            for(uint8 i=0;i<canshu_number;i++) //��ֹFLASH��
            {
              canshu[i] = fuwei[i];
              BUZZER_ON;
            }
          }
        } //ģʽһ����
            
 /****************************************ģʽ��************************************************/
        if(show_flag_2==0&&show_flag_1) //�����˹�����ģʽ
        {
          Round_AI();  
        }
/*********************************************************************************************/ 
/*************************************ģʽ��**************************************************/
      if(show_flag_4==0)//Բ������ģʽ�����һ������
      {
        jiansu_flag=1;
      }
      
/********************************************************************************************/
/*****************************ģʽ��***************************************************/
      if(show_flag_3==0) //��ʱ���٣�����������
      {
        dingshijiansu=1;
      }
/**************************************************************************************/
          key_value=0;   //��ֵ����
          systick_delay_ms(5);//��ʱ5ms
    
    } //����ѭ������
          //BUZZER_OFF;
           OLED_CLS();                 
          readfromflash();    //��flash�е����ݶ�ȡ��    
          ad_readfromflash();
          SpeedGivenTemp = SpeedGiven; //�ٶ��ݴ�
          Round_Mode();
          systick_delay_ms(400);
          systick_delay_ms(400);
        
          enable_irq(PIT_CH0_IRQn);			
          gpio_init(I2,GPI,0);
          port_pull(I2);
          EnableInterrupts;					//�������ж�         
          for(;;)    //����forִ���ٶȿ�
          {
              vcan_sendware(var, sizeof(var));           
          //   Get_Gyro(); 
          //   mpu_gyro_z=mpu_gyro_z/10;
              if(show_flag_1&&show_flag_2) //1111
               {
                OLED_P6x8Str(0,0,"   Lpw Lpn Rpn Rpw  ");  
                OLED_P6x8Str(0,1,"        Lx Rx    "); 
                OLED_ShowData(AD_Value[1], 0, 2);
                OLED_ShowData(AD_Value[2], 32, 2);                
                OLED_ShowData(AD_Value[5], 64, 2);
                OLED_ShowData(AD_Value[0], 96, 2);
                
                OLED_ShowData(AD_Value[4], 32, 4);   /*��ʾ      �� �� ��
                                                               ��б  ��б      */
                OLED_ShowData(AD_Value[3], 64, 4);
                OLED_ShowData((int)AD_Error, 0, 6);
                OLED_ShowData((int)Middle_Err, 64, 6);
        //        OLED_ShowData(mpu_gyro_z,32,6);
        
                }
               else if(!show_flag_1&&show_flag_2)//0111
              {
                OLED_ShowData(AD_MAX[1], 0, 2);
                OLED_ShowData(AD_MAX[2], 32, 2);
                OLED_ShowData(AD_MAX[5], 64, 2);
                OLED_ShowData(AD_MAX[0], 96, 2);
                OLED_ShowData(AD_MAX[4], 32, 4);                                                                  
                OLED_ShowData(AD_MAX[3], 64, 4); 
                OLED_P6x8Str(6,0,"   Lpw Lpn Rpn Rpw  ");  
                OLED_P6x8Str(7,1,"        Lx Rx    "); 
               }
               if(!show_flag_2) //��ʾ����
               {
                OLED_P6x8Str(0,0,"ADError3");
                OLED_ShowData((int)AD_Error_3,80,0);
                OLED_P6x8Str(0,1,"CircleFlag");
                OLED_ShowData(circle_Flag,80,1);               
                OLED_P6x8Str(0,2,"TurnFlag");
                OLED_ShowData(Turn_Flag,80,2); //�� ��
                OLED_P6x8Str(0,3,"SpeedGiven");
                OLED_ShowData((int)SpeedGiven,80,3);                
                OLED_P6x8Str(0,4,"OutCircle");
                OLED_ShowData(Go_Out_Circle,80,4);
                OLED_P6x8Str(0,5,"CircleNum");
                OLED_ShowData(huandaonum,80,5);
                OLED_P6x8Str(0,6,"Guo0");
                OLED_ShowData(turn_Error,80,6);
                OLED_P6x8Str(0,7,"ADErr3Max");
                OLED_ShowData((int)AD_Error_3_Max,80,7);
                //OLED_P6x8Str(0,4,"Guo0Err");
               // OLED_ShowData(turn_Error,42,4);  */        
                /*   OLED_P14x16Str(0,0,"��������",0);
                   OLED_ShowData(AD_Error_3,60,1);
                   OLED_P14x16Str(0,2,"��ʼ���",1);
                   OLED_ShowData(Turn_Flag,60,3);
                   OLED_P14x16Str(0,4,"��ǽ���",2);
                   OLED_ShowData(Turn_Flag2,60,5);
                   OLED_P14x16Str(0,6,"������־",3);
                   OLED_ShowData(Go_Out_Circle,60,7);
                   if(circle_Flag==1) OLED_P6x8Str(90,0,"circle");*/
               }
               /**
                * ����ʾ�������Ͳ���ģʽ
                **/
               if(show_flag_2)     DataScope_SendMode=0;//1111 
               else if(!show_flag_2)  DataScope_SendMode=1;//1011
            
          
                OLED_DLY_ms(5);         
          }//��̬ͣ��
   
}



/********************************************************************************************************************
 * @brief  ���ֳ�ʼ������
 ********************************************************************************************************************/
/*-----------------------��FLASH----------------------------------*/
void readfromflash(void)
{
    canshu[0]=flash_read(253,0,uint16);   //�ٶȻ���253����
    canshu[1]=flash_read(253,4,uint16);
    canshu[8]=flash_read(253,8,uint16);
    canshu[9]=flash_read(253,12,uint16);
    
    
    canshu[2]=flash_read(254,0,uint16);    //ת�򻷲���
    canshu[3]=flash_read(254,4,uint16);
    canshu[4]=flash_read(254,8,uint16);
    
    canshu[5]=flash_read(255,0,uint16);
    canshu[6]=flash_read(255,4,uint16);
    canshu[7]=flash_read(255,8,uint16);

    canshu[10]=flash_read(254,12,uint16);
    canshu[11]=flash_read(255,12,uint16);
    canshu[12]=flash_read(248,0,uint16);
    canshu[13]=flash_read(248,4,uint16);
    canshu[14]=flash_read(248,8,uint16);
    
   
    AI_FLASH_Read(); //AI��FLASH
    
            Speed_P=canshu[0];    //������ֵ
            Speed_I=canshu[1]; 
            Turn_P=canshu[2]; 
            Turn_D=canshu[3];
            Turn_I=canshu[4];
            Turn2P=canshu[5];
            Turn2D=canshu[6]; 
            SpeedGiven=canshu[7];
            Round_P=canshu[8];
            Round_D=canshu[9];
            M_P=canshu[10];
            BT=canshu[11];
            X1=canshu[12];
            X2=canshu[13];
            SpeedCut=canshu[14];
           
            
           
}

void ad_readfromflash(void)
{  
   AD_MAX[5] = flash_read(249,4,uint16);
   AD_MIN[5] = flash_read(249,0,uint16);
   
   AD_MAX[0] = flash_read(251,0,uint16);
   AD_MAX[1] = flash_read(252,0,uint16);
   AD_MAX[2] = flash_read(252,4,uint16);
   AD_MAX[3] = flash_read(252,8,uint16);
   AD_MAX[4] = flash_read(252,12,uint16);
  
   
   AD_MIN[0] = flash_read(250,0,uint16);
   AD_MIN[1] = flash_read(250,4,uint16);
   AD_MIN[2] = flash_read(250,8,uint16);
   AD_MIN[3] = flash_read(250,12,uint16);
   AD_MIN[4] = flash_read(251,4,uint16);

   
}
/*----------------------------дFLASH----------------------*/
void writetoflash(void)
{
  
      FLASH_EraseSector(255);   //������һ������
      FLASH_EraseSector(254);   //��������ڶ������� 
      FLASH_EraseSector(253);   //�����������������
      FLASH_EraseSector(248);   //����������ĸ�����
      FLASH_EraseSector(247);   //����������������
      
       
    AI_FLASH_Write(); //AIдFLASH
     
     FLASH_WriteSector(253,(const uint8 *)&canshu[0],2,0); 
     FLASH_WriteSector(253,(const uint8 *)&canshu[1],2,4);
     FLASH_WriteSector(253,(const uint8 *)&canshu[8],2,8);
     FLASH_WriteSector(253,(const uint8 *)&canshu[9],2,12);
     
     FLASH_WriteSector(254,(const uint8 *)&canshu[2],2,0);
     FLASH_WriteSector(254,(const uint8 *)&canshu[3],2,4);
     FLASH_WriteSector(254,(const uint8 *)&canshu[4],2,8);
     
     FLASH_WriteSector(255,(const uint8 *)&canshu[5],2,0);
     FLASH_WriteSector(255,(const uint8 *)&canshu[6],2,4);
     FLASH_WriteSector(255,(const uint8 *)&canshu[7],2,8);
     FLASH_WriteSector(254,(const uint8 *)&canshu[10],2,12);
     FLASH_WriteSector(255,(const uint8 *)&canshu[11],2,12);
     
      FLASH_WriteSector(248,(const uint8 *)&canshu[12],2,0);
      FLASH_WriteSector(248,(const uint8 *)&canshu[13],2,4);
       FLASH_WriteSector(248,(const uint8 *)&canshu[14],2,8);
     
}
void ad_writetoflash(void)
{
     FLASH_EraseSector(252);
     FLASH_EraseSector(251);  
     FLASH_EraseSector(250); 
     FLASH_EraseSector(249);
     
     FLASH_WriteSector(249,(const uint8 *)&AD_MIN[5],2,0);
     FLASH_WriteSector(249,(const uint8 *)&AD_MAX[5],2,4);
     
     FLASH_WriteSector(250,(const uint8 *)&AD_MIN[0],2,0);
     FLASH_WriteSector(250,(const uint8 *)&AD_MIN[1],2,4);
     FLASH_WriteSector(250,(const uint8 *)&AD_MIN[2],2,8);
     FLASH_WriteSector(250,(const uint8 *)&AD_MIN[3],2,12);
     FLASH_WriteSector(251,(const uint8 *)&AD_MIN[4],2,4);
     
     
     FLASH_WriteSector(251,(const uint8 *)&AD_MAX[0],2,0); //��������    
     FLASH_WriteSector(252,(const uint8 *)&AD_MAX[1],2,0); 
     FLASH_WriteSector(252,(const uint8 *)&AD_MAX[2],2,4);
     FLASH_WriteSector(252,(const uint8 *)&AD_MAX[3],2,8);
     FLASH_WriteSector(252,(const uint8 *)&AD_MAX[4],2,12);
}
/*----------------------------------��ʼ��---------------------*/
void System_Init(void)
{
    uint8 i;
/*****������ʾ*****/ 
   // pit_delay_ms(pit0,1000);
   gpio_init(G0,GPO,1);
    gpio_set(G0,1);
    gpio_init(G1,GPO,1);
    gpio_set(G1,1);
    gpio_init(G2,GPO,1);
    gpio_set(G2,1);
    gpio_init(G3,GPO,1);
    gpio_set(G3,1);
 
    for(i=0;i<2;i++)
   {
    pit_delay_ms(pit0,250);
    gpio_turn(G0);
    gpio_turn(G1);
    gpio_turn(G2);
    gpio_turn(G3);
      
  }

   //�������ڳ�ʼ��
   gpio_init(H0,GPO,0);
   port_pull(H0);
   /*BUZZER_ON;
   pit_delay_ms(pit0,200);
   BUZZER_OFF;
   pit_delay_ms(pit0,100);*/
   BUZZER_ON;
   pit_delay_ms(pit0,200);
   BUZZER_OFF;
   //ͣ����ʼ��
    gpio_init(H2,GPI,0);
    port_pull(H2);
    
  //����INT�ڳ�ʼ��
    gpio_init(A1,GPI,0);
    port_pull(A1);    //A1����������
    
  //��������ʼ��
    ftm_count_init(ftm0);
    gpio_init(E1,GPI,0);
    port_pull(E1);
    port_pull(E0);
 
    ftm_count_init(ftm1);
    gpio_init(H5,GPI,0);
    port_pull(E7);
    port_pull(H5);
    
    //���Զ˿�
    gpio_init(D7,GPO,0);
    
    port_pull(D7);
    
    
    
    //ADͨ����ʼ��
    adc_init(ADC0_SE13);
    adc_init(ADC0_SE7);
    adc_init(ADC0_SE6);
    adc_init(ADC0_SE14);
    adc_init(ADC0_SE12);
    adc_init(ADC0_SE5);//
    
  //���ͨ����ʼ��
    ftm_pwm_mux(ftm2,ftm_ch3);
    ftm_pwm_mux(ftm2,ftm_ch2);
    ftm_pwm_mux(ftm2,ftm_ch4);
    ftm_pwm_mux(ftm2,ftm_ch5);
    
    ftm_pwm_init(ftm2,ftm_ch2,13000,0);
    ftm_pwm_duty(ftm2,ftm_ch2,0);
  
    ftm_pwm_init(ftm2,ftm_ch3,13000,0);
    ftm_pwm_duty(ftm2,ftm_ch3,0);
    
    ftm_pwm_init(ftm2,ftm_ch4,13000,0);
    ftm_pwm_duty(ftm2,ftm_ch4,0);
    
    ftm_pwm_init(ftm2,ftm_ch5,13000,0);
    ftm_pwm_duty(ftm2,ftm_ch5,0);
    
   
}

