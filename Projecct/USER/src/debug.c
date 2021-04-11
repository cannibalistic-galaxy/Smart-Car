#include "headfile.h"
/***************************************************************************************************************
 * @file  �ټ�����������λ�����ͳ���
 * @author hzy
 ***************************************************************************************************************/
char date[34];
void data_send(int data1,int data2,int data3,int data4,int data5,int data6)
{
  if(data1<0) data1 = 10000+abs(data1);
  if(data2<0) data2 = 10000+abs(data2);
  if(data3<0) data3 = 10000+abs(data3);
  if(data4<0) data4 = 10000+abs(data4);
  if(data5<0) data5 = 10000+abs(data5);
  if(data6<0) data6 = 10000+abs(data6);
    
  sprintf(date,"%c%c%05d%05d%05d%05d%05d%05d%c%c",'$','$',data1,data2,data3,data4,data5,data6,'#','#');
}


/***************************************************************************************************************
 * DataScope��λ�����ͳ���
 ***************************************************************************************************************/
 
//#include "DataScope_DP.h"
/*--------------------------����ʾ��������---------------------------------------------------------------------------*/
unsigned char i;          //�������� 
unsigned char DataScope_OutPut_Buffer[42] = {0};	   //���ڷ��ͻ�����
unsigned char Send_Count; //������Ҫ���͵����ݸ���
extern uint8 DataScope_SendMode;
/*-------------------------------------------------------------------------------------------------------------------*/
//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ 
//����˵�����û�����ֱ�Ӳ����˺��� 
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷��� 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷��� 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10�����0��ֱ����������ִ�к���
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//����˵�������� DataScopeV1.0 ����ȷʶ���֡��ʽ
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //֡ͷ
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6; break;   
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10; break;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; break;
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18; break;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22; break; 
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26; break;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; break;
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; break;
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38; break;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; break;
   }	 
  }
	return 0;
}



void Debug_DataScope(void)
{ 
            if(DataScope_SendMode==0)
            {
                 DataScope_Get_Channel_Data((float)(AD_Error), 1 ); //������ 1.0  д��ͨ�� 1
                 DataScope_Get_Channel_Data((float)(Error_Delta) , 2 ); //������ 2.0  д��ͨ�� 2
                 DataScope_Get_Channel_Data((float)(AD_Second_Derivative), 3 ); //������ 3.0  д��ͨ�� 3
                 DataScope_Get_Channel_Data( (float)(1000*circle_Flag), 4 ); //������ 4.0  д��ͨ�� 4
		 DataScope_Get_Channel_Data((float)(1000*zhuan), 5 ); //������ 5.0  д��ͨ�� 5
                 DataScope_Get_Channel_Data((float)(1000*Turn_Right_Flag), 6 ); //������ 6.0  д��ͨ�� 6
		 DataScope_Get_Channel_Data((float)(1000*Turn_Flag), 7 ); //������ 7.0  д��ͨ�� 7
                 DataScope_Get_Channel_Data((float)(1000*Turn_Flag2), 8 ); //������ 8.0  д��ͨ�� 8
                 DataScope_Get_Channel_Data((float)1000*Go_Out_Circle, 9 ); //������ 9.0  д��ͨ�� 9
               //  DataScope_Get_Channel_Data((float)Speed_now, 10); //������ 10.0 д��ͨ�� 10*/
	  	 Send_Count = DataScope_Data_Generate(9); //����10��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���		
		  for( i = 0 ; i < Send_Count; i++)  //ѭ������,ֱ���������   
	 	  {
                   uart_putchar(uart1,DataScope_OutPut_Buffer[i]);
		  }
              //    systick_delay_ms(50);
            }
            else if(DataScope_SendMode==1)
            {
                 DataScope_Get_Channel_Data((float)(dis_AD[0]), 1 ); //������ 1.0  д��ͨ�� 1
                 DataScope_Get_Channel_Data((float)(dis_AD[1]) ,2 ); //������ 2.0  д��ͨ�� 2
                 DataScope_Get_Channel_Data((float)(dis_AD[2]), 3 ); //������ 3.0  д��ͨ�� 3
                 DataScope_Get_Channel_Data((float)(dis_AD[3]), 4 ); //������ 4.0  д��ͨ�� 4
		 DataScope_Get_Channel_Data((float)(dis_AD[4]), 5 ); //������ 5.0  д��ͨ�� 5
                 DataScope_Get_Channel_Data((float)(dis_AD[5]), 6 ); //������ 6.0  д��ͨ�� 6
		 DataScope_Get_Channel_Data((float)(1000*circle_Flag), 7 ); //������ 7.0  д��ͨ�� 7
                 DataScope_Get_Channel_Data((float)(1000*Turn_Left_Flag), 8 ); //������ 8.0  д��ͨ�� 8
                 DataScope_Get_Channel_Data((float)1000*Turn_Flag, 9 ); //������ 9.0  д��ͨ�� 9
                 DataScope_Get_Channel_Data((float)1000*Turn_Right_Flag, 10); //������ 10.0 д��ͨ�� 10*/
	  	 Send_Count = DataScope_Data_Generate(10); //����10��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���		
		  for( i = 0 ; i < Send_Count; i++)  //ѭ������,ֱ���������   
	 	  {
                   uart_putchar(uart1,DataScope_OutPut_Buffer[i]);
		  }
            
            }
}







