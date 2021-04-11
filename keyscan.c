/***************************************************
 * ģʽ��д������
 * ����ɨ��ûд
 ***************************************************/
#include "keyscan.h"
#include "headfile.h"
#define sw1 A0
#define sw2 C7
#define sw3 C6
#define sw4 I3
uint8 sw1_status;
uint8 sw2_status;
uint8 sw3_status;
uint8 sw4_status;

int8 key_number;
uint8 key_value;
uint8 bomamode;


void boma_init(void)
{
  gpio_init(sw1,GPI,0);
  gpio_init(sw2,GPI,0);
  gpio_init(sw3,GPI,0);
  gpio_init(sw4,GPI,0);
  port_pull(sw1);
  port_pull(sw2);
  port_pull(sw3);
  port_pull(sw4);
}
/***************************************************************************
 * ˵�������뿪�ز����ұ���1�����������0����������Ϊsw1--4
 * ��ʼ״̬:1111
 ***************************************************************************/
void boma_mode(void)
{
  sw1_status = gpio_get(sw1);
  sw2_status = gpio_get(sw2);
  sw3_status = gpio_get(sw3);
  sw4_status = gpio_get(sw4);
  if(sw1_status==0&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //ģʽ0 0000
    bomamode=0;
  if(sw1_status==0&&sw2_status==0&&sw3_status==0&&sw4_status==1)  //ģʽ1 0001
    bomamode=1;
  if(sw1_status==0&&sw2_status==0&&sw3_status==1&&sw4_status==0)  //ģʽ2 0010
    bomamode=2;
  if(sw1_status==0&&sw2_status==0&&sw3_status==1&&sw4_status==1)  //ģʽ3 0011
    bomamode=3;
  if(sw1_status==0&&sw2_status==1&&sw3_status==0&&sw4_status==0)  //ģʽ4 0100
    bomamode=4;
  if(sw1_status==0&&sw2_status==1&&sw3_status==0&&sw4_status==1)  //ģʽ5 0101
    bomamode=5;
  if(sw1_status==0&&sw2_status==1&&sw3_status==1&&sw4_status==0)  //ģʽ6 0110
    bomamode=6;
  if(sw1_status==0&&sw2_status==1&&sw3_status==1&&sw4_status==1)  //ģʽ7 0111
    bomamode=7;
  if(sw1_status==1&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //ģʽ8 1000
    bomamode=8;
  if(sw1_status==1&&sw2_status==0&&sw3_status==0&&sw4_status==1)  //ģʽ9 1001
    bomamode=9;
  if(sw1_status==1&&sw2_status==0&&sw3_status==1&&sw4_status==0)  //ģʽ10 1010
    bomamode=10;
  if(sw1_status==1&&sw2_status==0&&sw3_status==1&&sw4_status==1)  //ģʽ11 1011
    bomamode=11;
  if(sw1_status==1&&sw2_status==1&&sw3_status==0&&sw4_status==0)  //ģʽ12 1100
    bomamode=12;
  if(sw1_status==1&&sw2_status==1&&sw3_status==0&&sw4_status==1)  //ģʽ13 1101
    bomamode=13;
  if(sw1_status==1&&sw2_status==1&&sw3_status==1&&sw4_status==0)  //ģʽ14 1110
    bomamode=14;
  if(sw1_status==1&&sw2_status==1&&sw3_status==1&&sw4_status==1)  //ģʽ15 1111
    bomamode=15;
}


