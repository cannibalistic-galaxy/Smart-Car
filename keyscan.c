/***************************************************
 * 模式刚写了三个
 * 键盘扫描没写
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
 * 说明：拨码开关拨向右边是1，拨向左边是0；从上往下为sw1--4
 * 初始状态:1111
 ***************************************************************************/
void boma_mode(void)
{
  sw1_status = gpio_get(sw1);
  sw2_status = gpio_get(sw2);
  sw3_status = gpio_get(sw3);
  sw4_status = gpio_get(sw4);
  if(sw1_status==0&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //模式0 0000
    bomamode=0;
  if(sw1_status==0&&sw2_status==0&&sw3_status==0&&sw4_status==1)  //模式1 0001
    bomamode=1;
  if(sw1_status==0&&sw2_status==0&&sw3_status==1&&sw4_status==0)  //模式2 0010
    bomamode=2;
  if(sw1_status==0&&sw2_status==0&&sw3_status==1&&sw4_status==1)  //模式3 0011
    bomamode=3;
  if(sw1_status==0&&sw2_status==1&&sw3_status==0&&sw4_status==0)  //模式4 0100
    bomamode=4;
  if(sw1_status==0&&sw2_status==1&&sw3_status==0&&sw4_status==1)  //模式5 0101
    bomamode=5;
  if(sw1_status==0&&sw2_status==1&&sw3_status==1&&sw4_status==0)  //模式6 0110
    bomamode=6;
  if(sw1_status==0&&sw2_status==1&&sw3_status==1&&sw4_status==1)  //模式7 0111
    bomamode=7;
  if(sw1_status==1&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //模式8 1000
    bomamode=8;
  if(sw1_status==1&&sw2_status==0&&sw3_status==0&&sw4_status==1)  //模式9 1001
    bomamode=9;
  if(sw1_status==1&&sw2_status==0&&sw3_status==1&&sw4_status==0)  //模式10 1010
    bomamode=10;
  if(sw1_status==1&&sw2_status==0&&sw3_status==1&&sw4_status==1)  //模式11 1011
    bomamode=11;
  if(sw1_status==1&&sw2_status==1&&sw3_status==0&&sw4_status==0)  //模式12 1100
    bomamode=12;
  if(sw1_status==1&&sw2_status==1&&sw3_status==0&&sw4_status==1)  //模式13 1101
    bomamode=13;
  if(sw1_status==1&&sw2_status==1&&sw3_status==1&&sw4_status==0)  //模式14 1110
    bomamode=14;
  if(sw1_status==1&&sw2_status==1&&sw3_status==1&&sw4_status==1)  //模式15 1111
    bomamode=15;
}


