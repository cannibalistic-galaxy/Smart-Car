/***************************************************
 * 模式刚写了三个
 * 键盘扫描没写
 ***************************************************/
#include "keyscan.h"
#include "headfile.h"

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

int boma_mode(void)
{
  sw1_status = gpio_get(sw1);
  sw2_status = gpio_get(sw2);
  sw3_status = gpio_get(sw3);
  sw4_status = gpio_get(sw4);
  if(sw1_status==0&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //模式0
    return (0);
  if(sw1_status==1&&sw2_status==0&&sw3_status==0&&sw4_status==0)  //模式1
    return (1);
  if(sw1_status==0&&sw2_status==1&&sw3_status==0&&sw4_status==0)  //模式2
    return (2);
}

void keyscan(void);
