/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		模拟M_IIC
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 * @note	
					接线定义
					------------------------------------ 
						SCL                 查看SEEKFREE_M_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_M_IIC文件内的SEEKFREE_SDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/



#ifndef _MPU_IIC_h
#define _MPU_IIC_h



#include "headfile.h"



#define MPU_MSCL        H4 //A3                           //定义SCL引脚  可任意更改为其他IO
#define MPU_MSDA        H3//A2                           //定义SDA引脚  可任意更改为其他IO

typedef enum M_IIC       //DAC模块
{
    M_IIC,
   M_SCCB
} M_IIC_type;



void  M_IIC_start(void);
void  M_IIC_stop(void);
void  M_IIC_ack_main(uint8 ack_main);
void  m_send_ch(uint8 c);
uint8 m_read_ch(uint8 ack);
void  mpu_write_reg(uint8 dev_add, uint8 reg, uint8 dat);
uint8 mpu_read_reg(uint8 dev_add, uint8 reg, M_IIC_type type);
void mpu_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, M_IIC_type type);
void  M_IIC_init(void);











#endif

