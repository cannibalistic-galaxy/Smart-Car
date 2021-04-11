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
						MSCL                 查看SEEKFREE_M_IIC文件内的MPU_MSCL宏定义
						MSDA                 查看SEEKFREE_M_IIC文件内的MPU_MSDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/



#include "MPU_IIC.h"


#define MSDA             gpio_get (MPU_MSDA)
#define MSDA0()          gpio_set (MPU_MSDA, 0)		//IO口输出低电平
#define MSDA1()          gpio_set (MPU_MSDA, 1)		//IO口输出高电平  
#define MSCL0()          gpio_set (MPU_MSCL, 0)		//IO口输出低电平
#define MSCL1()          gpio_set (MPU_MSCL, 1)		//IO口输出高电平
#define MPU_DIR_OUT()       gpio_ddr (MPU_MSDA, GPO)    //输出方向
#define MPU_DIR_IN()        gpio_ddr (MPU_MSDA, GPI)    //输入方向


//内部数据定义
uint8 M_IIC_ad_main; //器件从地址	    
uint8 M_IIC_ad_sub;  //器件子地址	   
uint8 *M_IIC_buf;    //发送|接收数据缓冲区	    
uint8 M_IIC_num;     //发送|接收数据个数	     

#define mpu_ack 1      //主应答
#define no_mpu_ack 0   //从应答	 



//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟M_IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果M_IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
void mpu_delay(void)
{
	//j=10通讯速率大约为100K （内核频率40M）
    //j=0 通讯速率大约为140K （内核频率40M）
    uint16 j=10;   
	while(j--);
}


//内部使用，用户无需调用
void M_IIC_start(void)
{
	MSDA1();
	MSCL1();
	mpu_delay();
	MSDA0();
	mpu_delay();
	MSCL0();
}

//内部使用，用户无需调用
void M_IIC_stop(void)
{
	MSDA0();
	MSCL0();
	mpu_delay();
	MSCL1();
	mpu_delay();
	MSDA1();
	mpu_delay();
}

//主应答(包含mpu_ack:MSDA=0和no_mpu_ack:MSDA=0)
//内部使用，用户无需调用
void M_I2C_SendACK(unsigned char mpu_ack_dat)
{
    MSCL0();
	mpu_delay();
	if(mpu_ack_dat) MSDA0();
    else    	MSDA1();

    MSCL1();
    mpu_delay();
    MSCL0();
    mpu_delay();
}


static int M_SCCB_WaitAck(void)
{
    MSCL0();
	MPU_DIR_IN();
	mpu_delay();
	
	MSCL1();
    mpu_delay();
	
    if(MSDA)           //应答为高电平，异常，通信失败
    {
        MPU_DIR_OUT();
        MSCL0();
        return 0;
    }
    MPU_DIR_OUT();
    MSCL0();
	mpu_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void m_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	MSDA1();//MSDA 输出数据
        else			MSDA0();
        c <<= 1;
        mpu_delay();
        MSCL1();                //MSCL 拉高，采集信号
        mpu_delay();
        MSCL0();                //MSCL 时钟线拉低
    }
	M_SCCB_WaitAck();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|M_IIC_mpu_ack_main()使用
//内部使用，用户无需调用
uint8 m_read_ch(uint8 mpu_ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    MSCL0();
    mpu_delay();
    MSDA1();             //置数据线为输入方式
    MPU_DIR_IN();
    for(i=0;i<8;i++)
    {
        mpu_delay();
        MSCL0();         //置时钟线为低，准备接收数据位
        mpu_delay();
        MSCL1();         //置时钟线为高，使数据线上数据有效
        mpu_delay();
        c<<=1;
        if(MSDA) c+=1;   //读数据位，将接收的数据存c
    }
    MPU_DIR_OUT();
	MSCL0();
	mpu_delay();
	M_I2C_SendACK(mpu_ack_x);    //主机读完发送应答信号
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟M_IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	M_IIC_start();
 //   m_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
        m_send_ch(dev_add); 
	m_send_ch( reg );   				 //发送从机寄存器地址
	m_send_ch( dat );   				 //发送需要写入的数据
	M_IIC_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟M_IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是M_IIC  还是 M_SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 mpu_read_reg(uint8 dev_add, uint8 reg, M_IIC_type type)
{
	uint8 dat;
	M_IIC_start();
  //  m_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
        m_send_ch(dev_add); 
	m_send_ch( reg );   //发送从机寄存器地址
	if(type == M_SCCB)M_IIC_stop();
	
	M_IIC_start();
//	m_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
        m_send_ch(dev_add+0x01); 
        
	dat = m_read_ch(no_mpu_ack);   				//读取数据
	M_IIC_stop();
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟M_IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是M_IIC  还是 M_SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, M_IIC_type type)
{
	M_IIC_start();
 //   m_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	m_send_ch(dev_add); 
        m_send_ch( reg );   				//发送从机寄存器地址
	if(type == M_SCCB)M_IIC_stop();
	
	M_IIC_start();
	//m_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
        m_send_ch(dev_add+0x01); 
    while(--num)
    {
        *dat_add = m_read_ch(mpu_ack); //读取数据
        dat_add++;
    }
    *dat_add = m_read_ch(no_mpu_ack); //读取数据
	M_IIC_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟M_IIC端口初始化
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void M_IIC_init(void)
{
	gpio_init (MPU_MSCL, GPO,1);
	gpio_init (MPU_MSDA, GPO,1);

	port_pull (MPU_MSCL);
	port_pull (MPU_MSDA);
}

