
#include "includes.h"


//TM1640是从LSB起传输，不同于其他IIC设备从MSB起传输 ，所以需要转换数据位序先

unsigned char data_bitreverse(unsigned char data)
{
	static unsigned char temp_data=0;
	for(unsigned char i=0;i<8;i++)//MSB
	{
    temp_data<<=1;
		temp_data^=data&0x01;
		data>>=1;
	}
	return temp_data;
}	


//--------------标准I2C协议框架----------------------------

void iic_start(i2c_st *const i2c_me)
{
	i2c_me->sda_mode(1);
	i2c_me->sda_out(1);
	i2c_me->sck_out(1);		
	i2c_me->delay_us(4);
	i2c_me->sda_out(0);	
	i2c_me->delay_us(4);
	i2c_me->sck_out(0);	
}

void iic_stop(i2c_st *const i2c_me)
{
	i2c_me->sda_mode(1);//sda线输出
	i2c_me->sck_out(0);		
	i2c_me->sda_out(0);//STOP:when CLK is high DATA change form low to high
	i2c_me->delay_us(4);
	i2c_me->sck_out(1);		
	i2c_me->sda_out(1);//发送I2C总线结束信号
	i2c_me->delay_us(4);
}

unsigned char iic_wait_ack(i2c_st *const i2c_me)
{
	u8 ucErrTime=0;
	
	i2c_me->sda_mode(0);      //SDA设置为输入  

	i2c_me->sda_out(1);
	i2c_me->delay_us(1);		   
	i2c_me->sck_out(1);
	i2c_me->delay_us(1);		 
	while(i2c_me->sda_in())
	{
		if(ucErrTime++>250)
		{
			iic_stop(i2c_me);
			return 1;
		}
	}
	i2c_me->sck_out(0);//时钟输出0 	   
	return 0;  	
}

void iic_ack(i2c_st *const i2c_me)
{
	i2c_me->sck_out(0);
	i2c_me->sda_mode(1);
	i2c_me->sda_out(0);
	i2c_me->delay_us(2);	
	i2c_me->sck_out(1);
	i2c_me->delay_us(2);	
	i2c_me->sck_out(0);
}

void iic_nack(i2c_st *const i2c_me)
{
	i2c_me->sck_out(0);
	i2c_me->sda_mode(1);
	i2c_me->sda_out(1);
	i2c_me->delay_us(2);	
	i2c_me->sck_out(1);
	i2c_me->delay_us(2);	
	i2c_me->sck_out(0);	
}

void iic_delayms(i2c_st *const i2c_me,unsigned short int s)
{
	i2c_me->delay_ms(s);
}


void iic_sendbyte(i2c_st *const i2c_me,unsigned char tbyte)
{		
 unsigned char t,temp_tbyte=tbyte;
	if(i2c_me->trans_mode) //LSB模式
		temp_tbyte=data_bitreverse(tbyte);//位序颠倒
 //默认是MSB位开始传输
 i2c_me->sda_mode(1); 	 
 i2c_me->sck_out(0);	//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{     			
		i2c_me->sda_out((temp_tbyte&0x80)>>7);
		temp_tbyte<<=1; 	  
		i2c_me->sck_out(1);
		i2c_me->delay_us(2);	
		i2c_me->sck_out(0);		
		i2c_me->delay_us(2);	
	}	 	
}

unsigned char iic_readbyte(i2c_st *const i2c_me,unsigned char ack)
{
	unsigned char i,receive=0;
  i2c_me->sda_mode(0); 	//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		i2c_me->sck_out(0); 
		i2c_me->delay_us(2);	
		i2c_me->sck_out(1);
		receive<<=1;
		if(i2c_me->sda_in())
		{
			receive++; 
		}					
		i2c_me->delay_us(2);	
  }					 
    if (!ack) iic_nack(i2c_me);//发送nACK
    else      iic_ack(i2c_me); //发送ACK   
    return receive;	
}

void iic_init(i2c_st *const i2c_me)
{
	i2c_me->gpio_init();
	i2c_me->sck_out(1);
	i2c_me->sda_out(1);	
}
