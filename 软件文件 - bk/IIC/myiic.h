
#ifndef __MYIIC_H
#define __MYIIC_H

#include "../SYSTEM/sys.h" 
		   

//----------i2c对接接口函数指针-------
typedef struct
{
	unsigned char trans_mode;//传输模式，0:默认是从MSB开始传输（通用型），1：从LSB开始传输（特别）
	void (*gpio_init)(void);
	void (*sda_mode)(unsigned char );	
	unsigned char  (*sda_in)(void);
	void (*sda_out)(unsigned char );
	void (*sck_out)(unsigned char );
	void (*delay_us)(unsigned short int);
	void (*delay_ms)(unsigned short int);	
}i2c_st;

//-----------i2c标准接口---------
void iic_start(i2c_st *const i2c_me);
void iic_stop(i2c_st *const i2c_me);
unsigned char iic_wait_ack(i2c_st *const i2c_me);
void iic_ack(i2c_st *const i2c_me);
void iic_nack(i2c_st *const i2c_me);
void iic_delayms(i2c_st *const i2c_me,unsigned short int s);
void iic_sendbyte(i2c_st *const i2c_me,unsigned char tbyte);
unsigned char iic_readbyte(i2c_st *const i2c_me,unsigned char ack);
void iic_init(i2c_st *const i2c_me);
//-----------------END-----------------
#endif

