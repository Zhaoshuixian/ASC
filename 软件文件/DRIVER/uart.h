
#ifndef  __UART_H__
#define  __UART_H__

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define UART_BUFF_SIZE  (512) //串口1接收最大缓存字节数
typedef struct 
{
  unsigned char rx_buff[UART_BUFF_SIZE];//接收缓存区 
  unsigned int  rx_len; //接收长度
  unsigned char rx_frame_flag;
}uart_st;

extern UART_HandleTypeDef huart1; //usart1
extern UART_HandleTypeDef huart2; //usart2
extern uart_st uart1,uart2;

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_DMA_Init(void);
void device_uart_handle(void);
void device_uart_handle(void);


#endif


