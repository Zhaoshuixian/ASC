

#include "uart.h"
#include "misc.h"

UART_HandleTypeDef huart1; //usart1
UART_HandleTypeDef huart2; //usart2
DMA_HandleTypeDef  hdma_usart1_rx;//usart1_dma
DMA_HandleTypeDef  hdma_usart2_rx;//usart2_dma

uart_st uart1,uart2;

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{
  huart1.Instance            = USART1; //
  huart1.Init.BaudRate       = 115200;
  huart1.Init.WordLength     = UART_WORDLENGTH_8B;
  huart1.Init.StopBits       = UART_STOPBITS_1;
  huart1.Init.Parity         = UART_PARITY_NONE;
  huart1.Init.Mode           = UART_MODE_TX_RX;//收发模式
  huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling   = UART_OVERSAMPLING_16; //16Bit过采样
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // 1位过采样禁能
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;// 没有串口高级功能初始化
	
	
  if(HAL_UART_Init(&huart1) != HAL_OK)  Error_Handler();
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//使能空闲中断
	HAL_UART_Receive_DMA(&huart1,uart1.rx_buff,UART_BUFF_SIZE);//打开DMA接收，数据缓存至数组缓存区

	__HAL_UART_CLEAR_FLAG(&huart1,UART_CLEAR_IDLEF);//清除标志位
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{
  huart2.Instance        = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
  if (HAL_UART_Init(&huart2) != HAL_OK)  Error_Handler();
	
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);//使能IDEL中断	
	HAL_UART_Receive_IT(&huart2,(uint8_t*)&uart2.rx_buff,UART_BUFF_SIZE);//打开DMA接收，数据缓存至数组缓存区
	__HAL_UART_CLEAR_FLAG(&huart2,UART_CLEAR_IDLEF);//清除标志位

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 3);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/*
***获取DMA接收数据长度
*/
static unsigned int uart_dma_recvlen(UART_HandleTypeDef *huart)
{
	return (UART_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx));//获取接收数据长度 		
}

/*
***重置DMA配置
*/
static void uart_dma_reset(UART_HandleTypeDef *huart)
{
  #define __HAL_DMA_SET_COUNTER(__HANDLE__,reload_value) ((__HANDLE__)->Instance->CNDTR=reload_value)  	
	
	__HAL_DMA_DISABLE(huart->hdmarx);//关闭DMA
	__HAL_DMA_SET_COUNTER(huart->hdmarx,UART_BUFF_SIZE);//重装DMA数值	
	__HAL_DMA_ENABLE(huart->hdmarx);//使能DMA	 	
}
/*
***串口1/2接收任务
*/
static void uart_rx_task(UART_HandleTypeDef *huart)
{
  if(USART1==huart->Instance)//UART1
  {
    if(uart1.rx_frame_flag)//帧标志
    {
      uart1.rx_frame_flag=0;
      uart1.rx_len=uart_dma_recvlen(&huart1);
			#ifdef DEBUG_UART1
      UART1_LOG("Recv size:%d\r\n",uart1.rx_len);
      UART1_LOG("Recv data:%s\r\n",uart1.rx_buff);
			#endif			
      uart_dma_reset(&huart1);
      memset(uart1.rx_buff,0,sizeof(uart1.rx_buff));  
    }
  }
  else if(USART2==huart->Instance)//UART2
  {
    if(uart2.rx_frame_flag)
    {
      uart2.rx_frame_flag=0;      
      uart2.rx_len=uart_dma_recvlen(&huart1);
			#ifdef DEBUG_UART2
      UART2_LOG("Recv size:%d\r\n",uart2.rx_len);
      UART2_LOG("Recv data:%s\r\n",uart2.rx_buff);
			#endif
      uart_dma_reset(&huart2);
      memset(uart2.rx_buff,0,sizeof(uart2.rx_buff));     
    }
  }
}

/*
***双串口处理
*/
void device_uart_handle(void)
{
  uart_rx_task(&huart1);
  uart_rx_task(&huart2);
}

/*
***串口数据发送
*/
void uart_tx_data(UART_HandleTypeDef *huart,unsigned char *tdata,unsigned int tzise)
{
  HAL_UART_Transmit(huart, (uint8_t *)&tdata, tzise, 0xffff);
}

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

