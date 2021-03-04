


#include "spi.h"
#include "misc.h"

SPI_HandleTypeDef  hspi3; //spi3

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI3_Init(void)
{
  hspi3.Instance         = SPI3;               //配置为SPI3          
  hspi3.Init.Mode        = SPI_MODE_MASTER;    //配置为Master      
  hspi3.Init.Direction   = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize    = SPI_DATASIZE_8BIT;     //8位数据模式
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;//SCLK下降沿
  hspi3.Init.CLKPhase    = SPI_PHASE_1EDGE;
  hspi3.Init.NSS         = SPI_NSS_SOFT;             //NSS为软件控制    
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//配置SPI3分频系数 SPI最高频率有限制
  hspi3.Init.FirstBit    = SPI_FIRSTBIT_MSB;   
  hspi3.Init.TIMode      = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial  = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode  = SPI_NSS_PULSE_ENABLE;
	
  if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();
}


