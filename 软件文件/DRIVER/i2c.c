
#include "i2c.h"
#include "misc.h"

I2C_HandleTypeDef  hi2c1; //i2c1

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{
  hi2c1.Instance              = I2C1;	
  hi2c1.Init.Timing           = 0x10D05E82;
  hi2c1.Init.OwnAddress1      = 0x0; //设置I2C从机地址
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;//7位地址模式
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;//非双寻址模式
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	//配置初始化
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  //模拟滤波配置
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)  Error_Handler();
  //数字滤波配置
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}
