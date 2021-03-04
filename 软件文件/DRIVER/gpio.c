

#include "gpio.h"
#include "misc.h"


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); 
	
  /*Configure GPIO pin : NC_1_Pin */
  GPIO_InitStruct.Pin  = NC_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_1_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pin : NC_2_Pin */
  GPIO_InitStruct.Pin  = NC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEI_SIN_Pin */
  GPIO_InitStruct.Pin  = KEY_SIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_SIN_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pins : J1_1_Pin /J1_2_Pin /J1_4_Pin /J1_5_Pin */
  GPIO_InitStruct.Pin   = J1_1_Pin|J1_2_Pin|J1_3_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : */
  GPIO_InitStruct.Pin   = AD7193_VDD_SWITCH_Pin|TEMPER_VDD_SWITCH_Pin|EXT_VDD_SWITCH_Pin|BMI160_VDD_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMPER_SIN_Pin /DEV_LED_Pin */
  GPIO_InitStruct.Pin   = TEMPER_SIN_Pin|DEV_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
 /*Configure GPIO pins :  AD7193_CS_Pin */	
  GPIO_InitStruct.Pin   = AD7193_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin : EXT_TRIG_Pin */
  GPIO_InitStruct.Pin  = EXT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXT_TRIG_GPIO_Port, &GPIO_InitStruct);

  //外部中断线配置
  HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);//EXT_TRIG
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);//KEY
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  //配置引脚上电初始端口电平状态
  HAL_GPIO_WritePin(GPIOA, J1_1_Pin|J1_2_Pin|J1_3_Pin|TEMPER_SIN_Pin ,GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, AD7193_VDD_SWITCH_Pin|DEV_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, TEMPER_VDD_SWITCH_Pin, GPIO_PIN_SET); //ADC_VTEMP_VDD_SWITCH
	HAL_GPIO_WritePin(GPIOA, BMI160_VDD_SWITCH_Pin, GPIO_PIN_SET); //PA5
	HAL_GPIO_WritePin(GPIOA, EXT_VDD_SWITCH_Pin,    GPIO_PIN_SET); //EXT_VDD_SWITCH
	HAL_GPIO_WritePin(GPIOA, AD7193_CS_Pin, GPIO_PIN_SET); //PA15
}


/*
***GPIO外部中断回调函数
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //判断进入中断的GPIOs
  if(EXT_TRIG_Pin == GPIO_Pin)//外部触发引脚
  {
    //SleepMode_Enter_Flag=0;//退出SLEEP MODE 
  }
  else if(KEY_SIN_Pin == GPIO_Pin)//按键引脚
  {
    //SleepMode_Enter_Flag=0;//退出SLEEP MODE 
  }
}



