

#include "wtd.h"
#include "misc.h"

IWDG_HandleTypeDef hiwdg; //iwdg

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
 /*
 - LSI(32KHz)为WDG的时钟源 -
 配置IWDG 32分频（也就是32KHz /32 = 1KHz = 1us），
 配置重载计数值为1000（也就是装载满一次需要1000*1us = 1s）。
 或者大家使用公式计算：Tout=(4*(2^prer)*rlr)/32 (ms)。此例程prer = 3，rlr = 1000。
 */
void MX_IWDG_Init(void)
{
  hiwdg.Instance       = IWDG; // 配置为IWDG
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;// 32分频，prer = 3
  hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload    = 1000;
	
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

/*
**独立看门狗喂狗函数，也就是清除计数值。
*/
void IWDG_Feed(void)
{
  HAL_IWDG_Refresh(&hiwdg); 	// 喂狗
}






