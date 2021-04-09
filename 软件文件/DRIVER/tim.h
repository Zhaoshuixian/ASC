
#ifndef __TIM_H__
#define __TIM_H__

#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef    htim2;

void MX_TIM2_Init(unsigned int arr,unsigned int psc);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


#endif



