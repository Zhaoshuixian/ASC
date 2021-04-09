

#include "tim.h"
#include "misc.h"


TIM_HandleTypeDef    htim2;
/*
初始化：Tout=(ARR+1)(PSC+1)/TIMxCLK (us)。ARR代表预分频，PSC代表自动装载值。
*/
void MX_TIM2_Init(unsigned int arr,unsigned int psc)
{
  htim2.Instance = TIM2;                        // 配置为TIM2
  htim2.Init.Prescaler = arr-1;                 // 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;  // 向上计数
  htim2.Init.Period = psc-1;                  
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // 不分频，
  
  // 初始化TIM2，出错则进入错误处理函数
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  Error_Handler();    
   HAL_TIM_Base_Start_IT(&htim2);      // 开启TIM2，并且使能中断
}

/*
中断配置
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();            //使能定时器TIM2
    HAL_NVIC_SetPriority(TIM2_IRQn,10,0);   //设置中断优先级10，子优先级0--自行按需修改
    HAL_NVIC_EnableIRQ(TIM2_IRQn);          //使能ITM3中断
  }
}

/*
回调
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static unsigned int cnt=0;
  if(htim == (&htim2))
  {
		cnt++;
		if(0==cnt%50)
		{
			cnt=0;
		  HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
		}
    //ADD USER CODE...
  }
}



