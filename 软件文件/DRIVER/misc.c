

#include "misc.h"


uint32_t task1_timeout=0;
uint32_t task2_timeout=0;
uint32_t task3_timeout=0;
uint32_t task4_timeout=0;
uint32_t task5_timeout=0;
uint32_t task6_timeout=0;

/**
  BMI160 VCC SWITCH
	*/
void bmi160_pwr_switch(unsigned char x) 
{
	if(x)
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);		
	}
	else
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);		
	}	
}

/**
  AD7193 VCC SWITCH
	*/
void ad7193_pwr_switch(unsigned char x)  
{
	if(x)
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);		
	}
	else
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);		
	}
}

/**
  EXTTEMP VCC SWITCH
	*/
void temp_pwr_switch(unsigned char x) 
{
	if(x)
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);		
	}
	else
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);		
	}	
}

void task_sheduler(void (*fuc)(),uint32_t task_time,uint8_t id)  
{
	switch(id)
	{
		case 0:
			break;		
		case 1: 
		if((task1_timeout+task_time)<=HAL_GetTick())
		{
			task1_timeout = HAL_GetTick();//  
			(*fuc)();
		} 						
		break;	
		case 2: 
		if((task2_timeout+task_time)<=HAL_GetTick())
		{
			task2_timeout = HAL_GetTick();//  
			(*fuc)();
		} 						
		break;		
		case 3: 
		if((task3_timeout+task_time)<=HAL_GetTick())
		{
			task3_timeout = HAL_GetTick();//  
			(*fuc)();
		}  						
		break;	
		case 4: 
		if((task4_timeout+task_time)<=HAL_GetTick())
		{
			task4_timeout = HAL_GetTick();//  
			(*fuc)();
		} 						
		break;
		case 5: 
		if((task5_timeout+task_time)<=HAL_GetTick())
		{
			task5_timeout = HAL_GetTick();//  
			(*fuc)();
		} 						
		break;	
		case 6: 
		if((task6_timeout+task_time)<=HAL_GetTick())
		{
			task6_timeout = HAL_GetTick();//  
			(*fuc)();
		} 						
		break;
	}
}

