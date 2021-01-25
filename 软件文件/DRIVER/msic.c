

#include "msic.h"

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
/**
   LED 
  */
void device_led_switch(unsigned char x)
{
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	if(x)
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);		
	}
	else
	{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);		
	}		
}
