

#ifndef __MSIC_H__
#define __MSIC_H__

#include "stm32l4xx_hal.h"

void bmi160_pwr_switch(unsigned char x);
void ad7193_pwr_switch(unsigned char x);
void temp_pwr_switch(unsigned char x);
void device_led_switch(unsigned char x);


#endif



