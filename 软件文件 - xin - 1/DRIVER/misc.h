

#ifndef __MISC_H__
#define __MISC_H__

#include "stm32l4xx_hal.h"

#include "main.h"

void bmi160_pwr_switch(unsigned char x);
void ad7193_pwr_switch(unsigned char x);
void temp_pwr_switch(unsigned char x);

void tasks_create(void (*fuc)(),uint32_t task_time,uint8_t id)  ;
#endif



