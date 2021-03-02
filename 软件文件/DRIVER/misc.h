

#ifndef __MISC_H__
#define __MISC_H__

#include "stm32l4xx_hal.h"
#include "main.h"

typedef void     (*ptask)(void);
typedef uint32_t (*ptick)(void);

typedef struct 
{
   uint8_t id;
   uint32_t timer;//任务超时计时器
   uint32_t rtime;//运行时间
   ptick tick_func;
   ptask task_func;
}task_st;


void tasks_os_run(task_st *const ptask,unsigned char task_num);
#endif



