

#include "misc.h"

void tasks_os_run(task_st *const ptask,unsigned char task_num)
{
	for(unsigned char i=0;i<task_num;i++)
	{
		if((ptask[i].timer+ptask[i].rtime)<=ptask[i].tick_func())
		{
			ptask[i].timer = ptask[i].tick_func();//  
			ptask[i].task_func();
		}
	}
}


typedef struct
{
	unsigned char value;
	unsigned int timeout;
}sem_st;

void sem_take(sem_st *const sem_me)
{
	sem_me->value=0;
}

unsigned char sem_wait(sem_st *const sem_me,unsigned int timeout)
{
	sem_me->timeout=timeout;//等待时间
  
	while(sem_me->timeout)//
	{
		if(0xFFFFFFFF!=timeout)//设置最大值时，则代表永久等待
		{
			sem_me->timeout--;
		}
		
		if(sem_me->value) break;//拿到信号量，则立即退出
	}
	
	return 0;
}

void sem_release(sem_st *const sem_me)
{
	sem_me->value=1;
}



