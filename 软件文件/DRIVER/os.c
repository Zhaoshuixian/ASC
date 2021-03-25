
#include "os.h"


//伪OS

/*
***多任务调度器
*/
void tasks_os_run(task_st *const ptask,unsigned char task_num)
{
	for(unsigned char i=0;i<task_num;i++)
	{
    	if(ptask[i].attr)//非分时任务
		{
			ptask[i].task_func();
		}
		else //分时任务
		{
			if((ptask[i].timer+ptask[i].rtime)<=ptask[i].tick_func())
			{
				ptask[i].timer = ptask[i].tick_func();//  
				ptask[i].task_func();
			}
	    }
	}
}

/*
***创建信号量
*/
void sem_create(sem_st *const sem_me)
{
	sem_me->value=0;
	sem_me->timeout=0;
}

/*
***拿取信号量
*/
unsigned char sem_take(sem_st *const sem_me)
{
	if(sem_me->value)
	{
		return 1;
	}
	else
	{
		return 0;		
	}
}

/*
***等待信号量
*/
unsigned char sem_wait(sem_st *const sem_me,unsigned int timeout)
{
	sem_me->timeout=timeout;//重载等待时间
	
    if(0==sem_me->timeout) //0等待 
	{
		if(sem_me->value)
		{
		  return 1;//拿到
		}
		else
		{
		  return 0;			
		}
	}
	while(0 < sem_me->timeout)//非0等待
	{
		if(0xFFFFFFFF!=timeout)//设置最大值时，则代表永久等待
		{
			sem_me->timeout--;
		}
		if(sem_me->value) break;//拿到信号量，则立即退出
	}
	
	return 1;//拿到
}

/*
***释放信号量
*/
void sem_release(sem_st *const sem_me)
{
	__disable_irq();//临界区保护
	sem_me->value=1;
	__enable_irq();
}


