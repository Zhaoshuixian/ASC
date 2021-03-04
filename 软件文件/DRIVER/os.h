

#ifndef __OS_H__
#define __OS_H__


typedef unsigned int (*ptick)(void);
typedef void     (*ptask)(void);

typedef struct 
{
   unsigned char id;  //任务ID
	 unsigned char attr;//任务属性 0:分时 | 1:不分时
   unsigned int  timer;//任务超时计时器
   unsigned int  rtime;//运行时间
   ptick tick_func;
   ptask task_func;
}task_st;


/*
***信号量定义
*/
typedef struct
{
	unsigned char value;
	unsigned int timeout;
}sem_st;

void sem_create(sem_st *const sem_me);
unsigned char sem_take(sem_st *const sem_me);
unsigned char sem_wait(sem_st *const sem_me,unsigned int timeout);
void sem_release(sem_st *const sem_me);


void tasks_os_run(task_st *const ptask,unsigned char task_num);



#endif


