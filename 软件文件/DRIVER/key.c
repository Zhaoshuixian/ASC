

#include "key.h"
#include "misc.h"


typedef struct
{
   
}key_st;

key_st key_arr[]=
{

};

unsigned char key_read(key_st *const key_me)
{
    for(unsigned char i=0;i<sizeof(key_st)/sizeof(*key_me);i++)
    {
        key_me[i].status =HAL_GPIO_ReadPin(key_me[i].gpio,key_me[i].pin);
        if(0!=key_me[i].status) break;
    }

    return i;
}

