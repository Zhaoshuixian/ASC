

#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32l4xx_hal.h"


#define EEPROM_START_ADDRESS     ((uint32_t)0x0803E000) /* EEPROM start address in Flash */
#define FLASH_BASE_ADDRESS       ((uint32_t)0x08000000)  /* the start address of the chip */
#define FLASH_ALL_SIZE           (1024*256)//256KB (0-0x3FFFF)



#define  DOUBLE_2WORD (0) //采用4字节模式写入

#if DOUBLE_2WORD
void flash_read(unsigned int ReadAddress, unsigned int *pBuffer, unsigned  int NumToRead);
void flash_write(unsigned int WriteAddress, unsigned int *pBuffer, unsigned int NumToWrite);
#else
void flash_read(unsigned int ReadAddress, unsigned char *pBuffer, unsigned  int NumToRead);
void flash_write(unsigned int WriteAddress, unsigned char *pBuffer, unsigned int NumToWrite);
#endif



#endif


