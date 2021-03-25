

#include "flash.h"

/*开辟个人用户FLASH存储空间 8K*/
#define FLASH_USER_START_ADDR   (ADDR_FLASH_PAGE_124)                      /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_124+FLASH_PAGE_SIZE - 1)  /* End @ of user Flash area */

/**
  * @brief  Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t stm32_flash_init(void)
{
  /* Unlock the internal flash */
  HAL_FLASH_Unlock();

  return FLASH_OK;
}

/**
  * @brief  De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t stm32_flash_deinit(void)
{
  /* Lock the internal flash */
  HAL_FLASH_Lock();

  return FLASH_OK;
}

/**
  * @brief  Gets the page of a given address
  * @param  rd_addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
unsigned int stm32_flash_page_get(unsigned int rd_addr)
{
  unsigned int page = 0;

  if (rd_addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (rd_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (rd_addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

#if 0
/**
  * @brief  Gets the bank of a given address
  * @param  rd_addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
unsigned int stm32_flash_bank_get(unsigned int rd_addr)
{
  unsigned int bank = 0;

  if (0== READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE))
  {
    /* No Bank swap */
    (rd_addr < (FLASH_BASE + FLASH_BANK_SIZE))?(bank = FLASH_BANK_1):(bank = FLASH_BANK_2);
  }
  else
  {
    /* Bank swap */
    (rd_addr < (FLASH_BASE + FLASH_BANK_SIZE))?(bank = FLASH_BANK_2):(bank = FLASH_BANK_1)
  }

  return bank;
}
#endif

/**
  * @brief  Erases sector.
  * @param  er_addr: Address of sector to be erased.
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
unsigned char stm32_flash_erase(unsigned int er_addr)
{
  unsigned int PageError = 0;

  FLASH_EraseInitTypeDef eraseinitstruct;

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

   /* Get the number of sector to erase from 1st sector*/
  eraseinitstruct.Banks     = FLASH_BANK_1;//for L432
  //eraseinitstruct.Banks     = stm32_flash_bank_get(er_addr);//for L476
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.Page      = stm32_flash_page_get(er_addr);

  if (HAL_FLASHEx_Erase(&eraseinitstruct, &PageError) != HAL_OK)  return FLASH_ERR;

  return FLASH_OK;
}

/**
  * @brief  Writes Data into Memory.
  * @param  wr_buff: Pointer to the source buffer. Address to be written to.
  * @param  wr_addr: Pointer to the destination buffer.
  * @param  wr_len: Number of data to be written (in bytes).
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
unsigned char stm32_flash_write(unsigned char *wr_buff, unsigned int wr_addr, unsigned int wr_len)
{
  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  for(unsigned int i = 0; i < wr_len; i += 8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by byte */
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (unsigned int)(wr_addr+i), *(uint64_t*)(wr_buff+i)) == HAL_OK)
    {
     /* Check the written value */
      if(*(uint64_t *)(wr_buff + i) != *(uint64_t*)(wr_addr+i))
      {
        /* Flash content doesn't match SRAM content */
        return FLASH_CHECK_ERR;
      }
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return FLASH_ERR;
    }
  }
  return FLASH_OK;
}

/**
  * @brief  Reads Data into Memory.
  * @param  rd_buff: Pointer to the source buffer. Address to be written to.
  * @param  rd_addr: Pointer to the destination buffer.
  * @param  rd_len: Number of data to be read (in bytes).
  * @retval return FLASH_OK.
  */
flash_status_t stm32_flash_read(unsigned char* rd_buff, unsigned int rd_addr, unsigned int rd_len)
{
    for(unsigned int i = 0; i < rd_len; i++)
	{
        rd_buff[i] = *(__IO unsigned char*)(rd_addr + i);
    }
	
    /* Return a valid address to avoid HardFault */
    return FLASH_OK;
}


 






