

#include "flash.h"
#include "string.h"
#include "misc.h"

#if DOUBLE_2WORD
#define MAX_SIZE  (FLASH_PAGE_SIZE/4) //设置缓存大小 4byte元
unsigned int pageBuffer[MAX_SIZE];//开辟缓存区 4*MAX_SIZE(byte)
#else
#define MAX_SIZE  (FLASH_PAGE_SIZE) //设置缓存大小 byte元
unsigned char pageBuffer[MAX_SIZE];//开辟缓存区
#endif
/**
  * @brief  Gets the page of a given address
  * @param  rd_addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static unsigned int flash_page_get(unsigned int rd_faddr)
{
  unsigned int page = 0;

  if (rd_faddr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (rd_faddr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (rd_faddr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Erases sector.
  * @param  er_addr: Address of sector to be erased.
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
static unsigned char FLASH_ErasePage(unsigned int er_faddr)
{
  unsigned int PageError = 0;

  FLASH_EraseInitTypeDef eraseinitstruct;

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

   /* Get the number of sector to erase from 1st sector*/
  eraseinitstruct.Banks     = FLASH_BANK_1;//for L432
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.Page      = flash_page_get(er_faddr);
  eraseinitstruct.NbPages   = 1;
  if (HAL_FLASHEx_Erase(&eraseinitstruct, &PageError) != HAL_OK)  return 1;

  return 0;
}

#if DOUBLE_2WORD
/*
***读FALSH特定地址上的数据，必须严格按照按4字节对齐
*/
static unsigned int FLASH_ReadWord(unsigned int ReadAddress)
{
	return *(__IO unsigned int*)ReadAddress; 	
}
#else
/*
***读FALSH特定地址上的数据，必须严格按照按1字节对齐
*/
static unsigned char FLASH_ReadByte(unsigned int ReadAddress)
{
	return *(__IO unsigned char*)ReadAddress; 
}
#endif

#if DOUBLE_2WORD
/*
***写FLASH数据
*/
static void FLASH_WriteNoCheck(unsigned int WriteAddress,unsigned int *pBuffer,unsigned int NumToWrite)   
{ 
	uint64_t doubledata=0;
	for(unsigned int i=0;i<NumToWrite/4;i++)
	{  
		doubledata=pBuffer[i*2];
		doubledata|=((uint64_t)pBuffer[i*2+1]<<32);			//合并为8字节数据写入
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddress+i*8,(uint64_t)pBuffer[i]);//强转为uint64_t	
	  #ifdef DEBUG_FLASH		
		FLASH_LOG("w-Addr:%#X -> pBuffer:%#X\r\n",WriteAddress+i*8,doubledata);		
		#endif		
	}  
} 
#else
/*
***写FLASH数据
*/
static void FLASH_WriteNoCheck(unsigned int WriteAddress,unsigned char *pBuffer,unsigned int NumToWrite)   
{ 
	uint64_t doubledata=0;
	
	unsigned char temp_buff[8]={0};
	unsigned int i=0;
	
	if(0==NumToWrite%8)//写入的字节正好是8的整数倍
	{
		for(i=0;i<NumToWrite/8;i++)//以为写入是按照8字节写入，对于unsigned int *pBuffer，在下面地址偏移次数计算时需要/2
		{  
			doubledata=pBuffer[i*8];
			doubledata|=((uint64_t)pBuffer[i*8+1]<<8);
			doubledata|=((uint64_t)pBuffer[i*8+2]<<16);
			doubledata|=((uint64_t)pBuffer[i*8+3]<<24);	
			doubledata|=((uint64_t)pBuffer[i*8+4]<<32);			
			doubledata|=((uint64_t)pBuffer[i*8+5]<<40);	
			doubledata|=((uint64_t)pBuffer[i*8+6]<<48);	
			doubledata|=((uint64_t)pBuffer[i*8+7]<<56);	
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddress+i*8,(uint64_t)doubledata);////地址按8个量偏移.	
			#ifdef DEBUG_FLASH		
			FLASH_LOG("w-Addr:%#X -> pBuffer:%#X\r\n",WriteAddress+i*8,doubledata);		
			#endif					
		}			
	}
	else//非8的整数倍数
	{
		if(8<NumToWrite)
		//先完成8个字节倍数的数据整合写入
		for(i=0;i<NumToWrite/8;i++)//以为写入是按照8字节写入，对于unsigned int *pBuffer，在下面地址偏移次数计算时需要/2
		{  
			doubledata=pBuffer[i*8];
			doubledata|=((uint64_t)pBuffer[i*8+1]<<8);
			doubledata|=((uint64_t)pBuffer[i*8+2]<<16);
			doubledata|=((uint64_t)pBuffer[i*8+3]<<24);	
			doubledata|=((uint64_t)pBuffer[i*8+4]<<32);			
			doubledata|=((uint64_t)pBuffer[i*8+5]<<40);	
			doubledata|=((uint64_t)pBuffer[i*8+6]<<48);	
			doubledata|=((uint64_t)pBuffer[i*8+7]<<56);	
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddress+i*8,(uint64_t)doubledata);////地址按8个量偏移.	
			#ifdef DEBUG_FLASH		
			FLASH_LOG("w-Addr:%#X -> pBuffer:%#X\r\n",WriteAddress+i*8,doubledata);		
			#endif					
		}
		//再整合余下的不足8个字节的数据，差缺部分用0xFF填充
		memset(&temp_buff[0],0xFF,8);//
		memcpy(&temp_buff[0],&pBuffer[i*8],NumToWrite%8);

		doubledata=temp_buff[0];
		doubledata|=((uint64_t)temp_buff[1]<<8);
		doubledata|=((uint64_t)temp_buff[2]<<16);
		doubledata|=((uint64_t)temp_buff[3]<<24);	
		doubledata|=((uint64_t)temp_buff[4]<<32);			
		doubledata|=((uint64_t)temp_buff[5]<<40);	
		doubledata|=((uint64_t)temp_buff[6]<<48);	
		doubledata|=((uint64_t)temp_buff[7]<<56);	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddress+i*8,(uint64_t)doubledata);////地址按8个量偏移.	
		#ifdef DEBUG_FLASH		
		FLASH_LOG("w-Addr:%#X -> pBuffer:%#X\r\n",WriteAddress+i*8,doubledata);			
		#endif				
	}


} 
#endif


#if DOUBLE_2WORD
/*
***读FLASH数据
*/
void flash_read(unsigned int ReadAddress, unsigned int *pBuffer, unsigned int NumToRead)
{
	for(unsigned int i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadWord(ReadAddress);
	  #ifdef DEBUG_FLASH		
		FLASH_LOG("r-Addr:%#X -> pBuffer[%d]:%#X\r\n",ReadAddress,i,pBuffer[i]);		
		#endif
		ReadAddress+=4;	//地址按4个量偏移.
	} 
}
#else
/*
***读FLASH数据
*/
void flash_read(unsigned int ReadAddress, unsigned char *pBuffer, unsigned int NumToRead)
{
	for(unsigned int i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadByte(ReadAddress+i);	//地址按1个量偏移.
	  #ifdef DEBUG_FLASH		
		FLASH_LOG("r-Addr:%#X -> pBuffer[%d]:%#X\r\n",ReadAddress+i,i,pBuffer[i]);		
		#endif
	} 
}
#endif

#if DOUBLE_2WORD
void flash_write(unsigned int WriteAddress, unsigned int *pBuffer, unsigned int NumToWrite)
{
  unsigned int pagePos;	       //扇区地址
	unsigned int pageOffset;	   //扇区内偏移地址
	unsigned int pageRemainSize; //扇区内剩余尺寸 
 	unsigned int i;    
	unsigned int offaddr;       //去掉0x08000000后的地址
    
	if(WriteAddress < FLASH_BASE_ADDRESS || WriteAddress >=(FLASH_BASE_ADDRESS + FLASH_ALL_SIZE ))//操作地址不在flash地址范围内
	{
			return;
	}
	/* 解锁FLASH */
	HAL_FLASH_Unlock();
	
	offaddr          = WriteAddress-FLASH_BASE_ADDRESS;  //实际偏移地址.
	pagePos          = offaddr/FLASH_PAGE_SIZE;			     //扇区地址  	
	pageOffset       = (offaddr%FLASH_PAGE_SIZE)/4;		   //在扇区内的偏移(4个字节为基本单位.)	
  pageRemainSize   = FLASH_PAGE_SIZE/4-pageOffset;		 //扇区剩余空间大小 
  
	if(NumToWrite<=pageRemainSize)
	{
		pageRemainSize =NumToWrite;//不大于该扇区范围	
	}
	while(1) 
	{	
		//按4字节地址偏移量读取
		flash_read(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS,pageBuffer,FLASH_PAGE_SIZE/4);//读出整个扇区的内容
		for(i=0;i<pageRemainSize;i++)//校验数据
		{
			//检查是否有需要擦除的数据				
			if(pageBuffer[pageOffset+i]!=0xFFFFFFFF)
			{
				#ifdef DEBUG_FLASH
				FLASH_LOG("SOME DATAS NEED TO ERASE!\r\n");
				FLASH_LOG("ERASE ADDR:%d\r\n",pageOffset+i);
				#endif				
				break;//需要擦除 
			}				
		}  
		if(i<pageRemainSize)//需要擦除
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO ERASE!\r\n");
			#endif			
			FLASH_ErasePage(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS);//擦除这个扇区
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO COPY!\r\n");
			#endif				
			for(i=0;i<pageRemainSize;i++)//复制
			{
				//将待欲写入的数据续存在之前的数据之后
				pageBuffer[i+pageOffset]=pBuffer[i];//保留页偏移地址以前的数据	 
			}
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO WRITE!\r\n");
			#endif					
			//按照8字节的偏移量写入
			FLASH_WriteNoCheck(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS,pageBuffer,FLASH_PAGE_SIZE/4);//写入整个扇区  
		}
		else //不需要擦除
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("NONE TO ERASE!\r\n");
			#endif
      //按照8字节的偏移量写入			
			FLASH_WriteNoCheck(WriteAddress,pBuffer,pageRemainSize);//写已经擦除了的,直接写入扇区剩余区间. 		
		}
		if(NumToWrite==pageRemainSize)
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("WRITE OVER !\r\n");
			#endif				
			break;//写入结束了
		}
		else//写入未结束
		{
			pagePos++;				//扇区地址增1
			pageOffset       = 0;				         //偏移位置为0 	 
			pBuffer         += pageRemainSize;  	//指针偏移
			WriteAddress    += pageRemainSize*4;	//写地址偏移	   
			NumToWrite      -= pageRemainSize;	  //字节数递减
			if(NumToWrite>(FLASH_PAGE_SIZE/4))
			{
					pageRemainSize=FLASH_PAGE_SIZE/4;//下一个扇区还是写不完
					#ifdef DEBUG_FLASH
					FLASH_LOG("WC-> pageRemainSize:%#X\r\n",pageRemainSize);
					#endif					
			}
      else 
			{
					pageRemainSize=NumToWrite;//下一个扇区可以写完了
					#ifdef DEBUG_FLASH
					FLASH_LOG("WC->WRITE OVER !\r\n");
					#endif					
			}
		}	 
	}
	/* 上锁FLASH */
	HAL_FLASH_Lock();
}

#else
void flash_write(unsigned int WriteAddress, unsigned char *pBuffer, unsigned int NumToWrite)
{
  unsigned int pagePos;	       //扇区地址
	unsigned int pageOffset;	   //扇区内偏移地址
	unsigned int pageRemainSize; //扇区内剩余尺寸 
 	unsigned int i;    
	unsigned int offaddr;       //去掉0x08000000后的地址
    
	if(WriteAddress < FLASH_BASE_ADDRESS || WriteAddress >=(FLASH_BASE_ADDRESS + FLASH_ALL_SIZE ))//操作地址不在flash地址范围内
	{
			return;
	}
	/* 解锁FLASH */
	HAL_FLASH_Unlock();
	
	offaddr          = WriteAddress-FLASH_BASE_ADDRESS;  //实际偏移地址.
	pagePos          = offaddr/FLASH_PAGE_SIZE;			     //扇区地址  	
	pageOffset       = (offaddr%FLASH_PAGE_SIZE);		   //在扇区内的偏移
  pageRemainSize   = FLASH_PAGE_SIZE-pageOffset;		 //扇区剩余空间大小 
  
	if(NumToWrite<=pageRemainSize)
	{
		pageRemainSize =NumToWrite;//不大于该扇区范围	
	}
	while(1) 
	{	
		//按4字节地址偏移量读取
		flash_read(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS,pageBuffer,sizeof(pageBuffer)/sizeof(*pageBuffer));//读出整个扇区的内容		
		for(i=0;i<pageRemainSize;i++)//校验数据
		{
			//检查是否有需要擦除的数据				
			if(pageBuffer[pageOffset+i]!=0xFF)
			{
				#ifdef DEBUG_FLASH
				FLASH_LOG("SOME DATAS NEED TO ERASE!\r\n");
				FLASH_LOG("ERASE ADDR:%d\r\n",pageOffset+i);
				#endif				
				break;//需要擦除 
			}				
		} 
		#ifdef DEBUG_FLASH
		FLASH_LOG("pageRemainSize:%d\r\n",pageRemainSize);
		#endif			
		if(i<pageRemainSize)//需要擦除
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO ERASE!\r\n");
			#endif			
			FLASH_ErasePage(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS);//擦除这个扇区
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO COPY!\r\n");
			#endif				
			for(i=0;i<pageRemainSize;i++)//复制
			{
				//将待欲写入的数据续存在之前的数据之后
				pageBuffer[i+pageOffset]=pBuffer[i];//保留页偏移地址以前的数据	 
			}
			#ifdef DEBUG_FLASH
			FLASH_LOG("TO WRITE!\r\n");
			#endif					
			//按照8字节的偏移量写入
			FLASH_WriteNoCheck(pagePos*FLASH_PAGE_SIZE+FLASH_BASE_ADDRESS,pageBuffer,FLASH_PAGE_SIZE);//写入整个扇区  
		}
		else //不需要擦除
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("NONE TO ERASE!\r\n");
			#endif
      //按照8字节的偏移量写入			
			FLASH_WriteNoCheck(WriteAddress,pBuffer,pageRemainSize);//写已经擦除了的,直接写入扇区剩余区间. 		
		}
		if(NumToWrite==pageRemainSize)
		{
			#ifdef DEBUG_FLASH
			FLASH_LOG("WRITE OVER !\r\n");
			#endif				
			break;//写入结束了
		}
		else//写入未结束
		{
			pagePos++;				//扇区地址增1
			pageOffset       = 0;				          //偏移位置为0 	 
			pBuffer         += pageRemainSize;  	//指针偏移
			WriteAddress    += pageRemainSize;	  //写地址偏移	   
			NumToWrite      -= pageRemainSize;	  //字节数递减
			if(NumToWrite>(FLASH_PAGE_SIZE))
			{
					pageRemainSize=FLASH_PAGE_SIZE;//下一个扇区还是写不完
					#ifdef DEBUG_FLASH
					FLASH_LOG("WC-> pageRemainSize:%#X\r\n",pageRemainSize);
					#endif					
			}
      else 
			{
					pageRemainSize=NumToWrite;//下一个扇区可以写完了
					#ifdef DEBUG_FLASH
					FLASH_LOG("WC->WRITE OVER !\r\n");
					#endif					
			}
		}	 
	}
	/* 上锁FLASH */
	HAL_FLASH_Lock();
}


#endif






