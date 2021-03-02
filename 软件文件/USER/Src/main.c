
/*
	Project: Anchor Cable Sensor (STM32L432KBU6 + MDK)
	Create Data: 2021/02/02
	Modify Data: 2021/03/02
	Author By:   Zhao Shuixian
  Brief：
  1. 实现多任务调度功能
  2. 实现LED功能
  3. 实现按键功能
  4. 实现串口USART1打印功能
  5. 实现AD7193数据读取
  6. 实现BMI160数据读取
  7. 实现RTC定时唤醒
*/

#include "main.h"
#include "misc.h"
#include "ad7193.h"
#include "bmi160_bsp.h"

/*系统时基时间 ：1ms*/
#define SYSTEM_TICK_TIME  (1)//
/*任务执行时间*/
#define LED_TASK_TIME     (500/SYSTEM_TICK_TIME)      //500ms
#define BMI160_TASK_TIME  (1000/SYSTEM_TICK_TIME)     //1000ms
#define AD7193_TASK_TIME  (1500/SYSTEM_TICK_TIME)     //1000ms
#define TEMPER_TASK_TIME  (10/SYSTEM_TICK_TIME)       //10ms
#define KEY_TASK_TIME     (20/SYSTEM_TICK_TIME)       //20ms
#define HOOK_TASK_TIME    (30000/SYSTEM_TICK_TIME)    //30s

/*外设定义*/
I2C_HandleTypeDef  hi2c1; //i2c1
SPI_HandleTypeDef  hspi3; //spi3
RTC_HandleTypeDef  hrtc;  //rtc
IWDG_HandleTypeDef hiwdg; //iwdg
UART_HandleTypeDef huart1; //usart1
UART_HandleTypeDef huart2; //usart2
DMA_HandleTypeDef hdma_usart1_rx;//usart1_dma
DMA_HandleTypeDef hdma_usart2_rx;//usart2_dma


/*函数声明*/
static void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void IWDG_Feed(void);
static void System_Config_When_Enter_StopMode(void);
static void device_led_contrl(void);
static void device_bmi160_read(void);
static void device_ad7193_read(void);
static void device_temper_read(void);
static void device_key_read(void);
static void hook_idel_handle(void);

//任务列表
task_st multi_task[] =
{
  {1, 0, LED_TASK_TIME,    HAL_GetTick, device_led_contrl },
  {2, 0, BMI160_TASK_TIME, HAL_GetTick, device_bmi160_read},  
  {3, 0, AD7193_TASK_TIME, HAL_GetTick, device_ad7193_read},
  {4, 0, TEMPER_TASK_TIME, HAL_GetTick, device_temper_read},  
  {5, 0, KEY_TASK_TIME   , HAL_GetTick, device_key_read   },
  {6, 0, HOOK_TASK_TIME  , HAL_GetTick, hook_idel_handle  }, //暂时在HOOK中设计一个运行30s后自动进入休眠的任务，以验证休眠功能
  //...Add task	
};

uint8_t SleepMode_Enter_Flag=0;//0:Normal | 1:Sleep
uint8_t LED_Status_Flag=0;     //0:FLASH  | 1:LIGHT | 2:DARK | 3:TWINKLE

/*
**串口打印重映射
*/
int fputc(int ch, FILE *f)
{
  // 配置格式化输出到串口USART1
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

/*
**LED状态控制
*/
void device_led_contrl(void)
{
	switch(LED_Status_Flag)
	{
		case 0://闪烁
			HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
			break;
		case 1://常亮
			HAL_GPIO_WritePin(GPIOA,DEV_LED_Pin,GPIO_PIN_RESET);
			break;    
		case 2://常灭
			HAL_GPIO_WritePin(GPIOA,DEV_LED_Pin,GPIO_PIN_SET);
			break;
		case 3://眨闪
			HAL_GPIO_WritePin(GPIOA,DEV_LED_Pin,GPIO_PIN_RESET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOA,DEV_LED_Pin,GPIO_PIN_SET);
			break;         
	}
}

/*
**AD7193相关数据读取
*/
typedef struct
{
	unsigned int	raw_data;
	float volt;
}ch_st;	
	
ch_st ch[5]={0};

void device_ad7193_read(void)
{
	/*
	如果使能多个通道，则每次切换通道时，ADC会给滤波器留出完整的建立时间，以便产生有效转换结果。
	AD7193将通过以下序列自动处理这种状况：

	1. 选择某个通道时，调制器和滤波器将复位。
	2. AD7193允许完整的建立时间以产生有效转换结果。
	3. DOUT/RDY会在有效转换结果可用时给出提示。
	4. AD7193选择下一个使能通道，并在该通道上执行转换。
	5. 当ADC在下一个通道上执行转换时，用户可以读取数据寄存器。

	*/
	unsigned char status_reg_val=0;
	status_reg_val=ad7193_get_register_value(AD7193_REG_STAT,1,1);//读状态寄存器
	status_reg_val&=0x0F;  
  
	switch(status_reg_val)//当前数据所在通道的指示
	{
		case AD7193_CH_0:
    #ifdef DEBUG_MODE
		printf("CH0...\r\n");
    #endif
		ch[0].raw_data= ad7193_continuous_readavg(10);//连续转换
		ch[0].volt= ad7193_convert_to_volts(ch[0].raw_data,5.0);				
		break;
		case AD7193_CH_1:
    #ifdef DEBUG_MODE
		printf("CH1...\r\n");
    #endif
		ch[1].raw_data = ad7193_continuous_readavg(10);	
		ch[1].volt = ad7193_convert_to_volts(ch[1].raw_data,5.0);				
		break;
		case AD7193_CH_2:
    #ifdef DEBUG_MODE
		printf("CH2...\r\n");
    #endif
		ch[2].raw_data = ad7193_continuous_readavg(10);
		ch[2].volt = ad7193_convert_to_volts(ch[2].raw_data,5.0);				
		break;
		case AD7193_CH_3:
    #ifdef DEBUG_MODE
		printf("CH3...\r\n");
    #endif
		ch[3].raw_data = ad7193_continuous_readavg(10);
		ch[3].volt = ad7193_convert_to_volts(ch[3].raw_data,5.0);		
		break;   
		case AD7193_CH_TEMP:
    #ifdef DEBUG_MODE    
		printf("CHTEMP...\r\n");
    #endif    
		ch[4].raw_data = ad7193_continuous_readavg(10);//连续转换
		ch[4].volt = ad7193_temperature_read(ch[4].raw_data);
		break; 
		default:  
    #ifdef DEBUG_MODE     
		printf("UNKOWN %d...\r\n",status_reg_val);	
    #endif			
		break; 		
	}
#ifdef DEBUG_MODE
	static ch_st ch_tmp[5];
	for(unsigned char i=0;i< 4;i++)
	{
		if(ch_tmp[i].volt!=ch[i].volt)
		{
			ch_tmp[i].volt=ch[i].volt;
      #ifdef DEBUG_MODE 
			printf("CH%d Volts is %fV...\r\n",i,ch_tmp[i].volt);	
      #endif      	 
		}    
	}
	if(ch_tmp[4].volt!=ch[4].volt)
	{
    ch_tmp[4].volt=ch[4].volt;
    #ifdef DEBUG_MODE      
    printf("Current Chip temperature is %fC...\r\n",ch_tmp[4].volt);
    #endif         
	}   
#endif
}

/*
**BMI160相关数据读取
*/
void device_bmi160_read(void)
{
  bmi160_read_sensor_data(&sensor_bmi160);
}

/*
**按键状态读取及处理
*/
void device_key_read(void)
{
  static uint8_t last_key,now_key,key_count;
  
  //按键状态读取
  (0==HAL_GPIO_ReadPin(KEY_SIN_GPIO_Port,KEY_SIN_Pin))?(now_key=1):(now_key=0);

  //按键动作变化处理
  if(last_key!=now_key)
  {
    if(0!=now_key)//按下
    {
      #ifdef DEBUG_MODE
		  printf(">> The Key Pessed !!!\r\n");	
      #endif	
			(0==++key_count%2)?(LED_Status_Flag=0):(LED_Status_Flag=3);
    }   
  }
	last_key = now_key;
}

/*
**外部温度传感器读取
*/
void device_temper_read(void)
{

}

/*
***hook空闲函数
*/
void hook_idel_handle(void)
{
   #ifdef DEBUG_MODE
   printf(">> Enter Sleep...\r\n");
   #endif 
	 SleepMode_Enter_Flag=1;//进入SLEEP MODE  
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
	__TODO:	
  SystemClock_Config();
  SystemPower_Config(); 
  MX_GPIO_Init();
  HAL_Delay(1000);//等待电源稳定
  MX_DMA_Init();
  MX_I2C1_Init();  //for BMI160
  MX_SPI3_Init();  //for AD7193
  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();	
	if(DEVICE_INIT_OK!=bmi160_bsp_init(&sensor_bmi160)) 
	{
      #ifdef DEBUG_MODE    
		  printf(">> BMI160 Init failed...\r\n");
      #endif          
			while(1) //失败进入阻塞提示
			{
				HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
				HAL_Delay(150);//LED闪烁指示
			}
	}	
  #ifdef DEBUG_MODE 
  printf(">> BMI160 Chip ID is %#X...\r\n",sensor_bmi160.chip_id);
  #endif    
	bmi160_config_init();

	if(DEVICE_INIT_OK!=ad7193_init())//设备初始化失败
	{
    #ifdef DEBUG_MODE
		printf(">> AD7193 Init failed...\r\n");	
    #endif	
		while(1) //失败进入阻塞提示
		{
			HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
			HAL_Delay(100);//LED闪烁指示
		}
	}
  ad7193_config_init();
	
	#ifdef DEBUG_MODE
  printf("\r\n-------Guangdong Tek Smart Sensor Ltd.,Company-------\r\n");
  printf("\r\n------------Make Data: %s-%s-------\r\n",(const char *)__TIME__,(const char *)__DATE__);
  printf("\r\n------------------All Init OK------------------------\r\n");
  #endif
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);//禁止RTC周期唤醒中断
//	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	//MX_IWDG_Init();
  while (1)
  {
  /* hrtc Wakeup Interrupt Generation: 
    the wake-up counter is set to its maximum value to yield the longuest
    stop time to let the current reach its lowest operating point.
    The maximum value is 0xFFFF, corresponding to about 33 sec. when 
    RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

    Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
    Wakeup Time = Wakeup Time Base * WakeUpCounter 
      = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
      ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
  
    To configure the wake up timer to 60s the WakeUpCounter is set to 0xFFFF:
    Wakeup Time Base = 16 /(~32.000KHz) = ~0.5 ms
    Wakeup Time = 0.5 ms  * WakeUpCounter
    Therefore, with wake-up counter =  0xFFFF  = 65,535 
     Wakeup Time =  0,5 ms *  65,535 = 32,7675 s ~ 33 sec. 
    */
		tasks_os_run(multi_task,sizeof(multi_task)/sizeof(*multi_task));//任务启动
    if(1==SleepMode_Enter_Flag)	
		{
			System_Config_When_Enter_StopMode();// 
			SleepMode_Enter_Flag=0;//RTC唤醒之后，清除标志
			goto __TODO;//唤醒系统
		}
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    IWDG_Feed();
}

/*
***GPIO外部中断回调函数
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //判断进入中断的GPIOs
  if(EXT_TRIG_Pin == GPIO_Pin)//外部触发引脚
  {
    //SleepMode_Enter_Flag=0;//退出SLEEP MODE 
  }
  else if(KEY_SIN_Pin == GPIO_Pin)//按键引脚
  {
    SleepMode_Enter_Flag=0;//退出SLEEP MODE 
  }
}

#define UART1_BUFF_SIZE  (255) //串口1接收最大缓存字节数
unsigned int rx_len; //接收长度
unsigned char rx_buff[UART1_BUFF_SIZE]={0};//接收缓存区

/*
***DMA发送
*/
void DMA_UART1_Send(uint8_t *data,uint8_t len)//串口发送
{
  if(HAL_UART_Transmit_DMA(&huart1,data,len)!= HAL_OK) Error_Handler();
}

/*
***DMA接收
*/
void DMA_UART1_Read(uint8_t *data,uint8_t len)//串口接收
{
	 HAL_UART_Receive_DMA(&huart1,data,len);//重新打开DMA接收
}

/*
***UART串口IDEL中断回调函数
*/
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);	//清除空闲中断标志
		//__HAL_DMA_DISABLE(huart->hdmarx);//关闭DMA
	  HAL_UART_DMAStop(huart); // 
		rx_len = UART1_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);//获取接收数据长度
		//memcpy(aRxBuffer_Save,aRxBuffer,aRX_Count);	//获取接收的数据
		//__HAL_DMA_RESET_HANDLE_STATE(huart->hdmarx);	//复位DMA接收设置	
		//__HAL_DMA_ENABLE(huart->hdmarx);//使能DMA
		HAL_UART_Receive_DMA(&huart1,rx_buff,UART1_BUFF_SIZE);//重新打开DMA接收
	}
}

/*
**系统进入休眠状态(停止模式)
*/
void System_Config_When_Enter_StopMode(void)
{ 
  //为了将功耗降到最低，将引脚恢复初始化状态
  GPIO_InitTypeDef GPIO_InitStructure;	
	
	HAL_SPI_MspDeInit(&hspi3);  //缺省SPI设备
	HAL_I2C_MspDeInit(&hi2c1);  //缺省I2C设备	
	HAL_UART_MspDeInit(&huart1);//缺省串口1设备	
	HAL_UART_MspDeInit(&huart2);//缺省串口2设备			

#if 0
	//除了休眠唤醒引脚、其余皆设置为模拟输入
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_All);
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_All);
	HAL_GPIO_DeInit(GPIOC,GPIO_PIN_All);

  /*Configure GPIO pin : EXT_TRIG_Pin */
  GPIO_InitStructure.Pin  = EXT_TRIG_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /*Configure GPIO pin : KEI_SIN_Pin */
  GPIO_InitStructure.Pin  = KEY_SIN_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
#else
  HAL_GPIO_DeInit(GPIOA,J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin|TEMPER_SIN_Pin|DEV_LED_Pin);
/*IC POWER CONTRL*/
  HAL_GPIO_DeInit(GPIOA,AD7193_VDD_SWITCH_Pin|TEMPER_VDD_SWITCH_Pin|BMI160_VDD_SWITCH_Pin|EXT_VDD_SWITCH_Pin);
/*SWD DEBUG*/
  HAL_GPIO_DeInit(GPIOA,SWD_DIO_Pin|SWD_SCK_Pin);
/*NULL*/
  HAL_GPIO_DeInit(GPIOB,NC_1_Pin);
  HAL_GPIO_DeInit(GPIOC,NC_2_Pin);	
#endif
	//关闭端口时钟
	__HAL_RCC_GPIOA_CLK_DISABLE();	
	__HAL_RCC_GPIOB_CLK_DISABLE();	//PB0  -> EXT_TRIG_SINGAL
	//__HAL_RCC_GPIOC_CLK_DISABLE();  //PC15 -> KEY_BUTTON_SINGAL
  __HAL_RCC_PWR_CLK_ENABLE();  //打开电源管理时钟
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x0FFFF, RTC_WAKEUPCLOCK_RTCCLK_DIV16); //激活RTC周期唤醒中断，并设置周期唤醒时长为：33s
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI); //进入停止模式（这边配置的是STOP2） 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	/*
    可以使用四种不同的时钟源来驱动系统时钟(SYSCLK)： • HSI （高速内部） 16 MHz RC 振荡器时钟
    • MSI （多速内部）RC 振荡器时钟
    • HSE 振荡器时钟，4 至 48 MHz
    • PLL 时钟
    从复位中启动后， MSI用作系统时钟源，配置为4 MHz。
    器件具有以下附加时钟源：
    • 32 kHz低速内部RC (LSI RC)，该 RC 用于驱动独立看门狗，
      也可选择提供给hrtc 用于停止 / 待机模式下的自动唤醒。
    • 32.768 kHz低速外部晶振(LSE 晶振),用于驱动实时时钟 (RTCCLK)
    对于每个时钟源来说，在未使用时都可单独打开或者关闭，以降低功耗。
    
    MSI 时钟信号是从内部 RC 振荡器生成的。其频率范围可通过时钟控制寄存器 （RCC_CR）
    中的 MSIRANGE[3:0] 位进行软件调节。有 12 个频率范围可用：100 kHz、200 kHz、400 kHz、
    800 kHz、1 MHz、2 MHz、4 MHz（默认值）、8 MHz、16 MHz、24 MHz、32 MHz和 48 MHz。
    在从复位重启、从待机、关断低功耗模式后唤醒， MSI 时钟被用作系统时钟。从复位重启后，MSI 频率被置位其默认值 4 MHz。
	*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;// 设置需要配置的振荡器为MSI/LSI
  RCC_OscInitStruct.LSIState       = RCC_LSI_ON;   // 激活LSI时钟(32kHz低速内部RC振荡器时钟)
  RCC_OscInitStruct.MSIState       = RCC_MSI_ON;   // 激活MSI时钟(100KHz-48MHz多速内部RC振荡器时钟)
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange  =  RCC_MSIRANGE_11; //配置为48MHz
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE; 
	
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // 需要配置的时钟HCLK(APB1/APB2总线时钟源)、SYSCLK(系统时钟源)、PCLK1(APB1上外设时钟源)、PCLK2(APB2上外设时钟源)
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;// 配置系统时钟为MSI输入，48MHz
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;     // AHB时钟为系统时钟1分频，48MHz/1 =48MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;       // APB1时钟为系统时钟1分频，48MHz/1 =48MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;       // APB2时钟为系统时钟1分频，48MHz/1 =48MHz  
  
/*
--@from --RM0390 Reference manual---
	
--->>FLASH_ACR	

	0 WS (1 CPU cycle)    0 < HCLK ≤ 30    0 <HCLK ≤ 24      0 < HCLK ≤ 22      0 < HCLK ≤ 20
	1 WS (2 CPU cycles)  30 < HCLK ≤ 60   24 < HCLK ≤ 48    22 < HCLK ≤ 44     20 < HCLK ≤ 40
	2 WS (3 CPU cycles)  60 < HCLK ≤ 90   48 < HCLK ≤ 72    44 < HCLK ≤ 66     40 < HCLK ≤ 60
	3 WS (4 CPU cycles)  90 < HCLK ≤ 120  72 < HCLK ≤ 96    66 < HCLK ≤ 88     60 < HCLK ≤ 80
	4 WS (5 CPU cycles) 120 < HCLK ≤ 150  96 < HCLK ≤ 120   88 < HCLK ≤ 110    80 < HCLK ≤ 100
	5 WS (6 CPU cycles) 150 < HCLK ≤ 180 120 < HCLK ≤ 144  110 < HCLK ≤ 132   100 < HCLK ≤ 120
	6 WS (7 CPU cycles)                  144 < HCLK ≤ 168  132 < HCLK ≤ 154   120 < HCLK ≤ 140
	7 WS (8 CPU cycles)                  168 <HCLK ≤ 180   154 < HCLK ≤ 176   140 < HCLK ≤ 160
	8 WS (9 CPU cycles)                                    176 < HCLK ≤ 180   160 < HCLK ≤ 168	
	*/	
	
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

  // 需要初始化的外设时钟:USART1/USART2/I2C1/RTC
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
	
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1; 
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;//RTC时钟源为LSI	

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
  // 配置内部主稳压器输出电压，配置为稳压器输出电压范围1模式，也就是：典型输出电压为1.2V，系统频率高达80MHz
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
	
	__HAL_RCC_RTC_ENABLE();//RTC时钟使能
	
  // 配置系统定时器中断时间，配置为HCLK的千分频
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  // 配置系统定时器，配置为HCLK
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  // 系统定时器中断配置，设置系统定时器中断优先级最高（为0），且子优先级最高（为0）
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
	*            + hrtc Clocked by LSI
	*            + VREFINT OFF, with fast wakeup enabled
	*            + No IWDG
	*            + Automatic Wakeup using hrtc clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
	#define RTC_ASYNCH_PREDIV    0x7F
	#define RTC_SYNCH_PREDIV     0xF9  /* 32Khz/128 - 1 */	
	
  __HAL_RCC_PWR_CLK_ENABLE();//打开设备管理时钟
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);//配置MST为唤醒时钟源	
  /* Configure hrtc */
  hrtc.Instance = RTC;	
  /* Set the hrtc time base to 1s */
  /* Configure hrtc prescaler and hrtc data registers as follow:
    - Hour Format   = Format 24
    - Asynch Prediv = Value according to source clock
    - Synch Prediv  = Value according to source clock
    - OutPut = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType     = Open Drain 
	*/
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler(); 
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance              = I2C1;	
  hi2c1.Init.Timing           = 0x10D05E82;
  hi2c1.Init.OwnAddress1      = 0x0; //设置I2C从机地址
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;//7位地址模式
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;//非双寻址模式
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	//配置初始化
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  //模拟滤波配置
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)  Error_Handler();
  //数字滤波配置
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
 /*
 - LSI(32KHz)为WDG的时钟源 -
 配置IWDG 32分频（也就是32KHz /32 = 1KHz = 1us），
 配置重载计数值为1000（也就是装载满一次需要1000*1us = 1s）。
 或者大家使用公式计算：Tout=(4*(2^prer)*rlr)/32 (ms)。此例程prer = 3，rlr = 1000。
 */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance       = IWDG; // 配置为IWDG
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;// 32分频，prer = 3
  hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload    = 1000;
	
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

/*
**独立看门狗喂狗函数，也就是清除计数值。
*/
static void IWDG_Feed(void)
{
  HAL_IWDG_Refresh(&hiwdg); 	// 喂狗
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  hspi3.Instance         = SPI3;               //配置为SPI3          
  hspi3.Init.Mode        = SPI_MODE_MASTER;    //配置为Master      
  hspi3.Init.Direction   = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize    = SPI_DATASIZE_8BIT;     //8位数据模式
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;//SCLK下降沿
  hspi3.Init.CLKPhase    = SPI_PHASE_1EDGE;
  hspi3.Init.NSS         = SPI_NSS_SOFT;             //NSS为软件控制    
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//配置SPI3分频系数 SPI最高频率有限制
  hspi3.Init.FirstBit    = SPI_FIRSTBIT_MSB;   
  hspi3.Init.TIMode      = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial  = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode  = SPI_NSS_PULSE_ENABLE;
	
  if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance            = USART1; //
  huart1.Init.BaudRate       = 115200;
  huart1.Init.WordLength     = UART_WORDLENGTH_8B;
  huart1.Init.StopBits       = UART_STOPBITS_1;
  huart1.Init.Parity         = UART_PARITY_NONE;
  huart1.Init.Mode           = UART_MODE_TX_RX;//收发模式
  huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling   = UART_OVERSAMPLING_16; //16Bit过采样
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // 1位过采样禁能
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;// 没有串口高级功能初始化
	
  if(HAL_UART_Init(&huart1) != HAL_OK)  Error_Handler();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance        = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//使能IDEL中断
  if (HAL_UART_Init(&huart2) != HAL_OK)  Error_Handler();

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); 
	
  /*Configure GPIO pin : NC_1_Pin */
  GPIO_InitStruct.Pin  = NC_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_1_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pin : NC_2_Pin */
  GPIO_InitStruct.Pin  = NC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEI_SIN_Pin */
  GPIO_InitStruct.Pin  = KEY_SIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_SIN_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pins : J1_1_Pin /J1_2_Pin /J1_4_Pin /J1_5_Pin */
  GPIO_InitStruct.Pin   = J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : */
  GPIO_InitStruct.Pin   = AD7193_VDD_SWITCH_Pin|TEMPER_VDD_SWITCH_Pin|EXT_VDD_SWITCH_Pin|BMI160_VDD_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMPER_SIN_Pin /DEV_LED_Pin */
  GPIO_InitStruct.Pin   = TEMPER_SIN_Pin|DEV_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
 /*Configure GPIO pins :  AD7193_CS_Pin */	
  GPIO_InitStruct.Pin   = AD7193_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin : EXT_TRIG_Pin */
  GPIO_InitStruct.Pin  = EXT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXT_TRIG_GPIO_Port, &GPIO_InitStruct);

  //外部中断线配置
  HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);//EXT_TRIG
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);//KEY
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  //配置引脚上电初始端口电平状态
  HAL_GPIO_WritePin(GPIOA, J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin|TEMPER_SIN_Pin ,GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, AD7193_VDD_SWITCH_Pin|DEV_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, TEMPER_VDD_SWITCH_Pin, GPIO_PIN_SET); //ADC_VTEMP_VDD_SWITCH
	HAL_GPIO_WritePin(GPIOA, BMI160_VDD_SWITCH_Pin, GPIO_PIN_SET); //PA5
	HAL_GPIO_WritePin(GPIOA, EXT_VDD_SWITCH_Pin,    GPIO_PIN_SET); //EXT_VDD_SWITCH
	HAL_GPIO_WritePin(GPIOA, AD7193_CS_Pin, GPIO_PIN_SET); //PA15
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
