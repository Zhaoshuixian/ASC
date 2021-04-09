
/*
	Project: Anchor Cable Sensor (STM32L432 + MDK)
	Create Data: 2021/02/02
	Modify Data: 2021/03/27
	Author By:   Zhao Shuixian
  Brief：
  1. 实现多任务调度功能
  2. 实现LED功能
  3. 实现按键功能
  4. 实现串口USART1打印功能
  5. 实现AD7193数据读取
  6. 实现BMI160数据读取
  7. 实现RTC定时休眠唤醒
	8. 实现UART_DMA接收（BUG:首上电产生IDLE中断）
	9. 实现片上FLASH存储操作
*/

#include "misc.h"
#include "gpio.h"
#include "ad7193.h"
#include "bmi160_bsp.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "wtd.h"
#include "os.h"
#include "flash.h"
#include "tim.h"

/*外设定义*/
RTC_HandleTypeDef  hrtc;  //rtc

/*函数声明*/
static void SystemClock_Config(void);
static void SystemPower_Config(void);
static void System_Config_When_Enter_StopMode(void);

static void device_led_handle(void);
static void device_temper_handle(void);
static void device_key_handle(void);
static void device_lpw_handle(void);
static void hook_idle_handle(void);

/*系统时基时间 ：1ms*/
#define SYSTEM_TICK_TIME  (1)//
/*任务执行时间*/
#define LED_TASK_TIME     (500/SYSTEM_TICK_TIME)      //500ms
#define BMI160_TASK_TIME  (1000/SYSTEM_TICK_TIME)     //1000ms
#define AD7193_TASK_TIME  (1500/SYSTEM_TICK_TIME)     //1000ms
#define TEMPER_TASK_TIME  (10/SYSTEM_TICK_TIME)       //10ms
#define KEY_TASK_TIME     (20/SYSTEM_TICK_TIME)       //20ms
#define UART_TASK_TIME    (0)
#define LPW_TASK_TIME     (0)
#define HOOK_TASK_TIME    (30000/SYSTEM_TICK_TIME)    //30s

/*任务列表*/
task_st multi_task[] =
{
  {1, 0,  0, LED_TASK_TIME,    HAL_GetTick, device_led_handle   },
  {2, 0,  0, BMI160_TASK_TIME, HAL_GetTick, device_bmi160_handle},  
  {3, 0,  0, AD7193_TASK_TIME, HAL_GetTick, device_ad7193_handle},
  {4, 0,  0, TEMPER_TASK_TIME, HAL_GetTick, device_temper_handle},  
  {5, 0,  0, KEY_TASK_TIME   , HAL_GetTick, device_key_handle   },
  {6, 1,  0, UART_TASK_TIME  , HAL_GetTick, device_uart_handle  }, // 
  {7, 0,  0, HOOK_TASK_TIME  , HAL_GetTick, hook_idle_handle    }, //暂时在HOOK中设计一个运行30s后自动进入休眠的任务，以验证休眠功能
	{8, 1,  0, LPW_TASK_TIME  ,  HAL_GetTick, device_lpw_handle   }
  //...Add task	
};

uint8_t LED_Status_Flag=0;     //0:FLASH  | 1:LIGHT | 2:DARK | 3:TWINKLE
sem_st slp_sem;
/*
**LED状态控制
*/
void device_led_handle(void)
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
**按键状态读取及处理
*/
void device_key_handle(void)
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
void device_temper_handle(void)
{
   //ADC OR DIGITAL??
}

/*
***设备低功耗
*/
void device_lpw_handle(void)
{
	if(0!=sem_wait(&slp_sem,0))
	{
		System_Config_When_Enter_StopMode();//进入低功耗前的配置
	}
}

/*
***hook空闲函数
*/
void hook_idle_handle(void)
{
   #ifdef SYSTEM_LOG
   SYSTEM_LOG("Enter Sleep...\r\n");
   #endif 
	 sem_release(&slp_sem); //释放信号量	
}

/*
***定时唤醒事件回调处理
*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  IWDG_Feed();//喂狗
}

/*
**系统进入休眠状态(停止模式)
*/
void System_Config_When_Enter_StopMode(void)
{ 
  //为了将功耗降到最低，将引脚恢复初始化状态
	HAL_SPI_MspDeInit(&hspi3);  //缺省SPI设备
	HAL_I2C_MspDeInit(&hi2c1);  //缺省I2C设备	
	HAL_UART_MspDeInit(&huart1);//缺省串口1设备	
	HAL_UART_MspDeInit(&huart2);//缺省串口2设备			
#if 0
  GPIO_InitTypeDef GPIO_InitStructure;	
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
  HAL_GPIO_DeInit(GPIOA,J1_1_Pin|J1_2_Pin|J1_3_Pin|TEMPER_SIN_Pin|DEV_LED_Pin);
/*IC POWER CONTRL*/
  HAL_GPIO_DeInit(GPIOA,AD7193_VDD_SWITCH_Pin|TEMPER_VDD_SWITCH_Pin|BMI160_VDD_SWITCH_Pin|EXT_VDD_SWITCH_Pin);
/*SWD DEBUG*/
  HAL_GPIO_DeInit(GPIOA,SWD_DIO_Pin|SWD_SCK_Pin);
/*NULL*/
  HAL_GPIO_DeInit(GPIOB,NC_1_Pin);
  HAL_GPIO_DeInit(GPIOC,NC_2_Pin);	
#endif
	//关闭端口时钟
  //__HAL_RCC_GPIOA_CLK_DISABLE();	
  //__HAL_RCC_GPIOB_CLK_DISABLE();	//PB0  -> EXT_TRIG_SINGAL
  //__HAL_RCC_GPIOC_CLK_DISABLE();  //PC15 -> KEY_BUTTON_SINGAL
  __HAL_RCC_PWR_CLK_ENABLE();  //打开电源管理时钟
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
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();

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
	#define RTC_ASYNCH_PREDIV    (0x7F)
	#define RTC_SYNCH_PREDIV     (0xF9) /* 32Khz/128 - 1 */	
	
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
  MX_USART2_UART_Init();
#if 0	
	MX_TIM2_Init(48,10000);//
#endif
	#ifdef DEBUG_SYSTEM    
	SYSTEM_LOG("Startup...\r\n");
	#endif	
//	if(DEVICE_INIT_OK!=bmi160_bsp_init(&sensor_bmi160)) 
//	{
//		#ifdef DEBUG_BMI160    
//		BMI160_LOG("Init failed...\r\n");
//		#endif          
//		while(1) //失败进入阻塞提示
//		{ 
//			HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
//			HAL_Delay(150);//LED闪烁指示
//		}
//	}	
//  #ifdef DEBUG_BMI160 
//  BMI160_LOG("CHIP ID:%#X...\r\n",sensor_bmi160.chip_id);
//  #endif    
//	bmi160_config_init();
//	if(DEVICE_INIT_OK!=ad7193_init())//设备初始化失败
//	{
//    #ifdef DEBUG_AD7193
//		AD7193_LOG("Init failed...\r\n");	
//    #endif	
//		while(1) //失败进入阻塞提示
//		{
//			HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
//			HAL_Delay(100);//LED闪烁指示
//		}
//	}
//  ad7193_config_init();
	#ifdef DEBUG_SYSTEM
  SYSTEM_LOG("-------Guangdong Tek Smart Sensor Ltd.,Company-------\r\n");
  SYSTEM_LOG("------------Make Data: %s-%s-------\r\n",(const char *)__TIME__,(const char *)__DATE__);
  SYSTEM_LOG("------------------All Init OK------------------------\r\n");
  #endif
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);//禁止RTC周期唤醒中断
	
	#ifdef DEBUG_FLASH_EXAMPLE
  #if DOUBLE_2WORD
	uint32_t wdata_arr[16]={0x1122,0x2233,0x3344,0x4455,0x5566,0x6677,0x7788,0x8899,0x99AA,0xAABB,0xBBCC,0xCCEE,0XEEFF,0xFF00};
	uint32_t rdata_arr[16]={0};
	#else
	uint8_t wdata_arr[17]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0XEE,0xFF,0x85,0x74,0x52};
	uint8_t rdata_arr[17]={0};
	
	uint8_t s_wdata1=0x12;//addr-0x25
	uint8_t s_wdata2=0x34;//addr-0x30
	
	uint8_t s_rdata1=0;//addr-0x1	
	uint8_t s_rdata2=0;	//addr-0x15	
	#endif	
  //flash_read(EEPROM_START_ADDRESS, &rdata_arr[0],17);	
  flash_read(EEPROM_START_ADDRESS+0x8, &s_rdata1,1);	
  flash_read(EEPROM_START_ADDRESS+0xC, &s_rdata2,1);		
//	if(0x11!=rdata_arr[0])
//	{
//		 flash_write(EEPROM_START_ADDRESS,&wdata_arr[0],17);		
//	}
	if(0x12!=s_rdata1)
	{
     flash_write(EEPROM_START_ADDRESS+0x8,&s_wdata1,1);	
	}
	if(0x34!=s_rdata2)
	{
     flash_write(EEPROM_START_ADDRESS+0xC,&s_wdata2,1);	
	}	
	FLASH_LOG("s_rdata1;%#x\r\n",s_rdata1);
	FLASH_LOG("s_rdata2;%#x\r\n",s_rdata2);	
	#endif
	//MX_IWDG_Init(); 
	sem_create(&slp_sem);
  while (1)
  {
		tasks_os_run(multi_task,sizeof(multi_task)/sizeof(*multi_task));//多任务启动
		if(0!=sem_take(&slp_sem)) goto __TODO;//唤醒系统后，重新进入初始化
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
