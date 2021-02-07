
/*
	Project: Anchor Cable Sensor (STM32L432KBU6 + MDK)
	
	Create Data: 2021/02/02
	Modify Data: 2021/02/03
	
	Author By:   Zhao Shuixian
*/


#include "main.h"
#include "misc.h"
#include "ad7193.h"
#include "bmi160_bsp.h"
#include "stdio.h"

struct bmi160_dev sensor_bmi160;
uint8_t SleepMode_Enter_Flag=0;
uint8_t LED_Status_Flag=0;

I2C_HandleTypeDef  hi2c1; //i2c1
SPI_HandleTypeDef  hspi3; //spi3
IWDG_HandleTypeDef hiwdg; //iwdg

UART_HandleTypeDef huart1; //usart1
UART_HandleTypeDef huart2; //usart2

DMA_HandleTypeDef hdma_usart1_rx;//usart1_dma
DMA_HandleTypeDef hdma_usart2_rx;//usart2_dma

void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void IWDG_Feed(void);

/*任务执行时间*/
#define LED_TASK_TIME     (500)
#define BMI160_TASK_TIME  (10)
#define AD7193_TASK_TIME  (10)
#define TEMPER_TASK_TIME  (10)
#define KEY_TASK_TIME     (20)

unsigned long t=0;
float v=0.0;


//串口打印重映射
int fputc(int ch, FILE *f)
{
  // 配置格式化输出到串口USART1
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

/*
---<<< 进入Stop模式的流程 >>>---

1.关闭程序中的IO和外设功能
2.调用进入stop模式的函数

---<<< 退出Stop模式的流程 >>>---

1.唤醒引脚外部中断触发
2.配置系统时钟源 
3.程序从外部中断回调处继续运行

*/
void device_led_toggle(void)
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
     case 3:
           break;         
  }

}

void device_ad7193_read(void)
{
  ad7193_range_setup(1, AD7193_CONF_GAIN_1);/* Select unipolar operation and ADC's input range to +-2.5V. */		
  
  ad7193_channel_select(AD7193_CH_0);/* Select channel AIN1(+) - AIN2(-) */
  ad7193_channel_select(AD7193_CH_1);/* Select channel AIN3(+) - AIN4(-) */
  ad7193_channel_select(AD7193_CH_2);/* Select channel AIN5(+) - AIN6(-) */
  ad7193_channel_select(AD7193_CH_3);/* Select channel AIN7(+) - AIN8(-) */	
  
  /* Returns the average of several conversion results. */
  /* Converts 24-bit raw data to volts. */
  v = ad7193_convert_to_volts(ad7193_continuous_readavg(10),5.0);
  t = ad7193_temperature_read();	/* Read the temperature. */	
}

void device_bmi160_read(void)
{

}

void device_key_read(void)
{
  static uint8_t last_key,now_key,key_count;
  
  //按键状态读取
  if(0==HAL_GPIO_ReadPin(KEY_SIN_GPIO_Port,KEY_SIN_Pin))//按下
  {
    now_key=1;
  }
  else //释放
  {
    now_key=0;
    last_key = now_key;
  }

  //按键动作变化处理
  if(last_key!=now_key)
  {
		printf("The Key has Trigged...\r\n");		
    if(0!=now_key)//按下
    {
      (2<key_count)?(key_count=1):(key_count++);

      if(0!=key_count%2)
      {
        if(0==LED_Status_Flag)//处于闪烁状态
        {
          LED_Status_Flag=1;//切换为常亮状态
        }
      }
      else
      {
        if(0!=LED_Status_Flag)//处于非亮状态
        {
          LED_Status_Flag=0;//切换为闪烁状态
        }        
      }
    }   
  }
}

void device_temper_read(void)
{

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();//for BMI160
  MX_SPI3_Init();  //for AD7193
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();	

	if(DEVICE_INIT_OK!=bmi160_bsp_init(&sensor_bmi160)) 
	{
		  printf("BMI160 Init failed...\r\n");
			while(1)
			{
				HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
				HAL_Delay(150);//LED闪烁指示
			}
	}	
	/*
		AD7193具体步骤：
		1.Init
		2.Reset
		3.Calibrate(zero&full)  <-> 校准
		4.RangeSetup(1,gain1)   <-> 单双极性以及增益选择
		5.ChannelSelect         <-> 通道选择
		6.SingleConversion      <-> 获取单次AD采样值
		7.ConvertToVolts        <-> 转换成电压
		8.ContinuousReadAvg(10) <-> 获取10次采样求均值
		9.ConvertToVolts        <-> 转换成电压
		10.TemperatureRead      <-> 获得温度AD
	*/
	if(DEVICE_INIT_OK!=ad7193_init())//设备初始化失败
	{
		printf("AD7193 Init failed...\r\n");		
		while(1)
		{
			HAL_GPIO_TogglePin(GPIOA,DEV_LED_Pin);
			HAL_Delay(100);//LED闪烁指示
		}
	}
  ad7193_calibrate(AD7193_MODE_CAL_INT_ZERO, AD7193_CH_0);
  ad7193_calibrate(AD7193_MODE_CAL_INT_FULL, AD7193_CH_0);

  ad7193_calibrate(AD7193_MODE_CAL_INT_ZERO, AD7193_CH_1);
  ad7193_calibrate(AD7193_MODE_CAL_INT_FULL, AD7193_CH_1);

  ad7193_calibrate(AD7193_MODE_CAL_INT_ZERO, AD7193_CH_2);
  ad7193_calibrate(AD7193_MODE_CAL_INT_FULL, AD7193_CH_2);  

  ad7193_calibrate(AD7193_MODE_CAL_INT_ZERO, AD7193_CH_3);
  ad7193_calibrate(AD7193_MODE_CAL_INT_FULL, AD7193_CH_3); 

  //MX_IWDG_Init();
  printf("\r\n-------Guangdong Tek Smart Sensor Ltd,.Company-------\r\n");
  printf("\r\n--------------Make Data: %s-%s-----------------------\r\n",(const char *)__TIME__,(const char *)__DATE__);
  printf("\r\n------------------All Init OK------------------------\r\n");
  while (1)
  {
    if(1!= SleepMode_Enter_Flag)//1. -> NORMAL MODE
    {
      //1.1 -- LED --	
      task_sheduler(device_led_toggle, LED_TASK_TIME,    1); //OK
      //1.2 -- BMI160 --	
      //task_sheduler(device_bmi160_read,BMI160_TASK_TIME, 2);
      //1.3 -- AD7193 --
      task_sheduler(device_ad7193_read,AD7193_TASK_TIME, 3);    
      //1.4 -- KEY_BUTTON --	
      task_sheduler(device_key_read,   KEY_TASK_TIME,    4);    
      //1.5 -- TEMPER_SENSOR --
      //task_sheduler(device_temper_read,TEMPER_TASK_TIME, 5);    
      //1.6 -- EXT_TRIG_SINGAL --
    }
    else//2. -> SLEEP MODE 
    {
      // System_Enter_StopMode();
      // SystemClock_ReConfig_When_Exit_StopMode();
      while(1);
    }
  }
}

//GPIO中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //判断进入中断的GPIOs
  if(EXT_TRIG_Pin == GPIO_Pin)//外部触发引脚
  {
    //SleepMode_Enter_Flag=1;//进入SLEEP MODE 
  }
  else if(KEY_SIN_Pin == GPIO_Pin)//按键引脚
  {

  }
}
/*
系统进入休眠状态(停止模式)
*/
void System_Enter_StopMode(void)
{ 
  //为了将功耗降到最低，将引脚恢复初始化状态
  GPIO_InitTypeDef GPIO_InitStructure;	

	HAL_SPI_MspDeInit(&hspi3);  //缺省SPI设备
	HAL_I2C_MspDeInit(&hi2c1);  //缺省I2C设备	
	HAL_UART_MspDeInit(&huart1);//缺省串口1设备	
	HAL_UART_MspDeInit(&huart2);//缺省串口2设备			
	
	//除了休眠唤醒引脚、其余皆设置为模拟输入
	GPIO_InitStructure.Pin  = J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin|AD7193_PWR_SWITCH_Pin| \
	TEMPER_SIN_Pin|DEV_LED_Pin|TEMPER_PWR_SWITCH_Pin|BMI160_PWR_SWITCH_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin  = NC_1_Pin|AD7193_SCK_Pin|AD7193_MISO_Pin|AD7193_MOSI_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin  = NC_2_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;	
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	//关闭端口时钟
	__HAL_RCC_GPIOA_CLK_DISABLE();	
	//__HAL_RCC_GPIOB_CLK_DISABLE();	//PB0  -> EXT_TRIG_SINGAL
	//__HAL_RCC_GPIOC_CLK_DISABLE();  //PC15 -> KEY_BUTTON_SINGAL
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI); // 进入停止模式（这边配置的是STOP2） 
}

/*
  从停止模式下唤醒，重新配置时钟
*/
void SystemClock_ReConfig_When_Exit_StopMode(void)
{
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_11);// 配置MSI为48MHz
	
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET);//等待准备就绪
	
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_MSI);  //配置MSI为系统时钟源
  while (__HAL_RCC_GET_SYSCLK_SOURCE()== RESET);  //等待准备就绪
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
      也可选择提供给RTC 用于停止 / 待机模式下的自动唤醒。
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
  RCC_OscInitStruct.MSIClockRange  = RCC_MSIRANGE_11; //配置为48MHz
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE; 
	
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  // 需要配置的时钟HCLK(APB1/APB2总线时钟源)、SYSCLK(系统时钟源)、PCLK1(APB1上外设时钟源)、PCLK2(APB2上外设时钟源)
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
	
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  // 需要初始化的外设时钟:USART1/USART2/I2C1
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;  

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  // 配置内部主稳压器输出电压，配置为稳压器输出电压范围1模式，也就是：典型输出电压为1.2V，系统频率高达80MHz
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  // 配置系统定时器中断时间，配置为HCLK的千分频
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  // 配置系统定时器，配置为HCLK
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  // 系统定时器中断配置，设置系统定时器中断优先级最高（为0），且子优先级最高（为0）
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
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

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)//配置初始化
  {
    Error_Handler();
  }
  
  // //模拟滤波配置
  // if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // //数字滤波配置
  // if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  
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
  hiwdg.Instance = IWDG; // 配置为IWDG
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;// 32分频，prer = 3
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
/*独立看门狗喂狗函数，也就是清除计数值。*/
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
  hspi3.Init.Mode        = SPI_MODE_MASTER;   //配置为Master      
  hspi3.Init.Direction   = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize    = SPI_DATASIZE_8BIT;     //8位数据模式
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase    = SPI_PHASE_1EDGE;
  hspi3.Init.NSS         = SPI_NSS_SOFT;             //NSS为软件控制    
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//配置SPI3分频系数 SPI最高频率有限制
  hspi3.Init.FirstBit    = SPI_FIRSTBIT_MSB;   
  hspi3.Init.TIMode      = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial  = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode  = SPI_NSS_PULSE_ENABLE;
	
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_1_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pin : NC_2_Pin */
  GPIO_InitStruct.Pin  = NC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NC_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEI_SIN_Pin */
  GPIO_InitStruct.Pin  = KEY_SIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_SIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J1_1_Pin /J1_2_Pin /J1_4_Pin /J1_5_Pin /AD7193_PWR_SWITCH_Pin */
  GPIO_InitStruct.Pin   = J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin|AD7193_PWR_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMPER_SIN_Pin /DEV_LED_Pin /TEMPER_PWR_SWITCH_Pin /AD7193_CS_Pin */
  GPIO_InitStruct.Pin   = TEMPER_SIN_Pin|DEV_LED_Pin|TEMPER_PWR_SWITCH_Pin|BMI160_PWR_SWITCH_Pin;
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
  HAL_GPIO_WritePin(GPIOA, J1_1_Pin|J1_2_Pin|J1_4_Pin|J1_5_Pin|TEMPER_SIN_Pin|TEMPER_PWR_SWITCH_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, AD7193_PWR_SWITCH_Pin|DEV_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AD7193_CS_Pin, GPIO_PIN_SET); //PA15
	HAL_GPIO_WritePin(GPIOA, BMI160_PWR_SWITCH_Pin, GPIO_PIN_SET); //PA5

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
