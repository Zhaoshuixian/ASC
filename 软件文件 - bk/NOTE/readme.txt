

----功能引脚----

//PA0 - PA1 - PA2(U2TX) - PA32(U2RX) - PA4 
 
//PA5(BMI160-VCC-SWITCH)

//PA6(AD7193-VCC-SWITCH)

//PB0(EXT-INT-TRIG)

//PA8(LED)

//PA9(U1TX) PA10(U1RX)

//PA11(数字TEMP信号)  //若是ADC型需要作变更

//PA12(TEMP-VCC-SWITCH)

//PA15(SPI1-CS) PB3-PB5

//PB6-PB7(I2C1)

//PC15(KEY1)

//PC14 PB1 PA13 PA14  --(NC)


进低功耗前的配置：
1. 如果有独立看门狗，需要在上电初始化时，通过修改FLASH寄存器的相关位，使看门狗在进入stop模式后停止计数，就不会引起看门狗复位了
2.对应的外设SPI，调用对应外设的对应DeInit函数，要注意的是17版的库，SPI的DeInit函数有BUG，
需要按照我另一篇文章（STM32L4退出低功耗后SPI读写出错）进行修改，不然低功耗唤醒后，SPI读写会异常
调用函数： HAL_SPI_MspDeInit(&hspi2);
3.外设ADC，需要关闭，不然会增加功耗，
4.usart关闭
调用函数：HAL_UART_DeInit(&huart3); __HAL_RCC_USART3_CLK_DISABLE();
5.定时器，可以关闭也可以不关闭，不受影响
调用函数： HAL_TIM_Base_Stop_IT(&htim2);
6.GPIO配置，所有管脚设置为模拟输入模式，降低功耗
7.设置唤醒管脚和RTC唤醒时钟
8.调用进入低功耗的函数，进低功耗

出低功耗后的配置：
1.恢复时钟配置（若是用的MSI时钟可以不进行配置）
2.GPIO初始化
3.配置外设SPI，USART，ADC，定时器
4.关闭RTC周期唤醒（防止在程序正常运行时，进入RTC周期唤醒中断）
stop模式下停止看门狗计数

