

#ifndef __MISC_H__
#define __MISC_H__

#include "stm32l4xx_hal.h"



#define DEBUG_MODE

#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#define UART1_LOG(...)   LOG("[UART1]"##__VA_ARGS__) 
#define UART2_LOG(...)   LOG("[UART2]"##__VA_ARGS__) 
#define BMI160_LOG(...)  LOG("[BMI160]"##__VA_ARGS__) 
#define AD7193_LOG(...)  LOG("[AD7193]"##__VA_ARGS__) 

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32l4xx_hal.h"

#define DEVICE_INIT_OK     (0)
#define DEVICE_INIT_ERROR  (1)

void Error_Handler(void);


/*PORT A*/
#define J1_1_Pin GPIO_PIN_0
#define J1_1_GPIO_Port GPIOA

#define J1_2_Pin GPIO_PIN_1
#define J1_2_GPIO_Port GPIOA

#define U2_TX_Pin GPIO_PIN_2
#define U2_TX_GPIO_Port GPIOA

#define U2_RX_Pin GPIO_PIN_3
#define U2_RX_GPIO_Port GPIOA

#define J1_3_Pin GPIO_PIN_4
#define J1_3_GPIO_Port GPIOA

#define BMI160_VDD_SWITCH_GPIO_Port GPIOA
#define BMI160_VDD_SWITCH_Pin  GPIO_PIN_5

#define AD7193_VDD_SWITCH_Pin GPIO_PIN_6
#define AD7193_VDD_SWITCH_GPIO_Port GPIOA

#define EXT_VDD_SWITCH_Pin GPIO_PIN_7
#define EXT_VDD_SWITCH_GPIO_Port GPIOA

#define DEV_LED_Pin GPIO_PIN_8
#define DEV_LED_GPIO_Port GPIOA

/*串口1*/
#define U1_TX_Pin GPIO_PIN_9
#define U1_TX_GPIO_Port GPIOA

#define U1_RX_Pin GPIO_PIN_10
#define U1_RX_GPIO_Port GPIOA

/*外部温度传感器*/
#define TEMPER_SIN_Pin GPIO_PIN_11
#define TEMPER_SIN_GPIO_Port GPIOA

#define TEMPER_VDD_SWITCH_Pin GPIO_PIN_12
#define TEMPER_VDD_SWITCH_GPIO_Port GPIOA

/*SWD DEBUG*/
#define SWD_DIO_Pin GPIO_PIN_13
#define SWD_DIO_GPIO_Port GPIOA

#define SWD_SCK_Pin GPIO_PIN_14
#define SWD_SCK_GPIO_Port GPIOA

/*AD7193 SOFT CS*/
#define AD7193_CS_Pin GPIO_PIN_15
#define AD7193_CS_GPIO_Port GPIOA

/*PORT B*/

#define EXT_TRIG_Pin GPIO_PIN_0
#define EXT_TRIG_GPIO_Port GPIOB
#define EXT_TRIG_EXTI_IRQn EXTI0_IRQn

#define NC_1_Pin GPIO_PIN_1
#define NC_1_GPIO_Port GPIOB

#define AD7193_SCK_Pin GPIO_PIN_3
#define AD7193_SCK_GPIO_Port GPIOB

#define AD7193_MISO_Pin GPIO_PIN_4
#define AD7193_MISO_GPIO_Port GPIOB

#define AD7193_MOSI_Pin GPIO_PIN_5
#define AD7193_MOSI_GPIO_Port GPIOB

#define BMI160_SCL_Pin GPIO_PIN_6
#define BMI160_SCL_GPIO_Port GPIOB

#define BMI160_SDA_Pin GPIO_PIN_7
#define BMI160_SDA_GPIO_Port GPIOB


/*PORT C*/
#define NC_2_Pin GPIO_PIN_14
#define NC_2_GPIO_Port GPIOC

#define KEY_SIN_Pin GPIO_PIN_15
#define KEY_SIN_GPIO_Port GPIOC
#define KEI_SIN_EXTI_IRQn EXTI15_10_IRQn

#ifdef __cplusplus
}
#endif


#endif



