/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32l4xx_hal.h"

#define DEVICE_INIT_OK     (0)
#define DEVICE_INIT_ERROR  (1)

void Error_Handler(void);


#define NC_2_Pin GPIO_PIN_14
#define NC_2_GPIO_Port GPIOC
#define KEY_SIN_Pin GPIO_PIN_15
#define KEY_SIN_GPIO_Port GPIOC
#define KEI_SIN_EXTI_IRQn EXTI15_10_IRQn
#define J1_1_Pin GPIO_PIN_0
#define J1_1_GPIO_Port GPIOA
#define J1_2_Pin GPIO_PIN_1
#define J1_2_GPIO_Port GPIOA
#define U2_TX_Pin GPIO_PIN_2
#define U2_TX_GPIO_Port GPIOA
#define U2_RX_Pin GPIO_PIN_3
#define U2_RX_GPIO_Port GPIOA
#define J1_4_Pin GPIO_PIN_4
#define J1_4_GPIO_Port GPIOA
#define J1_5_Pin GPIO_PIN_5
#define J1_5_GPIO_Port GPIOA
#define AD7193_PWR_SWITCH_Pin GPIO_PIN_6
#define AD7193_PWR_SWITCH_GPIO_Port GPIOA
#define TEMPER_SIN_Pin GPIO_PIN_7
#define TEMPER_SIN_GPIO_Port GPIOA
#define EXT_TRIG_Pin GPIO_PIN_0
#define EXT_TRIG_GPIO_Port GPIOB
#define EXT_TRIG_EXTI_IRQn EXTI0_IRQn
#define NC_1_Pin GPIO_PIN_1
#define NC_1_GPIO_Port GPIOB
#define DEV_LED_Pin GPIO_PIN_8
#define DEV_LED_GPIO_Port GPIOA
#define U1_TX_Pin GPIO_PIN_9
#define U1_TX_GPIO_Port GPIOA
#define U1_RX_Pin GPIO_PIN_10
#define U1_RX_GPIO_Port GPIOA
#define TEMPER_PWR_SWITCH_Pin GPIO_PIN_12
#define TEMPER_PWR_SWITCH_GPIO_Port GPIOA
#define SWD_DIO_Pin GPIO_PIN_13
#define SWD_DIO_GPIO_Port GPIOA
#define SWD_SCK_Pin GPIO_PIN_14
#define SWD_SCK_GPIO_Port GPIOA
#define AD7193_CS_Pin GPIO_PIN_15
#define AD7193_CS_GPIO_Port GPIOA
#define AD7193_SCK_Pin GPIO_PIN_3
#define AD7193_SCK_GPIO_Port GPIOB
#define AD7193_MISO_Pin GPIO_PIN_4
#define AD7193_MISO_GPIO_Port GPIOB
#define AD7193_MOSI_Pin GPIO_PIN_5
#define AD7193_MOSI_GPIO_Port GPIOB


#define BMI160_PWR_SWITCH_GPIO_Port GPIOA
#define BMI160_PWR_SWITCH_Pin  GPIO_PIN_5

#define BMI160_SCL_Pin GPIO_PIN_6
#define BMI160_SCL_GPIO_Port GPIOB

#define BMI160_SDA_Pin GPIO_PIN_7
#define BMI160_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
