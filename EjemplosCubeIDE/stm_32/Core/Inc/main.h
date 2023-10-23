/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t digital_read(void);//return the value of digital inputs PB2, PB11, PB12, PA11, PA12, PC5, PC6, PC8
void digital_write(uint8_t);//write in digital outputs  PA10, PB5, PB4, PB10, PA8, PA9, PC7 , PB6

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DI_6_Pin GPIO_PIN_5
#define DI_6_GPIO_Port GPIOC
#define DI_1_Pin GPIO_PIN_2
#define DI_1_GPIO_Port GPIOB
#define DO_4_Pin GPIO_PIN_10
#define DO_4_GPIO_Port GPIOB
#define DI_2_Pin GPIO_PIN_11
#define DI_2_GPIO_Port GPIOB
#define DI_3_Pin GPIO_PIN_12
#define DI_3_GPIO_Port GPIOB
#define sel_0_Pin GPIO_PIN_13
#define sel_0_GPIO_Port GPIOB
#define sel_1_Pin GPIO_PIN_14
#define sel_1_GPIO_Port GPIOB
#define sel_2_Pin GPIO_PIN_15
#define sel_2_GPIO_Port GPIOB
#define DI_7_Pin GPIO_PIN_6
#define DI_7_GPIO_Port GPIOC
#define DO_7_Pin GPIO_PIN_7
#define DO_7_GPIO_Port GPIOC
#define DI_8_Pin GPIO_PIN_8
#define DI_8_GPIO_Port GPIOC
#define DO_5_Pin GPIO_PIN_8
#define DO_5_GPIO_Port GPIOA
#define DO_6_Pin GPIO_PIN_9
#define DO_6_GPIO_Port GPIOA
#define DO_1_Pin GPIO_PIN_10
#define DO_1_GPIO_Port GPIOA
#define DI_4_Pin GPIO_PIN_11
#define DI_4_GPIO_Port GPIOA
#define DI_5_Pin GPIO_PIN_12
#define DI_5_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DO_3_Pin GPIO_PIN_4
#define DO_3_GPIO_Port GPIOB
#define DO_2_Pin GPIO_PIN_5
#define DO_2_GPIO_Port GPIOB
#define DO_8_Pin GPIO_PIN_6
#define DO_8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
