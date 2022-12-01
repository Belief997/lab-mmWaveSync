/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DIO0_Pin GPIO_PIN_0
#define DIO0_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_1
#define DIO1_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_4
#define DIO2_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_0
#define DIO3_GPIO_Port GPIOB
#define DIO4_Pin GPIO_PIN_1
#define DIO4_GPIO_Port GPIOB
#define DIO5_Pin GPIO_PIN_10
#define DIO5_GPIO_Port GPIOB
#define LED_KEY_Pin GPIO_PIN_12
#define LED_KEY_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI4_15_IRQn
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI4_15_IRQn
#define BTN3_Pin GPIO_PIN_15
#define BTN3_GPIO_Port GPIOB
#define BTN3_EXTI_IRQn EXTI4_15_IRQn
#define Pulse_Pin GPIO_PIN_8
#define Pulse_GPIO_Port GPIOA
#define MS_Select_Pin GPIO_PIN_11
#define MS_Select_GPIO_Port GPIOA
#define Lora_Rst_Pin GPIO_PIN_12
#define Lora_Rst_GPIO_Port GPIOA
#define Lora_CS_Pin GPIO_PIN_8
#define Lora_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
