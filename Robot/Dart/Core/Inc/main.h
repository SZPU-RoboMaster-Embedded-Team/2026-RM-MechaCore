/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Limit_Pin GPIO_PIN_4
#define Limit_GPIO_Port GPIOE
#define LimitF0_Pin GPIO_PIN_0
#define LimitF0_GPIO_Port GPIOF
#define HX711_DT_Pin GPIO_PIN_1
#define HX711_DT_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOG
#define LEDG7_Pin GPIO_PIN_7
#define LEDG7_GPIO_Port GPIOG
#define LEDG6_Pin GPIO_PIN_6
#define LEDG6_GPIO_Port GPIOG
#define LEDG5_Pin GPIO_PIN_5
#define LEDG5_GPIO_Port GPIOG
#define LEDG4_Pin GPIO_PIN_4
#define LEDG4_GPIO_Port GPIOG
#define LEDG3_Pin GPIO_PIN_3
#define LEDG3_GPIO_Port GPIOG
#define LEDG2_Pin GPIO_PIN_2
#define LEDG2_GPIO_Port GPIOG
#define LimitC0_Pin GPIO_PIN_0
#define LimitC0_GPIO_Port GPIOC
#define LimitC1_Pin GPIO_PIN_1
#define LimitC1_GPIO_Port GPIOC
#define LEDG1_Pin GPIO_PIN_1
#define LEDG1_GPIO_Port GPIOG
#define LimitA4_Pin GPIO_PIN_4
#define LimitA4_GPIO_Port GPIOA
#define LimitB1_Pin GPIO_PIN_1
#define LimitB1_GPIO_Port GPIOB
#define LimitB0_Pin GPIO_PIN_0
#define LimitB0_GPIO_Port GPIOB
#define LimitE12_Pin GPIO_PIN_12
#define LimitE12_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
