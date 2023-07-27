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
#include "stm32l4xx_hal.h"

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
#define SPI1_NSS1_Pin GPIO_PIN_4
#define SPI1_NSS1_GPIO_Port GPIOA
#define SPI1_NSS2_Pin GPIO_PIN_5
#define SPI1_NSS2_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_10
#define PWM4_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_11
#define PWM5_GPIO_Port GPIOB
#define FAN_RPM4_Pin GPIO_PIN_14
#define FAN_RPM4_GPIO_Port GPIOB
#define FAN_RPM5_Pin GPIO_PIN_15
#define FAN_RPM5_GPIO_Port GPIOB
#define FAN_RPM1_Pin GPIO_PIN_6
#define FAN_RPM1_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_7
#define A0_GPIO_Port GPIOC
#define FAN_RPM3_Pin GPIO_PIN_8
#define FAN_RPM3_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_9
#define A1_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_10
#define A2_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_11
#define A3_GPIO_Port GPIOC
#define STAT_Pin GPIO_PIN_12
#define STAT_GPIO_Port GPIOC
#define FAN_RPM2_Pin GPIO_PIN_5
#define FAN_RPM2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
