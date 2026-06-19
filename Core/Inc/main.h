/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_Pin GPIO_PIN_0
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_1
#define ADC2_GPIO_Port GPIOA
#define MOTOR6_Pin GPIO_PIN_2
#define MOTOR6_GPIO_Port GPIOA
#define HEATER_Pin GPIO_PIN_3
#define HEATER_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_4
#define ADC3_GPIO_Port GPIOA
#define POLAR_Pin GPIO_PIN_5
#define POLAR_GPIO_Port GPIOA
#define SERVO1_Pin GPIO_PIN_6
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_7
#define SERVO2_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_1
#define SERVO4_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_8
#define MOTOR1_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_9
#define MOTOR2_GPIO_Port GPIOA
#define MOTOR3_Pin GPIO_PIN_10
#define MOTOR3_GPIO_Port GPIOA
#define MOTOR4_Pin GPIO_PIN_15
#define MOTOR4_GPIO_Port GPIOA
#define MOTOR5_Pin GPIO_PIN_3
#define MOTOR5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
