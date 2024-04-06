/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define ENCODER_FL_Pin GPIO_PIN_6
#define ENCODER_FL_GPIO_Port GPIOA
#define ENCODER_FR_Pin GPIO_PIN_7
#define ENCODER_FR_GPIO_Port GPIOA
#define ENCODER_BL_Pin GPIO_PIN_0
#define ENCODER_BL_GPIO_Port GPIOB
#define ENCODER_BR_Pin GPIO_PIN_1
#define ENCODER_BR_GPIO_Port GPIOB
#define MOTOR_FL_DIR_Pin GPIO_PIN_12
#define MOTOR_FL_DIR_GPIO_Port GPIOB
#define MOTOR_FR_DIR_Pin GPIO_PIN_13
#define MOTOR_FR_DIR_GPIO_Port GPIOB
#define MOTOR_BL_DIR_Pin GPIO_PIN_14
#define MOTOR_BL_DIR_GPIO_Port GPIOB
#define MOTOR_BR_DIR_Pin GPIO_PIN_15
#define MOTOR_BR_DIR_GPIO_Port GPIOB
#define MOTOR_FL_PWM_Pin GPIO_PIN_8
#define MOTOR_FL_PWM_GPIO_Port GPIOA
#define MOTOR_FR_PWM_Pin GPIO_PIN_9
#define MOTOR_FR_PWM_GPIO_Port GPIOA
#define MOTOR_BL_PWM_Pin GPIO_PIN_10
#define MOTOR_BL_PWM_GPIO_Port GPIOA
#define MOTOR_BR_PWM_Pin GPIO_PIN_11
#define MOTOR_BR_PWM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
