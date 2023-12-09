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
#include "stm32g0xx_hal.h"

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
void SetNumber(uint8_t, uint8_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HL_Switch1_Pin GPIO_PIN_13
#define HL_Switch1_GPIO_Port GPIOC
#define HL_Switch2_Pin GPIO_PIN_14
#define HL_Switch2_GPIO_Port GPIOC
#define HL_Switch3_Pin GPIO_PIN_15
#define HL_Switch3_GPIO_Port GPIOC
#define T_SENSOR_Pin GPIO_PIN_0
#define T_SENSOR_GPIO_Port GPIOA
#define EN_SW_Pin GPIO_PIN_5
#define EN_SW_GPIO_Port GPIOA
#define EN_SW_EXTI_IRQn EXTI4_15_IRQn
#define EN_A_Pin GPIO_PIN_6
#define EN_A_GPIO_Port GPIOA
#define EN_B_Pin GPIO_PIN_7
#define EN_B_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOB
#define HL_DP_Pin GPIO_PIN_15
#define HL_DP_GPIO_Port GPIOA
#define HL_A_Pin GPIO_PIN_3
#define HL_A_GPIO_Port GPIOB
#define HL_B_Pin GPIO_PIN_4
#define HL_B_GPIO_Port GPIOB
#define HL_C_Pin GPIO_PIN_5
#define HL_C_GPIO_Port GPIOB
#define HL_D_Pin GPIO_PIN_6
#define HL_D_GPIO_Port GPIOB
#define HL_E_Pin GPIO_PIN_7
#define HL_E_GPIO_Port GPIOB
#define HL_F_Pin GPIO_PIN_8
#define HL_F_GPIO_Port GPIOB
#define HL_G_Pin GPIO_PIN_9
#define HL_G_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
