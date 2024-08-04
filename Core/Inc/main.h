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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define V_C_Pin GPIO_PIN_2
#define V_C_GPIO_Port GPIOA
#define I_C_Pin GPIO_PIN_3
#define I_C_GPIO_Port GPIOA
#define V_B_Pin GPIO_PIN_4
#define V_B_GPIO_Port GPIOA
#define I_B_Pin GPIO_PIN_5
#define I_B_GPIO_Port GPIOA
#define V_A_Pin GPIO_PIN_6
#define V_A_GPIO_Port GPIOA
#define I_A_Pin GPIO_PIN_7
#define I_A_GPIO_Port GPIOA
#define V_VIN_Pin GPIO_PIN_0
#define V_VIN_GPIO_Port GPIOB
#define TEMP_C_Pin GPIO_PIN_1
#define TEMP_C_GPIO_Port GPIOB
#define TEMP_B_Pin GPIO_PIN_2
#define TEMP_B_GPIO_Port GPIOB
#define TEMP_A_Pin GPIO_PIN_10
#define TEMP_A_GPIO_Port GPIOB
#define POT_Pin GPIO_PIN_12
#define POT_GPIO_Port GPIOB
#define GLA_Pin GPIO_PIN_13
#define GLA_GPIO_Port GPIOB
#define GLB_Pin GPIO_PIN_14
#define GLB_GPIO_Port GPIOB
#define GLC_Pin GPIO_PIN_15
#define GLC_GPIO_Port GPIOB
#define GHA_Pin GPIO_PIN_8
#define GHA_GPIO_Port GPIOA
#define GHB_Pin GPIO_PIN_9
#define GHB_GPIO_Port GPIOA
#define GHC_Pin GPIO_PIN_10
#define GHC_GPIO_Port GPIOA
#define TOGGLE_SW_Pin GPIO_PIN_11
#define TOGGLE_SW_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_1
#define LED_R_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
