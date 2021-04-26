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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define PRINT_RAW_PEDALS 0
#define DEBUG_AMS 1
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
#define DASH_LED_POWER_Pin GPIO_PIN_3
#define DASH_LED_POWER_GPIO_Port GPIOC
#define BRAKE_PRESSURE_Pin GPIO_PIN_2
#define BRAKE_PRESSURE_GPIO_Port GPIOA
#define BRAKE_PEDAL_1_Pin GPIO_PIN_3
#define BRAKE_PEDAL_1_GPIO_Port GPIOA
#define PEDAL_ACCEL_1_Pin GPIO_PIN_4
#define PEDAL_ACCEL_1_GPIO_Port GPIOA
#define BRAKE_PEDAL_2_Pin GPIO_PIN_5
#define BRAKE_PEDAL_2_GPIO_Port GPIOA
#define PEDAL_ACCEL_2_Pin GPIO_PIN_6
#define PEDAL_ACCEL_2_GPIO_Port GPIOA
#define PEDAL_ACCEL_3_Pin GPIO_PIN_4
#define PEDAL_ACCEL_3_GPIO_Port GPIOC
#define HSOUT_RTD_LED_Pin GPIO_PIN_8
#define HSOUT_RTD_LED_GPIO_Port GPIOE
#define DASH_POWER_Pin GPIO_PIN_11
#define DASH_POWER_GPIO_Port GPIOE
#define RTD_INPUT_Pin GPIO_PIN_15
#define RTD_INPUT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
