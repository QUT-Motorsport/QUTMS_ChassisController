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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define PRINT_RAW_PEDALS 0
#define DEBUG_AMS 0

// 0 - brakes must be actuated for RTD, 1 - brakes not required
#define RTD_DEBUG 0

// is brakes missing an issue
#define BRAKE_NON_CRITICAL 1

// is steering missing an issue
#define STEERING_NON_CRITICAL 1
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
#define STEERING_1_Pin GPIO_PIN_3
#define STEERING_1_GPIO_Port GPIOA
#define PEDAL_ACCEL_1_Pin GPIO_PIN_4
#define PEDAL_ACCEL_1_GPIO_Port GPIOA
#define STEERING_2_Pin GPIO_PIN_5
#define STEERING_2_GPIO_Port GPIOA
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

// number of MCISO boards in car
#define MCISO_COUNT 2

// 100ms
#define PERIPHERAL_RETRY 100

// 1000ms
#define PERIPHERAL_TIMEOUT 1000

// 500ms
#define SENSOR_RETRY 500

// 5000ms
#define SENSOR_TIMEOUT 5000

// 300ms
#define HEARTBEAT_TIMEOUT 300

// 250ms
#define HEARTBEAT_PRINT_TIME 250

// 1000ms
#define UART_TIMEOUT 1000

// 10% of 1000
#define APPS_DIFF 300

// usually +-2 of each other, so 10 is massive bad
#define STEER_IMP_DIFF 10

// 100ms
#define SENSOR_IMPLAUSIBILITY_TIMEOUT 100

// 50mv
#define ADC_DISCONNECT_CUTOFF 50

// inverter settings

#define INV_MAX_CURRENT 120
#define INV_DEADZONE_MIN 400
#define INV_DEADZONE_MAX 500

#define INV_REGEN_ENABLE 1
#define INV_REGEN_KMH_CUTOFF 10
#define INV_REGEN_MAX_CURRENT 60

#define INV_TV_ENABLE 0
#define INV_TV_DEADZONE 10
#define INV_TV_SCALAR 0
#define INV_TV_BOOST 0

#define WHEEL_RADIUS 0.4064

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
