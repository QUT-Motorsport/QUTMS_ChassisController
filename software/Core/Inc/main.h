/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SEM_ACQUIRE_TIMEOUT 32U // Milliseconds
#define SEM_ACQUIRE_GLOBALSTATE_TIMEOUT 64U // Milliseconds, might need a longer timeout for global states.
#define CC_HEARTBEAT_PERIOD 75U // Milliseconds
#define CC_IDC_PERIOD 250U // Milliseconds
#define CC_CAN_QUEUESIZE 10
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void CC_LogInfo(char* msg, size_t length);
__NO_RETURN void fsm_thread_mainLoop(void* arg);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BRAKE_PRESSURE_Pin GPIO_PIN_2
#define BRAKE_PRESSURE_GPIO_Port GPIOA
#define BRAKE_PEDAL_ONE_Pin GPIO_PIN_3
#define BRAKE_PEDAL_ONE_GPIO_Port GPIOA
#define BRAKE_PEDAL_TWO_Pin GPIO_PIN_5
#define BRAKE_PEDAL_TWO_GPIO_Port GPIOA
#define HSOUT_RTD_LED_Pin GPIO_PIN_8
#define HSOUT_RTD_LED_GPIO_Port GPIOE
#define RTD_INPUT_Pin GPIO_PIN_15
#define RTD_INPUT_GPIO_Port GPIOE
#define RTD_INPUT_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
