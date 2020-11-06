/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FSM.h"
#include "CC_FSM_States.h"
#include "CC_CAN_Messages.h"
#include "PDM_CAN_Messages.h"
#include "AMS_CAN_Messages.h"
#include "SHDN_IMD_CAN_Messages.h"
#include "SHDN_BSPD_CAN_Messages.h"
#include "SHDN_CAN_Messages.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
osThreadId_t fsmThread;
const osThreadAttr_t fsmThreadAttr = {
		.stack_size = 2048
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  MX_CAN3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Create CAN Filter & Apply it to &CANBUS41, &CANBUS2 and &CANBUS3 */
	CAN_FilterTypeDef sFilterConfig1;

	sFilterConfig1.FilterBank = 0;
	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig1.FilterIdHigh = 0x0000;
	sFilterConfig1.FilterIdLow = 0x0001;
	sFilterConfig1.FilterMaskIdHigh = 0x0000;
	sFilterConfig1.FilterMaskIdLow = 0x0000;
	sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig1.FilterActivation = ENABLE;
	sFilterConfig1.SlaveStartFilterBank = 14;

	CAN_FilterTypeDef sFilterConfig2;

	sFilterConfig2.FilterBank = 14;
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
	sFilterConfig2.FilterIdLow = 0x0001;
	sFilterConfig2.FilterMaskIdHigh = 0x0000;
	sFilterConfig2.FilterMaskIdLow = 0x0000;
	sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig2.FilterActivation = ENABLE;
	sFilterConfig2.SlaveStartFilterBank = 14;

	CAN_FilterTypeDef sFilterConfig3;

	sFilterConfig3.FilterBank = 0;
	sFilterConfig3.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig3.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig3.FilterIdHigh = 0x0000;
	sFilterConfig3.FilterIdLow = 0x0001;
	sFilterConfig3.FilterMaskIdHigh = 0x0000;
	sFilterConfig3.FilterMaskIdLow = 0x0000;
	sFilterConfig3.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig3.FilterActivation = ENABLE;
	sFilterConfig3.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_ConfigFilter(&hcan3, &sFilterConfig3) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	//Create FSM instance
	fsm_t *fsm = fsm_new(&startState);

	// Create a new thread, where our FSM will run.
	osThreadNew(fsm_thread_mainLoop, fsm, &fsmThreadAttr);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Creates and logs an error string to huart3
 * @note The form of the log message is as so: "TAG_subsystem: error"
 * @param TAG Primary System eg. "Chassis Controller"
 * @param Subsystem of error eg. "CAN SEND"
 * @param error Full error string
 * @retval None
 */
void CC_LogInfo(char* msg, size_t length)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)msg, length, HAL_MAX_DELAY);
}

/**
 * @brief FSM thread main loop task for RTOS
 * @param fsm the FSM object passed to the loop
 * @retval None
 */
__NO_RETURN void fsm_thread_mainLoop(void *fsm)
{
	CC_LogInfo("Entering FSM Thread\r\n", strlen("Entering FSM Thread\r\n"));
	fsm_setLogFunction(fsm, &CC_LogInfo);
	fsm_reset(fsm, &startState);
	fsm_changeState(fsm, &debugState, "Forcing debug state");
	for(;;)
	{
		while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
		{
			CC_CAN_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &(msg.header), msg.data);
			osMessageQueuePut(CC_GlobalState->CANQueue, &msg, 0U, 0U);
			char x[80];
			int len = sprintf(x, "[%li] Got CAN msg from CAN1: %02lX\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, msg.header.ExtId);
			//CC_LogInfo(x, len);
		}

		while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
		{
			CC_CAN_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &(msg.header), msg.data);
			osMessageQueuePut(CC_GlobalState->CANQueue, &msg, 0U, 0U);
			char x[80];
			int len = sprintf(x, "[%li] Got CAN msg from CAN2: %02lX\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, msg.header.ExtId);
			//CC_LogInfo(x, len);
		}

		while(HAL_CAN_GetRxFifoFillLevel(&hcan3, CAN_RX_FIFO0) > 0)
		{
			CC_CAN_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan3, CAN_RX_FIFO0, &(msg.header), msg.data);
			osMessageQueuePut(CC_GlobalState->CANQueue, &msg, 0U, 0U);
			char x[80];
			int len = sprintf(x, "[%li] Got CAN msg from CAN3: %02lX\r\n", (HAL_GetTick() - CC_GlobalState->startupTicks)/1000, msg.header.ExtId);
			//CC_LogInfo(x, len);
		}

		osDelay(20);
		fsm_iterate(fsm);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
