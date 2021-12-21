/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <QUTMS_can.h>

message_queue_t queue_CAN1;
message_queue_t queue_CAN2;
message_queue_t queue_CAN3;

uint32_t txMailbox_CAN1;
uint32_t txMailbox_CAN2;
uint32_t txMailbox_CAN3;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}
/* CAN3 init function */
void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;
static uint32_t HAL_RCC_CAN3_CLK_ENABLED=0;
static uint32_t HAL_RCC_CAN2_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    HAL_RCC_CAN3_CLK_ENABLED++;
    if(HAL_RCC_CAN3_CLK_ENABLED==1){
      __HAL_RCC_CAN3_CLK_ENABLE();
    }
    HAL_RCC_CAN2_CLK_ENABLED++;
    if(HAL_RCC_CAN2_CLK_ENABLED==1){
      __HAL_RCC_CAN2_CLK_ENABLE();
    }
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspInit 0 */

  /* USER CODE END CAN3_MspInit 0 */
    /* CAN3 clock enable */
    HAL_RCC_CAN3_CLK_ENABLED++;
    if(HAL_RCC_CAN3_CLK_ENABLED==1){
      __HAL_RCC_CAN3_CLK_ENABLE();
    }
    HAL_RCC_CAN2_CLK_ENABLED++;
    if(HAL_RCC_CAN2_CLK_ENABLED==1){
      __HAL_RCC_CAN2_CLK_ENABLE();
    }
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN3 GPIO Configuration
    PA8     ------> CAN3_RX
    PA15     ------> CAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN3 interrupt Init */
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN3_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX1_IRQn);
  /* USER CODE BEGIN CAN3_MspInit 1 */

  /* USER CODE END CAN3_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN3_CLK_ENABLED--;
    if(HAL_RCC_CAN3_CLK_ENABLED==0){
      __HAL_RCC_CAN3_CLK_DISABLE();
    }
    HAL_RCC_CAN2_CLK_ENABLED--;
    if(HAL_RCC_CAN2_CLK_ENABLED==0){
      __HAL_RCC_CAN2_CLK_DISABLE();
    }
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspDeInit 0 */

  /* USER CODE END CAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN3_CLK_ENABLED--;
    if(HAL_RCC_CAN3_CLK_ENABLED==0){
      __HAL_RCC_CAN3_CLK_DISABLE();
    }
    HAL_RCC_CAN2_CLK_ENABLED--;
    if(HAL_RCC_CAN2_CLK_ENABLED==0){
      __HAL_RCC_CAN2_CLK_DISABLE();
    }
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN3 GPIO Configuration
    PA8     ------> CAN3_RX
    PA15     ------> CAN3_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_15);

    /* CAN3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_RX1_IRQn);
  /* USER CODE BEGIN CAN3_MspDeInit 1 */

  /* USER CODE END CAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
bool setup_CAN() {
	// setup CAN queues
	queue_init(&queue_CAN1, sizeof(CAN_MSG_Generic_t));
	queue_init(&queue_CAN2, sizeof(CAN_MSG_Generic_t));
	queue_init(&queue_CAN3, sizeof(CAN_MSG_Generic_t));

	// start CAN lines
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		printf("ERROR: FAILED TO START CAN1\r\n");
		return false;
	}
	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		printf("ERROR: FAILED TO START CAN2\r\n");
		return false;
	}
	if (HAL_CAN_Start(&hcan3) != HAL_OK) {
		printf("ERROR: FAILED TO START CAN3\r\n");
		return false;
	}

	// setup CAN filters
	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0001;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		printf("failed to config filter on can1\r\n");
		return false;
	}

	sFilterConfig.FilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		printf("failed to config filter on can2\r\n");
		return false;
	}

	sFilterConfig.FilterBank = 28;

	if (HAL_CAN_ConfigFilter(&hcan3, &sFilterConfig) != HAL_OK) {
		printf("failed to config filter on can3\r\n");
		return false;
	}

	// activate notifications / interrupts
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN1 notification on RX0");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN1 notification on RX1");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN2 notification on RX0");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN2 notification on RX1");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN3 notification on RX0");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CAN3 notification on RX1");
		return false;
	}

	return true;
}

void handle_CAN_interrupt(CAN_HandleTypeDef *hcan, int fifo) {
	__disable_irq();
	//int fill = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);

	CAN_MSG_Generic_t msg;
	CAN_RxHeaderTypeDef header;
	CAN_TxHeaderTypeDef headerTx;

	while(HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
	//for (int i = 0; i < fill; i++) {
		if (HAL_CAN_GetRxMessage(hcan, fifo, &header, msg.data) != HAL_OK) {
			printf("failed to read CAN msg");
		}

		msg.hcan = hcan;
		msg.ID = header.IDE == CAN_ID_EXT ? header.ExtId : header.StdId;
		msg.ID_TYPE = header.IDE == CAN_ID_EXT ? 1 : 0;
		msg.DLC = header.DLC;
		msg.timestamp = HAL_GetTick();

		// add to CAN recieve queue
		if (hcan == &hcan1) {
			// if it's an inverter msg, forward onto CAN2
			headerTx.ExtId = header.ExtId;
			headerTx.StdId = header.StdId;
			headerTx.IDE = header.IDE;
			headerTx.RTR = header.RTR;
			headerTx.DLC = header.DLC;
			HAL_CAN_AddTxMessage(&hcan2, &headerTx, msg.data, &txMailbox_CAN2);

			queue_add(&queue_CAN1, &msg);
		} else if (hcan == &hcan2) {
			queue_add(&queue_CAN2, &msg);
		} else if (hcan == &hcan3) {
			queue_add(&queue_CAN3, &msg);
		}

		// add to CAN logging queue
		//queue_add(&queue_CAN_log, &msg);
	}
	__enable_irq();
}

uint8_t msg_count[3];
int test = 0;

HAL_StatusTypeDef CC_send_can_msg(CAN_HandleTypeDef *hcan,
		CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]) {
	// pull out the CAN msg for logging
	CAN_MSG_Generic_t msg;

	for (int i = 0; i < pHeader->DLC; i++) {
		msg.data[i] = aData[i];
	}

	msg.hcan = hcan;
	msg.ID = pHeader->IDE == CAN_ID_EXT ? pHeader->ExtId : pHeader->StdId;
	msg.ID_TYPE = pHeader->IDE == CAN_ID_EXT ? 1 : 0;
	msg.DLC = pHeader->DLC;
	msg.timestamp = HAL_GetTick();

	int can_idx = 0;

	uint32_t *pTxMailbox;
	if (hcan == &hcan1) {
		pTxMailbox = &txMailbox_CAN1;
		can_idx = 0;


	} else if (hcan == &hcan2) {
		pTxMailbox = &txMailbox_CAN2;
		can_idx = 1;
	} else if (hcan == &hcan3) {
		pTxMailbox = &txMailbox_CAN3;
		can_idx = 2;
	}


	// finally send CAN msg
	HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(hcan, pHeader, aData,
			pTxMailbox);
	if (result != HAL_OK) {
		printf("FAILED TO SEND CAN %i - e: %lu\r\n", can_idx + 1,
				hcan->ErrorCode);
	}

	return result;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
