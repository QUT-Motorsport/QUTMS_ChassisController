/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <Timer.h>
#include <queue.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

/* USER CODE BEGIN Private defines */
#define CAN_QUEUE_SIZE 50

extern ms_timer_t timer_CAN_queue;
extern message_queue_t queue_CAN1;
extern message_queue_t queue_CAN2;
extern message_queue_t queue_CAN3;

extern uint32_t txMailbox_CAN1;
extern uint32_t txMailbox_CAN2;
extern uint32_t txMailbox_CAN3;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_CAN3_Init(void);

/* USER CODE BEGIN Prototypes */
void setup_CAN();
void CAN_timer_cb(void *args);

HAL_StatusTypeDef CC_send_can_msg(CAN_HandleTypeDef *hcan,
		CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

// called from CAN interrupts, just adds any messages to the CAN queues
void handle_CAN_interrupt(CAN_HandleTypeDef *hcan, int fifo);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
