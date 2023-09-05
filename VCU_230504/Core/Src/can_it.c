/*
 * can_it.c
 *
 *  Created on: Feb 28, 2023
 *      Author: joy54
 */

#include "can_it.h"
#include <stdio.h>
#include "stm32f4xx_it.h"

extern CAN_HandleTypeDef hcan1;
CAN_RxHeaderTypeDef CAN_RXH;


uint32_t Mailbox;
uint8_t RXData[8] = {0};
uint8_t MCUData[8] = {0};
uint8_t BMSData[8] = {0};
uint8_t GyroData[8] = {0};
uint32_t txcount = 0;
uint32_t rxcount = 0;
uint16_t Motor_RPM_PM100dx=0;
uint8_t SOC_100 = 0;
//uint32_t count=0;

void CAN_Filter_defunc(CAN_FilterTypeDef* filter)
{
	filter->FilterActivation = CAN_FILTER_ENABLE; // filter on,off
	filter->FilterBank = 1; // filterbank initialize single can = 0~13, dual can = 0~27
	filter->FilterFIFOAssignment = CAN_FILTER_FIFO0; // fifo assgin 0 or 1
	filter->FilterIdHigh = 0x0000; //
	filter->FilterIdLow = 0x0000;
	filter->FilterMaskIdHigh = 0x0000;
	filter->FilterMaskIdLow = 0x0000;
	filter->FilterMode = CAN_FILTERMODE_IDMASK; // filter mode -> mask or list
	filter->FilterScale = CAN_FILTERSCALE_16BIT; // filter scale
	// filtername.SlaveStartFilterBank // only dual can

	HAL_CAN_ConfigFilter(&hcan1, filter);
}

void CAN_TX_Header_defunc(CAN_TxHeaderTypeDef* TX_Header, uint32_t ID)
{
	TX_Header->DLC = 8;
	TX_Header->IDE = CAN_ID_STD;
	TX_Header->RTR = CAN_RTR_DATA;
	TX_Header->TransmitGlobalTime = DISABLE;
	TX_Header->StdId = ID;
}

void CAN_Error_Handler(CAN_HandleTypeDef* hcan, CAN_FilterTypeDef* filter)
{
	if (HAL_CAN_ConfigFilter(hcan, filter) != HAL_OK)
	{
		// Filter configuration Error
		Error_Handler();
	}

	// Can Start
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		// Start Error
		Error_Handler();
	}

	// Activate CAN RX notification
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		// Notification Error
		Error_Handler();
	}
}

void Transmit_CAN(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TX_Header, uint8_t data[], uint32_t *pTxMailbox)
{
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan))
	{
		HAL_CAN_AddTxMessage(hcan, TX_Header, data, pTxMailbox);
		txcount++;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	rxcount++;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RXH, RXData) != HAL_OK)
    {
      Error_Handler();
    }

				switch(CAN_RXH.StdId)
				{
								case 0xa5: // pm100dx rpm
								 for(int i = 0;i<8;i++)
									 Motor_RPM_PM100dx =RXData[3]<<8 | RXData[2];
								break;

								case 0x09: // soc_100_dash
									SOC_100 = RXData[0];
									break;
				}
	}

