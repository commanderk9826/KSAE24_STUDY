/*
 * can_it.h
 *
 *  Created on: Feb 28, 2023
 *      Author: joy54
 */

#ifndef INC_CAN_IT_H_
#define INC_CAN_IT_H_

#include "main.h"
#include "can.h"
#include "stm32f4xx_it.h"

void CAN_Filter_defunc(CAN_FilterTypeDef* filter);
void CAN_TX_Header_defunc(CAN_TxHeaderTypeDef* TX_Header, uint32_t ID);
void CAN_Error_Handler(CAN_HandleTypeDef* hcan, CAN_FilterTypeDef* filter);
void Transmit_CAN(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TX_Header, uint8_t data[], uint32_t *pTxMailbox);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);

extern uint8_t MCUData[8];
extern uint8_t BMSData[8];
extern uint8_t GyroData[8];
extern uint8_t TXData[8];

#endif /* INC_CAN_IT_H_ */
