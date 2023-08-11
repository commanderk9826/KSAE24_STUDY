/*
 * can_it.c
 *
 *  Created on: 2023. 2. 21.
 *      Author: KSB
 */

#include "can_it.h"
#include <stdio.h>
#include "main.h"
#include <math.h>
#define BUFFER_LEN 88

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef CAN_TXH;
CAN_RxHeaderTypeDef CAN_RXH;
CAN_FilterTypeDef filtername;
uint32_t Mailbox;

uint8_t TXData[8]  = {6,0,0,0,0,0,0,0}; // MCU CAN isn't used.
uint8_t RXData[8]     = {0}; // Whole CAN bus data.
// uint8_t VCUData[8] = {0}; // VCUData separated in RPM and Brake.
uint8_t BMSData_1[8]  = {0};
uint8_t BMSData_2[8]  = {0};
uint8_t BMSData_3[8]  = {0};
uint8_t BrakePData[8] = {0};
uint8_t GyroData[8]   = {0};
uint8_t test_count = 0;
extern uint8_t TX_BUFFER[BUFFER_LEN];

void CAN_RX_Header_defunc() //CAN_RX_HEADER
{
	filtername.FilterActivation = CAN_FILTER_ENABLE; // filter on,off
	filtername.FilterBank = 1; // filterbank initialize single can = 0~13, dual can = 0~27
	filtername.FilterFIFOAssignment = CAN_FILTER_FIFO0; // fifo assgin 0 or 1
	filtername.FilterIdHigh = 0x0000;
	filtername.FilterIdLow = 0x0000;
	filtername.FilterMaskIdHigh = 0x0000;
	filtername.FilterMaskIdLow = 0x0000;
	filtername.FilterMode = CAN_FILTERMODE_IDMASK; // filter mode -> mask or list
	filtername.FilterScale = CAN_FILTERSCALE_16BIT; // filter scale
	// filtername.SlaveStartFilterBank // only dual can

	HAL_CAN_ConfigFilter(&hcan1, &filtername);
}

void CAN_TX_Header_defunc() //CAN_TX_HEADER
{
	CAN_TXH.DLC = 8;
	CAN_TXH.IDE = CAN_ID_STD;
	CAN_TXH.RTR = CAN_RTR_DATA;
	CAN_TXH.StdId = 0xf3;
	CAN_TXH.TransmitGlobalTime = DISABLE;
}

void CAN_Error_Handler() // CAN_START & CAN_ERROR
{
	if (HAL_CAN_ConfigFilter(&hcan1, &filtername) != HAL_OK)
	{
		// Filter configuration Error
		Error_Handler();
	}

	// Can Start
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		// Start Error
		Error_Handler();
	}

	// Activate CAN RX notification
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK)
	{
		// Notification Error
		Error_Handler();
	}
}

/*void callbackSystick() //1ms count CAN_TX
{

	static int count=0;
	count++;
	if (count == 10)  //10ms send can tx message
	{

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			test_count++;

			HAL_CAN_AddTxMessage(&hcan1, &CAN_TXH, TXData, &Mailbox);
		}
		count = 0;
	}
}*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle) //CAN_RX
{
	if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &CAN_RXH, RXData) != HAL_OK)
	{
		Error_Handler();
	}

	switch (CAN_RXH.StdId)
	// selecting CAN id
	{
	case 0x01: // BMS #1
		for (int i = 0; i < 7; i++)
		{
			TX_BUFFER[i] = RXData[i];
		}
		break;

	case 0x02: // BMS #2
		for (int i = 0; i < 5; i++)
		{
			TX_BUFFER[i+7] = RXData[i];
		}
		break;
	case 0x03: // BMS #3
		for (int i = 0; i < 5; i++)
		{
			TX_BUFFER[i+12] = RXData[i];
		}
		break;

	case 0x104: // LV BMS bq76920
		TX_BUFFER[17] = RXData[1]; // pack voltage msb
		TX_BUFFER[18] = RXData[2]; // pack voltage lsb
		TX_BUFFER[82] = RXData[0]; // bms stat
		TX_BUFFER[83] = RXData[3]; // ntc temp msb
		TX_BUFFER[84] = RXData[4]; // ntc temp lsb
		TX_BUFFER[85] = RXData[5]; // curent msb
		TX_BUFFER[86] = RXData[6]; // current lsb
		TX_BUFFER[87] = RXData[7]; // count
		break;

	case 0x09: //SOC
		TX_BUFFER[19] = RXData[0]; // SOC uint8_t
		TX_BUFFER[20] = RXData[1]; // SOC .xx
		TX_BUFFER[21] = RXData[2]; // current uint8_t
		TX_BUFFER[22] = RXData[3]; // current .xx
		TX_BUFFER[23] = RXData[4]; // max current uint8_t
		break;

	case 0xa0: // Temperatures #1 GDB Temp.
		TX_BUFFER[24] = RXData[6];
		TX_BUFFER[25] = RXData[7];

		break;

	case 0xa1: // Temperatures #2 PCB Temp.
		TX_BUFFER[26] = RXData[0];
		TX_BUFFER[27] = RXData[1];

		break;

	case 0xa2: // Temperatures #3 no working  and Coolant Temp.
		TX_BUFFER[28] = RXData[0]; // no working
		TX_BUFFER[29] = RXData[1];
		TX_BUFFER[30] = RXData[4]; // motor temp
		TX_BUFFER[31] = RXData[5];
		break;


	case 0xa3: //Analog Input Voltages  10bit , apps

		TX_BUFFER[32] = RXData[0];
		TX_BUFFER[33] = RXData[1];
		break;

	case 0xa5: //Motor Position Information (rpm) Motor Angular velocity
		TX_BUFFER[34] = RXData[2];
		TX_BUFFER[35] = RXData[3];
		break;

	case 0xa6: //Current Information
		TX_BUFFER[36] = RXData[0];
		TX_BUFFER[37] = RXData[1]; // Phase A
		TX_BUFFER[38] = RXData[2];
		TX_BUFFER[39] = RXData[3]; // Phase B
		TX_BUFFER[40] = RXData[4];
		TX_BUFFER[41] = RXData[5]; // Phase C
		TX_BUFFER[42] = RXData[6];
		TX_BUFFER[43] = RXData[7]; // DC Bus
		break;

	case 0xa7: //Voltage Information
		TX_BUFFER[44] = RXData[0];
		TX_BUFFER[45] = RXData[1]; // DC Bus
		TX_BUFFER[46] = RXData[2];
		TX_BUFFER[47] = RXData[3]; // Output
		TX_BUFFER[48] = RXData[4];
		TX_BUFFER[49] = RXData[5]; // VAB_Vd
		TX_BUFFER[50] = RXData[6];
		TX_BUFFER[51] = RXData[7]; // VAB_Vq
		break;

	case 0xa8: //dq
		TX_BUFFER[52] = RXData[4];
		TX_BUFFER[53] = RXData[5]; // ID_current
		TX_BUFFER[54] = RXData[6];
		TX_BUFFER[55] = RXData[7]; // IQ_current
		break;

	case 0xaa: //Internal States
		TX_BUFFER[56] = RXData[0]; // VSM State
		TX_BUFFER[57] = RXData[2]; // Inverter State
		break;

	case 0xac: //Torque & Timer Information
		TX_BUFFER[58] = RXData[0];
		TX_BUFFER[59] = RXData[1]; // Commanded Torque
		TX_BUFFER[60] = RXData[2];
		TX_BUFFER[61] = RXData[3]; // Torque Feedback
		break;

	case 0x1b1: // Brake pressure
		TX_BUFFER[62] = RXData[0];
		TX_BUFFER[63] = RXData[1];
		break;

	case 0xff: //GYRO_CAN
		for (int i = 0; i < 8; i++)
			GyroData[i] = RXData[i];

		if (GyroData[1] == 0x33) //acc
		{
			TX_BUFFER[64] = GyroData[2];
			TX_BUFFER[65] = GyroData[3];
			TX_BUFFER[66] = GyroData[4];
			TX_BUFFER[67] = GyroData[5];
			TX_BUFFER[68] = GyroData[6];
			TX_BUFFER[69] = GyroData[7];
		}
		if (GyroData[1] == 0x34) // gyro
		{
			TX_BUFFER[70] = GyroData[2];
			TX_BUFFER[71] = GyroData[3];
			TX_BUFFER[72] = GyroData[4];
			TX_BUFFER[73] = GyroData[5];
			TX_BUFFER[74] = GyroData[6];
			TX_BUFFER[75] = GyroData[7];
		}
		if (GyroData[1] == 0x35) // ang
		{
			TX_BUFFER[76] = GyroData[2];
			TX_BUFFER[77] = GyroData[3];
			TX_BUFFER[78] = GyroData[4];
			TX_BUFFER[79] = GyroData[5];
			TX_BUFFER[80] = GyroData[6];
			TX_BUFFER[81] = GyroData[7];
		}
		break;
	} //switch end

}
