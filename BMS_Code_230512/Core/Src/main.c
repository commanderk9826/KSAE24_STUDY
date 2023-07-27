/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct StructureSLAVEBMS
{
	float CV[18U];
	float GV[9U];
	float REF;
	float SC;
	float ITMP;
	float VA;
	float VD;
	float VUV;	/* CFGR */
	float VOV;	/* CFGR */
	float DCTO;
	float Max_Voltage;
	float Min_Voltage;
	uint8_t MaxTemp;

	unsigned short iITMP;

	unsigned char COV[18U];
	unsigned char CUV[18U];
	unsigned char REV;
	unsigned char MUXFAIL;
	unsigned char THSD;
	unsigned char GPIO_SET[9U];	/* CFGR */
	unsigned char REFON;		/* CFGR */
	unsigned char DTEN;			/* CFGR */
	unsigned char ADCOPT;		/* CFGR */
	unsigned char DCC[18U];
	unsigned char MUTE;
	unsigned char FDRF;
	unsigned char PS;
	unsigned char DTMEN;
	unsigned char DCC0;

} StructureSLAVEBMS;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SPI control definitions */
#define SLAVEBMSFORWARD_CS_H()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_SET);
#define SLAVEBMSFORWARD_CS_L()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_RESET);
#define SLAVEBMSREVERSE_CS_H()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_SET);
#define SLAVEBMSREVERSE_CS_L()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_RESET);
#define SLAVEBMSACCESS_FWD			(unsigned char)(0x01)
#define SLAVEBMSACCESS_REV			(unsigned char)(0x02)
#define SLAVEBMSACCESS_CRC_MISMATCH	(unsigned char)(0xFF)
#define SLAVEBMSACCESS_CRC_MATCH	(unsigned char)(0x00)

/* SLAVEBMS Order */
#define SLAVEBMS_CMD_WRCFGA			(unsigned short)(0x0001)
#define SLAVEBMS_CMD_WRCFGB			(unsigned short)(0x0024)
#define SLAVEBMS_CMD_WRPWM			(unsigned short)(0x0020)
#define SLAVEBMS_CMD_WRPSB			(unsigned short)(0x001C)
#define SLAVEBMS_CMD_RDCFGA			(unsigned short)(0x0002)
#define SLAVEBMS_CMD_RDCFGB			(unsigned short)(0x0026)
#define SLAVEBMS_CMD_RDCVA			(unsigned short)(0x0004)
#define SLAVEBMS_CMD_RDCVB			(unsigned short)(0x0006)
#define SLAVEBMS_CMD_RDCVC			(unsigned short)(0x0008)
#define SLAVEBMS_CMD_RDCVD			(unsigned short)(0x000A)
#define SLAVEBMS_CMD_RDCVE			(unsigned short)(0x0009)
#define SLAVEBMS_CMD_RDCVF			(unsigned short)(0x000B)
#define SLAVEBMS_CMD_RDAUXA			(unsigned short)(0x000C)
#define SLAVEBMS_CMD_RDAUXB			(unsigned short)(0x000E)
#define SLAVEBMS_CMD_RDAUXC			(unsigned short)(0x000D)
#define SLAVEBMS_CMD_RDAUXD			(unsigned short)(0x000F)
#define SLAVEBMS_CMD_RDSTATA		(unsigned short)(0x0010)
#define SLAVEBMS_CMD_RDSTATB		(unsigned short)(0x0012)
#define SLAVEBMS_CMD_ADCV			(unsigned short)(0x0260)
#define SLAVEBMS_CMD_ADAX			(unsigned short)(0x0460)
#define SLAVEBMS_CMD_ADSTAT			(unsigned short)(0x0468)
#define SLAVEBMS_CMD_MUTE			(unsigned short)(0x0028)
#define SLAVEBMS_CMD_UNMUTE			(unsigned short)(0x0029)

#define SLAVEBMS_CMD_CLRCELL		(unsigned short)(0x0711)	/* Clear cell voltage Data_Array group */
#define SLAVEBMS_CMD_CLRAUX			(unsigned short)(0x0712)	/* Clear auxiliary Data_Array group */
#define SLAVEBMS_CMD_CLRSTAT   		(unsigned short)(0x0713)	/* Clear status Data_Array group */

#define SLAVEBMS_CMD_PLADC			(unsigned short)(0x0714)	/* Poll ADC conversion status */
#define SLAVEBMS_CBD_MD_27K			(unsigned short)((unsigned short)(0x0001) << 7)		/* ADCOPT = 0, FAST      */
#define SLAVEBMS_CBD_MD_7K			(unsigned short)((unsigned short)(0x0002) << 7)		/* ADCOPT = 0, NORMAL */
#define SLAVEBMS_CBD_MD_26			(unsigned short)((unsigned short)(0x0003) << 7)		/* ADCOPT = 0, FILTERED */
#define SLAVEBMS_CBD_MD_14K			(unsigned short)((unsigned short)(0x0001) << 7)		/* ADCOPT = 1, FAST      */
#define SLAVEBMS_CBD_MD_3K			(unsigned short)((unsigned short)(0x0002) << 7)		/* ADCOPT = 1, NORMAL */
#define SLAVEBMS_CBD_MD_2K			(unsigned short)((unsigned short)(0x0003) << 7)		/* ADCOPT = 1, FILTERED */
#define SLAVEBMS_CBD_DCP			(unsigned short)(0x0010)	/* Discharge permit */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader1;
CAN_TxHeaderTypeDef TxHeader2;
CAN_TxHeaderTypeDef TxHeader3;
//CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilterConfig;

extern SPI_HandleTypeDef hspi1;

StructureSLAVEBMS SLAVEBMSA;
StructureSLAVEBMS SLAVEBMSB;
StructureSLAVEBMS SLAVEBMSC;
StructureSLAVEBMS SLAVEBMSD;
StructureSLAVEBMS SLAVEBMSE;

float Max_Voltage, Min_Voltage, MaxMin, Voltage_diff;

float c1[14] = {0}, c2[14] = {0}, c3[14] = {0}, c4[14] = {0}, c5[14] = {0};

uint16_t Max_Voltage_int, Min_Voltage_int;
uint8_t bb = 0, aa = 0;
uint8_t Max_temp, Min_temp;
uint16_t temp00, temp01, temp02, temp03, temp04, temp05, temp06, temp07, temp08, temp09, temp10, temp11, temp12, temp13, temp14,
		 temp15, temp16, temp17, temp18, temp19, temp20, temp21, temp22, temp23, temp24, temp25, temp26, temp27, temp28, temp29,
		 temp30, temp31, temp32, temp33, temp34, temp35, temp36, temp37, temp38, temp39, temp40, temp41, temp42, temp43, temp44;
uint32_t duty = 0;
uint32_t k = 0;

uint16_t Sum_of_Voltage = 0;

uint16_t vol_stack = 0, temp_stack_NTC = 0;

uint8_t tx_data1[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t tx_data2[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t tx_data3[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint32_t Mailbox;

uint8_t seg = 0, cell = 0, time = 0, Voltage_time = 0, Temp_time = 0;

uint32_t  oldccr1 = 0, oldccr2 = 0, oldccr3 = 0, oldccr4 = 0, oldccr5 = 0, newccr1, newccr2, newccr3, newccr4, newccr5;
float rpm1, rpm2, rpm3, rpm4, rpm5;
uint8_t ch1done = 0, ch2done = 0, ch3done = 0, ch4done = 0, ch5done = 0;

double A = -5.811324394798985 * 10, B = 5.322242441512193 * 0.01, C = 9.500402599146873 * 0.0001, D = -2.170326385759362 * 0.0000001;

uint8_t state = 0;

uint32_t can_count1 = 0, can_count2 = 0, can_count3 = 0;

unsigned short SLAVEBMS_CRC_Table[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
unsigned char SLAVEBMS_Access(unsigned char Value);
void SLAVEBMS_Wakeup(unsigned char direction);
void SLAVEBMS_InitCRCTable(void);
unsigned short SLAVEBMS_CRC_calc(unsigned char len, unsigned char *data);
unsigned char SLAVEBMS_ReadDataGroup(unsigned char direction, unsigned short Order, unsigned char Counting, unsigned char *BF);
void SLAVEBMS_WriteSingleCmd(unsigned char direction, unsigned short RequestCMD);
void SLAVEBMS_PollADCDone(unsigned char direction, unsigned char *Result);
void Delay_(void);

void Reading_Voltage(void);
void Voltage_Drop_Sense(void);
void Voltage_Rise_Sense(void);
void Sum_Voltage_Error(void);
void Control_Cell_UVE(void);
void Control_Cell_OVE(void);
void Transmit_Data();
void Select_Address(unsigned char number);
void Reading_Temp(void);
void Temp_Error(void);
void CAN_TX_Config(CAN_TxHeaderTypeDef* Header, uint32_t ID);
void Voltage_MinMax(void);
void Reading_Voltage_Config(void);
void Seg_MaxMin_Voltage(void);
void S_balance(void);
void Balancing(void);
void Set_PWM_duty(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void calc_RPM();
void delay_us(volatile uint16_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  SLAVEBMS_InitCRCTable();

  Reading_Voltage_Config();

  CAN_TX_Config(&TxHeader1, 0x01);

  CAN_TX_Config(&TxHeader2, 0x02);

  CAN_TX_Config(&TxHeader3, 0x03);

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);

  //S_balance();  // Turn ON during Balancing

  //SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_MUTE);  // Turn ON during Balancing

  HAL_Delay(2);

  Reading_Voltage(); //SLAVEBMS A ~ E

  Voltage_MinMax(); //Max & Min Voltage

  //Control_Cell_UVE();

  //Control_Cell_OVE();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_Delay(10);

	  //Reading_Voltage(); //SLAVEBMS A ~ E

	  //Voltage_MinMax(); //Max & Min Voltage

	  //Voltage_Rise_Sense(); //with voltage

	  //Voltage_Drop_Sense();  //Monitoring low voltage(2.5V)

	  Reading_Temp();  //Measuring  Battery Temperature

	  //Temp_Error();
	  calc_RPM();

	  //HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);

	  //Balancing();  // Turn OFF during driving
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Delay_(void)
{
	volatile unsigned long Delay = 4444;
	while(Delay--);
}

void SLAVEBMS_InitCRCTable(void)
{
	unsigned short Indicator = 0U;
	unsigned short Bit;
	unsigned short PEC;

	for(Indicator = 0U; Indicator < 256; Indicator++)
	{
		PEC = Indicator << 7U;
		for(Bit = 8U; Bit > 0; --Bit)
		{
			if(PEC & 0x4000)
			{
				PEC = (PEC << 1U);
				PEC = (PEC ^ (unsigned short)(0x4599));
			}
			else
			{
				PEC = (PEC << 1U);
			}
		}
		SLAVEBMS_CRC_Table[Indicator] = PEC & 0xFFFF;
	}
}

unsigned short SLAVEBMS_CRC_calc(unsigned char len, unsigned char *data)
{
	unsigned short Indicator;
	unsigned short PEC;
	unsigned short address;

	PEC = 16;
	for(Indicator = 0; Indicator < len; Indicator++)
	{
		address = ((PEC >> 7) ^ data[Indicator]) & 0xff;
		PEC = (PEC << 8) ^ SLAVEBMS_CRC_Table[address];
	}

	return (PEC*2);
}

unsigned char SLAVEBMS_Access(unsigned char Value)
{
	HAL_SPI_TransmitReceive(&hspi1, &Value, &Value, 1, 10);
	return (unsigned char)(Value);
}

void SLAVEBMS_Wakeup(unsigned char direction)
{
	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		SLAVEBMS_Access(0x00);
		SLAVEBMS_Access(0x00);
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		SLAVEBMS_Access(0x00);
		SLAVEBMS_Access(0x00);
		SLAVEBMSREVERSE_CS_H();
	}
}

unsigned char SLAVEBMS_ReadDataGroup(unsigned char direction, unsigned short Order, unsigned char Counting, unsigned char *BF)
{
	unsigned char TX_BF[4U];
	unsigned short CRC_calc;
	unsigned short CRC_extracted;
	unsigned char Indicator;
	unsigned char ReadCount = Counting * 8U;

	for(Indicator = 0U; Indicator < ReadCount; Indicator++)
	{
		*(BF + Indicator) = 0x00;
	}

	TX_BF[0U] = (unsigned char)((Order >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((Order >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Delay_();
		for(Indicator = 0U; Indicator < ReadCount; Indicator++)
		{
			*(BF + Indicator) = SLAVEBMS_Access((unsigned char)(0x00));
		}
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Delay_();
		for(Indicator = 0U; Indicator < ReadCount; Indicator++)
		{
			*(BF + Indicator) = SLAVEBMS_Access((unsigned char)(0x00));
		}
		SLAVEBMSREVERSE_CS_H();
	}

	for(Indicator = 0U; Indicator < Counting; Indicator++)
	{
		CRC_calc = SLAVEBMS_CRC_calc(6U, (BF + (Indicator * 8U)));
		CRC_extracted = ((unsigned short)(*(BF + (Indicator * 8U) + 6)) << 8);
		CRC_extracted |= *(BF + (Indicator * 8U) + 7);

		if(CRC_calc != CRC_extracted)
		{
			return SLAVEBMSACCESS_CRC_MISMATCH;
		}
	}

	return SLAVEBMSACCESS_CRC_MATCH;
}

void SLAVEBMS_WriteSingleCmd(unsigned char direction, unsigned short RequestCMD)
{
	unsigned short CRC_calc;
	unsigned char TX_BF[4U];
	unsigned char Indicator;

	TX_BF[0U] = (unsigned char)((RequestCMD >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((RequestCMD >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		SLAVEBMSREVERSE_CS_H();
	}
}

void SLAVEBMS_PollADCDone(unsigned char direction, unsigned char *Result)
{
	unsigned short CRC_calc;
	unsigned char TX_BF[4U];
	unsigned char Indicator;

	TX_BF[0U] = (unsigned char)((SLAVEBMS_CMD_PLADC >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((SLAVEBMS_CMD_PLADC >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Indicator = 0U;
		while(1)
		{
			HAL_Delay(1);
			if(SLAVEBMS_Access((unsigned char)(0xFF)) == (unsigned char)(0xFF))
			{
				SLAVEBMSFORWARD_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MATCH;
				break;
			}
			Indicator++;
			if(Indicator > 16)
			{
				SLAVEBMSFORWARD_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MISMATCH;
				break;
			}
		}
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Indicator = 0U;
		while(1)
		{
			HAL_Delay(1);
			if(SLAVEBMS_Access((unsigned char)(0xFF)) == (unsigned char)(0xFF))
			{
				SLAVEBMSREVERSE_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MATCH;
				break;
			}
			Indicator++;
			if(Indicator > 16)
			{
				SLAVEBMSREVERSE_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MISMATCH;
				break;
			}
		}
	}
}

void Reading_Voltage_Config(void)
{
	unsigned char Data_Array[44U]; //Order array 4 + (Configuration array 6 + CRC 2)*5
	unsigned short CRC_calc;

	SLAVEBMS_Wakeup(SLAVEBMSACCESS_FWD);
	HAL_Delay(10);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0xFE);
	Data_Array[5] = (unsigned char)(0x0E); //VUV = 3.3V
	Data_Array[6] = (unsigned char)(0x18); //VOV = 4.2V
	Data_Array[7] = (unsigned char)(0xA4);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0xF0);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0xFE);
	Data_Array[13] = (unsigned char)(0x0E);
	Data_Array[14] = (unsigned char)(0x18);
	Data_Array[15] = (unsigned char)(0xA4);
	Data_Array[16] = (unsigned char)(0x00);
	Data_Array[17] = (unsigned char)(0xE0);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0xFE);
	Data_Array[21] = (unsigned char)(0x0E);
	Data_Array[22] = (unsigned char)(0x18);
	Data_Array[23] = (unsigned char)(0xA4);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0xFE);
	Data_Array[29] = (unsigned char)(0x0E);
	Data_Array[30] = (unsigned char)(0x18);
	Data_Array[31] = (unsigned char)(0xA4);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0xFE);
	Data_Array[37] = (unsigned char)(0x0E);
	Data_Array[38] = (unsigned char)(0x18);
	Data_Array[39] = (unsigned char)(0xA4);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);


	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0x0F);
	Data_Array[5] = (unsigned char)(0x8F);
	Data_Array[6] = (unsigned char)(0x00);
	Data_Array[7] = (unsigned char)(0x00);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0x0F);
	Data_Array[13] = (unsigned char)(0x8F);
	Data_Array[14] = (unsigned char)(0x00);
	Data_Array[15] = (unsigned char)(0x00);
	Data_Array[16] = (unsigned char)(0x00);
	Data_Array[17] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0x0F);
	Data_Array[21] = (unsigned char)(0x8F);
	Data_Array[22] = (unsigned char)(0x00);
	Data_Array[23] = (unsigned char)(0x00);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0x0F);
	Data_Array[29] = (unsigned char)(0x8F);
	Data_Array[30] = (unsigned char)(0x00);
	Data_Array[31] = (unsigned char)(0x00);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0x0F);
	Data_Array[37] = (unsigned char)(0x8F);
	Data_Array[38] = (unsigned char)(0x00);
	Data_Array[39] = (unsigned char)(0x00);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_SET);
}

void Reading_Voltage(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	unsigned char Data_Array[40U]; //(Configuration array 6 + CRC 2)*5
	unsigned char ConversionResult;

	SLAVEBMS_Wakeup(SLAVEBMSACCESS_FWD);
	HAL_Delay(10);

	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRCELL);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADCV | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);
	if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
	{
		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVA, 5, Data_Array);	//기존 코딩?  ?   ?  ?   ( 기존?   CRC Mismatch ?  ?  ?  ?   CV?   ???  ?? ?   ????????????????????????? ????????????????????????? ?   ????????????????????????? ?  ?  )


		SLAVEBMSA.CV[0] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[1] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[2] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[0] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[1] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[2] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[0] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[1] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[2] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[0] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[1] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[2] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[0] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[1] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[2] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVB, 5, Data_Array);

		SLAVEBMSA.CV[3] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[4] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[5] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[3] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[4] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[5] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[3] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[4] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[5] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[3] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[4] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[5] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[3] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[4] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[5] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVC, 5, Data_Array);

		SLAVEBMSA.CV[6] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[7] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[8] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[6] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[7] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[8] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[6] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[7] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[8] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[6] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[7] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[8] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[6] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[7] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[8] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVD, 5, Data_Array);

		SLAVEBMSA.CV[9] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[10] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[11] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[9] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[10] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[11] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[9] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[10] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[11] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[9] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[10] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[11] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[9] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[10] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[11] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVE, 5, Data_Array);

		SLAVEBMSA.CV[12] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[13] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[14] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[12] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[13] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[14] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[12] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[13] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[14] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[12] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[13] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[14] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[12] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[13] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[14] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);

		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVF, 5, Data_Array);

		SLAVEBMSA.CV[15] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.CV[16] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.CV[17] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.CV[15] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.CV[16] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.CV[17] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.CV[15] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.CV[16] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.CV[17] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.CV[15] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.CV[16] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.CV[17] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.CV[15] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.CV[16] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.CV[17] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);

	}


	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRAUX);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADAX | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);

	if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
	{
		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXA, 5, Data_Array);


		SLAVEBMSA.GV[0] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.GV[1] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.GV[2] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.GV[0] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.GV[1] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.GV[2] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.GV[0] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.GV[1] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.GV[2] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.GV[0] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.GV[1] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.GV[2] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.GV[0] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.GV[1] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.GV[2] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXB, 5, Data_Array);

		SLAVEBMSA.GV[3] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.GV[4] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.REF = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.GV[3] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.GV[4] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.REF = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.GV[3] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.GV[4] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.REF = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.GV[3] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.GV[4] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.REF = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.GV[3] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.GV[4] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.REF = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXC, 5, Data_Array);


		SLAVEBMSA.GV[5] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSA.GV[6] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
		SLAVEBMSA.GV[7] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.GV[5] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSB.GV[6] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
		SLAVEBMSB.GV[7] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.GV[5] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSC.GV[6] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
		SLAVEBMSC.GV[7] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.GV[5] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSD.GV[6] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
		SLAVEBMSD.GV[7] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.GV[5] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
		SLAVEBMSE.GV[6] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
		SLAVEBMSE.GV[7] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXD, 5, Data_Array);


		SLAVEBMSA.GV[8] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSB.GV[8] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSC.GV[8] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSD.GV[8] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSE.GV[8] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
	}

	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRSTAT);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADSTAT | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);
	if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
	{
		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDSTATA, 5, Data_Array);


		SLAVEBMSA.SC = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.003);
		SLAVEBMSA.ITMP = ((float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
		SLAVEBMSA.VA = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

		SLAVEBMSB.SC = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.003);
		SLAVEBMSB.ITMP = ((float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
		SLAVEBMSB.VA = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

		SLAVEBMSC.SC = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.003);
		SLAVEBMSC.ITMP = ((float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
		SLAVEBMSC.VA = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

		SLAVEBMSD.SC = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.003);
		SLAVEBMSD.ITMP = ((float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
		SLAVEBMSD.VA = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

		SLAVEBMSE.SC = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.003);
		SLAVEBMSE.ITMP = ((float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
		SLAVEBMSE.VA = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


		SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDSTATB, 5, Data_Array);

		SLAVEBMSA.VD = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
		SLAVEBMSB.VD = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
		SLAVEBMSC.VD = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
		SLAVEBMSD.VD = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
		SLAVEBMSE.VD = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
	}

	/*for(int8_t i = 0; i < 15; i++)
	{
		if((SLAVEBMSA.CV[i] <= 2.50) || (SLAVEBMSB.CV[i] <= 2.50) || (SLAVEBMSC.CV[i] <= 2.50) || (SLAVEBMSD.CV[i] <= 2.50) || (SLAVEBMSE.CV[i] <= 2.50))
		{
			SLAVEBMSA.CV[i] = 3.50;
			SLAVEBMSB.CV[i] = 3.80;
			SLAVEBMSC.CV[i] = 3.80;
			SLAVEBMSD.CV[i] = 3.80;
			SLAVEBMSE.CV[i] = 3.80;
		}
		else if((SLAVEBMSA.CV[i] >= 4.20) || (SLAVEBMSB.CV[i] >= 4.20) || (SLAVEBMSC.CV[i] >= 4.20) || (SLAVEBMSD.CV[i] >= 4.20) || (SLAVEBMSE.CV[i] >= 4.20))
		{
			SLAVEBMSA.CV[i] = 3.50;
			SLAVEBMSB.CV[i] = 3.80;
			SLAVEBMSC.CV[i] = 3.80;
			SLAVEBMSD.CV[i] = 3.80;
			SLAVEBMSE.CV[i] = 3.80;
		}
	}*/

	Sum_of_Voltage = (SLAVEBMSA.SC * 100) + (SLAVEBMSB.SC * 100)+ (SLAVEBMSC.SC * 100) + (SLAVEBMSD.SC * 100) + (SLAVEBMSE.SC * 100);
}

void Voltage_Drop_Sense(void)
{
	int cnt = 0;

	for(uint8_t i = 0; i < 14; i++)
	{
		if((SLAVEBMSA.CV[i] <= 2.50) || (SLAVEBMSB.CV[i] <= 2.50) || (SLAVEBMSC.CV[i] <= 2.50) || (SLAVEBMSD.CV[i] <= 2.50) || (SLAVEBMSE.CV[i] <= 2.50))
		{
			while(Voltage_time <= 3)
			{
				if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
				{
					__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
					Voltage_time++;
				}
				Reading_Voltage();

				Sum_Voltage_Error();

				cnt++;
			}

			for(int i = 0; i < 14; i++)
			{
				c1[i] /= cnt;
				c2[i] /= cnt;
				c3[i] /= cnt;
				c4[i] /= cnt;
				c5[i] /= cnt;

				if((c1[i] <= 2.50) || (c2[i] <= 2.50) || c3[i] <= 2.50 || c4[i] <= 2.50 || c5[i] <= 2.50)
				{
					HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);

					while(1)
					{
						for (uint8_t i = 0; i < 4; i++)
						{
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
							HAL_Delay(200);
						}
						HAL_Delay(2800);	// Drop Sence Error LED 2
					}
				}
				else
				{
					Voltage_time = 0;
				}
			}
		}
	}
}


void Voltage_Rise_Sense(void)
{
	int cnt = 0;

	for(uint8_t i = 0; i < 14; i++)
	{
		if((SLAVEBMSA.CV[i] >= 4.20) || (SLAVEBMSB.CV[i] >= 4.20) || (SLAVEBMSC.CV[i] >= 4.20) || (SLAVEBMSD.CV[i] >= 4.20) || (SLAVEBMSE.CV[i] >= 4.20))
		{
			while(Voltage_time <= 3)
			{
				if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
				{
					__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
					Voltage_time++;
				}
				Reading_Voltage();

				Sum_Voltage_Error();

				cnt++;
			}

			for(int i = 0; i < 14; i++)
			{
				c1[i] /= cnt;
				c2[i] /= cnt;
				c3[i] /= cnt;
				c4[i] /= cnt;
				c5[i] /= cnt;

				if((c1[i] >= 4.20) || (c2[i] >= 4.20) || c3[i] >= 4.20 || c4[i] >= 4.20 || c5[i] >= 4.20)
				{
					HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);//(1)

					while(1)
					{
						for (uint8_t i = 0; i < 6; i++)
						{
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
							HAL_Delay(200);
						}
						HAL_Delay(2800);	// Rise Sence Error LED 3
					}
				}
				else
				{
					Voltage_time = 0;
				}
			}
		}
	}
}

void Control_Cell_UVE(void)  //control with voltage
{
	// UVR : 2.50V Per Cell -> DSG ON>=2.50

	for(uint8_t i = 0; i < 14; i++)
	{
		if((SLAVEBMSA.CV[i] <= 2.50) || (SLAVEBMSB.CV[i] <= 2.50) || (SLAVEBMSC.CV[i] <= 2.50) || (SLAVEBMSD.CV[i] <= 2.50) || (SLAVEBMSE.CV[i] <= 2.50))
		{
			HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);//(2)

			while(1)
			{
				Reading_Voltage(); //SLAVEBMSA + B + C + D + E

				for (uint8_t i = 0; i < 4; i++)
				{
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					HAL_Delay(200);
				}
				HAL_Delay(2800);		// Under Voltage Error LED 2
			}
		}
	}
}

void Control_Cell_OVE(void)  // OVR : 4.20V Per Cell -> CHG ON>=4.20
{

	for(uint8_t i = 0; i < 14; i++)
	{
		if((SLAVEBMSA.CV[i] >= 4.20) || (SLAVEBMSB.CV[i] >= 4.20) || (SLAVEBMSC.CV[i] >= 4.20) || (SLAVEBMSD.CV[i] >= 4.20) || (SLAVEBMSE.CV[i] >= 4.20))
		{
			HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);//(1)
			//HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

			while(1)
			{
				Reading_Voltage(); //SLAVEBMSA + B + C + D + E

				//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				for (uint8_t i = 0; i < 6; i++)
				{
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					HAL_Delay(200);
				}
				HAL_Delay(2800);		// Over Voltage Eror LED 3
			}
		}
	}
}

void Sum_Voltage_Error(void)
{
	c1[0] += SLAVEBMSA.CV[0];
	c1[1] += SLAVEBMSA.CV[1];
	c1[2] += SLAVEBMSA.CV[2];
	c1[3] += SLAVEBMSA.CV[3];
	c1[4] += SLAVEBMSA.CV[4];
	c1[5] += SLAVEBMSA.CV[5];
	c1[6] += SLAVEBMSA.CV[6];
	c1[7] += SLAVEBMSA.CV[7];
	c1[8] += SLAVEBMSA.CV[8];
	c1[9] += SLAVEBMSA.CV[9];
	c1[10] += SLAVEBMSA.CV[10];
	c1[11] += SLAVEBMSA.CV[11];
	c1[12] += SLAVEBMSA.CV[12];
	c1[13] += SLAVEBMSA.CV[13];

	c2[0] += SLAVEBMSB.CV[0];
	c2[1] += SLAVEBMSB.CV[1];
	c2[2] += SLAVEBMSB.CV[2];
	c2[3] += SLAVEBMSB.CV[3];
	c2[4] += SLAVEBMSB.CV[4];
	c2[5] += SLAVEBMSB.CV[5];
	c2[6] += SLAVEBMSB.CV[6];
	c2[7] += SLAVEBMSB.CV[7];
	c2[8] += SLAVEBMSB.CV[8];
	c2[9] += SLAVEBMSB.CV[9];
	c2[10] += SLAVEBMSB.CV[10];
	c2[11] += SLAVEBMSB.CV[11];
	c2[12] += SLAVEBMSB.CV[12];
	c2[13] += SLAVEBMSB.CV[13];

	c3[0] += SLAVEBMSC.CV[0];
	c3[1] += SLAVEBMSC.CV[1];
	c3[2] += SLAVEBMSC.CV[2];
	c3[3] += SLAVEBMSC.CV[3];
	c3[4] += SLAVEBMSC.CV[4];
	c3[5] += SLAVEBMSC.CV[5];
	c3[6] += SLAVEBMSC.CV[6];
	c3[7] += SLAVEBMSC.CV[7];
	c3[8] += SLAVEBMSC.CV[8];
	c3[9] += SLAVEBMSC.CV[9];
	c3[10] += SLAVEBMSC.CV[10];
	c3[11] += SLAVEBMSC.CV[11];
	c3[12] += SLAVEBMSC.CV[12];
	c3[13] += SLAVEBMSC.CV[13];

	c4[0] += SLAVEBMSD.CV[0];
	c4[1] += SLAVEBMSD.CV[1];
	c4[2] += SLAVEBMSD.CV[2];
	c4[3] += SLAVEBMSD.CV[3];
	c4[4] += SLAVEBMSD.CV[4];
	c4[5] += SLAVEBMSD.CV[5];
	c4[6] += SLAVEBMSD.CV[6];
	c4[7] += SLAVEBMSD.CV[7];
	c4[8] += SLAVEBMSD.CV[8];
	c4[9] += SLAVEBMSD.CV[9];
	c4[10] += SLAVEBMSD.CV[10];
	c4[11] += SLAVEBMSD.CV[11];
	c4[12] += SLAVEBMSD.CV[12];
	c4[13] += SLAVEBMSD.CV[13];

	c5[0] += SLAVEBMSE.CV[0];
	c5[1] += SLAVEBMSE.CV[1];
	c5[2] += SLAVEBMSE.CV[2];
	c5[3] += SLAVEBMSE.CV[3];
	c5[4] += SLAVEBMSE.CV[4];
	c5[5] += SLAVEBMSE.CV[5];
	c5[6] += SLAVEBMSE.CV[6];
	c5[7] += SLAVEBMSE.CV[7];
	c5[8] += SLAVEBMSE.CV[8];
	c5[9] += SLAVEBMSE.CV[9];
	c5[10] += SLAVEBMSE.CV[10];
	c5[11] += SLAVEBMSE.CV[11];
	c5[12] += SLAVEBMSE.CV[12];
	c5[13] += SLAVEBMSE.CV[13];
}

void Transmit_Data()
{
	if(state == 0)
	{
		uint8_t Sum_of_Voltage_1 = (Sum_of_Voltage & 0x00ff);
		uint8_t Sum_of_Voltage_2 = ((Sum_of_Voltage & 0xff00)>>8);

		tx_data1[0] = Sum_of_Voltage_1;
		tx_data1[1] = Sum_of_Voltage_2;
		tx_data1[2] = (uint8_t)((int)(SLAVEBMSA.Max_Voltage * 100) & 0x00ff) - 255;
		tx_data1[3] = (uint8_t)((int)(SLAVEBMSB.Max_Voltage * 100) & 0x00ff) - 255;
		tx_data1[4] = (uint8_t)((int)(SLAVEBMSC.Max_Voltage * 100) & 0x00ff) - 255;
		tx_data1[5] = (uint8_t)((int)(SLAVEBMSD.Max_Voltage * 100) & 0x00ff) - 255;
		tx_data1[6] = (uint8_t)((int)(SLAVEBMSE.Max_Voltage * 100) & 0x00ff) - 255;
		tx_data1[7] = state;
		state = 1;
		Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(Mailbox)
		{
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, tx_data1, &Mailbox);
			can_count1++;
		}
	}
	else if(state == 1)
	{
		tx_data2[0] = (uint8_t)((int)(SLAVEBMSA.Min_Voltage * 100) & 0x00ff) - 255;
		tx_data2[1] = (uint8_t)((int)(SLAVEBMSB.Min_Voltage * 100) & 0x00ff) - 255;
		tx_data2[2] = (uint8_t)((int)(SLAVEBMSC.Min_Voltage * 100) & 0x00ff) - 255;
		tx_data2[3] = (uint8_t)((int)(SLAVEBMSD.Min_Voltage * 100) & 0x00ff) - 255;
		tx_data2[4] = (uint8_t)((int)(SLAVEBMSE.Min_Voltage * 100) & 0x00ff) - 255;
		tx_data2[5] = state;
		tx_data2[6] = 0;
		tx_data2[7] = 0;
		state = 2;
		Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(Mailbox)
		{
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, tx_data2, &Mailbox);
			can_count2++;
		}
	}
	else if(state == 2)
	{
		tx_data3[0] = SLAVEBMSA.MaxTemp;
		tx_data3[1] = SLAVEBMSB.MaxTemp;
		tx_data3[2] = SLAVEBMSC.MaxTemp;
		tx_data3[3] = SLAVEBMSD.MaxTemp;
		tx_data3[4] = SLAVEBMSE.MaxTemp;
		tx_data3[5] = 0;
		tx_data3[6] = 0;
		tx_data3[7] = 0;
		state = 0;
		Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(Mailbox)
		{
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader3, tx_data3, &Mailbox);
			can_count3++;
		}
	}

}

void Select_Address(unsigned char number)  //for temperature sensor
{
	switch(number)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//1
			//0000
			break;

		case 1:
			HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_SET);//2
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_RESET);//0001
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_SET);//3
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A0_Pin, GPIO_PIN_RESET);//0010
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOC, A1_Pin|A0_Pin, GPIO_PIN_SET);//4
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin, GPIO_PIN_RESET);//0011
			break;

		case 4:
			HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_SET);//5
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//0100
			break;

		case 5:
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A0_Pin, GPIO_PIN_SET);//6
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin, GPIO_PIN_RESET);//0101
			break;

		case 6:
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_SET);//7
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A0_Pin, GPIO_PIN_RESET);//0110
			break;

		case 7:
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_SET);//8
			HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_RESET);//0111
			break;

		case 8:
			HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_SET);//9
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//1000
			break;

		case 9:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A0_Pin, GPIO_PIN_SET);//10
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_RESET);//1001
			break;

		case 10:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A0_Pin, GPIO_PIN_SET);//11
			HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_RESET);//1010
			break;

		case 11:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin|A0_Pin, GPIO_PIN_SET);//12
			HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_RESET);//1011
			break;

		case 12:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin, GPIO_PIN_SET);//13
			HAL_GPIO_WritePin(GPIOC, A1_Pin|A0_Pin, GPIO_PIN_RESET);//1100
			break;

		case 13:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A0_Pin, GPIO_PIN_SET);//14
			HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_RESET);//1101
			break;

		case 14:
			HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_SET);//15
			HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_RESET);//1110
			break;
	}
}

void Reading_Temp(void)
{
	uint8_t R_Temp[45];
	uint32_t temp_value[45] = {0};
	uint8_t index = 0;


		for(uint8_t i = 0; i < 15; i++)
		{
			index = 3 * i;
			Select_Address(i);
			HAL_Delay(40);
			HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
			HAL_Delay(1);
			//HAL_Delay(1);
			HAL_ADC_Stop_DMA(&hadc1);
		}

/*
	index = 0;
	Select_Address(0);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);
	index = 3;
	Select_Address(1);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 6;
	Select_Address(2);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 9;
	Select_Address(3);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 12;
	Select_Address(4);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 15;
	Select_Address(5);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 18;
	Select_Address(6);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 21;
	Select_Address(7);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 24;
	Select_Address(8);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 27;
	Select_Address(9);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 30;
	Select_Address(10);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 33;
	Select_Address(11);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 36;
	Select_Address(12);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 39;
	Select_Address(13);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);

	index = 42;
	Select_Address(14);
	delay_us(500);
	HAL_ADC_Start_DMA(&hadc1, &temp_value[index], 3);
	delay_us(500);
	//HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(10);
*/
	for(uint8_t i = 0; i < 45; i++)
	{
		R_Temp[i] = (A + (B * temp_value[i])) / (1 + (C * temp_value[i]) + D * temp_value[i] * temp_value[i]);
	}

	temp00 = R_Temp[0];
	temp01 = R_Temp[1];
	temp02 = R_Temp[2];
	temp03 = R_Temp[3];
	temp04 = R_Temp[4];
	temp05 = R_Temp[5];
	temp06 = R_Temp[6];
	temp07 = R_Temp[7];
	temp08 = R_Temp[8];
	temp09 = R_Temp[9];
	temp10 = R_Temp[10];
	temp11 = R_Temp[11];
	temp12 = R_Temp[12];
	temp13 = R_Temp[13];
	temp14 = R_Temp[14];
	temp15 = R_Temp[15];
	temp16 = R_Temp[16];
	temp17 = R_Temp[17];
	temp18 = R_Temp[18];
	temp19 = R_Temp[19];
	temp20 = R_Temp[20];
	temp21 = R_Temp[21];
	temp22 = R_Temp[22];
	temp23 = R_Temp[23];
	temp24 = R_Temp[24];
	temp25 = R_Temp[25];
	temp26 = R_Temp[26];
	temp27 = R_Temp[27];
	temp28 = R_Temp[28];
	temp29 = R_Temp[29];
	temp30 = R_Temp[30];
	temp31 = R_Temp[31];
	temp32 = R_Temp[32];
	temp33 = R_Temp[33];
	temp34 = R_Temp[34];
	temp35 = R_Temp[35];
	temp36 = R_Temp[36];
	temp37 = R_Temp[37];
	temp38 = R_Temp[38];
	temp39 = R_Temp[39];
	temp40 = R_Temp[40];
	temp41 = R_Temp[41];
	temp42 = R_Temp[42];
	temp43 = R_Temp[43];
	temp44 = R_Temp[44];
	Max_temp = R_Temp[0];
	SLAVEBMSA.MaxTemp = 0;
	SLAVEBMSB.MaxTemp = 0;
	SLAVEBMSC.MaxTemp = 0;
	SLAVEBMSD.MaxTemp = 0;
	SLAVEBMSE.MaxTemp = 0;
	for(uint8_t q = 0; q < 44; q++)
	{
		if(Max_temp < R_Temp[q])
		{
			Max_temp = R_Temp[q];
		}

		if(q < 9)
		{
			if(SLAVEBMSA.MaxTemp < R_Temp[q])
			{
				SLAVEBMSA.MaxTemp = R_Temp[q];
			}
		}
		else if(q >= 9 && q < 18)
		{
			if(SLAVEBMSB.MaxTemp < R_Temp[q])
			{
				SLAVEBMSB.MaxTemp = R_Temp[q];
			}
		}
		else if(q >= 18 && q < 27)
		{
			if(SLAVEBMSC.MaxTemp < R_Temp[q])
			{
				SLAVEBMSC.MaxTemp = R_Temp[q];
			}
		}
		else if(q >= 27 && q < 36)
		{
			if(SLAVEBMSD.MaxTemp < R_Temp[q])
			{
				SLAVEBMSD.MaxTemp = R_Temp[q];
			}
		}
		else if(q >= 36 && q < 45)
		{
			if(SLAVEBMSE.MaxTemp < R_Temp[q])
			{
				SLAVEBMSE.MaxTemp = R_Temp[q];
			}
		}
	}

	Min_temp = R_Temp[0];

	for(uint8_t q = 0; q < 44; q++)
	{
		if(Min_temp > R_Temp[q])
		{
			Min_temp = R_Temp[q];
		}
	}

	Set_PWM_duty();
    /*for(uint8_t i = 0; i < 30; i++)
    {
    	R_Temp[i] = 0;
    }*/
}

void Temp_Error(void)
{
	while(Max_temp > 60)	//Max_temp > 60'C
	{
		if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
		{
			__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
			Temp_time++;
		}
		Reading_Temp();
		HAL_Delay(1);

		if(Temp_time >= 5)
		{
			HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, GPIO_PIN_RESET);//(1)
		//	HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

			while(1)
			{
				for(uint8_t i = 0; i < 8; i++)
				{
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					HAL_Delay(200);
				}

				HAL_Delay(2800);  // Temp Error LED four time
			}
		}
	}
	Temp_time = 0;
}

void CAN_TX_Config(CAN_TxHeaderTypeDef* Header, uint32_t ID)
{
	Header->StdId = ID;
	Header->ExtId = 0x0000;
	Header->IDE = CAN_ID_STD;
	Header->RTR = CAN_RTR_DATA;
	Header->DLC = 8;
	Header->TransmitGlobalTime = DISABLE;

	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;
}

void Voltage_MinMax(void)
{
	Max_Voltage = 0;
	SLAVEBMSA.Max_Voltage = 0;
	SLAVEBMSB.Max_Voltage = 0;
	SLAVEBMSC.Max_Voltage = 0;
	SLAVEBMSD.Max_Voltage = 0;
	SLAVEBMSE.Max_Voltage = 0;

	for(uint8_t i = 0; i < 14; i++)
	{
		Max_Voltage = (Max_Voltage > SLAVEBMSA.CV[i]) ? Max_Voltage : SLAVEBMSA.CV[i];
		Max_Voltage = (Max_Voltage > SLAVEBMSB.CV[i]) ? Max_Voltage : SLAVEBMSB.CV[i];
		Max_Voltage = (Max_Voltage > SLAVEBMSC.CV[i]) ? Max_Voltage : SLAVEBMSC.CV[i];
		Max_Voltage = (Max_Voltage > SLAVEBMSD.CV[i]) ? Max_Voltage : SLAVEBMSD.CV[i];
		Max_Voltage = (Max_Voltage > SLAVEBMSE.CV[i]) ? Max_Voltage : SLAVEBMSE.CV[i];
		SLAVEBMSA.Max_Voltage = (SLAVEBMSA.Max_Voltage > SLAVEBMSA.CV[i]) ? SLAVEBMSA.Max_Voltage : SLAVEBMSA.CV[i];
		SLAVEBMSB.Max_Voltage = (SLAVEBMSB.Max_Voltage > SLAVEBMSB.CV[i]) ? SLAVEBMSB.Max_Voltage : SLAVEBMSB.CV[i];
		SLAVEBMSC.Max_Voltage = (SLAVEBMSC.Max_Voltage > SLAVEBMSC.CV[i]) ? SLAVEBMSC.Max_Voltage : SLAVEBMSC.CV[i];
		SLAVEBMSD.Max_Voltage = (SLAVEBMSD.Max_Voltage > SLAVEBMSD.CV[i]) ? SLAVEBMSD.Max_Voltage : SLAVEBMSD.CV[i];
		SLAVEBMSE.Max_Voltage = (SLAVEBMSE.Max_Voltage > SLAVEBMSE.CV[i]) ? SLAVEBMSE.Max_Voltage : SLAVEBMSE.CV[i];
	}

	Min_Voltage = 10;
	SLAVEBMSA.Min_Voltage = 10;
	SLAVEBMSB.Min_Voltage = 10;
	SLAVEBMSC.Min_Voltage = 10;
	SLAVEBMSD.Min_Voltage = 10;
	SLAVEBMSE.Min_Voltage = 10;

	for(uint8_t i = 0; i < 14; i++)
	{
		Min_Voltage = (Min_Voltage < SLAVEBMSA.CV[i]) ? Min_Voltage : SLAVEBMSA.CV[i];
		Min_Voltage = (Min_Voltage < SLAVEBMSB.CV[i]) ? Min_Voltage : SLAVEBMSB.CV[i];
		Min_Voltage = (Min_Voltage < SLAVEBMSC.CV[i]) ? Min_Voltage : SLAVEBMSC.CV[i];
		Min_Voltage = (Min_Voltage < SLAVEBMSD.CV[i]) ? Min_Voltage : SLAVEBMSD.CV[i];
		Min_Voltage = (Min_Voltage < SLAVEBMSE.CV[i]) ? Min_Voltage : SLAVEBMSE.CV[i];
		SLAVEBMSA.Min_Voltage = (SLAVEBMSA.Min_Voltage < SLAVEBMSA.CV[i]) ? SLAVEBMSA.Min_Voltage : SLAVEBMSA.CV[i];
		SLAVEBMSB.Min_Voltage = (SLAVEBMSB.Min_Voltage < SLAVEBMSB.CV[i]) ? SLAVEBMSB.Min_Voltage : SLAVEBMSB.CV[i];
		SLAVEBMSC.Min_Voltage = (SLAVEBMSC.Min_Voltage < SLAVEBMSC.CV[i]) ? SLAVEBMSC.Min_Voltage : SLAVEBMSC.CV[i];
		SLAVEBMSD.Min_Voltage = (SLAVEBMSD.Min_Voltage < SLAVEBMSD.CV[i]) ? SLAVEBMSD.Min_Voltage : SLAVEBMSD.CV[i];
		SLAVEBMSE.Min_Voltage = (SLAVEBMSE.Min_Voltage < SLAVEBMSE.CV[i]) ? SLAVEBMSE.Min_Voltage : SLAVEBMSE.CV[i];
	}

	Voltage_diff = Max_Voltage - Min_Voltage;
}

void Seg_MaxMin_Voltage(void)
{
	Max_Voltage = 0;

	for(uint8_t i = 0; i < 14; i++)
	{
		if(Max_Voltage <= SLAVEBMSA.CV[i])
		{
			Max_Voltage = SLAVEBMSA.CV[i];
			seg = 1;
			cell = i + 1;
		}

		if(Max_Voltage <= SLAVEBMSB.CV[i])
		{
			Max_Voltage = SLAVEBMSB.CV[i];
			seg = 2;
			cell = i + 1;
		}

		if(Max_Voltage <= SLAVEBMSC.CV[i])
		{
			Max_Voltage = SLAVEBMSC.CV[i];
			seg = 3;
			cell = i + 1;
		}

		if(Max_Voltage <= SLAVEBMSD.CV[i])
		{
			Max_Voltage = SLAVEBMSD.CV[i];
			seg = 4;
			cell = i + 1;
		}

		if(Max_Voltage <= SLAVEBMSE.CV[i])
		{
			Max_Voltage = SLAVEBMSE.CV[i];
			seg = 5;
			cell = i + 1;
		}
	}

	Min_Voltage = 10;

	for(uint8_t i = 0; i < 14; i++)
	{
		if(Min_Voltage >= SLAVEBMSA.CV[i])
		{
			Min_Voltage = SLAVEBMSA.CV[i];
		}

		if(Min_Voltage >= SLAVEBMSB.CV[i])
		{
			Min_Voltage = SLAVEBMSB.CV[i];
		}

		if(Min_Voltage >= SLAVEBMSC.CV[i])
		{
			Min_Voltage = SLAVEBMSC.CV[i];
		}

		if(Min_Voltage >= SLAVEBMSD.CV[i])
		{
			Min_Voltage = SLAVEBMSD.CV[i];
		}

		if(Min_Voltage >= SLAVEBMSE.CV[i])
		{
			Min_Voltage = SLAVEBMSE.CV[i];
		}
	}

	MaxMin = Max_Voltage - Min_Voltage;

	if(MaxMin <= 0.02)
	{
		seg = 0;
		cell = 0;
	}
}

void S_balance(void)
{
	unsigned char Data_Array[44U];
	unsigned short CRC_calc;
	unsigned short SA_con[3], SB_con[3], SC_con[3], SD_con[3], SE_con[3];

	for(uint8_t i = 0; i < 3; i++)
	{
		SA_con[i] = 0x00;
		SB_con[i] = 0x00;
		SC_con[i] = 0x00;
		SD_con[i] = 0x00;
		SE_con[i] = 0x00;
	}

	if(seg == 5)
	{
		if(cell <= 8)
		{
			SA_con[0] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 12)
		{
			SA_con[1] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 16)
		{
			SA_con[2] = (0x01 << (cell - 1) % 8);
		}
	}

	if(seg == 4)
	{
		if(cell <= 8)
		{
			SB_con[0] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 12)
		{
			SB_con[1] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 16)
		{
			SB_con[2] = (0x01 << (cell - 1) % 8);
		}
	}

	if(seg == 3)
	{
		if(cell <= 8)
		{
			SC_con[0] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 12)
		{
			SC_con[1] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 16)
		{
			SC_con[2] = (0x01 << (cell - 1) % 8);
		}
	}

	if(seg == 2)
	{
		if(cell <= 8)
		{
			SD_con[0] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 12)
		{
			SD_con[1] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 16)
		{
			SD_con[2] = (0x01 << (cell - 1) % 8);
		}
	}

	if(seg == 1)
	{
		if(cell <= 8)
		{
			SE_con[0] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 12)
		{
			SE_con[1] = (0x01 << (cell - 1) % 8);
		}
		else if(cell <= 16)
		{
			SE_con[2] = (0x01 << (cell - 1) % 8);
		}
	}

	SLAVEBMS_Wakeup(SLAVEBMSACCESS_FWD);
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0xFE);
	Data_Array[5] = (unsigned char)(0x00);
	Data_Array[6] = (unsigned char)(0x00);
	Data_Array[7] = (unsigned char)(0x00);
	Data_Array[8] = (unsigned char)SA_con[0];
	Data_Array[9] = (unsigned char)(0xF0|SA_con[1]);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0xFE);
	Data_Array[13] = (unsigned char)(0x00);
	Data_Array[14] = (unsigned char)(0x00);
	Data_Array[15] = (unsigned char)(0x00);
	Data_Array[16] = (unsigned char)SB_con[0];
	Data_Array[17] = (unsigned char)(0xF0|SB_con[1]);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0xFE);
	Data_Array[21] = (unsigned char)(0x00);
	Data_Array[22] = (unsigned char)(0x00);
	Data_Array[23] = (unsigned char)(0x00);
	Data_Array[24] = (unsigned char)SC_con[0];
	Data_Array[25] = (unsigned char)(0xF0|SC_con[1]);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0xFE);
	Data_Array[29] = (unsigned char)(0x00);
	Data_Array[30] = (unsigned char)(0x00);
	Data_Array[31] = (unsigned char)(0x00);
	Data_Array[32] = (unsigned char)SD_con[0];
	Data_Array[33] = (unsigned char)(0xF0|SD_con[1]);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0xFE);
	Data_Array[37] = (unsigned char)(0x00);
	Data_Array[38] = (unsigned char)(0x00);
	Data_Array[39] = (unsigned char)(0x00);
	Data_Array[40] = (unsigned char)SE_con[0];
	Data_Array[41] = (unsigned char)(0xF0|SE_con[1]);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)SA_con[2];
	Data_Array[5] = (unsigned char)(0x00);
	Data_Array[6] = (unsigned char)(0x00);
	Data_Array[7] = (unsigned char)(0x00);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)SB_con[2];
	Data_Array[13] = (unsigned char)(0x00);
	Data_Array[14] = (unsigned char)(0x00);
	Data_Array[15] = (unsigned char)(0x00);
	Data_Array[16] = (unsigned char)(0x00);;
	Data_Array[17] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)SC_con[2];
	Data_Array[21] = (unsigned char)(0x00);
	Data_Array[22] = (unsigned char)(0x00);
	Data_Array[23] = (unsigned char)(0x00);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)SD_con[2];
	Data_Array[29] = (unsigned char)(0x00);
	Data_Array[30] = (unsigned char)(0x00);
	Data_Array[31] = (unsigned char)(0x00);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)SE_con[2];
	Data_Array[37] = (unsigned char)(0x00);
	Data_Array[38] = (unsigned char)(0x00);
	Data_Array[39] = (unsigned char)(0x00);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRPWM >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRPWM >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0xFF);
	Data_Array[5] = (unsigned char)(0xFF);
	Data_Array[6] = (unsigned char)(0xFF);
	Data_Array[7] = (unsigned char)(0xFF);
	Data_Array[8] = (unsigned char)(0xFF);
	Data_Array[9] = (unsigned char)(0xFF);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0xFF);
	Data_Array[13] = (unsigned char)(0xFF);
	Data_Array[14] = (unsigned char)(0xFF);
	Data_Array[15] = (unsigned char)(0xFF);
	Data_Array[16] = (unsigned char)(0xFF);
	Data_Array[17] = (unsigned char)(0xFF);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0xFF);
	Data_Array[21] = (unsigned char)(0xFF);
	Data_Array[22] = (unsigned char)(0xFF);
	Data_Array[23] = (unsigned char)(0xFF);
	Data_Array[24] = (unsigned char)(0xFF);
	Data_Array[25] = (unsigned char)(0xFF);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0xFF);
	Data_Array[29] = (unsigned char)(0xFF);
	Data_Array[30] = (unsigned char)(0xFF);
	Data_Array[31] = (unsigned char)(0xFF);
	Data_Array[32] = (unsigned char)(0xFF);
	Data_Array[33] = (unsigned char)(0xFF);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0xFF);
	Data_Array[37] = (unsigned char)(0xFF);
	Data_Array[38] = (unsigned char)(0xFF);
	Data_Array[39] = (unsigned char)(0xFF);
	Data_Array[40] = (unsigned char)(0xFF);
	Data_Array[41] = (unsigned char)(0xFF);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRPSB >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRPSB >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0xFF);
	Data_Array[5] = (unsigned char)(0xFF);
	Data_Array[6] = (unsigned char)(0xFF);
	Data_Array[7] = (unsigned char)(0x00);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0xFF);
	Data_Array[13] = (unsigned char)(0xFF);
	Data_Array[14] = (unsigned char)(0xFF);
	Data_Array[15] = (unsigned char)(0x00);
	Data_Array[16] = (unsigned char)(0x00);
	Data_Array[17] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0xFF);
	Data_Array[21] = (unsigned char)(0xFF);
	Data_Array[22] = (unsigned char)(0xFF);
	Data_Array[23] = (unsigned char)(0x00);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0xFF);
	Data_Array[29] = (unsigned char)(0xFF);
	Data_Array[30] = (unsigned char)(0xFF);
	Data_Array[31] = (unsigned char)(0x00);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0xFF);
	Data_Array[37] = (unsigned char)(0xFF);
	Data_Array[38] = (unsigned char)(0xFF);
	Data_Array[39] = (unsigned char)(0x00);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);
}

void Balancing(void)
{
	if(time == 10)
	{
		SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_MUTE);
		HAL_Delay(1);

		Reading_Voltage();

		SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_UNMUTE);
		HAL_Delay(1);
		time = 0;
	}

	Seg_MaxMin_Voltage();
	S_balance();

	if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
		time++;
	}
}

void Set_PWM_duty(void)
{
	/*htim1.Instance->CCR1 = duty * 2;
	htim1.Instance->CCR2 = duty * 1.5;
	htim1.Instance->CCR3 = duty;
	htim2.Instance->CCR3 = duty * 1.5;
	htim2.Instance->CCR4 = duty * 2;*/

	if(SLAVEBMSA.MaxTemp <= 50 && SLAVEBMSB.MaxTemp <= 50 && SLAVEBMSC.MaxTemp <= 50 && SLAVEBMSD.MaxTemp <= 50 && SLAVEBMSE.MaxTemp <= 50)
	{
		htim1.Instance->CCR1 = SLAVEBMSA.MaxTemp * 1.4;
		htim1.Instance->CCR2 = SLAVEBMSB.MaxTemp * 1.1;
		htim1.Instance->CCR3 = SLAVEBMSC.MaxTemp * 0.8;
		htim2.Instance->CCR3 = SLAVEBMSD.MaxTemp * 1.1;
		htim2.Instance->CCR4 = SLAVEBMSE.MaxTemp * 1.4;
	}
	else
	{
		htim1.Instance->CCR1 = 70;
		htim1.Instance->CCR2 = 55;
		htim1.Instance->CCR3 = 40;
		htim2.Instance->CCR3 = 55;
		htim2.Instance->CCR4 = 70;
	}

	//if (Max_temp < 50)
	//{
		/*htim1.Instance->CCR1 = (Max_temp);
		htim1.Instance->CCR2 = (Max_temp);
		htim1.Instance->CCR3 = (Max_temp);
		htim2.Instance->CCR3 = (Max_temp);
		htim2.Instance->CCR4 = (Max_temp);*/
		/*htim1.Instance->CCR1 = duty;
		htim1.Instance->CCR2 = duty;
		htim1.Instance->CCR3 = duty;
		htim2.Instance->CCR3 = duty;
		htim2.Instance->CCR4 = duty;*/
		/*htim1.Instance->CCR1 = 30;
		htim1.Instance->CCR2 = 25;
		htim1.Instance->CCR3 = 20;
		htim2.Instance->CCR3 = 25;
		htim2.Instance->CCR4 = 30;*/

	//}

	/*else
	{
		htim1.Instance->CCR1 = 100;
		htim1.Instance->CCR2 = 100;
		htim1.Instance->CCR3 = 100;
		htim1.Instance->CCR3 = 100;
		htim1.Instance->CCR4 = 100;
	}*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim6.Instance)
	{
		Reading_Voltage(); //SLAVEBMS A ~ E

		Voltage_MinMax(); //Max & Min Voltage

		Transmit_Data();

	}
}

void calc_RPM()
{
	if (ch1done)
	{
		newccr1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
		if (oldccr1 <= newccr1)
		{
			rpm1 = 30 * htim3.Instance->ARR / (newccr1 - oldccr1);
		}
		else
		{
			rpm1 = 30 * htim3.Instance->ARR / (htim3.Instance->ARR + newccr1 - oldccr1);
		}
		oldccr1 = newccr1;
		ch1done = 0;
	}
	if (ch2done)
	{
		newccr2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
		if (oldccr2 <= newccr2)
		{
			rpm2 = 30 * htim3.Instance->ARR / (newccr2 - oldccr2);
		}
		else
		{
			rpm2 = 30 * htim3.Instance->ARR / (htim3.Instance->ARR + newccr2 - oldccr2);
		}
		oldccr2 = newccr2;
		ch2done = 0;
	}
	if (ch3done)
	{
		newccr3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
		if (oldccr3 <= newccr3)
		{
			rpm3 = 30 * htim3.Instance->ARR / (newccr3 - oldccr3);
		}
		else
		{
			rpm3 = 30 * htim3.Instance->ARR / (htim3.Instance->ARR + newccr3 - oldccr3);
		}
		oldccr3 = newccr3;
		ch3done = 0;
	}
	if (ch4done)
	{
		newccr4 = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
		if (oldccr4 <= newccr4)
		{
			rpm4 = 30 * htim15.Instance->ARR / (newccr4 - oldccr4);
		}
		else
		{
			rpm4 = 30 * htim15.Instance->ARR / (htim15.Instance->ARR + newccr4 - oldccr4);
		}
		oldccr4 = newccr4;
		ch4done = 0;
	}
	if (ch5done)
	{
		newccr5 = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
		if (oldccr5 <= newccr5)
		{
			rpm5 = 30 * htim15.Instance->ARR / (newccr5 - oldccr5);
		}
		else
		{
			rpm5 = 30 * htim15.Instance->ARR / (htim15.Instance->ARR + newccr5 - oldccr5);
		}
		oldccr5 = newccr5;
		ch5done = 0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			ch1done = 1;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			ch2done = 1;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			ch3done = 1;
		}
	}
	else if (htim->Instance == htim15.Instance)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			ch4done = 1;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			ch5done = 1;
		}
	}
}

void delay_us(volatile uint16_t us)
{
	htim16.Instance->CNT = 0;
	while (htim16.Instance->CNT < us)
	{

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
