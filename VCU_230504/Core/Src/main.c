/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_it.h"
#include "stm32f4xx_it.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef int bool;
typedef struct RPM_Variable
{
	uint32_t RPM;
	uint32_t Velocity;
	float freq;
	uint32_t period;
	bool done;
} RPM_Variable;

typedef enum
{
	brake = 0,
	RPM = 1,
} TXFlag;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PPR 4
#define true  1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t capture1[2];
uint16_t capture2[2];
uint16_t capture3[2];
uint16_t capture4[2];
uint16_t Apps =0;
uint32_t ADC_Value[1];
RPM_Variable Front_L;
RPM_Variable Rear_L;
RPM_Variable Front_R;
RPM_Variable Rear_R;
float table[288] = {0}, k = 0.486; // apps table, k = torque const
uint16_t Total_Torque = 0; //
uint32_t dac_value=0;
uint32_t KPH_velocity=0;
uint32_t mailbox;
uint8_t tx_flag = 0;
uint8_t TXData_RPM[8];
uint8_t TXData_Brake[8];
uint32_t pressure;
uint32_t tim2_test=0;
extern uint16_t Motor_RPM_PM100dx;
extern uint8_t SOC_100;

CAN_TxHeaderTypeDef TXH_RPM;
CAN_TxHeaderTypeDef TXH_Brake;
CAN_FilterTypeDef filtername;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void CAN_Header_Config(CAN_TxHeaderTypeDef *TX_Header);
void Measure(RPM_Variable *Front_R_Wheel, RPM_Variable *Front_L_Wheel, RPM_Variable *Rear_R_Wheel, RPM_Variable *Rear_L_Wheel, uint8_t TXData[8]);
void Calc_RPM(RPM_Variable *Front_R_Wheel, RPM_Variable *Front_L_Wheel, RPM_Variable*Rear_R_Wheel, RPM_Variable*Rear_L_Wheel, uint8_t TXData[8]);
//void CAN_Tx(CAN_TxHeaderTypeDef *Tx_Header, uint8_t TXData[8], uint32_t *Mailbox);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void callbackSystick();
void Brake_Pressure(uint8_t TXData[8]);
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)capture1, 2);
  //htim3.State = HAL_TIM_STATE_READY;
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)capture2, 2);
  HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)capture3, 2);
  HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)capture4, 2);
  CAN_Filter_defunc(&filtername);
  CAN_TX_Header_defunc(&TXH_RPM, 0x1b0);
  CAN_TX_Header_defunc(&TXH_Brake, 0x1b1);
  CAN_Error_Handler(&hcan1, &filtername);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // speed (km/h)
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2); // SOC


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Measure(&Front_R, &Front_L, &Rear_R, &Rear_L, TXData_Brake);
	  KPH_velocity = Motor_RPM_PM100dx*0.01924685;
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, map(KPH_velocity, 0, 120, 0, 4095));
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, map(SOC_100, 0, 100, 0, 4095));


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Measure(RPM_Variable *Front_R_Wheel, RPM_Variable *Front_L_Wheel, RPM_Variable *Rear_R_Wheel, RPM_Variable *Rear_L_Wheel, uint8_t TXData[8])
{
	Brake_Pressure(TXData);
	Calc_RPM(Front_R_Wheel, Front_L_Wheel, Rear_R_Wheel, Rear_L_Wheel, TXData);
}

void Brake_Pressure(uint8_t TXData[8])
{
	HAL_ADC_Start_DMA(&hadc1, ADC_Value, 1);
	//pressure =  map(ADC_Value[0], 410, 3724, 0, 1600);
	TXData[0] = (uint8_t)ADC_Value[0];
	TXData[1] = (uint8_t)(ADC_Value[0] >> 8);
	HAL_ADC_Stop_DMA(&hadc1);

}

void Calc_RPM(RPM_Variable *Front_R_Wheel, RPM_Variable *Front_L_Wheel, RPM_Variable *Rear_R_Wheel, RPM_Variable *Rear_L_Wheel, uint8_t TXData[8])
{
	if(Front_R_Wheel->done)
	{
		if(capture1[0] > capture1[1])
		{
			Front_R_Wheel->period = htim1.Instance->ARR + capture1[1] - capture1[0];
		}
		else
		{
			Front_R_Wheel->period = capture1[1] - capture1[0];
		}
		Front_R_Wheel->freq = htim1.Instance->ARR / Front_R_Wheel->period;
		Front_R_Wheel->RPM = (uint32_t)(15 * Front_R_Wheel->freq);
		Front_R_Wheel->done = false;
		TXData_RPM[0] = (uint8_t)Front_R_Wheel->RPM;
		TXData_RPM[1] = (uint8_t)Front_R_Wheel->RPM >> 8;
	}

	if(Front_L_Wheel->done)
	{
		if(capture2[0] > capture2[1])
		{
			Front_L_Wheel->period = htim2.Instance->ARR + capture2[1] - capture2[0];
		}
		else
		{
			Front_L_Wheel->period = capture2[1] - capture2[0];
		}
		Front_L_Wheel->freq = htim2.Instance->ARR / Front_L_Wheel->period;
		Front_L_Wheel->RPM = (uint32_t)(15 * Front_L_Wheel->freq);
		Front_L_Wheel->done = false;
		TXData_RPM[2] = (uint8_t)Front_L_Wheel->RPM;
		TXData_RPM[3] = (uint8_t)Front_L_Wheel->RPM >> 8;
	}

	if(Rear_R_Wheel->done)
		{
			if(capture3[0] > capture3[1])
			{
				Rear_R_Wheel->period = htim3.Instance->ARR + capture3[1] - capture3[0];
			}
			else
			{
				Rear_R_Wheel->period = capture3[1] - capture3[0];
			}
			Rear_R_Wheel->freq = htim3.Instance->ARR / Rear_R_Wheel->period;
			Rear_R_Wheel->RPM = (uint32_t)(15 * Rear_R_Wheel->freq);
			Rear_R_Wheel->done = false;
			TXData_RPM[4] = (uint8_t)Rear_R_Wheel->RPM;
			TXData_RPM[5] = (uint8_t)Rear_R_Wheel->RPM >> 8;
		}

	if(Rear_L_Wheel->done)
		{
			if(capture4[0] > capture4[1])
			{
				Rear_L_Wheel->period = htim4.Instance->ARR + capture4[1] - capture4[0];
			}
			else
			{
				Rear_L_Wheel->period = capture4[1] - capture4[0];
			}
			Rear_L_Wheel->freq = htim4.Instance->ARR / Rear_L_Wheel->period;
			Rear_L_Wheel->RPM = (uint32_t)(15 * Rear_L_Wheel->freq);
			Rear_L_Wheel->done = false;
			TXData_RPM[6] = (uint8_t)Rear_L_Wheel->RPM;
			TXData_RPM[7] = (uint8_t)Rear_L_Wheel->RPM >> 8;
		}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		Front_R.done = true;
	}
	if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		Front_L.done = true;
	}
	if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		Rear_R.done = true;
	}
	if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		Rear_L.done = true;
	}
}


uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void callbackSystick() //1ms count
{
	static int count = 0;
	count++;
	if (count == 5)  //10ms send can tx message
	{
		if(tx_flag == brake)
		{
			Transmit_CAN(&hcan1, &TXH_Brake, TXData_Brake, &mailbox);
			tx_flag = RPM;
		}
		else if(tx_flag == RPM)
		{
			Transmit_CAN(&hcan1, &TXH_RPM, TXData_RPM, &mailbox);
			tx_flag = brake;
		}
		count = 0;
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
