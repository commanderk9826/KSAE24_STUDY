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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN 88
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
uint32_t rx_count = 0;
uint32_t SOC , Current_Sensor = 0 ;
uint8_t RX_BUFFER[BUFFER_LEN] = { 0 };
uint8_t TX_BUFFER[BUFFER_LEN] = { 0 };
uint16_t F_L_RPM_Sensor = 0;
uint16_t F_R_RPM_Sensor = 0;
uint16_t R_L_RPM_Sensor = 0;
uint16_t R_R_RPM_Sensor = 0;
int8_t acc_x1, acc_x2, acc_y1, acc_y2, acc_z1, acc_z2, gyr_x1, gyr_x2, gyr_y1,
		gyr_y2, gyr_z1, gyr_z2 = 0;
int8_t ang_x1, ang_x2, ang_y1, ang_y2, ang_z1, ang_z2 = 0;
int16_t acc_X, acc_Y, acc_Z, gyr_X, gyr_Y, gyr_Z, ang_X, ang_Y, ang_Z = 0;
int16_t GDBT, CBT, coolant_Temp, accel_transform, Motor_Temp, Motor_RPM,
		phaseA_Current, phaseB_Current, phaseC_Current, DCBus_Current,
		DCBus_Voltage, Output_Voltage, Vd_Voltage, Vq_Voltage, Id_Current,
		Iq_Current, commanded_torque, feedback_torque = 0;
int8_t VSM_state, Inverter_state = 0;
uint16_t F_L_Linear,F_R_Linear,R_L_Linear,R_R_Linear,Steering_Linear = 0;
uint16_t Sum_Voltage = 0;
uint16_t A_Max_Voltage,B_Max_Voltage,C_Max_Voltage,D_Max_Voltage,E_Max_Voltage = 0;
uint16_t A_Min_Voltage,B_Min_Voltage,C_Min_Voltage,D_Min_Voltage,E_Min_Voltage = 0;
uint8_t A_Temp,B_Temp,C_Temp,D_Temp,E_Temp = 0;
uint16_t Brake_Pressure = 0;
uint8_t Max_Current = 0;
float Lv_Voltage , Lv_Temp , Lv_Current = 0;
uint8_t Lv_C_Count, Lv_Bms_stat = 0;
float Lv_Temp = 0;
float Adc1, Adc2 = 0;
long double Adc_Temp, Ntc_R = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Sum_Voltage = (RX_BUFFER[1] << 8) + RX_BUFFER[0];
		A_Max_Voltage = RX_BUFFER[2] + 255;
		B_Max_Voltage = RX_BUFFER[3] + 255;
		C_Max_Voltage = RX_BUFFER[4] + 255;
		D_Max_Voltage = RX_BUFFER[5] + 255;
		E_Max_Voltage = RX_BUFFER[6] + 255;
		A_Min_Voltage = RX_BUFFER[7] + 255;
		B_Min_Voltage = RX_BUFFER[8] + 255;
		C_Min_Voltage = RX_BUFFER[9] + 255;
		D_Min_Voltage = RX_BUFFER[10] + 255;
		E_Min_Voltage = RX_BUFFER[11] + 255;
		A_Temp = RX_BUFFER[12];
		B_Temp = RX_BUFFER[13];
		C_Temp = RX_BUFFER[14];
		D_Temp = RX_BUFFER[15];
		E_Temp = RX_BUFFER[16];
		Lv_Voltage = ((RX_BUFFER[17] << 8) | RX_BUFFER[18]) * 0.001532;
		SOC = RX_BUFFER[19];
		Current_Sensor = RX_BUFFER[21];
		Max_Current = RX_BUFFER[23];
		GDBT = ((RX_BUFFER[25] << 8) | RX_BUFFER[24]);
		CBT = ((RX_BUFFER[27] << 8) | RX_BUFFER[26]);
		coolant_Temp = ((RX_BUFFER[29] << 8) | RX_BUFFER[28]);
		accel_transform = ((RX_BUFFER[33] << 8) | RX_BUFFER[32]);
		accel_transform = (0b0000001111111111 && accel_transform);
		Motor_Temp = ((RX_BUFFER[31] << 8) | RX_BUFFER[30]);
		Motor_RPM = ((RX_BUFFER[35] << 8) | RX_BUFFER[34]);
		phaseA_Current = ((RX_BUFFER[37] << 8) | RX_BUFFER[36]);
		phaseB_Current = ((RX_BUFFER[39] << 8) | RX_BUFFER[38]);
		phaseC_Current = ((RX_BUFFER[41] << 8) | RX_BUFFER[40]);
		DCBus_Current = ((RX_BUFFER[43] << 8) | RX_BUFFER[42]);
		DCBus_Voltage = ((RX_BUFFER[45] << 8) | RX_BUFFER[44]);
		Output_Voltage = ((RX_BUFFER[47] << 8) | RX_BUFFER[46]);
		Vd_Voltage = ((RX_BUFFER[49] << 8) | RX_BUFFER[48]);
		Vq_Voltage = ((RX_BUFFER[51] << 8) | RX_BUFFER[50]);
		Id_Current = ((RX_BUFFER[53] << 8) | RX_BUFFER[52]);
		Iq_Current = ((RX_BUFFER[55] << 8) | RX_BUFFER[54]);
		VSM_state = RX_BUFFER[56];
		Inverter_state = RX_BUFFER[57];
		commanded_torque = ((RX_BUFFER[59] << 8) | RX_BUFFER[58]);
		feedback_torque = ((RX_BUFFER[61] << 8) | RX_BUFFER[60]);
		Brake_Pressure = (RX_BUFFER[63] << 8) + RX_BUFFER[62];
		acc_x1 = (int8_t) RX_BUFFER[64];
		acc_x2 = (int8_t) RX_BUFFER[65];
		acc_y1 = (int8_t) RX_BUFFER[66];
		acc_y2 = (int8_t) RX_BUFFER[67];
		acc_z1 = (int8_t) RX_BUFFER[68];
		acc_z2 = (int8_t) RX_BUFFER[69];

		acc_X = (acc_x2 << 8) + (uint8_t) acc_x1;
		acc_Y = (acc_y2 << 8) + (uint8_t) acc_y1;
		acc_Z = (acc_z2 << 8) + (uint8_t) acc_z1;

		gyr_x1 = (int8_t) RX_BUFFER[70];
		gyr_x2 = (int8_t) RX_BUFFER[71];
		gyr_y1 = (int8_t) RX_BUFFER[72];
		gyr_y2 = (int8_t) RX_BUFFER[73];
		gyr_z1 = (int8_t) RX_BUFFER[74];
		gyr_z2 = (int8_t) RX_BUFFER[75];

		gyr_X = (gyr_x2 << 8) + (uint8_t) gyr_x1;
		gyr_Y = (gyr_y2 << 8) + (uint8_t) gyr_y1;
		gyr_Z = (gyr_z2 << 8) + (uint8_t) gyr_z1;

		ang_x1 = (int8_t) RX_BUFFER[76];
		ang_x2 = (int8_t) RX_BUFFER[77];
		ang_y1 = (int8_t) RX_BUFFER[78];
		ang_y2 = (int8_t) RX_BUFFER[79];
		ang_z1 = (int8_t) RX_BUFFER[80];
		ang_z2 = (int8_t) RX_BUFFER[81];

		ang_X = (ang_x2 << 8) + (uint8_t) ang_x1;
		ang_Y = (ang_y2 << 8) + (uint8_t) ang_y1;
		ang_Z = (ang_z2 << 8) + (uint8_t) ang_z1;

		Lv_Bms_stat = RX_BUFFER[82];
		Adc_Temp = ((RX_BUFFER[83] << 8) + RX_BUFFER[84]);
		//Ntc_R = (Adc_Temp*(5/4096)/(5-Adc_Temp*(5/4096)))*10185;
		Adc1 = Adc_Temp*5;
		Adc1 = Adc1/4096;
		Adc2 = Adc_Temp * 5;
		Adc2 = Adc2/4096;
		Adc2 = 5 - Adc2;
		Ntc_R = Adc1/Adc2*10185;
		Lv_Temp = (float)1/(((float)1/3435)*log(Ntc_R/10000)+((float)1/298.15))-273.15;
		Lv_Current = ((RX_BUFFER[85] << 8) + RX_BUFFER[86]) * 0.00855;
		Lv_C_Count = RX_BUFFER[87];






	}

}
  /* USER CODE END 3 */


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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		HAL_UART_Receive_IT(&huart1, RX_BUFFER, sizeof(RX_BUFFER));
		rx_count++;
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
	while (1) {
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
