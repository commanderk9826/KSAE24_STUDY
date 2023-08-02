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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define SLAVE_ADDR 0xd0 //

#define ADDRESS_SMPLRT_DIV 0x19
#define ADDRESS_GYRO_CONFIG 0x1B
#define ADDRESS_ACCEL_CONFIG 0x1C
#define ADDRESS_PWR_MGMT_1 0x6B


#define ADDRESS_ACCEL_XOUT_H 0x3B  //ACCEL_XOUT[15:8]
#define ADDRESS_ACCEL_XOUT_L 0x3C  //ACCEL_XOUT[7:0]
#define ADDRESS_ACCEL_YOUT_H 0x3D  //ACCEL_YOUT[15:8]
#define ADDRESS_ACCEL_YOUT_L 0x3E  //ACCEL_YOUT[7:0]
#define ADDRESS_ACCEL_ZOUT_H 0x3F  //ACCEL_ZOUT[15:8]
#define ADDRESS_ACCEL_ZOUT_L 0x40  //ACCEL_ZOUT[7:0]

#define ADDRESS_GYRO_XOUT_H 0x43  //GYRO_XOUT[15:8]
#define ADDRESS_GYRO_XOUT_L 0x44  //GYRO_XOUT[7:0]
#define ADDRESS_GYRO_YOUT_H 0x45  //GYRO_YOUT[15:8]
#define ADDRESS_GYRO_YOUT_L 0x46  //GYRO_YOUT[7:0]
#define ADDRESS_GYRO_ZOUT_H 0x47  //GYRO_ZOUT[15:8]
#define ADDRESS_GYRO_ZOUT_L 0x48  //GYRO_ZOUT[7:0]


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
uint8_t READ_DATA[12] = {0};
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
uint8_t check = 0;
uint8_t stat = 0;
uint8_t stat2 = 0;
uint8_t stat3 = 0;
uint8_t stat4 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void BOOT_SET(I2C_HandleTypeDef* hi2c)
{

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // BOOT 10msec
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	uint8_t data;

	HAL_I2C_Init(&hi2c1);

	HAL_I2C_Mem_Read (&hi2c1, SLAVE_ADDR,0x75,1, &check, 1, 1000); //check slave addr
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDR, ADDRESS_PWR_MGMT_1, 1, &data, 1, 1000); // sleep(0x40) -> wake(0x00)
	data = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDR, ADDRESS_SMPLRT_DIV, 1, &data, 1, 1000); // sample rate 7
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDR, ADDRESS_GYRO_CONFIG, 1, &data, 1, 1000); // gyro scale 510 deg/s (0x00)
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDR, ADDRESS_ACCEL_CONFIG, 1, &data, 1, 1000); // acc scale 2g (0x00)

	HAL_Delay(2000);
}
void read_acc(void)
{
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x3B, 1, &READ_DATA[0], 1, 1000); // acc x (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x3C, 1, &READ_DATA[1], 1, 1000); // acc x (7:0)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x3D, 1, &READ_DATA[2], 1, 1000); // acc y (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x3E, 1, &READ_DATA[3], 1, 1000); // acc y (7:0)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x3F, 1, &READ_DATA[4], 1, 1000); // acc z (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x40, 1, &READ_DATA[5], 1, 1000); // acc z (7:0)
	acc_x = (int16_t)(READ_DATA[0] << 8 | READ_DATA[1]) / 16384.0;
	acc_y = (int16_t)(READ_DATA[2] << 8 | READ_DATA[3]) / 16384.0;
	acc_z = (int16_t)(READ_DATA[4] << 8 | READ_DATA[5]) / 16384.0;
}
void read_gyro(void)
{
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x43, 1, &READ_DATA[6], 1, 1000); // gyro x (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x44, 1, &READ_DATA[7], 1, 1000); // gyro x (7:0)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x45, 1, &READ_DATA[8], 1, 1000); // gyro y (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x46, 1, &READ_DATA[9], 1, 1000); // gyro y (7:0)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x47, 1, &READ_DATA[10], 1, 1000); // gyro z (15:8)
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDR, 0x48, 1, &READ_DATA[11], 1, 1000); // gyro z (7:0)
	gyro_x = (READ_DATA[6] << 8 | READ_DATA[7]) / 131.0;
	gyro_y = (READ_DATA[8] << 8 | READ_DATA[9]) / 131.0;
	gyro_z = (READ_DATA[10] << 8 | READ_DATA[11]) / 131.0;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_I2C_MspInit(&hi2c1);
  BOOT_SET(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_I2C_Mem_Read (&hi2c1, SLAVE_ADDR,0x75,1, &check, 1, 1000); //check slave addr
	  HAL_I2C_Mem_Read (&hi2c1, SLAVE_ADDR,0X6B,1, &stat, 1, 1000);
	  read_acc();
	  read_gyro();
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
