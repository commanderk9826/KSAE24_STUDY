/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "SX1278.h"

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

/*SD logging parameter*/
UINT bw;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t total;
uint32_t freespace;
uint32_t SDstate=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LORA(void);
void SDcard(void);
/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/

int _write(int file, char *ptr, int len) {
	int i;
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	for (i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int master;
int ret;
int i;
int t;
char buffer[512];
int message;
int message_length;
int Current=88;
int sum_Voltage=11;
int Linear_Left=22;
int Linear_Right=33;
int Max_Temperature=44;
int Min_Temperature=55;



/*SD data*/
char SDbuffer[512];
int time=0;
int linear=0;

uint16_t  Time, Battery_Sum_voltage, Frontright_motor_rpm, Frontleft_motor_rpm;

/*LORA data*/
uint16_t  Baterry_voltage[70];
uint16_t  Battery_temp, Motorcontroller_temp, Motor_temp, Motorcontroller_current, Motorcontroller_voltage, Motor_rpm, Motor_Torque;



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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
//Current=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
//	master = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
	/*	if (master == 1) {
			ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		} else {
			ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}
		*/

	//initialize LoRa module
	SX1278_hw.dio0.port = DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = DIO0_Pin;
	SX1278_hw.nss.port = NSS_GPIO_Port;
	SX1278_hw.nss.pin = NSS_Pin;
	SX1278_hw.reset.port = RESET_GPIO_Port;
	SX1278_hw.reset.pin = RESET_Pin;
	SX1278_hw.spi = &hspi2;

	SX1278.hw = &SX1278_hw;

	printf("Configuring LoRa module\r\n");
	SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);

	//	SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule\r\n");
//	ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);//16,2000
	ret = SX1278_LoRaEntryRx(&SX1278, 16, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    LORA();

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
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void LORA(void)
{
		//ret = SX1278_LoRaEntryRx(&SX1278, 16, 200);
		//printf("Slave ...\r\n");

		printf("Receiving package...\r\n");
		HAL_Delay(1000);
		ret = SX1278_LoRaRxPacket(&SX1278);
        printf("Received: %d\r\n", ret);
        if (ret > 0) {
        //	printf("Time Battery_Sum_voltage Battery_voltage_min Battery_voltage_max Battery_temp Motorcontroller_temp Motor_temp Motorcontroller_current Motorcontroller_voltage Motor_rpm Motor_Torque\r\n");
        //	printf("TIME Sum_of_Voltage Max_Voltage Min_Voltage Max_temp CAN_L_motor_temp, CAN_L_invertertemp, Motor_L_CommandedTorque, Motor_L_TorqueFeedback, CAN_L_current, CAN_L_voltage, CAN_L_VSM_State, CAN_L_Inverter_State, CAN_L_RPM, CAN_R_motor_temp, CAN_R_invertertemp, Motor_R_CommandedTorque, Motor_R_TorqueFeedback, CAN_R_current, CAN_R_voltage, CAN_R_VSM_State, CAN_R_Inverter_State, CAN_R_RPM\r\n");

            SX1278_read(&SX1278, (uint8_t*) buffer, ret);


       //     SX1278_read(&SX1278, (uint8_t*) buffer, ret);
           	printf("%s\r\n", buffer);
                  		  								  t=1;
                  		  								 printf("\r\nTIME: ");
                  		  								  for(i=0;i<246;i++){
                  		  								  if(buffer[i]==0) break;

                  		  								  if(buffer[i]!=32){
                  		  									 printf("%c",buffer[i]);
                  		  									 }

                  		  								  else if(buffer[i]==32&&t==1){
                  		  									 printf("\r\nSum_of_Voltage: ");
                  		  									 t++;
                  		  								  }
                  		  								  else if(buffer[i]==32&&t==2){
                  		  									 printf("\r\nMax_Voltage: ");
                  		  									 t++;
                  		  								  }
                  		  								  else if(buffer[i]==32&&t==3){
                  		  									 printf("\r\nMin_Voltage: ");
                  		  									 t++;
                  		  								  }
                  		  								  else if(buffer[i]==32&&t==4){
                  		  									 printf("\r\nMax_temp: ");
                  		  									 t++;
                  		  								  }

                  		  								  else if(buffer[i]==32&&t==5){
                  		  								 	 printf("\r\n\r\nCAN_L_motor_temp: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==6){
                  		  								 	 printf("\r\nCAN_L_invertertemp: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==7){
                   		  								 	 printf("\r\nMotor_L_CommandedTorque:  ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==8){
                   		  								 	 printf("\r\nMotor_L_TorqueFeedback: ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==9){
                   		  								 	 printf("\r\nCAN_L_current: ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==10){
                   		  								 	 printf("\r\nCAN_L_voltage: ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==11){
                   		  								 	 printf("\r\nCAN_L_VSM_State : ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==12){
                   		  								 	 printf("\r\nCAN_L_Inverter_State: ");
                   		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==13){
                  		  								 	 printf("\r\nCAN_L_RPM: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==14){
                  		  								 	 printf("\r\n\r\nCAN_R_motor_temp: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==15){
                  		  								 	 printf("\r\nCAN_R_invertertemp: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==16){
                  		  								 	 printf("\r\nMotor_R_CommandedTorque: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==17){
                  		  								 	 printf("\r\nMotor_R_TorqueFeedback ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==18){
                  		  								 	 printf("\r\nCAN_R_current: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==19){
                  		  								 	 printf("\r\nCAN_R_voltage: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==20){
                  		  								 	 printf("\r\nCAN_R_VSM_State: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==21){
                  		  								 	 printf("\r\nCAN_R_Inverter_State: ");
                  		  								 	 t++;
                  		  								}
                  		  								  else if(buffer[i]==32&&t==22){
                  		  								 	 printf("\r\nCAN_R_RPM ");
                  		  								 	 t++;
                  		  								}



                  		  						}

           // SDcard();
        }
        SDcard();
     /*   if (ret > 0) {
        		  								  SX1278_read(&SX1278, (uint8_t*) buffer, ret);
        		  								  t=0;
        		  								  for(i=0;i<246;i++){
        		  								  if(buffer[i]==0) break;

        		  								  if(buffer[i]!=32){
        		  									 printf("%c",buffer[i]);
        		  									 }
        		  								  else if(buffer[i]==32&&t==0){
        		  									 printf("\r\nTIME: ");
        		  									 t++;
        		  								  }
        		  								  else if(buffer[i]==32&&t==1){
        		  									 printf("\r\nSum_of_Voltage:");
        		  									 t++;
        		  								  }
        		  								  else if(buffer[i]==32&&t==2){
        		  									 printf("\r\nMax_Voltage: ");
        		  									 t++;
        		  								  }
        		  								  else if(buffer[i]==32&&t==3){
        		  									 printf("\r\nMin_Voltage: ");
        		  									 t++;
        		  								  }
        		  								  else if(buffer[i]==32&&t==4){
        		  									 printf("\r\nMax_temp: ");
        		  									 t++;
        		  								  }

        		  								  else if(buffer[i]==32&&t==5){
        		  								 	 printf("\r\n CAN_L_motor_temp CAN_L_invertertemp Motor_L_CommandedTorque Motor_L_TorqueFeedback CAN_L_current CAN_L_voltage CAN_L_VSM_State CAN_L_Inverter_State CAN_L_RPM \r\n");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==6){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==7){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==8){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==9){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==10){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==11){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==12){
         		  								 	 printf(" ");
         		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==13){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==14){
        		  								 	 printf("\r\n  CAN_R_motor_temp CAN_R_invertertem Motor_R_CommandedTorque Motor_R_TorqueFeedback CAN_R_current CAN_R_voltage CAN_R_VSM_State CAN_R_Inverter_State CAN_R_RPM \r\n");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==15){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==16){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==17){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==18){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==19){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==20){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==21){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}
        		  								  else if(buffer[i]==32&&t==22){
        		  								 	 printf(" ");
        		  								 	 t++;
        		  								}



        		  						}

        		  }

*/


        printf("\r\n------------------------------------------------------------------------------------------------------------------------------------\r\n");


}
void SDcard(void)
{
	 /* SD card -------------------------------------------------------------------------------------------------------------------*/
	 /* SDcard logging -------------------------------------------------------------------------------------------------------------------*/
	 /* Mount SD Card */

	/* start */
	if(SDstate==1){
		f_mount(&fs, "", 0);
		/* Open file to write */
		f_open(&fil, "DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	   /* Check free space */
	   f_getfree("", &fre_clust, &pfs);
	   total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	   freespace = (uint32_t)(fre_clust * pfs->csize * 0.5);
	   // Free space is less than 1kb

	   /* show start logging */
	   SDstate=2;
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	   f_puts("TIME Sum_of_Voltage Max_Voltage Min_Voltage Max_temp CAN_L_motor_temp CAN_L_invertertemp Motor_L_CommandedTorque Motor_L_TorqueFeedback CAN_L_current CAN_L_voltage CAN_L_VSM_State CAN_L_Inverter_State CAN_L_RPM CAN_R_motor_temp CAN_R_invertertemp Motor_R_CommandedTorque Motor_R_TorqueFeedback CAN_R_current CAN_R_voltage CAN_R_VSM_State CAN_R_Inverter_State CAN_R_RPM", &fil);
	   f_puts("\n", &fil);
			}


	/* running */
	else if(SDstate==2){
	  /* Writing text */
	  /*
	  //example how to write
	  sprintf(SDbuffer,"%d",Shock_linear);
	  f_puts("Shock_linear:", &fil);
	  f_write(&fil, SDbuffer , strlen(SDbuffer), &bw);
	  f_puts("\n", &fil);
	  Shock_linear++;
	  */

		//  f_write(&fil, SDbuffer , strlen(buffer), &bw);//roka rx sd Logging
			for(i=0;i<512;i++){
			SDbuffer[i]=buffer[i];
							}
			//  sprintf(SDbuffer,"%d:%d:%d, %d, %d, %d, %d, %d\n",time_data_min,time_data_sec, time_data, (int)Shock_linear[0], (int)Shock_linear[1], (int)Shock_linear[2], (int)Shock_linear[3],(int)Temperature);
			f_write(&fil, SDbuffer , strlen(SDbuffer), &bw);
			f_puts("\n", &fil);
	 }

	  /* Close file */
	else if(SDstate==3){
	     f_close(&fil);
	     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  }


}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{

		/*start*/
		if(SDstate==0){
			SDstate=1;
		}

		/*running*/
		else if(SDstate==2){
			SDstate=3;
		}

    	/* closed */
		else if(SDstate==3){
			SDstate=0;
		}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
