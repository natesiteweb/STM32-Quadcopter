/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"

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

uint8_t data[6] = "hello\n";

HAL_StatusTypeDef ret;
static const uint8_t GYRO_ADDR = 0x68 << 1;
uint8_t buf[40];
uint8_t test_gyro_send_buf[40];
volatile uint32_t val;
volatile uint32_t i2c_transmit_timer = 0;
float gyro_x;

uint8_t read_flag = 0;
uint8_t send_buffer[35];
uint8_t receive_buffer[35];

volatile uint8_t ack_rate_counter = 0;
uint8_t ack_rate = 20;//Every x ticks of the radio ask for data

volatile uint32_t current_ppm_capture = 0, last_ppm_capture = 0;
volatile uint32_t frequency_read = 1000;
volatile uint8_t current_ppm_channel = 0;
volatile uint32_t ppm_channels[6];//Channel - 1 because index 0 is channel 1

volatile uint32_t test_max_frequency = 0;

int16_t test_gyro_x = 0;


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

  HAL_Delay(500);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

  for(int i = 0; i < 32; i++)
  {
	  send_buffer[i] = '\0';
  }

  /*send_buffer[0] = 0x09;
  send_buffer[1] = 6;
  send_buffer[2] = 'h';
  send_buffer[3] = 'e';
  send_buffer[4] = 'l';
  send_buffer[5] = 'l';
  send_buffer[6] = 'o';
  send_buffer[7] = '\n';*/

  //sprintf((char*)send_buffer, "%l%s", test_capture_value, "\r\n");

  send_buffer[32] = 30;
  send_buffer[33] = 0;//Unreliable
  send_buffer[34] = 0;//No data

  buf[0] = 0x6B;
  buf[1] = 0x00;
  ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
  if(ret != HAL_OK)
  {
	  strcpy((char*)buf, "Error Tx\r\n");
  }

  HAL_Delay(10);

  buf[0] = 0x1B;
  buf[1] = 0x08;
  ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
  if(ret != HAL_OK)
  {
	  strcpy((char*)buf, "Error Tx\r\n");
  }

  HAL_Delay(10);

  buf[0] = 0x1A;
  buf[1] = 0x03;
  ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
  if(ret != HAL_OK)
  {
	  strcpy((char*)buf, "Error Tx\r\n");
  }

  //CDC_Transmit_FS(buf, strlen((char*)buf));

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(5);

	  if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY && send_buffer[34] == 0)
	  {
		  if(ack_rate_counter < 0xFF)
			  ack_rate_counter++;

		  for(int i = 0; i < 32; i++)
		  {
			  send_buffer[i] = '\0';
		  }

		  //test_gyro_x =

		  //sprintf((char*)send_buffer, "%ld%s", ppm_channels[2], "\r\n");//int32_t
		  sprintf((char*)send_buffer, "%lu%s%hd%s", ppm_channels[2], ":", test_gyro_x, "\r\n");//uint32_t

		  //strcpy((char*)send_buffer, "TEST\r\n");

		  if(ack_rate_counter == ack_rate)
		  {
			  //ack_rate_counter = 0;
			  send_buffer[34] = 1;
		  }
		  else
		  {
			  send_buffer[34] = 0;
		  }



		  send_buffer[32] = 30;
		  send_buffer[33] = 0;//Unreliable
		  //send_buffer[34] = 0;//No data

		  HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)send_buffer, 35);
		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }

	  if(read_flag == 1)
	  {
		  read_flag = 0;

		  test_gyro_x = (int16_t)((buf[0] << 8) | (buf[1]));

		  gyro_x = test_gyro_x / 65.5;

		  //i2c_transmit_timer = HAL_GetTick();
		  //i2c_transmit_timer = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
		  //HAL_Delay(10);



		  //HAL_I2C_Master_Transmit();
		  //HAL_I2C_Master_Transmit_IT();
		  //HAL_Delay(5);
		  //val = ((DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000)) - i2c_transmit_timer);
		  //val = (HAL_GetTick() - i2c_transmit_timer);

		  //val = (int16_t)((buf[0] << 8) | (buf[1]));
		  //gyro_x = val;
		  //itoa(val, buf, 10);
		  //itoa(val, buf, 10);
		  /*if(val > 0)
	  		{
	  			sprintf((char*)buf, "%s", "1\r\n");
	  		}
	  		else
	  		{
	  			sprintf((char*)buf, "%s", "0\r\n");
	  		}*/
		  //sprintf((char*)buf, "%lu%s", val, "\r\n");
		  //snprintf((char*)buf, 0, "%lu%s", val, "\r\n");

		  //CDC_Transmit_FS(buf, strlen((char*)buf));


		  //HAL_Delay(10);
		  //i2c_transmit_timer = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
		  //HAL_I2C_Master_Receive_IT(hi2c, DevAddress, pData, Size)
	  }

	  if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
	  {
		  HAL_I2C_Mem_Read_DMA(&hi2c1, GYRO_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, 2);
	  }

	  //ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, 0x43, 1, buf, 2, HAL_MAX_DELAY);
	  /*if(ret != HAL_OK)
	  	{
	  		strcpy((char*)buf, "Error Rx\r\n");
	  	}
	  	else
	  	{
	  		val = (int16_t)((buf[0] << 8) | (buf[1]));
	  		gyro_x = val;
	  		itoa(val, buf, 10);
	  		sprintf((char*)buf, "%s%s", buf, "\r\n");
	  	}*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		current_ppm_capture = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		{
			last_ppm_capture = current_ppm_capture;

			//&htim3->Instance->CCER |= TIM_CCER_CC1P;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			//frequency_read = current_ppm_capture - last_ppm_capture;

			/*if(frequency_read < 0)
			{
				frequency_read += 0xFFFFFFFF;
			}*/

			if (current_ppm_capture > last_ppm_capture)
			{
				frequency_read = current_ppm_capture - last_ppm_capture;
			}
			else if (current_ppm_capture <= last_ppm_capture)
			{
				/* 0x1000 is max overflow value */
				frequency_read = 0xFFFFFFFF + current_ppm_capture - last_ppm_capture;
			}

			frequency_read /= 2;

			if(frequency_read > 3000)
			{
				current_ppm_channel = 0;
			}
			else
			{
				current_ppm_channel++;
			}

			if(frequency_read > test_max_frequency)
			{
				test_max_frequency = frequency_read;
			}

			frequency_read += 400;

			if(current_ppm_channel >= 1 && current_ppm_channel <= 6)
			{
				ppm_channels[current_ppm_channel - 1] = frequency_read;
			}

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	read_flag = 1;
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
		if(ack_rate_counter == ack_rate)
		{
			//ack_rate_counter = 0;
			HAL_I2C_Master_Receive_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)receive_buffer, 34);
		}
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
		ack_rate_counter = 0;

		//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		send_buffer[34] = 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
