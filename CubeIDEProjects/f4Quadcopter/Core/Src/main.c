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
#include "telemetry.h"
#include "math.h"
#include "imu.h"
#include "control_logic.h"
#include "eeprom.h"

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
uint8_t test_gyro_send_buf[40];
volatile uint32_t val;
volatile uint32_t i2c_transmit_timer = 0;
float gyro_x;

uint8_t read_flag = 0;

volatile uint32_t current_ppm_capture = 0, last_ppm_capture = 0;
volatile uint32_t frequency_read = 1000;
volatile uint8_t current_ppm_channel = 0;
volatile uint32_t ppm_channels[7];//Channel - 1 because index 0 is channel 1
volatile uint32_t millis_timer_base;

volatile uint32_t test_max_frequency = 0;

uint32_t pwm_output_timer, main_loop_timer;
uint32_t how_long_to_loop_main;

uint32_t temp_led_timer;

uint8_t temp_test_text_buffer[35];

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
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim9);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);//Motor 1 - FL
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);//Motor 2 - FR
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);//Motor 3 - BR
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);//Motor 4 - BL

  auto_packet_buffer[0].total_width = 0;
  auto_packet_buffer[0].var_count = 0;
  auto_packet_buffer[0].id = 0x01;
  AddToAutoBuffer(0, &(raw_gyro_acc_data[0]), 2);
  AddToAutoBuffer(0, &(raw_gyro_acc_data[1]), 2);
  AddToAutoBuffer(0, &(raw_gyro_acc_data[2]), 2);
  AddToAutoBuffer(0, &gyro_x_angle, 4);
  AddToAutoBuffer(0, &gyro_y_angle, 4);
  AddToAutoBuffer(0, &gyro_z_angle, 4);
  AddToAutoBuffer(0, &how_long_to_loop_main, 4);
  AddToAutoBuffer(0, &(ppm_channels[2]), 4);
  auto_packet_count += 1;

  for(int i = 0; i < 6; i++)
  {
	  ppm_channels[i] = 1000;
  }

  for(int i = 0; i < 35; i++)
  {
	  empty_data_packet.payload[i] = '\0';
  }

  Setup_IMU();

  //CDC_Transmit_FS(buf, strlen((char*)buf));

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(GetMillisDifference(&temp_led_timer) > 500)
	  {
		  temp_led_timer = GetMillis();

		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }

	  if(GetMicrosDifference(&pwm_output_timer) >= 4000)
	  {
		  pwm_output_timer = GetMicros();
		  __HAL_TIM_SET_COUNTER(&htim8, 4999); //Reset motor PWN counter for fast response time
	  }

	  if(GetMicrosDifference(&main_loop_timer) >= 2000)
	  {
		  how_long_to_loop_main = GetMicrosDifference(&main_loop_timer);
		  main_loop_timer = GetMicros();

		  Read_IMU(0);

		  gyro_x = (float)raw_gyro_acc_data[0] / 65.5;
		  gyro_y = (float)raw_gyro_acc_data[1] / -65.5;
		  gyro_z = (float)raw_gyro_acc_data[2] / -65.5;

		  acc_magnitude = sqrt(((float)raw_gyro_acc_data[3] * (float)raw_gyro_acc_data[3]) + ((float)raw_gyro_acc_data[4] * (float)raw_gyro_acc_data[4]) + ((float)raw_gyro_acc_data[5] * (float)raw_gyro_acc_data[5]));

		  if(acc_magnitude != 0)
		  {
			  if(abs(raw_gyro_acc_data[4]) < acc_magnitude)
			  {
				  acc_x = asin((float)raw_gyro_acc_data[4] / acc_magnitude) * 57.296;
			  }

			  if(abs(raw_gyro_acc_data[3]) < acc_magnitude)
			  {
				  acc_y = asin((float)raw_gyro_acc_data[3] / acc_magnitude) * 57.296;
			  }
		  }

		  gyro_x_angle += (gyro_x) * ((float)how_long_to_loop_main / 1000000);
		  gyro_y_angle += (gyro_y) * ((float)how_long_to_loop_main / 1000000);
		  gyro_z_angle += (gyro_z) * ((float)how_long_to_loop_main / 1000000);

		  gyro_x_angle += (gyro_y_angle * sin(gyro_z * 0.01745 * ((float)how_long_to_loop_main / 1000000)));
		  gyro_y_angle -= (gyro_x_angle * sin(gyro_z * 0.01745 * ((float)how_long_to_loop_main / 1000000)));

		  gyro_x_angle = (gyro_x_angle * 0.9985) + (acc_x * (1.0000 - 0.9985));
		  gyro_y_angle = (gyro_y_angle * 0.9985) + (acc_y * (1.0000 - 0.9985));

		  if(gyro_z_angle < 0)
			  gyro_z_angle += 360;
		  if(gyro_z_angle >= 360)
			  gyro_z_angle -= 360;
	  }

	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, ppm_channels[2]);
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, ppm_channels[2]);
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ppm_channels[2]);
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ppm_channels[2]);

	  telem_loop();

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

			//frequency_read /= 2;

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

			if(frequency_read < 1000)
				frequency_read = 1000;
			else if(frequency_read > 2000)
				frequency_read = 2000;

			if(current_ppm_channel >= 1 && current_ppm_channel <= 6)
			{
				ppm_channels[current_ppm_channel - 1] = frequency_read;
			}

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim9)
	{
		//micros_timer_base += 65000;//65536;
		millis_timer_base += 65;//Overflow doesn't matter unless board is running for more than 49 days
	}
}

uint32_t GetMicros()
{
	//return micros_timer_base + __HAL_TIM_GET_COUNTER(&htim4);
	return __HAL_TIM_GET_COUNTER(&htim9);
}

uint32_t GetMillis()
{
	return millis_timer_base + (GetMicros() / 1000);
}

uint32_t GetMillisDifference(uint32_t *timer_counter_to_use)
{
	return GetMillis() - *timer_counter_to_use;
}

uint32_t GetMicrosDifference(uint32_t *timer_counter_to_use)
{
	uint32_t current_micros = GetMicros();
	uint32_t micros_difference = 0;

	if(current_micros > *timer_counter_to_use)
	{
		micros_difference = current_micros - *timer_counter_to_use;
	}
	else if(current_micros < *timer_counter_to_use)
	{
		micros_difference = 65000 + current_micros - *timer_counter_to_use;
	}

	return micros_difference;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
		tx_done = 1;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
		rx_done = 1;
		acks_counted++;
	}
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
	}
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

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
