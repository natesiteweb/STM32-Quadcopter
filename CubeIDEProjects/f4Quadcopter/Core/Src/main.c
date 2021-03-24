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
#include "bmp280.h"
#include "compass.h"

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
volatile int32_t ppm_channels[6];//Channel - 1 because index 0 is channel 1
volatile uint32_t millis_timer_base;

volatile uint32_t test_max_frequency = 0;

uint32_t pwm_output_timer, main_loop_timer;
uint32_t how_long_to_loop_main;
uint32_t main_cycle_counter = 0;
float how_long_to_loop_modifier = 1;
uint32_t temp_led_timer;

uint8_t temp_test_text_buffer[35];
uint32_t temp_control_loop_test_time = 3000;

uint8_t status_first = 0;

int16_t last_x_value, last_y_value, last_z_value;
int32_t x_deviation_sum = 10000, y_deviation_sum = 10000, z_deviation_sum = 10000;

/*
 * Control Boolean Variables
 */
uint8_t altitude_hold_flag = 0;

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
  auto_packet_buffer[0].id = GYRO_PACKET;
  auto_packet_buffer[0].send_rate = 1;
  AddToAutoBuffer(0, (uint8_t *)&(raw_gyro_acc_data[0]), 2);
  AddToAutoBuffer(0, (uint8_t *)&(raw_gyro_acc_data[1]), 2);
  AddToAutoBuffer(0, (uint8_t *)&(raw_gyro_acc_data[2]), 2);
  AddToAutoBuffer(0, (uint8_t *)&gyro_x_angle, 4);
  AddToAutoBuffer(0, (uint8_t *)&gyro_y_angle, 4);
  AddToAutoBuffer(0, (uint8_t *)&compass_heading, 4);
  AddToAutoBuffer(0, (uint8_t *)&how_long_to_loop_main, 4);
  AddToAutoBuffer(0, (uint8_t *)&(ppm_channels[2]), 4);
  AddToAutoBuffer(0, &status_first, 1);
  auto_packet_count += 1;

  auto_packet_buffer[1].total_width = 0;
  auto_packet_buffer[1].var_count = 0;
  auto_packet_buffer[1].id = PID_OUTPUT_PACKET;
  auto_packet_buffer[1].send_rate = 1;
  AddToAutoBuffer(1, (uint8_t *)&pid_roll_output, 4);
  AddToAutoBuffer(1, (uint8_t *)&pid_pitch_output, 4);
  AddToAutoBuffer(1, (uint8_t *)&pid_yaw_output, 4);
  //AddToAutoBuffer(1, &pid_pitch_output, 4);
  auto_packet_count += 1;

  auto_packet_buffer[2].total_width = 0;
  auto_packet_buffer[2].var_count = 0;
  auto_packet_buffer[2].id = ALTITUDE_PACKET;
  auto_packet_buffer[2].send_rate = 5;
  AddToAutoBuffer(2, (uint8_t *)&slow_bmp_altitude, 4);
  auto_packet_count += 1;

  for(int i = 0; i < 6; i++)
  {
	  ppm_channels[i] = 1000;
  }

  ppm_channels[4] = 2000;

  for(int i = 0; i < 35; i++)
  {
	  empty_data_packet.payload[i] = '\0';
  }

  Setup_IMU();
  Setup_BMP280();

  //Motor PID Gains
  EEPROM_Clear_Buffer();
  EEPROM_Read_Page(0, 24);
  eeprom_read_buffer_index = 0;
  EEPROM_Read_Buffer((uint8_t *)&kp_roll, 4);
  EEPROM_Read_Buffer((uint8_t *)&ki_roll, 4);
  EEPROM_Read_Buffer((uint8_t *)&kd_roll, 4);
  EEPROM_Read_Buffer((uint8_t *)&kp_yaw, 4);
  EEPROM_Read_Buffer((uint8_t *)&ki_yaw, 4);
  EEPROM_Read_Buffer((uint8_t *)&kd_yaw, 4);

  //Altitude and GPS PID Gains
  EEPROM_Clear_Buffer();
  EEPROM_Read_Page(32, 12);
  eeprom_read_buffer_index = 0;
  EEPROM_Read_Buffer((uint8_t *)&kp_alt, 4);
  EEPROM_Read_Buffer((uint8_t *)&ki_alt, 4);
  EEPROM_Read_Buffer((uint8_t *)&kd_alt, 4);

  //Compass Calibration Values
  EEPROM_Clear_Buffer();
  EEPROM_Read_Page(64, 12);
  eeprom_read_buffer_index = 0;
  EEPROM_Read_Buffer((uint8_t *)&compass_x_min, 2);
  EEPROM_Read_Buffer((uint8_t *)&compass_x_max, 2);
  EEPROM_Read_Buffer((uint8_t *)&compass_y_min, 2);
  EEPROM_Read_Buffer((uint8_t *)&compass_y_max, 2);
  EEPROM_Read_Buffer((uint8_t *)&compass_z_min, 2);
  EEPROM_Read_Buffer((uint8_t *)&compass_z_max, 2);

  Setup_Compass();

  //CDC_Transmit_FS(buf, strlen((char*)buf));

  //Calibrate_BMP280();
  //Calibrate_IMU();

  /*program_buffer[0] = 0x01;
  program_buffer[1] = 0x01;
  program_buffer[2] = 0x02;

  for(int i = 0; i < 4; i++)
  {
	  program_buffer[3+i] = *(((uint8_t *)&temp_control_loop_test_time) + i);
  }

  program_buffer[7] = 0x01;
  program_buffer[8] = 0x02;

  program_buffer[9] = 0x02;

  for(int i = 0; i < 4; i++)
  {
	  program_buffer[10+i] = *(((uint8_t *)&temp_control_loop_test_time) + i);
  }

  //program_buffer[14] = 0x03;//Restart Program
  program_buffer[14] = 0x04;*/

  HAL_Delay(2000);

  while(abs(x_deviation_sum) > 20 || abs(y_deviation_sum) > 20 || abs(z_deviation_sum) > 20)
  {
	  for(int i = 0; i < 200; i++)
	  {
		  if(i == 0)
		  {
			  x_deviation_sum = 0;
			  y_deviation_sum = 0;
			  z_deviation_sum = 0;
		  }

		  Read_IMU(0);

		  x_deviation_sum += abs(raw_gyro_acc_data[0] - last_x_value);
		  y_deviation_sum += abs(raw_gyro_acc_data[1] - last_y_value);
		  z_deviation_sum += abs(raw_gyro_acc_data[2] - last_z_value);

		  last_x_value = raw_gyro_acc_data[0];
		  last_y_value = raw_gyro_acc_data[1];
		  last_z_value = raw_gyro_acc_data[2];
		  HAL_Delay(5);
	  }

	  x_deviation_sum /= 200;
	  y_deviation_sum /= 200;
	  z_deviation_sum /= 200;
  }

  Calibrate_BMP280();
  Calibrate_IMU();
  ClearPrintBuffer();
  sprintf((char *)print_text_buffer, "%s", "Gyro Calibrated.\n");
  PrintManualPacket();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ppm_channels[4] < 1600)
	  {
		  manual_mode = 1;
		  status_first |= 1 << 1;

		  launched = 0;
		  landing = 0;
		  launching = 0;
		  altitude_hold_flag = 0;
	  }
	  else
	  {
		  manual_mode = 0;
		  status_first &= ~(1 << 1);

		  Control_Loop();
	  }

	  status_first = ((status_first | 0x01) * launched) + ((status_first & ~(0x01)) * (launched ^ 0x01));

	  if(GetMillisDifference(&temp_led_timer) > 500)
	  {
		  temp_led_timer = GetMillis();

		  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }

	  if(GetMicrosDifference(&pwm_output_timer) >= 4000)
	  {
		  pwm_output_timer = GetMicros();
		  //__HAL_TIM_SET_COUNTER(&htim8, 4999); //Reset motor PWN counter for fast response time(probably makes esc refresh rate faster)
	  }

	  if(GetMicrosDifference(&main_loop_timer) >= 2000)
	  {
		  if(main_cycle_counter > 399)
			  main_cycle_counter = 0;

		  if(main_cycle_counter % 4 == 0)//Every 4 clock cycles(500uS * 4 = 2000uS) NOT IN USE RIGHT NOW
		  {

		  }

		  how_long_to_loop_main = GetMicrosDifference(&main_loop_timer);
		  if(how_long_to_loop_main > 4000)
			  how_long_to_loop_main = 3000;
		  how_long_to_loop_modifier = (float)(round(((float)((float)how_long_to_loop_main / 2000)) * 100.0) / 100.0);
		  main_loop_timer = GetMicros();

		  if(launching && !launched)
		  {
			  //Launch logic here
			  Launch_Behavior();
		  }

		  if(landing && launched)
		  {
			  //Landing logic here
			  Land_Behavior();
		  }

		  if(main_cycle_counter % 20)
		  {
			  Read_Compass();
			  Read_BMP280_PressureTemperature();
			  if(altitude_hold_flag)
				  Calculate_Altitude_PID();
			  else
			  {
				  altitude_pid_output = 0;
				  pid_alt_last_error = 0;
				  pid_alt_i = 0;
			  }
		  }

		  Read_IMU(0);
		  Calculate_Attitude();
		  //Calculate all motors values, then immediately output them using oneshot125
		  Motor_PID();
		  Calculate_Motor_Outputs();

		  __HAL_TIM_SET_COUNTER(&htim8, 3999);

		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, esc1_output);
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, esc2_output);
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, esc3_output);
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, esc4_output);

		  main_cycle_counter++;
	  }

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
