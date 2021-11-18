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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "nrf24radio.h"
#include "gps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MY_ADDRESS 0x04

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

NRF24_RADIO nrf_radio;

volatile uint8_t i2c_receive_flag = 0;
volatile uint8_t new_packet_to_send_available = 0;
volatile uint8_t receieve_buffer[32];

data_packet empty_data_packet;
volatile data_packet *packet_to_send_to_master;

volatile uint32_t micros_timer_base;
volatile uint32_t millis_timer_base;

uint32_t time_since_last_radio_send = 0;

uint32_t test_led_timer = 0, test_i2c_last_received;

uint8_t buf[20];

volatile uint8_t telem_i2c_send_done = 0;


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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  nrf_radio.spiHandle = &hspi2;
  nrf_radio.csnPinPort = NRF_CSN_GPIO_Port;
  nrf_radio.cePinPort = NRF_CE_GPIO_Port;
  nrf_radio.csnPin = NRF_CSN_Pin;
  nrf_radio.cePin = NRF_CE_Pin;

  GPS_Init();

  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim4);

  empty_data_packet.width = 1;
  empty_data_packet.reliable = 0;

  unreliable_packet.width = 0;
  current_i2c_packet.width = 0;

  for(int i = 0; i < 35; i++)
  {
	  empty_data_packet.payload[i] = 0x00;
	  unreliable_packet.payload[i] = 0x00;
	  current_i2c_packet.payload[i] = 0x00;
  }

  packet_to_send_to_master = &empty_data_packet;

  NRF24_Init(&nrf_radio);

  HAL_Delay(500);

  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)current_i2c_packet.payload, 35);

  test_i2c_last_received = GetMillis();

  uint8_t temp_uart_buffer[32];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
	  {
		  //fail
	  }

	  /*if(GetMillisDifference(&test_i2c_last_received) > 900 && GetMillis() > 5000)
	  {
		  //HAL_I2C_DeInit(&hi2c1);
		  //MX_I2C1_Init();
		  //test_i2c_last_received = GetMillis();
		  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		  if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
		  {
			  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)current_i2c_packet.payload, 35);
		  }

		  sprintf((char *)temp_uart_buffer, "%lu%s", HAL_I2C_GetState(&hi2c1), "\r\n");
		  //hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
		  //HAL_I2C_STATE_BUSY;

		  HAL_UART_Transmit_IT(&huart1, (uint8_t *)temp_uart_buffer, sizeof((uint8_t *)temp_uart_buffer));
	  }
	  else
	  {
		  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  }*/

	  if(GetMillisDifference(&test_led_timer) > 500)
	  {
		  test_led_timer = GetMillis();
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }


	  if(radio_irq_flag == 1)
	  {
		  radio_irq_flag = 0;

		  uint8_t reg_value = NRF24_GetAddress(&nrf_radio, 7);

		  uint8_t tx_success = reg_value & (1 << 5);
		  uint8_t rx_success = reg_value & (1 << 6);
		  uint8_t tx_fail = reg_value & (1 << 4);

		  if(tx_success)
		  {
			  waiting_for_ack = 0;

			  if(reliable_packet_to_gcs_counter > 0)
			  {
				  //Get rid of first index of array and shift
				  for(int i = 0; i < reliable_packet_to_gcs_counter - 1; i++)
				  {
					  reliable_packets_to_gcs[i].width = reliable_packets_to_gcs[i+1].width;
					  reliable_packets_to_gcs[i].reliable = 1;

					  for(int j = 0; j < 35; j++)
					  {
						  reliable_packets_to_gcs[i].payload[j] = reliable_packets_to_gcs[i+1].payload[j];
					  }
				  }

				  reliable_packet_to_gcs_counter--;
			  }

			  if(rx_success)
			  {
				  NRF24_PacketRead(&nrf_radio);
				  NRF24_FlushRX(&nrf_radio);
				  NRF24_WriteBit(&nrf_radio, 7, 6, 1);
			  }

			  NRF24_WriteBit(&nrf_radio, 7, 5, 1);
		  }

		  if(tx_fail)
		  {
			  waiting_for_ack = 0;
			  NRF24_WriteBit(&nrf_radio, 7, 4, 1);
		  }
	  }

	  if(GetMicrosDifference(&time_since_last_radio_send) > 500 && (new_packet_to_send_available || reliable_packet_to_gcs_counter > 0) && waiting_for_ack == 0)
	  {
		  time_since_last_radio_send = GetMicros();

		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

		  if(reliable_packet_to_gcs_counter > 0)
		  {
			  NRF24_PacketSend(&nrf_radio, &reliable_packets_to_gcs[0]);
		  }
		  else
		  {
			  new_packet_to_send_available = 0;
			  NRF24_PacketSend(&nrf_radio, &unreliable_packet);
		  }
	  }

	  if(telem_i2c_send_done)
	  {
		  telem_i2c_send_done = 0;

		  if(packets_to_receive_counter > 0)
		  {
			  if(packets_to_receive[0].payload[0] == 0xFD)
				  waiting_for_gps_packet_sent = 0;

			  for(int i = 0; i < packets_to_receive_counter - 1; i++)
			  {
				  packets_to_receive[i].width = packets_to_receive[i+1].width;
				  packets_to_receive[i].reliable = 0;

				  for(int j = 0; j < 35; j++)
				  {
					  packets_to_receive[i].payload[j] = packets_to_receive[i+1].payload[j];
				  }
			  }

			  packets_to_receive_counter--;
		  }
	  }

	  if(i2c_receive_flag)
	  {
		  i2c_receive_flag = 0;
		  new_packet_to_send_available = 1;

		  current_i2c_packet.width = current_i2c_packet.payload[32];
		  current_i2c_packet.reliable = current_i2c_packet.payload[33];

		  if(current_i2c_packet.reliable && reliable_packet_to_gcs_counter < 31)
		  {
			  reliable_packets_to_gcs[reliable_packet_to_gcs_counter].width = current_i2c_packet.width;
			  reliable_packets_to_gcs[reliable_packet_to_gcs_counter].reliable = 1;
			  for(int i = 0; i < 35; i++)
			  {
				  reliable_packets_to_gcs[reliable_packet_to_gcs_counter].payload[i] = current_i2c_packet.payload[i];
			  }

			  reliable_packet_to_gcs_counter++;
		  }
		  else if(current_i2c_packet.reliable == 0)
		  {
			  unreliable_packet.width = current_i2c_packet.width;
			  unreliable_packet.reliable = 0;

			  for(int i = 0; i < 35; i++)
			  {
				  unreliable_packet.payload[i] = current_i2c_packet.payload[i];
			  }
		  }

		  if(packets_to_receive_counter > 0)
		  {
			  packet_to_send_to_master = &packets_to_receive[0];
		  }
		  else
		  {
			  packet_to_send_to_master = &empty_data_packet;
		  }
	  }

	  GPS_Read();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF_IRQ_Pin)
	{
		radio_irq_flag = 1;
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(AddrMatchCode == (uint8_t)(0x04 << 1))
	{
		if(TransferDirection == I2C_DIRECTION_TRANSMIT)
		{
			if(HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)current_i2c_packet.payload, 35, I2C_FIRST_FRAME) != HAL_OK)
			{
				//fail
			}
		}
		else
		{
			if(HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)(packet_to_send_to_master->payload), 34, I2C_LAST_FRAME) != HAL_OK)
			{
				//fail
			}
		}
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_receive_flag = 1;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	telem_i2c_send_done = 1;
	//HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)current_i2c_packet.payload, 35);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		//Look into overflow
		micros_timer_base += 65000;//65536;
		millis_timer_base += 65;//Overflow doesn't matter unless board is running for more than 49 days
	}
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	//HAL_I2C_DeInit(&hi2c1);
	//MX_I2C1_Init();
	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

	//HAL_I2C_STATE_BUSY;
	//HAL_I2C_GetError(&hi2c1);
	//sprintf((char *)temp_uart_buffer, "%lu%s", HAL_I2C_GetError(&hi2c1), "\r\n");

	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)temp_uart_buffer, sizeof((uint8_t *)temp_uart_buffer));
}

uint32_t GetMicros()
{
	//return micros_timer_base + __HAL_TIM_GET_COUNTER(&htim4);
	return __HAL_TIM_GET_COUNTER(&htim4);
	//return 0;
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
