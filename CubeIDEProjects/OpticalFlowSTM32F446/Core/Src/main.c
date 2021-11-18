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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "fonts.h"
#include "ov7670.h"
#include "i2c_sw.h"
#include "usbd_cdc_if.h"
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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

volatile uint8_t frame_captured_flag = 0;

uint32_t captured_frames = 0;
int32_t framerate = 0;
uint32_t framerate_timer;
volatile uint32_t millis_timer_base;

uint8_t frame_index = 0;
uint8_t *current_frame;
uint8_t *last_frame;

int32_t x_histogram1[200];
int32_t x_histogram2[200];
int32_t y_histogram1[200];
int32_t y_histogram2[200];

int32_t *x_histogram;
int32_t *x_histogram_prev;
int32_t *y_histogram;
int32_t *y_histogram_prev;

int32_t sobel_sum_x;
int32_t sobel_sum_y;
int32_t total_sum;

uint8_t command_packet[32];	//From FC to me
uint8_t response_packet[32];//From me to FC

uint8_t i2c_receive_flag = 0;
uint8_t telem_i2c_send_done = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
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

  HAL_Delay(2000);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  current_frame = frame1;
  last_frame = frame2;

  for(int i = 0; i < 32; i++)
  {
	  response_packet[i] = 0;
  }

  HAL_TIM_Base_Start(&htim9);
  HAL_TIM_Base_Start_IT(&htim4);
  //SW_I2C_initial();
  //i2c_port_initial(SW_I2C1);
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);

  //ST7735_Init(0);
  //fillScreen(BLACK);

  //fillScreen(GREEN);
  //fillScreen(BLACK);
  //testAll();
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);
  Camera_Init();
  HAL_Delay(500);
  HAL_DCMI_ConfigCrop(&hdcmi, 8, 8, 256, 128);
  HAL_DCMI_EnableCrop(&hdcmi);
  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)0x60000000, (256*128));
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)current_frame, (128*128));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK)
	  {
		  //fail
	  }

	  if(frame_captured_flag)
	  {
		  frame_captured_flag = 0;
		  captured_frames++;
		  HAL_DCMI_Suspend(&hdcmi);
		  HAL_DCMI_Stop(&hdcmi);

		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)last_frame, (128*128));

		  sobel_sum_x = 0;
		  sobel_sum_y = 0;

		  x_histogram[0] = 0;
		  y_histogram[0] = 0;
		  x_histogram[127] = 0;
		  y_histogram[95] = 0;

		  x_histogram_prev[0] = 0;
		  y_histogram_prev[0] = 0;
		  x_histogram_prev[127] = 0;
		  y_histogram_prev[95] = 0;

		  for(uint32_t y = 1; y < 95; y++)
		  {
			  y_histogram[y] = 0;
		  }

		  for(uint32_t x = 1; x < 127; x++)
		  {
			  x_histogram[x] = 0;

			  for(uint32_t y = 1; y < 95; y++)
			  {
				  sobel_sum_x = 0;
				  sobel_sum_y = 0;

				  sobel_sum_x -= current_frame[x - 1 + (y * 128)];
				  sobel_sum_x += current_frame[x + 1 + (y * 128)];

				  sobel_sum_x = abs(sobel_sum_x);

				  sobel_sum_y -= current_frame[x + ((y - 1) * 128)];
				  sobel_sum_y += current_frame[x + ((y + 1) * 128)];

				  sobel_sum_y = abs(sobel_sum_y);

				  total_sum = (sobel_sum_x * sobel_sum_x) + (sobel_sum_y * sobel_sum_y);

				  //frame3[x + (y * 128)] = 0;

				  if(total_sum > 1600)
				  {
					  x_histogram[x] += total_sum;
					  y_histogram[y] += total_sum;
					  //frame3[x + (y * 128)] = total_sum;
				  }

				  if(sobel_sum_x > 40)
				  {
					  //x_histogram[x] += sobel_sum_x;
					  //frame3[x + (y * 128)] = sobel_sum_x;
				  }

				  if(sobel_sum_y > 40)
				  {
					  //y_histogram[y] += sobel_sum_y;

					  //if(sobel_sum_y > frame3[x + (y * 128)])
						//  frame3[x + (y * 128)] = sobel_sum_y;
				  }
			  }
		  }

		  /*for(uint32_t y = 1; y < 95; y++)
		  {
			  y_histogram[y] = 0;

			  for(uint32_t x = 0; x < 128; x++)
			  {
				  sobel_sum_y = 0;

				  sobel_sum_y -= current_frame[x - 1 + ((y - 1) * 128)];
				  sobel_sum_y += current_frame[x + 1 + ((y + 1) * 128)];

				  sobel_sum_y = abs(sobel_sum_y);

				  //frame3[x + (y * 128)] = 0;

				  if(sobel_sum_y > 60)
				  {
					  y_histogram[y] += sobel_sum_y;
					  //frame3[x + (y * 128)] = 0xFF;
					  //intensity = 0xFF;
				  }
			  }
		  }*/

		  int32_t average_displacement_x = 0;
		  int32_t average_displacement_y = 0;

		  int32_t temp_displacement_x[200];
		  int32_t temp_displacement_y[200];
		  uint32_t SAD_temp[21];
		  int32_t border[2];

		  int32_t largest_displacement_x = 0;

		  border[0] = 19;
		  border[1] = 108;

		  for(int32_t x = border[0]; x < border[1]; x++)
		  {
			  temp_displacement_x[x] = 0;

			  for(int32_t c = -10; c <= 10; c++)
			  {
				  SAD_temp[c + 10] = 0;

				  for(int32_t r = -9; r <= 9; r++)
				  {
					  SAD_temp[c + 10] += abs(x_histogram[x+r] - x_histogram_prev[x+r+c]);

				  }
			  }

			  temp_displacement_x[x] = (int32_t)getMinimum(SAD_temp, 21) - 10;
			  average_displacement_x += temp_displacement_x[x];

			  //if(temp_displacement_x[x] > largest_displacement_x)
			//	  largest_displacement_x = temp_displacement_x[x];
		  }

		  average_displacement_x /= (border[1] - border[0]);

		  border[0] = 28;
		  border[1] = 68;

		  for(int32_t y = border[0]; y < border[1]; y++)
		  {
			  temp_displacement_y[y] = 0;

			  for(int32_t c = -10; c <= 10; c++)
			  {
				  SAD_temp[c + 10] = 0;

				  for(int32_t r = -9; r <= 9; r++)
				  {
					  SAD_temp[c + 10] += abs(y_histogram[y+r] - y_histogram_prev[y+r+c]);

				  }
			  }

			  temp_displacement_y[y] = (int32_t)getMinimum(SAD_temp, 21) - 10;
			  average_displacement_y += temp_displacement_y[y];

			  //if(temp_displacement_x[x] > largest_displacement_x)
				  //	  largest_displacement_x = temp_displacement_x[x];
		  }

		  average_displacement_y /= (border[1] - border[0]);

		  //fillScreen(BLACK);

		  /*ST7735_Select();

		  ST7735_SetAddressWindow(0, 0, 127, 127);
		  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
		  uint8_t *memread = (uint8_t *)(uint32_t)0x60000000;

		  for(int y = 0; y < 128; y++)
		  {
			  for(int x = 0; x < 128; x++)
			  {
				  //uint8_t intensity = memread[x + (y * 128)];
				  uint8_t intensity = test_frame[x + (y * 128)];

				  uint8_t red_channel = intensity;
				  uint8_t green_channel = intensity;
				  uint8_t blue_channel = intensity;

				  uint16_t color_to_send2 = ((red_channel & 0xF8) << 8) | ((green_channel & 0xFC) << 3) | ((blue_channel & 0xF8) >> 3);
				  uint8_t data[] = { color_to_send2 >> 8, color_to_send2 & 0xFF };

				  HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
			  }
		  }*/

		  response_packet[0] = 1;	//Use to detect if i2c happened during this loop

		  for(int i = 0; i < 4; i++)
		  {
			  response_packet[i+1] = *(((uint8_t *)&average_displacement_x) + i);
			  response_packet[i+5] = *(((uint8_t *)&average_displacement_y) + i);
			  response_packet[i+9] = *(((uint8_t *)&framerate) + i);
		  }

		  response_packet[0] = 0;



		  //CDC_Transmit_FS(current_frame, 128*128);
		  //HAL_Delay(175);

		  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)last_frame, (128*128));

		  if(frame_index == 0)
		  {
			  last_frame = frame2;
			  current_frame = frame1;

			  x_histogram_prev = x_histogram2;
			  y_histogram_prev = y_histogram2;

			  x_histogram = x_histogram1;
			  y_histogram = y_histogram1;
		  }
		  else if(frame_index == 1)
		  {
			  last_frame = frame1;
			  current_frame = frame2;

			  x_histogram_prev = x_histogram1;
			  y_histogram_prev = y_histogram1;

			  x_histogram = x_histogram2;
			  y_histogram = y_histogram2;
		  }

		  frame_index++;

		  if(frame_index > 1)
			  frame_index = 0;

		  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)0x60000000, (256*128));

		  //ST7735_Unselect();
	  }

	  if(GetMillisDifference(&framerate_timer) >= 1000)
	  {
		  framerate_timer = GetMillis();
		  framerate = captured_frames;
		  //fillScreen(BLACK);
		  //sprintf((char *)temp_data_line, "%lu", captured_frames);
		  //ST7735_WriteString(0, 0, (char *)temp_data_line, Font_11x18, RED,BLACK);

		  captured_frames = 0;
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

	hdma_dcmi.Instance = DMA2_Stream1;
	hdcmi.DMA_Handle = &hdma_dcmi;

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_OTHER;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_EVEN;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 176;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 64999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 179;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DCMI_RST_Pin */
  GPIO_InitStruct.Pin = DCMI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(AddrMatchCode == (uint8_t)(0x58 << 1))
	{
		if(TransferDirection == I2C_DIRECTION_TRANSMIT)
		{
			if(HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, (uint8_t *)command_packet, 32, I2C_FIRST_FRAME) != HAL_OK)
			{
				//fail
			}
		}
		else
		{
			if(HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, (uint8_t *)response_packet, 32, I2C_LAST_FRAME) != HAL_OK)
			{
				//fail
			}

			//HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
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
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c2)
	{
		HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

		if(I2C1->SR1 & I2C_SR1_BERR)//Bus error
		{
			I2C1->SR1 &= ~(I2C_SR1_BERR);
		}
		else if(I2C1->SR1 & I2C_SR1_AF)//Ack fail
		{
			I2C1->SR1 &= ~(I2C_SR1_AF);
		}
	}
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	frame_captured_flag = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		//micros_timer_base += 65000;//65536;
		millis_timer_base += 65;//Overflow doesn't matter unless board is running for more than 49 days
	}
}

uint32_t GetMicros()
{
	//return micros_timer_base + __HAL_TIM_GET_COUNTER(&htim4);
	return __HAL_TIM_GET_COUNTER(&htim4);
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
