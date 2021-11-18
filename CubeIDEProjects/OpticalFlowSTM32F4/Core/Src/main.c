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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "fonts.h"
#include "ov7670.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
DMA_HandleTypeDef hdma_tim1_ch3;

/* USER CODE BEGIN PV */

uint8_t test_print_buffer[100];

uint16_t test_circle_x = 64;
uint16_t test_circle_y = 64;

uint32_t avg_x = 0;
uint32_t avg_y = 0;
uint32_t avg_count = 0;

uint32_t millis_timer_base = 0;

uint32_t display_timer = 0;
uint32_t temp_timer = 0;
int32_t frame_counter = 0;

int32_t last_line = 0;
int32_t graph_x = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  ST7735_Init(0);
  fillScreen(BLACK);
  //testAll();
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  Camera_Init();
  HAL_Delay(500);
  //sprintf((char *)test_print_buffer, "%s", "testing\n");
  TIM3->DIER = TIM_DIER_CC3IE;
  TIM3->CR1 = TIM_CR1_CEN;

  //Check setRotation to see window order direction


  //CDC_Transmit_FS(test_print_buffer, strlen((char *)test_print_buffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //TIM3->DIER = TIM_DIER_CC3IE;
	  //TIM3->CR1 = TIM_CR1_CEN;

	  while(!frame_captured);
	  frame_counter++;

	  //fillScreen(BLACK);
	  //ST7735_SetRotation(0);
	  //sprintf((char *)temp_data_line, "%hd", ReadRegister(0xA2));
	  //ST7735_WriteString(0, 0, (char *)temp_data_line, Font_11x18, RED,BLACK);
	  //HAL_Delay(100);

	  /*ST7735_Select();

	  ST7735_SetAddressWindow(0, 0, 95, 127);
	  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);*/

	  temp_timer = GetMicros();

	  uint8_t *pixel_array;
	  int32_t *x_array;
	  int32_t *y_array;
	  int32_t *x_array_prev;
	  int32_t *y_array_prev;

	  if(which_frame)
	  {
		  pixel_array = temp_data_line;
		  x_array = x_histogram;
		  y_array = y_histogram;
		  x_array_prev = x_histogram_prev;
		  y_array_prev = y_histogram_prev;
	  }
	  else
	  {
		  pixel_array = temp_data_line2;
		  x_array = x_histogram_prev;
		  y_array = y_histogram_prev;
		  x_array_prev = x_histogram;
		  y_array_prev = y_histogram;
	  }

	  x_array[0] = 0;
	  y_array[0] = 0;
	  x_array[127] = 0;
	  y_array[95] = 0;

	  x_array_prev[0] = 0;
	  y_array_prev[0] = 0;
	  x_array_prev[127] = 0;
	  y_array_prev[95] = 0;

	  int32_t sobel_sum_x = 0;
	  int32_t sobel_sum_y = 0;

	  for(uint32_t i = 2; i < 254; i+=2)
	  {
		  x_array[i / 2] = 0;
		  //y_array[i] = 0;

		  for(uint32_t j = 0; j < 96; j++)
		  {
			  //uint16_t color_to_send = (uint16_t)((RED & (uint16_t)((uint16_t)temp_data_line[i+1 + (j * 256)] << 8))/* | (GREEN & (uint16_t)((uint16_t)temp_data_line[i + (j * 256)] << 5)) | (BLUE & (uint16_t)temp_data_line[i + (j * 256)])*/);
			  //uint16_t color_to_send = ((((uint16_t)temp_data_line[i + (j * 256)]) << 8)) | (((uint16_t)temp_data_line[i+1 + (j * 256)]));
			  //uint16_t color_to_send2 = ((color_to_send & RED) >> 11) | ((color_to_send & BLUE) << 11) | (color_to_send & GREEN);
			  //uint16_t color_to_send2 = color_to_send;//temp_data_line[i + (j * 256)] & BLUE;

			  //uint8_t intensity = pixel_array[i+1 + (j * 256)];

			  //uint8_t intensity = GetPixelIntensity(i, j);
			  //uint8_t last_intensity = ((temp_data_line[i+1 + (j * 256)]) & ~which_frame) + ((temp_data_line2[i+1 + (j * 256)]) & which_frame);

			  sobel_sum_x = 0;
			  //int16_t sobel_sum_y = 0;
			  //int16_t kernel[] = {-1, 0, 1};

			  //int32_t x_index = i + i + 1;
			  //int32_t y_index = (j * 256);

			  sobel_sum_x -= pixel_array[i - 1 + (j * 256)];
			  //sobel_sum_x += (0) * pixel_array[i + 1 + c + (j * 256)];
			  sobel_sum_x += pixel_array[i + 3 + (j * 256)];

			  /*for(int32_t c = -1; c < 2; c++)
			  {

				  sobel_sum_x += (c) * pixel_array[i + 1 + c + (j * 256)];// * (((temp_data_line[x_index+c + y_index]) & which_frame_byte) + ((temp_data_line2[x_index+c + y_index]) & ~which_frame_byte));
				  //sobel_sum_y += (c) * pixel_array[x_index + ((j + c) * 256)];// * (((temp_data_line[x_index + (y_index + c)]) & which_frame_byte) + ((temp_data_line2[x_index + (y_index+c)]) & ~which_frame_byte));
			  }*/

			  sobel_sum_x = abs(sobel_sum_x);
			  //sobel_sum_y = abs(sobel_sum_y);

			  //intensity = 0x00;

			  if(sobel_sum_x > 60)
			  {
				  x_array[i / 2] += sobel_sum_x;
				  //intensity = 0xFF;
			  }

			  //if(sobel_sum_y > 80)
			  //{
			//	  y_array[i] += sobel_sum_y;
			  //}

			  //intensity = pow();
			  //if(sqrt((sorbel_sum_x * sorbel_sum_x) + (sorbel_sum_y * sorbel_sum_y)) > 80)
			  //if((sorbel_sum_x | sorbel_sum_y) > 100)
			  //{
		//		  intensity = 0xFF;
			 // }
			  //else
				//  intensity = (sorbel_sum_x | sorbel_sum_y);

			  //uint32_t magnitude = (((i / 2) - test_circle_x) * ((i / 2) - test_circle_x)) + (j - test_circle_y)*(j - test_circle_y);
			  //if(magnitude > 34 && magnitude < 50)
				//  intensity = 0xFF;

			 /* if(((int32_t)intensity - (int32_t)last_intensity) * ((int32_t)intensity - (int32_t)last_intensity) > 8000)
			  {
				  intensity = 0xFF;
			  }
			  else
				  intensity = 0x00;


			  if((uint32_t)intensity * (uint32_t)intensity > 27000)
			  {
				  avg_x += i / 2;
				  avg_y += j;
				  avg_count++;
			  }*/

			  //uint8_t red_channel = intensity;
			  //uint8_t green_channel = intensity;
			  //uint8_t blue_channel = intensity;

			  /*if(i % 4 == 0)
			  {
				  red_channel += 1.402 * (float)((temp_data_line[i+2 + (j * 256)]) - 128);
				  green_channel -= 0.34414 * (float)((temp_data_line[i + (j * 256)]) - 128);
				  green_channel -= 0.71414 * (float)((temp_data_line[i+2 + (j * 256)]) - 128);
				  blue_channel += 1.772 * (float)((temp_data_line[i + (j * 256)]) - 128);
			  }
			  else
			  {
				  red_channel += 1.402 * (float)((temp_data_line[i + (j * 256)]) - 128);
				  green_channel -= 0.34414 * (float)((temp_data_line[i+2 + (j * 256)]) - 128);
				  green_channel -= 0.71414 * (float)((temp_data_line[i + (j * 256)]) - 128);
				  blue_channel += 1.772 * (float)((temp_data_line[i+2 + (j * 256)]) - 128);
			  }*/

			  //uint16_t color_to_send2 = ((red_channel & 0xF8) << 8) | ((green_channel & 0xFC) << 3) | ((blue_channel & 0xF8) >> 3);
			  //uint8_t data[] = { color_to_send2 >> 8, color_to_send2 & 0xFF };

			  //uint8_t data[] = { 0x00, ((temp_data_line[i + (j * 256)]) & BLUE) };
			  //ST7735_WriteData(data, sizeof(data));

			  //HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);

			  //ST7735_DrawPixel(i, (uint16_t)j, color_to_send);

			  //drawLine(i, 0, i, 20, color_to_send);
			  //HAL_Delay(5);
		  }
	  }

	  /*for(uint32_t j = 1; j < 95; j++)
	  {
		  y_array[j] = 0;

		  for(uint32_t i = 0; i < 128; i++)
		  {
			  sobel_sum_y = 0;

			  sobel_sum_y -= pixel_array[(i*2) + 1 + ((j - 1) * 256)];
			  sobel_sum_y += pixel_array[(i*2) + 1 + ((j + 1) * 256)];


			  //sobel_sum_y = abs(sobel_sum_y);

			  if(sobel_sum_y > 80 || sobel_sum_y < -80)
			  {
				  y_array[j] += sobel_sum_y;
			  }
		  }
	  }*/

	  int32_t average_displacement_x = 0;

	  int32_t temp_displacement_x[200];
	  int32_t temp_displacement_y[200];
	  uint32_t SAD_temp[21];
	  int32_t border[2];

	  border[0] = 19;
	  border[1] = 108;

	  //memset(temp_displacement_x, 0, 4*21);
	  //memset(temp_displacement_y, 0, 4*21);
	  //memset(SAD_temp, 0, 4*21);

	  for(int32_t x = border[0]; x < border[1]; x++)
	  {
		  temp_displacement_x[x] = 0;
		  //memset(SAD_temp, 0, 4*21);

		  for(int32_t c = -10; c <= 10; c++)
		  {
			  SAD_temp[c + 10] = 0;

			  for(int32_t r = -9; r <= 9; r++)
			  {
				  SAD_temp[c + 10] += abs(x_array[x+r] - x_array_prev[x+r+c]);
			  }
		  }

		  /*uint32_t i;
		  uint32_t min_ind = 0;
		  uint32_t min_err = SAD_temp[min_ind];
		  uint32_t min_err_tot = 0;
		  for (i = 1; i < 21; i++)
		  {
			  if (SAD_temp[i] <= min_err)
			  {
				  min_ind = i;
				  min_err = SAD_temp[i];
				  min_err_tot += min_err;
			  }
		  }*/

		  temp_displacement_x[x] = (int32_t)getMinimum(SAD_temp, 21) - 10;
		  average_displacement_x += temp_displacement_x[x];
	  }

	  average_displacement_x /= (border[1] - border[0]);

	  border[0] = 28;
	  border[1] = 68;

	  //memset(SAD_temp, 0, 4*21);

	  /*for(int32_t x = border[0]; x < border[1]; x++)
	  {
		  temp_displacement_y[x] = 0;
		  //memset(SAD_temp, 0, 4*21);

		  for(int32_t c = -10; c <= 10; c++)
		  {
			  SAD_temp[c + 10] = 0;

			  for(int32_t r = -9; r <= 9; r++)
			  {
				  SAD_temp[c + 10] += abs(y_array[x+r] - y_array_prev[x+r+c]);
			  }
		  }

		  temp_displacement_y[x] = (int32_t)getMinimum(SAD_temp, 21) - 10;
	  }*/

	  uint32_t timer_difference = GetMicrosDifference(&temp_timer);

	  if(GetMillisDifference(&display_timer) >= 100)
	  {
		  display_timer = GetMillis();

		  /*ST7735_Select();
		  ST7735_SetAddressWindow(0, 0, 10, 127);
		  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);

		  for(int x = 0; x < 10; x++)
		  {
			  for(int y = 0; y < 10; y++)
			  {
				  uint8_t data[] = { 0x00, 0x00 };

				  HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
			  }
		  }

		  if(frame_counter > 30)
		  {
			  ST7735_SetAddressWindow(0, 0, 10, 127);
			  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
			  for(int x = 0; x < 10; x++)
			  {
				  for(int y = 0; y < 10; y++)
				  {
					  uint8_t data[] = { 0xFF, 0xFF };

					  HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
				  }
			  }
		  }

		  ST7735_Unselect();*/

		  drawLine(graph_x, last_line + 64, graph_x + 1, (average_displacement_x * 3) + 64, 0xFFFF);
		  last_line = average_displacement_x * 3;

		  graph_x++;

		  if(graph_x == 127)
		  {
			  graph_x = 0;
			  fillScreen(BLACK);
		  }

		  //fillScreen(BLACK);
		  //ST7735_SetRotation(0);
		  //sprintf((char *)temp_data_line, "%ld", temp_displacement_x[40]);
		  //ST7735_WriteString(0, 0, (char *)temp_data_line, Font_11x18, RED,BLACK);

		  frame_counter = 0;
	  }

	  //ST7735_Unselect();

	  frame_captured = 0;

	  /*uint16_t temp_color = 0x4F7F;
	  uint8_t data2[] = { temp_color >> 8, temp_color & 0xFF };

	  test_circle_x = avg_x / avg_count;
	  test_circle_y = avg_y / avg_count;

	  ST7735_SetAddressWindow(test_circle_x - 4, test_circle_y - 4, test_circle_x + 4, test_circle_y + 4);
	  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
	  for(int x = 0; x < 8; x++)
	  {
		  for(int y = 0; y < 8; y++)
		  {
			  HAL_SPI_Transmit(&ST7735_SPI_PORT, data2, sizeof(data2), HAL_MAX_DELAY);
		  }
	  }

	  avg_x = 0;
	  avg_y = 0;
	  avg_count = 0;*/

	  //ST7735_Unselect();

	  //CDC_Transmit_FS(temp_data_line, 128*128*2);
	  //HAL_Delay(50);


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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
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
  hi2c2.Init.OwnAddress1 = 0;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim9.Init.Prescaler = 167;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 64999;
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
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_BS_Pin|TFT_CS_Pin|TFT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAM_RST_Pin */
  GPIO_InitStruct.Pin = CAM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_BS_Pin TFT_CS_Pin TFT_RST_Pin */
  GPIO_InitStruct.Pin = TFT_BS_Pin|TFT_CS_Pin|TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
