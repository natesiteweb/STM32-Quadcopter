/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ov7670.h"
#include "string.h"
#include "stdio.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "stm32f4xx_hal_dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_tim1_ch3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

	if (__HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim9, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim9, TIM_IT_UPDATE);
#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
			htim->PeriodElapsedCallback(&htim9);
#else
			HAL_TIM_PeriodElapsedCallback(&htim9);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
		}
	}

	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_BREAK) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_BREAK);
#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
			htim->BreakCallback(&htim1);
#else
			HAL_TIMEx_BreakCallback(&htim1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
		}
	}

	return;
  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

	if((TIM1->SR & TIM_SR_CC1IF) != 0x00)
	{
		//fillScreen(GREEN);
		TIM1->DIER &= ~TIM_DIER_CC3DE;
		TIM1->DIER &= ~TIM_DIER_CC1IE;
		TIM1->CCR3;
		TIM1->SR &= ~TIM_SR_CC3IF;
		//TIM2->DIER &= ~TIM_DIER_CC2IE;

		//HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_3);
		//HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)temp_data_line, (uint16_t)128);

		//HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_3);
		//HAL_DMA_Abort_IT((DMA_HandleTypeDef *)htim1.hdma);
		//DMA2_Stream6->CR = 0;
		//DMA2_Stream6->PAR = (uint32_t)&(GPIOA->IDR);
		//DMA2_Stream6->NDTR = (uint16_t)128;
		//DMA2_Stream6->M0AR = (uint32_t)(temp_data_line + (line_count * 128));
		//DMA2_Stream6->CR = DMA_SxCR_EN | DMA_SxCR_MINC | DMA_SxCR_PL;// | DMA_SxCR_CIRC;
		//HAL_DMA_Start_IT((DMA_HandleTypeDef *)htim1.hdma, (uint32_t)&(GPIOA->IDR), (uint32_t)(temp_data_line + (line_count * 128)), (uint32_t)128);

		DMA2_Stream6->CR = 0;
		//DMA2_Stream6->PAR = (uint32_t)&(GPIOA->IDR);
		DMA2_Stream6->NDTR = (uint16_t)256;
		DMA2_Stream6->M0AR = (uint32_t)(((uint32_t)temp_data_line & which_frame) + ((uint32_t)temp_data_line2 & ~which_frame) + (line_count * 256));
		//DMA2_Stream6->FCR = 0;
		DMA2_Stream6->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MINC;
		DMA2_Stream6->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
		DMA2_Stream6->CR |= DMA_IT_TC;
		DMA2_Stream6->CR |= DMA_SxCR_EN;

		//DMA2->

		//__HAL_TIM_ENABLE_DMA(asd, asd);
		//DMA2_Stream6->PAR = (uint32_t)&(GPIOA->IDR);
		//DMA2_Stream6->CR = DMA_SxCR_EN | DMA_SxCR_MINC | DMA_SxCR_PL;// | DMA_SxCR_CIRC;
		//TIM1->DIER |= TIM_DMA_CC3;
		//GPIOA->BSRR;
		//__HAL_TIM_ENABLE_DMA(asd, asdasd);
		line_count++;

		if(line_count > 95)
		{
			//fillScreen(GREEN);
			frame_captured = 1;

			//TIM3->DIER = 0;
			TIM1->DIER = 0;
			//TIM3->CR1 &= ~TIM_CR1_CEN;
			TIM1->CR1 &= ~TIM_CR1_CEN;
		}
		else
		{
			//HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)(temp_data_line + (line_count * 128)), (uint32_t)128);
			TIM1->DIER |= TIM_DIER_CC3DE;
			TIM1->DIER |= TIM_DIER_CC1IE;
		}
	}

	TIM1->CCR1;
	TIM1->SR &= ~TIM_SR_CC1IF;

	return;

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	if((TIM3->SR & TIM_SR_CC3IF) != 0x00)
	{
		//sprintf((char *)test_print_buffer, "%s", "worked\n");
		//CDC_Transmit_FS(test_print_buffer, strlen((char *)test_print_buffer));
		//fillScreen(GREEN);
		line_count = 0;
		//fillScreen(BLACK);

		if(~TIM1->CR1 & TIM_CR1_CEN)
		{
			//HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_3);
			if(which_frame == 0xFFFFFFFF)
				which_frame = 0x00000000;
			else
				which_frame = 0xFFFFFFFF;
			TIM1->DIER = TIM_DIER_CC1IE;
			TIM1->CR1 = TIM_CR1_CEN;
			//HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)temp_data_line, (uint16_t)128);
		}
	}

	TIM3->CCR3;
	TIM3->SR &= ~TIM_SR_CC3IF;
	return;

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */
	/*if(DMA2->HISR & DMA_HISR_TCIF6)
	{
		//IFCR = 0x3FU << hdma->StreamIndex;
		//fillScreen(RED);
	}*/

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch3);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
