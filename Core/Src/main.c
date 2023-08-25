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
#include "dac.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SW_RANDOMIZED_GLITCH_POS 0
#define SW_SWEEP_GLITCH_POS 1
#define SW_SWEEP_START_AT_STARTBIT 1


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define SET_REF_PIN (PATTERN_REF_Pin)
#define CLR_REF_PIN (PATTERN_REF_Pin << 16)
#define SET_DATA_PIN (PATTERN_DATA_Pin)
#define CLR_DATA_PIN ((PATTERN_DATA_Pin) << 16)

#define SET_RX_PIN (PATTERN_RX_Pin)
#define CLR_RX_PIN (PATTERN_RX_Pin << 16)

#define DAC_MAX ((1 << 12) - 1)

#define DATA_PIN_MASK (0xffffffff ^ SET_DATA_PIN ^ CLR_DATA_PIN)

#define BAUD_RATE 19200
#define BIT_WIDTH ((4000000 + BAUD_RATE /2) / BAUD_RATE)

#define GLITCH_WIDTH 1

#define  PATTERN_BUFFER_SIZE ((( BIT_WIDTH * 11) * 2) + BIT_WIDTH * 5)
#if SW_SWEEP_START_AT_STARTBIT
#define STATISTIC_COUNT PATTERN_BUFFER_SIZE
#define TIMER_FREQUENCY 4000000
#else
#define STATISTIC_COUNT (BIT_WIDTH + BIT_WIDTH / 5)
#endif

static uint32_t patternBuffer[PATTERN_BUFFER_SIZE];
static uint16_t dac_buffer[PATTERN_BUFFER_SIZE];
static uint32_t set_rx_pin[1];
static uint8_t rxBuffer[3];
extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_usart1_rx;
enum Parity
{
   none, odd, even
};
enum Parity parity = even;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char transmitData[] = "\x03";
int validCounter = 0;
int errorCounter = 0;
uint32_t glitchPosStatistic[STATISTIC_COUNT];
uint32_t glitchStart = 0;
uint32_t glitchEnd = 0;
int sweep = 0;
size_t byteWidth;
uint32_t bitPositions[32];
//---------------------------------------------------------------------------
void CalculateUart(uint32_t maskch1, uint32_t maskch2)
{
   size_t bitCount = (parity != none) ? 11 : 10;
   size_t y_val = 0;
   byteWidth = bitCount * TIMER_FREQUENCY / (BAUD_RATE);
   int y = 0;
   uint32_t ch1;
   uint32_t ch2;
   for (int n = 0; n < PATTERN_BUFFER_SIZE; n++)
   {
      y_val += bitCount;
      if (y_val >= byteWidth)
      {
         y ++;
         y_val -= byteWidth;
         bitPositions[y] = n;
      }
      if (y < 32)
      {
         ch1 = ((maskch1) & (1 << y)) != 0 ? SET_DATA_PIN : CLR_DATA_PIN;
         ch2 = ((maskch2) & (1 << y)) != 0 ? SET_REF_PIN : CLR_REF_PIN;
      }
      patternBuffer[n] = ch1 | ch2;
   }
}
//---------------------------------------------------------------------------
void CalculatePattern(uint16_t glitchPos, uint16_t glitchWidth, bool glitchLevel)
{
   const char *read = transmitData;
   uint8_t data = *read;
   uint8_t parityBit = data;
   parityBit = (parityBit >> 4) ^ parityBit;
   parityBit = (parityBit >> 2) ^ parityBit;
   parityBit = ((parityBit >> 1) ^ parityBit) & 1;
   uint32_t ref = (1 << 12) - 1;
   uint32_t mask = 0xffffffff;
   switch (parity)
   {

      case none:
         ref = (1 << 11) - 1;
         break;
      case odd:
         ref = (1 << 12) - 1;
         mask = (mask << 1) + (parityBit ^ 1);
         break;
      case even:
         ref = (1 << 11) - 1;
         mask = (mask << 1) + parityBit;
         break;
   }
   mask = (mask << 8) + data;
   mask = (mask << 1) + 0;
   mask = (mask << 1) + 1;
   CalculateUart(mask, ref << 1);
   patternBuffer[0] |= CLR_RX_PIN;
   patternBuffer[10] |= SET_RX_PIN;

   if (glitchWidth > 0)
   {
      uint32_t index = glitchStart + glitchPos;
      for (int n = 0; n < glitchWidth; n++)
      {
         patternBuffer[index + n] &= DATA_PIN_MASK;
         patternBuffer[index + n] |= (glitchLevel ? SET_DATA_PIN : CLR_DATA_PIN);
      }
   }
   uint32_t max = 1;
   for (int n = 0; n < STATISTIC_COUNT; n++)
   {
      if (glitchPosStatistic[n] > max)
      {
         max = glitchPosStatistic[n];
      }
   }
   for (int n = 0; n < STATISTIC_COUNT; n++)
   {
      #if SW_SWEEP_START_AT_STARTBIT
      uint32_t dacValue = (glitchPosStatistic[n] * DAC_MAX) / max;
      if (dacValue > 0)
      {
         dac_buffer[n] = dacValue;
      }
      patternBuffer[n] |= (glitchPosStatistic[n] > 0) ? SET_RX_PIN : CLR_RX_PIN;
      #else
      dac_buffer[glitchStart + n] = (glitchPosStatistic[n] * DAC_MAX) / max;
      #endif
   }

   // DMA, circular memory-to-peripheral mode, full word (32 bit) transfer
//   HAL_TIM_Base_DeInit(&htim2);
//   HAL_TIM_OC_DeInit(&htim2);
   HAL_TIM_Base_Stop(&htim2);
   HAL_TIM_Base_DeInit(&htim2);
   HAL_TIM_OC_DeInit(&htim2);
   MX_DMA_Init();
   MX_TIM2_Init();
   while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
   {
      huart1.Instance->RDR;
   }
   huart1.Instance->RQR = USART_RQR_RXFRQ;
   HAL_StatusTypeDef error0 = HAL_DMA_Start(&hdma_tim2_up, (uint32_t) patternBuffer, (uint32_t) &(GPIOC->BSRR), PATTERN_BUFFER_SIZE);
   HAL_StatusTypeDef error1 = HAL_DMA_Start(&hdma_tim2_ch1, (uint32_t) dac_buffer, (uint32_t) &(hdac1.Instance->DHR12R1), PATTERN_BUFFER_SIZE);
   HAL_StatusTypeDef error2 = HAL_DMA_Start(&hdma_usart1_rx, (uint32_t) set_rx_pin, (uint32_t) &(GPIOC->BSRR), 1);

   if (error0 == HAL_OK && error1 == HAL_OK && error2 == HAL_OK)
   {
      HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_Base_Start(&htim2);
   }
   else
   {
      printf("error: %u %u %u\n", error0, error1, error2);
   }
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
   MX_DMA_Init();
   MX_USART2_UART_Init();
   MX_TIM2_Init();
   MX_USART1_UART_Init();
   MX_RNG_Init();
   MX_DAC1_Init();
   /* USER CODE BEGIN 2 */
//   __disable_irq();
   HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
   uint16_t glitchPos = 0;
   uint16_t glitchWidth = GLITCH_WIDTH;
   glitchStart = 11 * BIT_WIDTH - 10;
   glitchEnd = 13 * BIT_WIDTH + 10;
   HAL_DMA_DeInit(&hdma_usart1_rx);
   hdma_usart1_rx.Init.Direction = DMA_MEMORY_TO_PERIPH;
   HAL_DMA_Init(&hdma_usart1_rx);
   ATOMIC_SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

   CalculatePattern(glitchPos, glitchWidth, false);
   memset(dac_buffer, 0, sizeof(dac_buffer));
   set_rx_pin[0] = 0;

   dac_buffer[0] = 4095;
   dac_buffer[1] = 4095;
   dac_buffer[2] = 4095;
   dac_buffer[3] = 4095;
   dac_buffer[4] = 4095;
   dac_buffer[5] = 4095;
   dac_buffer[6] = 4095;
   dac_buffer[7] = 4095;
   dac_buffer[8] = 4095;
   dac_buffer[9] = 4095;
   dac_buffer[10] = 4095;
   dac_buffer[11] = 4095;

   TIM2->DIER |= TIM_DIER_UDE;   // set UDE bit (update dma request enable)
   TIM2->DIER |= TIM_DIER_CC1DE;   // set UDE bit (update dma request enable)


   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
   while (1)
   {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      uint16_t rxEd = 0;
      HAL_StatusTypeDef uartExReceiveToIdle = HAL_UARTEx_ReceiveToIdle(&huart1, rxBuffer, sizeof(rxBuffer), &rxEd, 50);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      if (rxEd || uartExReceiveToIdle != HAL_OK)
      {
         validCounter++;
         rxBuffer[rxEd] = 0;
         int invalid = rxEd != strlen(transmitData);
         if (!invalid)
         {
            invalid = strcmp(rxBuffer, transmitData) != 0;
         }
         if (invalid)
         {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            errorCounter++;
            printf("[%5d/%5d] rx:%d", errorCounter, validCounter, rxEd);
            for (size_t index = 0; index < rxEd; index++)
            {
               printf(" %02x", rxBuffer[index]);
            }
            printf(", pos, width: %u, %u\n", glitchPos, glitchWidth);
            #if SW_BUTTON_WAIT
            while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
            {
            }
            #endif
            if (glitchPos < STATISTIC_COUNT)
            {
               glitchPosStatistic[glitchStart + glitchPos] += 1;
            }
         }
      }
      int rxTerminalCount = 0;
      while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
      {
         rxTerminalCount++;
         huart2.Instance->RDR;
      }
      if (rxTerminalCount)
      {
         uint32_t bitindex = 0;
         while (bitPositions[bitindex] < glitchStart)
         {
            bitindex++;
         }
         for (uint32_t n = glitchStart; n < glitchEnd; n++)
         {
            if (bitPositions[bitindex] == n)
            {
               printf("[%u]->%u\n", (unsigned int) n, (unsigned int) bitindex);
               bitindex++;
            }
            printf("[%3u]: %4u\n", (unsigned int) n, (unsigned int) glitchPosStatistic[n]);
         }
      }
      HAL_StatusTypeDef error0 = HAL_DMA_PollForTransfer(&hdma_tim2_ch1, HAL_DMA_FULL_TRANSFER, 100);
      HAL_StatusTypeDef error1 = HAL_DMA_PollForTransfer(&hdma_tim2_up, HAL_DMA_FULL_TRANSFER, 100);
      HAL_StatusTypeDef error2 = HAL_DMA_Abort(&hdma_usart1_rx);

      if (error0 == HAL_OK && error1 == HAL_OK && error2 == HAL_OK)
      {
         HAL_TIM_Base_Stop(&htim2);
         #if SW_RANDOMIZED_GLITCH_POS
         uint32_t nr;
         HAL_RNG_GenerateRandomNumber(&hrng, &nr);
         glitchPos = nr & 0x1ff;
         #endif
         #if SW_SWEEP_GLITCH_POS
         glitchPos += 1;
         if (glitchStart + glitchPos >= glitchEnd)
         {
            glitchPos = 0;
            sweep++;
            printf("sweep: %d\n", sweep);
         }
         #endif
         CalculatePattern(glitchPos, glitchWidth, false);
      }
      else
      {
         printf("error: %u %u %u\n", error0, error1, error2);
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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
int _write(int file, char *ptr, int len)
{
   (void) file;
   HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
   return len;
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
