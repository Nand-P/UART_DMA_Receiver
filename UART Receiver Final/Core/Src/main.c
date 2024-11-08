/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_PACKET_SIZE 255
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


struct connection {
    uint8_t packet_size; // Not inclusive of the 1 byte crc
    uint8_t num_of_packets; // Total number of packets to be sent
    uint8_t total_size; // Size of data not including padding (actual size)
    uint8_t crc;
    uint8_t handshake; // true if identical
};

struct data {
	double x_coord;
	double y_coord;
	double z_coord;

	uint8_t control;
};


uint8_t rx_buffer[26];
uint8_t packet[MAX_PACKET_SIZE];

struct connection* p_recv_init; // Pointer to recv_init struct, initialized in main

uint8_t calculate_crc(uint8_t* buffer, const size_t data_length) {
    /*
     * This function calculates an 8-bit CRC checksum given a buffer
     * of uint8_t data using the HAL_CRC_Calculate API.
     *
     * MX_CRC_Init() must be created from the .ioc setup and configured
     * to output an 8-bit CRC.
     *
     * If the buffer is terminated with an 8-bit checksum, it must be
     * excluded from this function by passing sizeof(buffer) - 1 for the
     * data_length parameter. This will prevent it from being included
     * as part of the checksum calculation.
     *
     * INPUTS
     * 		buffer : const uint8_t *
     * 		Buffer to calculate CRC from
     *
     * 		data_length : const size_t
     * 		Size of data in bytes. Equivalent to the number of elements
     * 		corresponding to data in the buffer.
     *
     * OUTPUTS
     * 		crc : uint8_t
     * 		8-bit CRC checksum
	*/

	uint8_t crc = 0;
	uint8_t *byte = buffer;

	// Sum all bytes of data
	for (int i=0; i < data_length; i++) {
		crc += *byte;
		byte++;
	}

	// Take 2s complement of the sum
	crc = (~crc) + 1;

    return crc;
}

void receive_gps_data(struct data *data, uint8_t* rx_buffer){
    double temp [3] = {0};
    for (int i =0; i <3; i++){
    	memcpy(&temp[i], rx_buffer, sizeof(double));  // Copy 8 bytes to temp[i]
    	rx_buffer += sizeof(double);  // Move the pointer by 8 bytes (size of double)
    }
	  data->x_coord = temp[0];
	  data->y_coord = temp[1];
	  data->z_coord = temp[2];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  struct connection recv_init;
  struct data test;

  p_recv_init = &recv_init;

//  test.x_coord = 2.22;
//  test.y_coord = 1.23;
//  test.z_coord = 4.45;
//  test.control = 64;
//
//  uint8_t* ptr = &test;
//  rx_buffer[0] = 3;
//  for (int i =1; i <sizeof(rx_buffer); i++){
//	  rx_buffer[i]= *ptr;
//	  ptr++;
//  }
//
//  uint8_t crc = calculate_crc(rx_buffer, sizeof(rx_buffer)-1);
  p_recv_init->handshake = 0;
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
  huart1.AdvancedInit.AutoBaudRateEnable = UART_ADVFEATURE_AUTOBAUDRATE_ENABLE;
  huart1.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : VCP_TX_Pin VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin|VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (p_recv_init->handshake == 0) { // If handshake failed, await new handshake
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
	} else if (p_recv_init->handshake == 1) { // If the handshake succeeded, receive
		HAL_UART_Receive_DMA(&huart1, &packet, p_recv_init->packet_size);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (p_recv_init->handshake == 0) { // Step 1: Handshake

	  /*
	   * Flag Value: Meaning -> Result
	   *
	   * 0: Matched CRC (Valid Data) -> Continue Transaction
	   * 1: Mismatched CRC (Corrupted Data) -> Send retry request to transmitter
	   *
	   * */

	  uint8_t flag = 0;
	  uint8_t recv_crc = calculate_crc((uint8_t*)p_recv_init, 4);

	  if (recv_crc != p_recv_init->crc) {
		  flag = 1;
		  HAL_UART_Transmit_DMA(&huart1, &flag, 1);
	  } else {
		  p_recv_init->handshake = 1;
		  HAL_UART_Transmit_DMA(&huart1, &flag, 1);
	  }

  } else if (recv_init.handshake == 1) { // Step 2: Receive Data

  }


  //HAL_UART_Receive_DMA(&huart1, rx_buffer + 1, sizeof(rx_buffer) - 1);
  uint8_t transmit_crc = rx_buffer[sizeof(rx_buffer)-1];

  if(calculate_crc(rx_buffer,sizeof(rx_buffer)-1) == transmit_crc ){
	  struct data out;
      uint8_t * receive_data = &rx_buffer;
      uint8_t data_avail = rx_buffer[0];
      receive_data++;
      if (data_avail == 1){
      } else if (data_avail == 2){
      } else if(data_avail == 3){
    	// Save buffer data to struct
        receive_gps_data(&out, receive_data);
        out.control = *receive_data;
      }

   }

  memset(rx_buffer, 0, sizeof(rx_buffer)); //!!! Clear buffer (or what should we do if CRCs don't match?)
  HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));

  return;
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
