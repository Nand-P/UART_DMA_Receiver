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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "uart_common.h"
#include "uart_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define MAX_PACKET_SIZE 255
//#define FULL_PACKET 4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    uint8_t corrupt_packets[MAX_PACKET_SIZE];
    uint8_t num_corrupt;
    uint8_t curr_pckt;
} internal_state;

uint8_t packets[MAX_PACKET_SIZE][PACKET_SIZE + 1];
uint8_t flag;
uint8_t received;
uint8_t padding;

// Global pointer to structs initialized in main
connection* p_recv_init;
internal_state* p_intl_state;

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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  received = 0;

  connection recv_init;
  internal_state intl_state;
  intl_state.curr_pckt = 0;
  intl_state.num_corrupt = 0;

  data test;

  p_recv_init = &recv_init;
  p_intl_state = &intl_state;

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
  memset(packets, 0, sizeof(packets));

  // Should I start a timeout timer here as well?
//  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (received == 1) {
		  // Extract GPS data from packets
		  int ret;
		  ret = receive_gps_data(packets, p_recv_init, &test);
		  char msg[128];
		  // Transmit message to computer via USB
		  if (ret == 0){ // Transmit data
			  // Can't transmit float due to FLASH size limitations (might work on different board)
//			  sprintf(msg, "COORDS: (%f,%f,%f)\r\nCTRL: %d\r\n\n", test.x_coord, test.y_coord, test.z_coord, test.control);

			  // For now we send raw bytes and computer can decode them as needed.
			  CDC_Transmit_FS((uint8_t*)&test, sizeof(test));
		  } else if (ret == 1){ // Transmit error message and exit program (maybe just reset instead?)
			  sprintf(msg, "TERMINATED: Size of data struct does not match size of received data.\n");
			  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
			  return 1;
		  }

		  // Cleanup
		  received = 0;
		  memset(packets, 0, sizeof(packets));
		  memset(p_recv_init, 0, sizeof(*p_recv_init));
		  memset(p_intl_state, 0, sizeof(*p_intl_state));
		  // Should I start a timeout timer here as well?
		//  HAL_TIM_Base_Start_IT(&htim3);
		  HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

	// Step 2: Either wait for new handshake or receive packet
	if (p_recv_init->handshake == 0) { // If handshake failed, await new handshake
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
	} else if (p_recv_init->handshake == 1) { // If the handshake succeeded, receive
		HAL_UART_Receive_DMA(&huart1, &packets[p_intl_state->curr_pckt], p_recv_init->packet_size + 1);
		p_intl_state->curr_pckt++;
	} else if (p_recv_init->handshake == 2) { // If we are expecting resent packets, receive
		p_intl_state->num_corrupt--;
		// Find target index in packets array
		uint8_t* dest = packets[p_intl_state->corrupt_packets[p_intl_state->num_corrupt]];
		HAL_UART_Receive_DMA(&huart1, dest, p_recv_init->packet_size + 1);
	}

	// Start timeout timer
	HAL_TIM_Base_Start_IT(&htim3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	// Stop and reset timeout timer
	HAL_TIM_Base_Stop_IT(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	if (p_recv_init->handshake == 0) { // Step 1: Handshake

	  /*
	   * Flag Value : Meaning -> Result
	   *
	   * 0 : Matched CRC (Valid Data) -> Continue Transaction
	   * 1 : Mismatched CRC (Corrupted Data) -> Send retry request to transmitter
	   *
	   * */

	  uint8_t recv_crc = calculate_crc((uint8_t*)p_recv_init, 3);

	  if (recv_crc != p_recv_init->crc) {
		  flag = 1;
	  } else {
		  p_recv_init->handshake = 1;
		  flag = 0;

		  // Calculate and save number of padded bytes
		  p_recv_init->padding = (p_recv_init->packet_size * p_recv_init->num_of_packets) - p_recv_init->total_size;
	  }

	  HAL_UART_Transmit_DMA(&huart1, &flag, 1);


	} else if (p_recv_init->handshake == 1) { // Step 3: Verify Received Data

	  // Check CRC
	  uint8_t recv_crc = calculate_crc(&packets[p_intl_state->curr_pckt - 1], p_recv_init->packet_size);
	  if (recv_crc != packets[p_intl_state->curr_pckt - 1][p_recv_init->packet_size]) {
		 p_intl_state->corrupt_packets[p_intl_state->num_corrupt] = p_intl_state->curr_pckt;
		 p_intl_state->num_corrupt++;
	  }

	  // Check total size
	  if (p_recv_init->total_size < p_recv_init->packet_size) {
		  p_recv_init->total_size = 0;
	  } else {
		  p_recv_init->total_size -= p_recv_init->packet_size;
	  }

	  // Receive only if we are expecting more packets
	  if (p_intl_state->curr_pckt != p_recv_init->num_of_packets) {
		  HAL_UART_Receive_DMA(&huart1, &packets[p_intl_state->curr_pckt], p_recv_init->packet_size + 1);
		  p_intl_state->curr_pckt++;

		  // Start timeout timer
		  HAL_TIM_Base_Start_IT(&htim3);
	  } else {
		  if (p_intl_state->num_corrupt > 0) {
			  p_recv_init->handshake = 2; // Step 4: Ask transmitter to re-send lost packets
			  HAL_UART_Transmit_DMA(&huart1, &p_intl_state->corrupt_packets[p_intl_state->num_corrupt - 1] , 1);
		  }

	  }
	} else if (p_recv_init->handshake == 2) {
	  if (p_intl_state->num_corrupt > 0) {
		  HAL_UART_Transmit_DMA(&huart1, &p_intl_state->corrupt_packets[p_intl_state->num_corrupt - 1] , 1);
	  } else { // Step 5: Done receiving packets, set received flag to do processing in main while loop
		  received = 1;
	  }

	}

//  //HAL_UART_Receive_DMA(&huart1, rx_buffer + 1, sizeof(rx_buffer) - 1);
//  uint8_t transmit_crc = rx_buffer[sizeof(rx_buffer)-1];
//
//  if(calculate_crc(rx_buffer,sizeof(rx_buffer)-1) == transmit_crc ){
//	  struct data out;
//      uint8_t * receive_data = &rx_buffer;
//      uint8_t data_avail = rx_buffer[0];
//      receive_data++;
//      if (data_avail == 1){
//      } else if (data_avail == 2){
//      } else if(data_avail == 3){
//    	// Save buffer data to struct
//        receive_gps_data(&out, receive_data);
//        out.control = *receive_data;
//      }
//
//   }
//
//  memset(rx_buffer, 0, sizeof(rx_buffer)); //!!! Clear buffer (or what should we do if CRCs don't match?)
//  HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));

  return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {  // Timer TIM3 timeout
    	HAL_UART_DMAStop(&huart1);  // Stops any UART DMA (both TX and RX)
    	memset(packets, 0, sizeof(packets));
		memset(p_recv_init, 0, sizeof(*p_recv_init));
		memset(p_intl_state, 0, sizeof(*p_intl_state));
		// Should I start a timeout timer here as well?
	  //  HAL_TIM_Base_Start_IT(&htim3);
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)p_recv_init, 4);
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
