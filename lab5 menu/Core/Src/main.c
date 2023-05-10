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
  *Phanwaphat Lohasummakul 64340500069
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[20];
uint8_t TxBuffer[150];
uint32_t timestamp = 0;
int Timehz = 0;
int Num = 0;
int U = 0;
int Hz = 0;
int CheckB1;
enum{
	Welcomeja, LED_Control, Button_Status
}state = Welcomeja;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTDMAConfig();
void LEDOn();
void LEDOff();
void welcome();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  welcome();
  UARTDMAConfig();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	  if((Num%2==1)&&(U>-5)){
		  LEDOn();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void welcome(){
	sprintf((char*)TxBuffer,"\r\nWelcome to Menu\r\n---------------\r\n0 : LED Control\r\n1 : Button Status\r\nPlease Select Menu\r\n");
	HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
}

void UARTDMAConfig()
{
	HAL_UART_Receive_DMA(&huart2, RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		RxBuffer[1] = '\0';
	  switch(state){
	  case Welcomeja:
		  if(RxBuffer[0]==48){
			  sprintf((char*)TxBuffer,"\r\nYou Select Menu %s : LED Control\r\nLED Control Menu\r\na : SpeedUp\r\ns : SpeedDown\r\nd : OnOff\r\nx : Back\r\n",RxBuffer);
			  state = LED_Control;
		  }
		  else if (RxBuffer[0]==49){
			  CheckB1 = 1;
			  sprintf((char*)TxBuffer,"\r\nYou Select Menu %s : Button Status\r\nButton Status Menu\r\nx : Back\r\nB1 : Show Button Status",RxBuffer);
			  state = Button_Status;
		  }
		  else{
			  sprintf((char*)TxBuffer,"\r\nYou Select Menu %s : Don't have this menu\r\nPlease select again\r\n",RxBuffer);
			  state = Welcomeja;
		  }
		  break;
	  case LED_Control:
		  if(RxBuffer[0] == 97){
			  if(Num%2==1){
				  U += 1;
				  Hz = 5 + U;
				  sprintf((char*)TxBuffer,"\r\nLED blink with a frequency of %d Hz\r\n",Hz);
			  }
			  else if(Num%2==0){
				  sprintf((char*)TxBuffer,"\r\nLED Off\r\nPlease On LED\r\n");
			  }
			  state = LED_Control;
		  }
		  else if(RxBuffer[0] == 115){
			  if(Num%2==1){
				  U -= 1;
				  Hz = 5 + U;
				  if(Hz > 0){
					  sprintf((char*)TxBuffer,"\r\nLED blink with a frequency of %d Hz\r\n",Hz);
				  }
				  else if(Hz <= 0){
					  U = -5;
					  LEDOff();
					  sprintf((char*)TxBuffer,"\r\nThe LED does not blink because the frequency is <= 0\r\nPlease increase the frequency\r\n");
				  }
			  }
			  else if(Num%2==0){
				  sprintf((char*)TxBuffer,"\r\nLED Off\r\nPlease On LED\r\n");
			  }
			  state = LED_Control;
		  }
		  else if(RxBuffer[0] == 100){
			  Num += 1;
			  if(Num % 2 == 1){
				  LEDOn();
				  sprintf((char*)TxBuffer,"\r\nLED On\r\n");
			  }
			  else if(Num % 2 == 0){
				  LEDOff();
				  sprintf((char*)TxBuffer,"\r\nLED Off\r\n");
			  }
			  state = LED_Control;
		  }
  		  else if(RxBuffer[0] == 120){
  			  LEDOff();
  			  U = 0;
  			  sprintf((char*)TxBuffer,"\r\nBack to main menu\r\n0 : LED Control\r\n1 : Button Status\r\nPlease Select Menu\r\n");
  			  state = Welcomeja;
  		  }
  		  else{
  			  sprintf((char*)TxBuffer,"\r\nYou Select Menu %s : Don't have this menu\r\nPlease select again\r\n",RxBuffer);
			  state = LED_Control;
  		  }
		  break;
	  case Button_Status:
		  if(RxBuffer[0] == 120){
			  CheckB1 = 0;
			  sprintf((char*)TxBuffer,"\r\nBack to main menu\r\n0 : LED Control\r\n1 : Button Status\r\nPlease Select Menu\r\n");
			  state = Welcomeja;
		  }
		  else{
			  sprintf((char*)TxBuffer,"\r\nYou Select Menu %s : Don't have this menu\r\nPlease select again\r\n",RxBuffer);
			  state = Button_Status;
		  }
		  break;
	  }
	  HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		if(CheckB1 == 1){
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0){
				sprintf((char*)TxBuffer,"\r\nThe button is pressed\r\n");
				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1){
				sprintf((char*)TxBuffer,"\r\nThe button is not pressed\r\n");
				HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));
			}
		}
	}
}

void LEDOn()
{
	if(HAL_GetTick() >= timestamp)
	{
		Timehz = 1000/(2*(5+U));
		timestamp = HAL_GetTick()+ Timehz;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void LEDOff()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
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
