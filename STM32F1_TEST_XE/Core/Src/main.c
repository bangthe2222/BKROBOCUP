/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PS2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
void Lui(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_RESET);
}

void Tien(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_SET);
}

void Phai(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_SET);
}

void Trai(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_RESET);
}

void Trai_Tai_Cho(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_SET);
}

void Phai_Tai_Cho(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_RESET);
}

void Dung(){
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Tren_Port, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_Duoi_Port, Duoi_Phai_2, GPIO_PIN_RESET);
}
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	PS2Buttons PS2;
	PS2_Init(&htim3, &PS2);
  while (1)
  {
    /* USER CODE END WHILE */	
		PS2_Update();
		if (PS2.UP) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Tien();
		} else if (PS2.DOWN) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Lui();
		}else if (PS2.LEFT) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Trai();
		}else if (PS2.RIGHT) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Phai();
		} else if (PS2.L1) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Trai_Tai_Cho();
		} else if (PS2.L2) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Phai_Tai_Cho();
		}  else {
			Dung();
		}
		HAL_Delay(2);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 71;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8
//                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA5 PA7 PA8
                           PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */

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