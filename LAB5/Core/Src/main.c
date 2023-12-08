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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A 0.0000264f
#define B 0.0024f
#define C 112.69f

#define PID_FAN_CYCLE_MAX 1000
#define PID_FAN_CYCLE_MIN 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void ledprint(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pwmFAN = 100;
float targetTemp = 20;

float Kp = 5;
float Ki = 0;
float Kd = 0;

float errPrev = 0;
float errCur = 0;
float errInteg = 0;
float errDiff = 0;

uint32_t timeCountMs = 0;


uint16_t ADC_raw[1];
float temp = 0;
uint8_t sch_100ms = 255;

void TempMeasure()
{
	if (sch_100ms)
	{
		HAL_ADC_Start_IT(&hadc1);
		temp = A*ADC_raw[0]*ADC_raw[0];
		temp+= B*ADC_raw[0];
		temp-= C;
		ledprint((uint8_t)temp);

		sch_100ms = 0;
	}
}

void FanPIDRegulator()
{
	float timeCountSec = (float)timeCountMs / 1000;

	errCur = temp - targetTemp;

	if ( (((Ki*errInteg)<=PID_FAN_CYCLE_MAX) && (errCur >=0)) || (((Ki*errInteg)>=PID_FAN_CYCLE_MIN) && (errCur < 0)) )
	{
		errInteg += errCur*timeCountSec;
	}

	errDiff = (errCur - errPrev)/timeCountSec;

  
	pwmFAN = Kp*errCur + Ki*errCur + Kd*errCur;

	if (pwmFAN < PID_FAN_CYCLE_MIN)
	{
		pwmFAN = PID_FAN_CYCLE_MIN;
	}
	if (pwmFAN > PID_FAN_CYCLE_MAX)
	{
		pwmFAN = PID_FAN_CYCLE_MAX;
	}

	errPrev = errCur;
	timeCountMs = 0;

}

uint8_t R1 = 255, R2 = 0, R3 = 0;

uint16_t count_encoder = 0;

void setnumber(uint8_t number)
{
	switch (number)
	{
	case 255:
			HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_SET);
			break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;

	case 4:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_SET);
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOB, HL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, HL_F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, HL_G_Pin, GPIO_PIN_RESET);
		break;
	}

}


void ledprint(uint16_t num)
{
	R3 = num%10;
	R2 = (num%100)/10;
	R1 = num/100;
	if (R1 == 0)
		R1=255;

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_raw,1);

  HAL_TIM_Base_Start_IT(&htim2);



  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);

  //TIM3->CCR4=PID_FAN_CYCLE_MIN;

  HAL_GPIO_WritePin(GPIOB,HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin|HL_E_Pin|HL_F_Pin|HL_G_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,HL_Switch1_Pin|HL_Switch2_Pin|HL_Switch3_Pin,GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  TempMeasure();

	  //FanPIDRegulator();
	  //ledprint(123);


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HL_Switch1_Pin|HL_Switch2_Pin|HL_Switch3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin
                          |HL_E_Pin|HL_F_Pin|HL_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HL_Switch1_Pin HL_Switch2_Pin HL_Switch3_Pin */
  GPIO_InitStruct.Pin = HL_Switch1_Pin|HL_Switch2_Pin|HL_Switch3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HL_A_Pin HL_B_Pin HL_C_Pin HL_D_Pin
                           HL_E_Pin HL_F_Pin HL_G_Pin */
  GPIO_InitStruct.Pin = HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin
                          |HL_E_Pin|HL_F_Pin|HL_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		sch_100ms = 255;
	}
	if (htim->Instance == TIM3)
	{
		TIM3->CCR4 = (uint16_t)pwmFAN;
		++timeCountMs;
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
