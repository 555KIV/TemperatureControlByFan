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
#include "filter_sma.h"
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
#define PID_FAN_CYCLE_MIN 50
#define PID_FAN_CYCLE_START 100

#define MENU_POS_MAX 5
#define MENU_POS_MIN 1

#define NULL_FLASH 0xffffffff
#define NUM_SETTING 4

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
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

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
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void LedPrint();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//---------------------------
float pwmFAN = 100;
float targetTemp = 20;
float targetTempEncoder = 20;

float Kp = 50;
float Ki = 0;
float Kd = 0;

int16_t KEncoder = 0;

float errPrev = 0;
float errCur = 0;
float errInteg = 0;
float errDiff = 0;

uint32_t timeCountMs = 0;

//----------------------------

uint16_t ADC_raw[1];
float temp = 0;
uint8_t sch_100ms = 255;

//----------------------------

uint8_t R1 = 0, R2 = 0, R3 = 0;

uint8_t segmentNumber[]={
		0x3f, // 0
		0x06, // 1
		0x5b, // 2
		0x4f, // 3
		0x66, // 4
		0x6d, // 5
		0x7d, // 6
		0x07, // 7
		0x7f, // 8
		0x67, // 9
		0x40, // - (10)
		0x00, // off (11)
		0x6d, // S (12)
		0x77, // A (13)
		0x3e, // V - U (14)
		0x73, // p (15)
		0x5e, // d	(16)
		0x30 // I (17)
};



//------------------------------

uint32_t prevCounter = 0;


uint8_t posMenu = 1;
uint8_t posMenuPID_Edit = 1;
uint8_t flagMenu = 0;
uint8_t flagMenuEditHidden = 0;
uint8_t flagMenuEdit = 0;
uint8_t flagMenuEditPID = 0;
uint8_t flagDot[3] = {0,0,0};

uint32_t counthidden = 0;



//------------------------------

const uint32_t flashAddress = 0x0801f800;



//------------------------------

void TempMeasure()
{
	uint16_t buf = 0;

	HAL_ADC_Start_IT(&hadc1);
	buf = ADC_raw[0];
	buf = Filter_Sma(ADC_raw[0]);
	//buf = Filter_RAA(ADC_raw[0]);
	temp = A*buf*buf;
	temp+= B*buf;
	temp-= C;
	temp*=10;
	//LedPrint((uint8_t)temp);

	sch_100ms = 0;

}

void FanPIDRegulator()
{
	float timeCountSec = (float)timeCountMs / 1000;

	errCur = (temp/10) - targetTemp;

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

	//TIM14->CCR1 = pwmFAN;
	TIM14->CCR1 = (uint16_t)pwmFAN;
	errPrev = errCur;
	timeCountMs = 0;

}


void SetNumber(uint8_t number, uint8_t flag)
{
	number = segmentNumber[number];

	HAL_GPIO_WritePin(GPIOB, HL_A_Pin, !((number>>0)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_B_Pin, !((number>>1)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_C_Pin, !((number>>2)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_D_Pin, !((number>>3)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_E_Pin, !((number>>4)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_F_Pin, !((number>>5)&0x01));
	HAL_GPIO_WritePin(GPIOB, HL_G_Pin, !((number>>6)&0x01));
	HAL_GPIO_WritePin(GPIOA,HL_DP_Pin, flag==1 ? GPIO_PIN_RESET : GPIO_PIN_SET);

}

void LoadSetting()
{
	uint32_t address = flashAddress;
	uint32_t data[NUM_SETTING] = {0,};

	for(uint16_t i = 0;i<NUM_SETTING;i++)
	{
		uint32_t buf = *(uint32_t*)address;
		if (buf != NULL_FLASH)
		{
			data[i] = buf;
		}
		address+=8;
	}

	targetTemp = (data[0]!=0)?((float)data[0]/10):20;
	targetTempEncoder = targetTemp;
	Kp = (data[1]!=0)?data[1]:50;
	Ki = (data[2]!=0)?data[2]:0;
	Kd = (data[3]!=0)?data[3]:0;
}

void SaveSetting()
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.Page = flashAddress;
	EraseInitStruct.Page = FLASH_PAGE_NB-1;
	EraseInitStruct.NbPages = 1;
	uint32_t pageErr = 0;

	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&EraseInitStruct, &pageErr);


	uint32_t address = flashAddress;
	uint64_t data[NUM_SETTING] = {targetTemp*10,Kp,Ki,Kd};

	//address +=4;

	for(uint16_t i = 0;i<NUM_SETTING;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data[i]);

		address +=8;
	}

	HAL_FLASH_Lock();


}

void RecordKEncoder()
{
	switch (posMenuPID_Edit)
		{
			case 1:
			{
				KEncoder = (int16_t)Kp;
				break;
			}
			case 2:
			{
				KEncoder = (int16_t)Ki;
				break;
			}
			case 3:
			{
				KEncoder = (int16_t)Kd;
				break;
			}
		}
}

void ApplyPIDSetting()
{
	switch (posMenuPID_Edit)
	{
		case 1:
		{
			Kp = KEncoder;
			break;
		}
		case 2:
		{
			Ki = KEncoder;
			break;
		}
		case 3:
		{
			Kd = KEncoder;
			break;
		}
	}
}

void EnterButton()
{
	if (flagMenu == 1)
	{
		flagMenu = 0;
		if (posMenu == 3)
		{
			flagMenuEditHidden = 0;
			flagMenuEdit = 1;
			//HAL_TIM_Base_Start_IT(&htim17);

		}
		return;
	}
	if (flagMenuEditPID == 1)
	{
		ApplyPIDSetting();
		//HAL_TIM_Base_Stop_IT(&htim17);
		flagMenuEdit = 0;
		flagMenuEditHidden = 0;
		flagMenuEditPID = 0;
		flagMenu = 1;
		return;
	}
	switch (posMenu)
	{
		case 1:
		{
			break;
		}
		case 2:
		{
			break;
		}
		case 3:
		{
			//HAL_TIM_Base_Stop_IT(&htim17);
			flagMenuEdit = 0;
			flagMenuEditHidden = 0;
			flagMenu = 1;
			targetTemp = targetTempEncoder;
			break;
		}
		case 4:
		{
			RecordKEncoder();
			//HAL_TIM_Base_Start_IT(&htim17);
			flagMenuEdit = 1;
			flagMenuEditPID = 1;
			break;
		}
		case 5:
		{
			SaveSetting();
			flagMenu = 1;
		}
	}

}

void LedPrint()
{
	if (flagMenu == 1)
	{
		R3 = 10; // symbol = '-'
		R1 = 10;
		R2 = posMenu;
		flagDot[1] = 0;
		return;
	}
	if (flagMenuEditHidden == 1)
	{
		R1 = 11;
		R2 = 11;
		R3 = 11;
		flagDot[1] = 0;
		return;
	}
	if (flagMenuEditPID == 1)
	{
		int16_t buf = KEncoder;
		R1 = abs(buf%10);
		if (buf < 0 && buf>-10)
		{
			R2 = 10;
			R3 = 11;
			return;
		}
		if (buf<0 && buf>-100)
		{
			R2 = abs(buf/10);
			R3 = 10;
			return;
		}
		if (buf<10)
		{
			R2 = 11;
			R3 = 11;
			return;
		}
		if (buf < 100)
		{
			R2 = (buf%100)/10;
			R3 = 11;
			return;
		}
		R2 = (buf%100)/10;
		R3 = buf/100;
		return;
	}

	switch (posMenu)
	{
		case 1:
		{
			uint16_t num = temp;
			R1 = num%10;
			R2 = (num%100)/10;
			if (num<100)
				R3 = 11;
			else
				R3 = num/100;
			flagDot[1] = 1;
			break;
		}
		case 2:
		{
			uint16_t buf = TIM14->CCR1 / 10;
			R1 = buf%10;
			if (buf<10)
				R2 = 11;
			else
				R2 = (buf%100)/10;
			if (buf/100 == 0)
				R3 = 11;
			else
				R3 = buf/100;
			flagDot[1] = 0;
			break;
		}
		case 3:
		{
			uint16_t target = targetTempEncoder*10;
			R1 = target%10;
			R2 = (target%100)/10;
			if (target<100)
				R3 = 11;
			else
				R3 = target/100;
			flagDot[1] = 1;
			break;
		}
		case 4:
		{
			R1 = 11;
			R3 = 11;

			R2 = 14+posMenuPID_Edit;
			break;
		}
		case 5:
		{
			R1 = 14;
			R2 = 13;
			R3 = 12;
			break;
		}
	}

}



void Encoder()
{
	uint32_t currCounter = __HAL_TIM_GET_COUNTER(&htim3);
	currCounter = 32767 - ((currCounter-1)&0xFFFF)/2;
	if (currCounter > 32768/2)
	{
		currCounter = currCounter - 32768;
	}
	if (currCounter != prevCounter)
	{
		uint32_t delta = currCounter - prevCounter;
		prevCounter = currCounter;
		if (delta>10)
		{
			if (flagMenu == 0 && posMenu == 3)
			{
				targetTempEncoder = (targetTempEncoder >= 99.9) ? 99.9 : targetTempEncoder+0.1;
				return;
			}
			if (flagMenu == 0 && posMenu == 4 && flagMenuEditPID == 0)
			{
				posMenuPID_Edit = (posMenuPID_Edit >= 3) ? 3 : posMenuPID_Edit+1;
				return;
			}
			if (flagMenuEditPID == 1)
			{
				KEncoder = (KEncoder >= 999) ? 999 : KEncoder+1;
				return;
			}
			posMenu = (posMenu >= MENU_POS_MAX) ? MENU_POS_MAX : posMenu+1;
		}
		if (delta<-10)
		{
			if (flagMenu == 0 && posMenu == 3)
			{
				targetTempEncoder = (targetTempEncoder <= 0) ? 0 : targetTempEncoder-0.1;
				return;
			}
			if (flagMenu == 0 && posMenu == 4 && flagMenuEditPID == 0)
			{
				posMenuPID_Edit = (posMenuPID_Edit <= 1) ? 1 : posMenuPID_Edit-1;
				return;
			}
			if (flagMenuEditPID == 1)
			{
				KEncoder = (KEncoder <= 0) ? 0 : KEncoder-1;
				return;
			}
			posMenu = posMenu <= MENU_POS_MIN ? MENU_POS_MIN : posMenu-1;
		}
		flagMenu = 1;
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_raw,1);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start_IT(&htim14,TIM_CHANNEL_1);

  TIM14->CCR1=PID_FAN_CYCLE_START;

  HAL_GPIO_WritePin(GPIOB,HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin|HL_E_Pin|HL_F_Pin|HL_G_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,HL_DP_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,HL_Switch1_Pin|HL_Switch2_Pin|HL_Switch3_Pin,GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim6);

  //HAL_TIM_Base_Start_IT(&htim17);

  LoadSetting();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (sch_100ms)
	  	{
		  TempMeasure();
	  	}
	  FanPIDRegulator();

	  Encoder();
	  LedPrint();


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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  htim6.Init.Period = 10;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  HAL_GPIO_WritePin(HL_DP_GPIO_Port, HL_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin
                          |HL_E_Pin|HL_F_Pin|HL_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HL_Switch1_Pin HL_Switch2_Pin HL_Switch3_Pin */
  GPIO_InitStruct.Pin = HL_Switch1_Pin|HL_Switch2_Pin|HL_Switch3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_SW_Pin */
  GPIO_InitStruct.Pin = EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EN_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HL_DP_Pin */
  GPIO_InitStruct.Pin = HL_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HL_DP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HL_A_Pin HL_B_Pin HL_C_Pin HL_D_Pin
                           HL_E_Pin HL_F_Pin HL_G_Pin */
  GPIO_InitStruct.Pin = HL_A_Pin|HL_B_Pin|HL_C_Pin|HL_D_Pin
                          |HL_E_Pin|HL_F_Pin|HL_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
	if (htim->Instance == TIM14)
	{
		TIM14->CCR1 = (uint16_t)pwmFAN;
		++timeCountMs;
	}
	if (htim->Instance == TIM17)
	{
		flagMenuEditHidden = (flagMenuEditHidden == 0)? 1: 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

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
