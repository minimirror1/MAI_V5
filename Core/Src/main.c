/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mai_init.h"
#include "mal_uart.h"
#include "mal_can.h"
#include "mal_systimer.h"
#include "mal_sensor_limit.h"
#include "mal_loop_process.h"
#include "mal_motor.h"
#include "mal_motor_acPana232Func.h"

#include "mal_board_info.h"

#include "eeprom.h"
#include "eeprom_data.h"
#include "mal_motor_acPanaCurve.h"

#include "Imonitor.h"



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
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

MAL_BoardManagerTypeDef mboard;

MAL_CAN_HandleTypeDef mcan1;
MAL_UART_HandleTypeDef muart1;
MAL_UART_HandleTypeDef muart2;

MAL_SENSOR_Limit_HandleTypeDef sensor[4];

MAL_MOTOR_HandleTypeDef mmotor[MOTOR_AXLE_CNT];

MAL_MOTOR_ACPANA_CurveTypeDef panaCurve = {0,};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define LIVE_DBG
#ifdef LIVE_DBG
#include "mal_motor_acPanasonic.h"
uint8_t live_1;
uint8_t live_2;
extern MAL_MOTOR_PanasonicHandleTypeDef mpanasonic;
#endif

/* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NB_OF_VAR];
//uint16_t VarDataTab[NB_OF_VAR] = { 'M', 'a', 't', 'e', 'u', 's', 'z', ' ', 'S', 'a', 'l', 'a', 'm', 'o', 'n', ' ', 'm', 's', 'a', 'l', 'a', 'm', 'o', 'n', '.', 'p', 'l' };
//uint8_t VarDataTabRead[NB_OF_VAR];
//uint16_t VarIndex, VarDataTmp = 0;

EEPemul_Data_TypeDef EepData;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t t_PulseSync;
	uint8_t toggle232=0;
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
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  HAL_IWDG_Refresh(&hiwdg);

	//my_can_id = MAL_Board_ID_GetValue();
	gm_motion_RX_LED_init(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	gm_motion_TX_LED_init(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
	//MAL_CAN_FilterConfig(&hcan1);

	MAL_MAI_V1_Init();
	//my_can_id = MAL_Board_ID_GetValue();

	//idtest

	add_my_can_sub_id(MAL_Board_ID_GetValue(), 1);


	can_init_data_save(&hcan1);
	MAL_Protocol_Ani_EventBootAlm();

	Imonitor_init();



/*eep example*/
////
////
//	//EE_emul_Init((uint16_t *)&EepData.flag,sizeof(EEPemul_Data_TypeDef));
//
//
//	//EE_ReadStrData((uint16_t *)&EepData.encoderSt1_cnt,sizeof(EepData.encoderSt1_cnt));
//
//	//EepData.encoderSt1_cnt = 0x11223344;
//	//EE_WriteStrData((uint16_t *)&EepData.encoderSt1_cnt,sizeof(EepData.encoderSt1_cnt));
//
//	//EepData.flag = 0x1212;
//	//EE_WriteStrData((uint16_t *)&EepData.flag,sizeof(EepData.flag));
//
//	/* Unlock the Flash Program Erase controller */
//
//	/* EEPROM Init */
//
//	// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

/*eep example*/
/*
	// Fill EEPROM variables addresses
	for (VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++) {
		VirtAddVarTab[VarIndex - 1] = VarIndex;
	}

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		 Sequence 1
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTab[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex], &VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);

	// Store revert Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
	HAL_UART_Transmit(&huart2, "Store revert values\n\r", 21, 100);

	//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		 Sequence 1
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTab[NB_OF_VAR - VarIndex - 1])) != HAL_OK) {
			Error_Handler();
		}
	}
//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

// Read values
	HAL_UART_Transmit(&huart2, "Read revert values\n\r", 20, 100);
	//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex], &VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read revert table: ", 19, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);

	// Store Values in EEPROM emulation
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
	HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

	// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		 Sequence 1
		if ((EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTab[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	// Read values
	HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		if ((EE_ReadVariable(VirtAddVarTab[VarIndex], &VarDataTabRead[VarIndex])) != HAL_OK) {
			Error_Handler();
		}
	}
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
	HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
	HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(MAL_SysTimer_Elapsed(t_PulseSync) >= 500)
		{
			if(toggle232 == 0)
			{
				MAL_Motor_AcPanasonic_232_GetPulseCounter();
				toggle232 = 1;
			}
			else if(toggle232 == 1)
			{
				MAL_Motor_AcPanasonic_232_GetAlmNumber();
				toggle232 = 2;
			}
			else
			{
				MAL_Motor_AcPanasonic_232_GetAbsoluteCounter();
				toggle232 = 0;
			}
			t_PulseSync = MAL_SysTimer_GetTickCount();
		}
		/*rs232 test*/
//
//		if (MAL_SysTimer_Elapsed(test232_t) >= 100) {
//			//MAL_Motor_AcPanasonic_232_SetAbsoluteClear();
//			MAL_Motor_AcPanasonic_232_GetAbsoluteCounter();
//			test232_t = MAL_SysTimer_GetTickCount();
//		}



		MAL_LOOP_ProcessHandler();
		proc_can_rx();
		proc_can_tx();


		/*	  HAL_Delay(100);
		 PWM_CCW_GPIO_Port->ODR |= PWM_CCW_Pin; // RESET
		 PWM_CW_GPIO_Port->ODR |= PWM_CW_Pin;
		 HAL_Delay(500);
		 PWM_CCW_GPIO_Port->ODR &= ~PWM_CCW_Pin; // SET
		 PWM_CW_GPIO_Port->ODR &= ~PWM_CW_Pin;*/
#ifdef IWDG_INIT
		HAL_IWDG_Refresh(&hiwdg);
#endif

#ifdef LIVE_DBG
	  live_1 = MAL_SENSOR_GetDetection(mpanasonic.cwSen);
	  live_2 = MAL_SENSOR_GetDetection(mpanasonic.ccwSen);
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 179;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 179;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3_Pin|LED_2_Pin|LED_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART1_RS485_EN_GPIO_Port, UART1_RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEN_2_Pin SEN_1_Pin ID_16_Pin ID_8_Pin
                           ID_4_Pin ID_2_Pin */
  GPIO_InitStruct.Pin = SEN_2_Pin|SEN_1_Pin|ID_16_Pin|ID_8_Pin
                          |ID_4_Pin|ID_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEN_1_DEC_Pin SEN_2_DEC_Pin */
  GPIO_InitStruct.Pin = SEN_1_DEC_Pin|SEN_2_DEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ID_1_Pin */
  GPIO_InitStruct.Pin = ID_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ID_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART1_RS485_EN_Pin */
  GPIO_InitStruct.Pin = UART1_RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UART1_RS485_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
