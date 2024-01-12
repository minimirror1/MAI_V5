/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mai_init.h"





/*************?ûê?ã†?óê RAMÎ≤ÑÌçº?óê ?î∞?ùº ?àò?†ï******************/
#define CAN_Q_BUFF_SIZE 	512   //  ((?ó§?çî 4 Byte + ?ç∞?ù¥?Ñ∞ 8 Byte) x 512(CAN_Q_BUFF_SIZE)) x 2(rx,tx) = 12,288 Byte
/*************?ûê?ã†?óê RAMÎ≤ÑÌçº?óê ?î∞?ùº ?àò?†ï******************/

// CAN 1Í∞úÏùº Í≤ΩÏö∞
#define CAN_1	0 // ?Ñ§?ù¥Î∞? ?ïå?ïÑ?Ñú define
#define CAN_CNT 1


#define IMONITOR_SLAVE
//#define IMONITOR_MASTER

#ifdef IMONITOR_SLAVE
#define IMONITOR_SLAVE_MAI
//#define IMONITOR_SLAVE_MBI
#endif

#include "dl_can.h"
#include "net_phd_pid.h"

#include "app_pid_edit_cmd.h"
#include "app_pid_error_cmd.h"
#include "app_pid_init_cmd.h"
#include "app_pid_init_cmd.h"
#include "app_pid_midi_cmd.h"
#include "app_pid_motion_cmd.h"
#include "app_pid_motion_cmd.h"
#include "app_pid_sensor_cmd.h"
#include "app_pid_speaker_cmd.h"
#include "app_pid_inspection_cmd.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define SEN_2_Pin GPIO_PIN_4
#define SEN_2_GPIO_Port GPIOC
#define SEN_1_Pin GPIO_PIN_5
#define SEN_1_GPIO_Port GPIOC
#define SEN_1_DEC_Pin GPIO_PIN_0
#define SEN_1_DEC_GPIO_Port GPIOB
#define SEN_2_DEC_Pin GPIO_PIN_1
#define SEN_2_DEC_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOB
#define ID_16_Pin GPIO_PIN_6
#define ID_16_GPIO_Port GPIOC
#define ID_8_Pin GPIO_PIN_7
#define ID_8_GPIO_Port GPIOC
#define ID_4_Pin GPIO_PIN_8
#define ID_4_GPIO_Port GPIOC
#define ID_2_Pin GPIO_PIN_9
#define ID_2_GPIO_Port GPIOC
#define ID_1_Pin GPIO_PIN_8
#define ID_1_GPIO_Port GPIOA
#define UART1_RS485_EN_Pin GPIO_PIN_15
#define UART1_RS485_EN_GPIO_Port GPIOA
#define PWM_CCW_Pin GPIO_PIN_4
#define PWM_CCW_GPIO_Port GPIOB
#define PWM_CW_Pin GPIO_PIN_6
#define PWM_CW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MOTOR_AXLE_CNT	1
#define HAL_MOTOR_MODULE_ENABLED
#define HAL_MOTOR_AC_MODULE_ENABLED
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
