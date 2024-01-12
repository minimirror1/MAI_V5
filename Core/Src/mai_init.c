/*
 * mai_init.c
 *
 *  Created on: 2020. 3. 18.
 *      Author: shin
 */


#include "mai_init.h"
#include "main.h"
#include "mal_uart.h"
#include "mal_can.h"
#include "mal_sensor_limit.h"
#include "mal_loop_process.h"
#include "mal_systimer.h"
#include "mal_motor_acPanasonic.h"
#include "mal_motor.h"

#include "mal_board_info.h"
#include "mal_motor_acPana232_v2.h"

#include "mal_motor_comBase.h"

#include "string.h"


/* hal */
extern CAN_HandleTypeDef hcan1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* mal */
extern MAL_BoardManagerTypeDef mboard;

extern MAL_CAN_HandleTypeDef mcan1;

extern MAL_UART_HandleTypeDef muart1;
extern MAL_UART_HandleTypeDef muart2;

extern MAL_MOTOR_PanasonicHandleTypeDef mpanasonic;

extern MAL_SENSOR_Limit_HandleTypeDef sensor[4];

extern MAL_MOTOR_HandleTypeDef mmotor[MOTOR_AXLE_CNT];

void MAL_MOTOR_Panasonic_Init(void);
void MAL_CAN_Init(void);
void MAL_UART_Init(void);
void MAL_SENSOR_Init(void);
void MAL_Motor_Init(void);
void MAL_Board_VerReg(void);
void MAL_RS232_Init();
void MAL_BASE_COM_Init(void);
//uint16_t IV[200];

uint32_t testCnt;
uint32_t testCntlog;
uint32_t t_test;
//=====
void testuart(void)
{
/*	testCnt +=testCntlog;
	testCntlog +=10;
	MAL_Motor_AcPanasonic_SetChangeTarget(&mpanasonic,testCnt);*/

	uint32_t he = 0;
	uint8_t pdata[8] = {0,};
	pdata[0] = MAL_Board_ID_GetValue();

	MAL_CAN_SendAddQueue_ExtData(&mcan1,he ,pdata , 8);
}
void test_init(void)
{
	MAL_SysTimer_FunctionCall(testuart, &t_test, 20);

}

void MAL_RUN_LED_Toggle(void)
{
	LED_1_GPIO_Port->ODR ^= LED_1_Pin;
}
void MAL_COM_LED_OFF(void)
{
	MAL_CAN_LED_Off(&mcan1.rxLed);
	MAL_CAN_LED_Off(&mcan1.txLed);
}


void MAL_LED_process(void)
{
	static uint32_t t_runLed;
	static uint32_t t_comLed;

	MAL_SysTimer_FunctionCall(MAL_RUN_LED_Toggle,&t_runLed,500);
	MAL_SysTimer_FunctionCall(MAL_COM_LED_OFF,&t_comLed,100);
}

//=====

void MAL_MAI_V1_Init(void)
{
	MAL_Motor_Init();





	MAL_Board_VerReg();
    MAL_CAN_Init();
	//MAL_UART_Init();
	MAL_SENSOR_Init();
	MAL_MOTOR_Panasonic_Init();

	MAL_RS232_Init();
	MAL_BASE_COM_Init();


	mboard.myCanId = MAL_Board_ID_GetValue();
    //my_can_id = MAL_Board_ID_GetValue();
	//MAL_LOOP_ProcessAddr(test_init);

	MAL_LOOP_ProcessAddr(MAL_LED_process);




}

void MAL_Board_VerReg(void)
{

	MAL_Board_Version_init(MAL_VERSION_MAJER, MAL_VERSION_MINOR, MAL_VERSION_BUILD);

	MAL_Board_ID_InitPort(ID_1_GPIO_Port, ID_1_Pin, GPIO_PIN_SET);
	MAL_Board_ID_InitPort(ID_2_GPIO_Port, ID_2_Pin, GPIO_PIN_SET);
	MAL_Board_ID_InitPort(ID_4_GPIO_Port, ID_4_Pin, GPIO_PIN_SET);
	MAL_Board_ID_InitPort(ID_8_GPIO_Port, ID_8_Pin, GPIO_PIN_SET);
	MAL_Board_ID_InitPort(ID_16_GPIO_Port, ID_16_Pin, GPIO_PIN_SET);

	MAL_Board_ID_SetReadValue();

}
void MAL_RS232_Init()
{
	MAL_UART_HandleMatching(&muart2,&huart2);
	MAL_UART_RxAppointment(&muart2);
	MAL_Motor_AcPanasonic_232_RegInit(&muart2);
	MAL_LOOP_ProcessAddr(MAL_Motor_AcPanasonic_232_Process);
}



void MAL_BASE_COM_Init(void)
{
	MAL_UART_HandleMatching(&muart1,&huart1);
	 MAL_UART_RS485Init(&muart1, UART1_RS485_EN_GPIO_Port, UART1_RS485_EN_Pin);
	MAL_UART_RxAppointment(&muart1);
	MAL_Motor_ComBase_RegInit(&muart1);
	MAL_LOOP_ProcessAddr(MAL_Motor_ComBase_Process);
}


void MAL_Motor_Init(void)
{
	//모터 구조체 초기화
	//MAL_Motor_CallBackInit(&mmotor[0], MAL_MOTOR_AC_PANASONIC );

	MAL_Motor_CallBackInit(&mmotor[0], (uint32_t *)&mpanasonic, MAL_MOTOR_AC_PANASONIC );
	/*
	MAL_Motor_ManagerInit((uint32_t *)&mcan1);

	MAL_LOOP_ProcessAddr(MAL_Protocol_Ani_Process);*/
}

void MAL_MOTOR_Panasonic_Init(void)
{
	MAL_Motor_AcPanasonic_TimerRegInit(&mpanasonic,&htim4,TIM_CHANNEL_1,&htim3,TIM_CHANNEL_1,&htim2);
	//MAL_Motor_AcPanasonic_TimerRegInit(&mpanasonic,&htim3,TIM_CHANNEL_1,&htim4,TIM_CHANNEL_1,&htim2);

	MAL_Motor_AcPanasonic_SensorLimRegInit(
			&mpanasonic,
			&sensor[0],
			&sensor[1]);

#ifdef MAL_TEST_MODE
	//MAL_Motor_AcPanasonic_SetSettingValTestValue(&mpanasonic);
#endif

	MAL_LOOP_ProcessAddr(MAL_Motor_AcPanasonic_Process);
}

void MAL_SENSOR_Init(void)
{
	MAL_SENSOR_LimitHandleInit(&sensor[0],0,0,SEN_1_GPIO_Port,SEN_1_Pin, MAL_SENSOR_CW, GPIO_PIN_SET);
	MAL_SENSOR_LimitHandleInit(&sensor[1],0,1,SEN_2_GPIO_Port,SEN_2_Pin, MAL_SENSOR_CCW, GPIO_PIN_SET);

#ifdef MAL_TEST_MODE
	MAL_LOOP_ProcessAddr(MAL_SENSOR_LimitRefresh);
#endif
}

void MAL_UART_Init(void)
{
	//dbg uart
	MAL_UART_HandleMatching(&muart2,&huart2);
	MAL_UART_RxAppointment(&muart2);
}

void MAL_CAN_Init(void)
{
	MAL_CAN_FilterConfig(&hcan1);
	MAL_CAN_HandleMatching(&mcan1,&hcan1);

	//MAL_CAN_LEDInit(&mcan1.txLed, LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	//MAL_CAN_LEDInit(&mcan1.rxLed, LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

	//MAL_LOOP_ProcessAddr(MAL_CAN_Process);
}
