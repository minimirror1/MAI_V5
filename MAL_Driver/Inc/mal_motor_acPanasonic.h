/*
 * mal_motor_acPanasonic.h
 *
 *  Created on: Mar 11, 2020
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_ACPANASONIC_H_
#define INC_MAL_MOTOR_ACPANASONIC_H_

#include "main.h"
#include "mal_hal_header.h"
#include "mal_sensor_limit.h"
#include "mal_motor.h"

#ifdef HAL_MOTOR_AC_MODULE_ENABLED

#define PANASONIC_CW_TIM_FREQ		(float)1/90000000
#define PANASONIC_CCW_TIM_FREQ		(float)1/90000000

#define PANASONIC_1US_PULSE_COUNT   91

#define PANASONIC_DMA_MEM_SIZE		10005

#define PANASONIC_POSI_MAX			4095//(uint16_t)0xFFFF






typedef enum __MAL_MOTOR_PanasonicLocationTypeDef
{
	MAL_PANASONIC_CW,
	MAL_PANASONIC_CCW
}MAL_MOTOR_PanasonicLocationTypeDef;

typedef enum __MAL_MOTOR_PanasonicTimStatusTypeDef
{
	MAL_PANASONIC_STOP,
	MAL_PANASONIC_RUN
}MAL_MOTOR_PanasonicTimStatusTypeDef;




typedef struct __MAL_MOTOR_PanasonicTimerTypeDef
{

	TIM_HandleTypeDef *htim;
	uint32_t Channel;

	float inPeriod; //타이머 공급 클럭 속도

	float tempPeriod; //주기 계산 임시저장
	float tempCnt; //주기 계산 임시저장
	uint32_t calcPeriod;//계산결과 주기

	int32_t oldDuty;

}MAL_MOTOR_PanasonicTimerTypeDef;

typedef struct __MAL_MOTOR_PanasonicPositionTypeDef
{
	int32_t now;
	int32_t target;
}MAL_MOTOR_PanasonicPositionTypeDef;

typedef struct __MAL_MOTOR_PanasonicStatusTypeDef
{

	MAL_MOTOR_PanasonicPositionTypeDef position;
	uint8_t axleNum;	//보드내부 축 번호
}MAL_MOTOR_PanasonicStatusTypeDef;


typedef struct __MAL_MOTOR_PanasonicControlTypeDef
{
	MAL_MOTOR_PanasonicLocationTypeDef location; //회전방향
	MAL_MOTOR_PanasonicTimStatusTypeDef runStatus; //DMA 출력 동작 플래그

	uint32_t calcCount;
	uint32_t setCount; //DMA 출력 카운트 =< 10000
	uint32_t checkCount; //출력 검사 카운트

}MAL_MOTOR_PanasonicControlTypeDef;

typedef struct __MAL_MOTOR_PanasonicSettingTypeDef
{
	uint8_t SensorDirection; 	//센서방향
	uint16_t OppositeLimit;		//반대편 한계 각도(카운터로 변환해야됨)
	uint16_t DefaultLocation;	//초기화 후 위치(초기위치)
	uint16_t ReductionRatio; 	//감속기 감속비
	MAL_MOTOR_SensorInit_TypeDef flag;				//세팅 여부 flag

	uint8_t offsetFlag;
	uint32_t t_offsetDelay;
	int32_t moveCnt;
	uint32_t t_offset;

	uint32_t t_emergency;

	int32_t ampZeroCnt;


	float reductionPuls;	//감속후 1도 움직임에 필요한 초당 펄스수
	float jogCount; 		//조그에 사용될 1ms 당 펄스수( 반올림)
	uint32_t t_jog;

	int32_t DefultLocTemp;
	uint32_t t_defLoc;

	//20201104
	int32_t DefaultLocationCnt;
	int32_t DefultLocTempCnt;
	uint8_t AbsoOffsetFlag;
	int32_t AbsoOffsetCnt;


	uint8_t absoReadFlag;//20201103
	uint8_t absoReadOk; //20201103
	uint8_t absoRetryCnt;//20201103
	int32_t absoCount;//20201103
	uint8_t absoStatus;//20201103	// 0:None, 1:ok, 2:battery error, 3:timeout

	float absoCntToCnt;//20201104 엡소 값 카운트 변경
}MAL_MOTOR_PanasonicSettingTypeDef;

typedef struct __MAL_MOTOR_PanasonicSettingConv_TypeDef {
	int32_t offset;
	int32_t cwLim;
	int32_t ccwLim;
	int32_t range;
}MAL_MOTOR_PanasonicSettingConv_TypeDef;


typedef struct __MAL_MOTOR_PanasonicJogConstantSpeed_TypeDef {
	uint8_t activeflag;
	int32_t jogCnt;
	uint32_t tJogMove;
}MAL_MOTOR_PanasonicJogConstantSpeed_TypeDef;


typedef struct __MAL_MOTOR_PanasonicHandleTypeDef
{

	MAL_MOTOR_PanasonicTimerTypeDef cwTim;
	MAL_MOTOR_PanasonicTimerTypeDef ccwTim;

	MAL_SENSOR_Limit_HandleTypeDef *cwSen;
	MAL_SENSOR_Limit_HandleTypeDef *ccwSen;

	TIM_HandleTypeDef *cyclehtim;

	uint16_t cwDMA[PANASONIC_DMA_MEM_SIZE];
	uint16_t ccwDMA[PANASONIC_DMA_MEM_SIZE];

	MAL_MOTOR_PanasonicStatusTypeDef status;

	MAL_MOTOR_PanasonicControlTypeDef control;

	MAL_MOTOR_PanasonicSettingTypeDef setting;

	MAL_MOTOR_PanasonicSettingConv_TypeDef settingConv;

	MAL_MOTOR_PanasonicJogConstantSpeed_TypeDef jogCtr;

}MAL_MOTOR_PanasonicHandleTypeDef;





extern void MAL_Motor_AcPanasonic_TimerRegInit(
		MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic,
		TIM_HandleTypeDef *cwhtim,
		uint32_t cwChannel,
		TIM_HandleTypeDef *ccwhtim,
		uint32_t ccwChannel,
		TIM_HandleTypeDef *cyclehtim);
extern void MAL_Motor_AcPanasonic_SensorLimRegInit(
		MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic,
		MAL_SENSOR_Limit_HandleTypeDef *cwSen,
		MAL_SENSOR_Limit_HandleTypeDef *ccwSen);

extern void MAL_Motor_AcPanasonic_TimerStart(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);

extern void MAL_Motor_AcPanasonic_Process(void);

extern void MAL_Motor_AcPanasonic_SetSettingValTestValue(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);

extern void MAL_Motor_AcPanasonic_IRQHandler(TIM_HandleTypeDef *htim);

extern void MAL_Motor_AcPanasonic_SetChangeTarget(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, int32_t target);

extern void MAL_Motor_AcPanasonic_CheckCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);
#endif


//protocol
//set
extern void MAL_Motor_AcPanasonic_SetLocation(uint32_t *pmpanasonic, uint16_t location);
extern void MAL_Motor_AcPanasonic_StartInit(uint32_t *pmpanasonic);
extern void MAL_Motor_AcPanasonic_SetSetting(
		uint32_t *pmpanasonic,
		uint8_t SensorDirection,
		uint16_t OppositeLimit,
		uint16_t DefaultLocation,
		uint16_t ReductionRatio);
extern uint8_t MAL_Motor_AcPanasonic_SetSetting_Absolute(
		uint32_t *pmpanasonic,
		uint8_t SensorDirection,
		uint16_t OppositeLimit,
		uint16_t DefaultLocation,
		uint16_t ReductionRatio);
extern void MAL_Motor_AcPanasonic_SetDefaultLocation(uint32_t *pmpanasonic);
extern void MAL_Motor_AcPanasonic_SetCounter(uint32_t *pmpanasonic, int16_t counter);
extern void MAL_Motor_AcPanasonic_SetLoadAbsoCnt(uint32_t *pmpanasonic, uint32_t loadAbsoCnt);//20201103

//get
extern uint8_t MAL_Motor_AcPanasonic_GetSettingFlag(uint32_t *pmpanasonic);
extern uint8_t MAL_Motor_AcPanasonic_GetAbsoStatusOk(uint32_t *pmpanasonic);
extern uint32_t MAL_Motor_AcPanasonic_GetAbsoCountOk(uint32_t *pmpanasonic);
extern uint8_t MAL_Motor_AcPanasonic_GetAbsoStatus(uint32_t *pmpanasonic);

//null
extern void MAL_Motor_AcPanasonic_NullFunction(uint32_t *pmpanasonic,uint16_t count, uint16_t rpm);


#endif /* INC_MAL_MOTOR_ACPANASONIC_H_ */
