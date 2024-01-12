/*
 * mal_motor_bldcMd.h
 *
 *  Created on: Mar 11, 2020
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_BLDCMD_H_
#define INC_MAL_MOTOR_BLDCMD_H_

#include "mal_uart.h"

#include "mal_sensor_limit.h"







typedef enum __MAL_MOTOR_BLDCMDLocationTypeDef
{
	MAL_BLDCMD_CW,
	MAL_BLDCMD_CCW
}MAL_MOTOR_BLDCMDLocationTypeDef;

typedef enum __MAL_MOTOR_BLDCMDTimStatusTypeDef
{
	MAL_BLDCMD_STOP,
	MAL_BLDCMD_RUN
}MAL_MOTOR_BLDCMDTimStatusTypeDef;




typedef struct __MAL_MOTOR_BLDCMDPositionTypeDef
{
	int32_t now;
	int32_t target;
}MAL_MOTOR_BLDCMDPositionTypeDef;

typedef struct __MAL_MOTOR_BLDCMDStatusTypeDef
{

	MAL_MOTOR_BLDCMDPositionTypeDef position;
	uint8_t axleNum;	//보드내부 축 번호
}MAL_MOTOR_BLDCMDStatusTypeDef;


typedef struct __MAL_MOTOR_BLDCMDControlTypeDef
{
	MAL_MOTOR_BLDCMDLocationTypeDef location; //회전방향
	MAL_MOTOR_BLDCMDTimStatusTypeDef runStatus; //DMA 출력 동작 플래그

	uint32_t calcCount;
	uint32_t setCount; //DMA 출력 카운트 =< 10000

}MAL_MOTOR_BLDCMDControlTypeDef;

typedef struct __MAL_MOTOR_BLDCMDSettingTypeDef
{
	uint8_t SensorDirection; 	//센서방향
	uint16_t OppositeLimit;		//반대편 한계 각도(카운터로 변환해야됨)
	uint16_t DefaultLocation;	//초기화 후 위치(초기위치)
	uint16_t ReductionRatio; 	//감속기 감속비
	MAL_MOTOR_SensorInit_TypeDef flag;				//세팅 여부 flag

	uint32_t t_emergency;

	uint8_t f_infoRev;		//설정정보 수신 플래그

	uint32_t initStCnt;//초기화 단계
	uint32_t count360;     //bldc 모터 감속끝단 360 카운터
	uint32_t countRatio;
	uint16_t Rpm;				//모터 구동 속도;

	float reductionPuls;	//감속후 1도 움직임에 필요한 초당 펄스수
	int32_t jogCount; 		//조그에 사용될 1ms 당 펄스수( 반올림)
	uint32_t t_jog;

	int32_t setPosiVal;	//센서 체크용 포지션

	uint8_t f_setStop;// 멈춤 채크 플래그
	uint32_t t_setPosi;//멈춤 0위치 설정

	uint32_t t_senRetry;// 센서 찍기 재시도

}MAL_MOTOR_BLDCMDSettingTypeDef;

typedef struct __MAL_MOTOR_BLDCMDSettingConv_TypeDef {
	int32_t offset;
	int32_t cwLim;
	int32_t ccwLim;
	int32_t range;
}MAL_MOTOR_BLDCMDSettingConv_TypeDef;


typedef struct __MAL_MOTOR_BLDCMDJogConstantSpeed_TypeDef {
	uint8_t activeflag;
	int32_t jogCnt;
	uint32_t tJogMove;
}MAL_MOTOR_BLDCMDJogConstantSpeed_TypeDef;


typedef struct __MAL_MOTOR_BLDCMDHandleTypeDef
{
	MAL_UART_HandleTypeDef *muart;

	MAL_SENSOR_Limit_HandleTypeDef *cwSen;
	MAL_SENSOR_Limit_HandleTypeDef *ccwSen;

	MAL_MOTOR_BLDCMDStatusTypeDef status;

	MAL_MOTOR_BLDCMDControlTypeDef control;

	MAL_MOTOR_BLDCMDSettingTypeDef setting;

	MAL_MOTOR_BLDCMDSettingConv_TypeDef settingConv;

	MAL_MOTOR_BLDCMDJogConstantSpeed_TypeDef jogCtr;

}MAL_MOTOR_BLDCMDHandleTypeDef;



//-------------------------------------------------------------------------------------------------------


extern void MAL_Motor_BldcMd_Process(void);


extern void MAL_Motor_BldcMd_UartReg(MAL_UART_HandleTypeDef *muart);
extern void MAL_Motor_BldcMd_SensorLimRegInit(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd,MAL_SENSOR_Limit_HandleTypeDef *cwSen,MAL_SENSOR_Limit_HandleTypeDef *ccwSen);


extern void MAL_Motor_BldcMd_SetBldcInfo(uint32_t *mbldcMd, uint16_t count, uint16_t rpm);

extern void MAL_Motor_BldcMd_SetDefaultLocation(uint32_t *mbldcMd);
extern void MAL_Motor_BldcMd_SetSettingVal(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation,
		uint16_t ReductionRatio);
extern void MAL_Motor_BldcMd_SetLocation(uint32_t *mbldcMd, uint16_t location);
extern void MAL_Motor_BldcMd_SetSetting(
		uint32_t *mbldcMd,
		uint8_t SensorDirection,
		uint16_t OppositeLimit,
		uint16_t DefaultLocation,
		uint16_t ReductionRatio);

extern void MAL_Motor_BldcMd_SetJogCounter(uint32_t *mbldcMd, int16_t counter);

extern void MAL_Motor_BldcMd_PID_INC_POSI_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t targetPosition, int16_t rpm);
extern void MAL_Motor_BldcMd_PID_POSI_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t targetPosition, int16_t setRpm);



extern uint8_t MAL_Motor_BldcMd_GetSettingFlag(uint32_t *mbldcMd);


#endif /* INC_MAL_MOTOR_BLDCMD_H_ */
