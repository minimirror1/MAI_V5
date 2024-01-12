/*
 * mal_sensor_limit.h
 *
 *  Created on: 2020. 3. 19.
 *      Author: shin
 */

#ifndef INC_MAL_SENSOR_LIMIT_H_
#define INC_MAL_SENSOR_LIMIT_H_


#define SENSOR_ADDR_REGIST_SIZE 10

//xxx 센서 감지시 콜백 함수 추가

typedef enum __MAL_SENSOR_LimitDetectionStatus
{
	MAL_SENSOR_RESET,
	MAL_SENSOR_SET
}MAL_SENSOR_LimitDetectionStatus;

typedef enum __MAL_SENSOR_LimitDirection
{
	MAL_SENSOR_CW,
	MAL_SENSOR_CCW
}MAL_SENSOR_LimitDirection;

typedef struct __MAL_SENSOR_LimitPortTypeDef
{
	GPIO_TypeDef* 	GPIOx;
	uint16_t 		GPIO_Pin;
}MAL_SENSOR_LimitPortTypeDef;

typedef struct __MAL_SENSOR_LimitIDTypeDef
{
	union{
		struct{
			uint8_t sen : 1;
			uint8_t axle : 7;
		};
		uint8_t sum;
	};
}MAL_SENSOR_LimitIDTypeDef;

typedef struct __MAL_SENSOR_LimitSettingTypeDef
{
	MAL_SENSOR_LimitDirection 		direction;		//모터 회전축 기준 센서 방향 cw, ccw
	GPIO_PinState 	detectionState;	//감지되었을때 핀 상태 SET, RESET
	MAL_SENSOR_LimitIDTypeDef id;
}MAL_SENSOR_LimitSettingTypeDef;

typedef struct __MAL_SENSOR_LimitStatusTypeDef
{
	MAL_SENSOR_LimitDetectionStatus detect;			//센서 감지 상태 SET,RESET

	MAL_SENSOR_LimitDetectionStatus new;
	MAL_SENSOR_LimitDetectionStatus old;

	uint8_t f_newEvent;
}MAL_SENSOR_LimitStatusTypeDef;

typedef struct __MAL_SENSOR_Limit_HandleTypeDef
{
	MAL_SENSOR_LimitPortTypeDef 		port;
	MAL_SENSOR_LimitSettingTypeDef 	setting;
	MAL_SENSOR_LimitStatusTypeDef 	status;
}MAL_SENSOR_Limit_HandleTypeDef;



/* sensor manager */
#pragma pack(1)
typedef struct __MAL_SENSOR_LimitRegistrationListTypeDef
{
	MAL_SENSOR_Limit_HandleTypeDef *pmsensor;
}MAL_SENSOR_LimitRegistrationListTypeDef;
#pragma pack()

#pragma pack(1)
typedef struct __MAL_SENSOR_LimitAddressRegistTypeDef
{
	MAL_SENSOR_LimitRegistrationListTypeDef list[SENSOR_ADDR_REGIST_SIZE];
	uint8_t registCnt;
}MAL_SENSOR_LimitAddressRegistTypeDef;
#pragma pack()

#pragma pack(1)
typedef struct __MAL_SENSOR_ManagerHandleTypeDef
{
	MAL_SENSOR_LimitAddressRegistTypeDef addrRegist;
}MAL_SENSOR_LimitManagerHandleTypeDef;
#pragma pack()
/* sensor manager */



extern void MAL_SENSOR_LimitHandleInit(
		MAL_SENSOR_Limit_HandleTypeDef *msensor,
		uint8_t axleId,
		uint8_t senId,
		GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin,
		MAL_SENSOR_LimitDirection direction,
		GPIO_PinState 	detectionState);

extern void MAL_SENSOR_LimitAddrRegist(MAL_SENSOR_Limit_HandleTypeDef *msensor);
extern void MAL_SENSOR_LimitRefresh(void);
extern void MAL_SENSOR_LimitRead(MAL_SENSOR_Limit_HandleTypeDef *msensor);


//get
extern MAL_SENSOR_LimitDetectionStatus MAL_SENSOR_GetDetection(MAL_SENSOR_Limit_HandleTypeDef *msensor);

#endif /* INC_MAL_SENSOR_LIMIT_H_ */
