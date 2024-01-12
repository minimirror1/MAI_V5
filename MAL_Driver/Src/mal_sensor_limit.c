/*
 * mal_sensor_limit.c
 *
 *  Created on: 2020. 3. 19.
 *      Author: shin
 */


#include "main.h"
#include "mal_motor.h"
#include "mal_sensor_limit.h"



MAL_SENSOR_LimitManagerHandleTypeDef msensorManger = {0,};


void MAL_SENSOR_LimitHandleInit(
		MAL_SENSOR_Limit_HandleTypeDef *msensor,
		uint8_t axleId,
		uint8_t senId,
		GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin,
		MAL_SENSOR_LimitDirection direction,
		GPIO_PinState 	detectionState);
void MAL_SENSOR_LimitAddrRegist(MAL_SENSOR_Limit_HandleTypeDef *msensor);
void MAL_SENSOR_LimitRefresh(void);
void MAL_SENSOR_LimitRead(MAL_SENSOR_Limit_HandleTypeDef *msensor);
void MAL_SENSOR_LimitTrigger(MAL_SENSOR_Limit_HandleTypeDef *msensor);

void MAL_SENSOR_LimitHandleInit(
		MAL_SENSOR_Limit_HandleTypeDef *msensor,
		uint8_t axleId,
		uint8_t senId,
		GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin,
		MAL_SENSOR_LimitDirection direction,
		GPIO_PinState 	detectionState)
{

	msensor->port.GPIOx = GPIOx;
	msensor->port.GPIO_Pin = GPIO_Pin;

	msensor->setting.id.axle = axleId;
	msensor->setting.id.sen = senId;

	msensor->setting.direction = direction;
	msensor->setting.detectionState = detectionState;

	MAL_SENSOR_LimitAddrRegist(msensor);
}

//센서 헨들을 리스트에 등록한다
void MAL_SENSOR_LimitAddrRegist(MAL_SENSOR_Limit_HandleTypeDef *msensor)
{
	msensorManger.addrRegist.list[msensorManger.addrRegist.registCnt].pmsensor = msensor;
	msensorManger.addrRegist.registCnt++;
	if(msensorManger.addrRegist.registCnt >= SENSOR_ADDR_REGIST_SIZE)
	{
		Error_Handler();
	}

}

void MAL_SENSOR_LimitRefresh(void)
{
	uint8_t cntTemp = msensorManger.addrRegist.registCnt;
	volatile uint8_t i = 0;
	for(i = 0; i < cntTemp; i++)
	{
		MAL_SENSOR_LimitRead (msensorManger.addrRegist.list[i].pmsensor);
		MAL_SENSOR_LimitTrigger (msensorManger.addrRegist.list[i].pmsensor);
	}
}


void MAL_SENSOR_LimitRead(MAL_SENSOR_Limit_HandleTypeDef *msensor)
{
	 if(HAL_GPIO_ReadPin(msensor->port.GPIOx, msensor->port.GPIO_Pin) == msensor->setting.detectionState)
		 msensor->status.detect = MAL_SENSOR_SET;
	 else
		 msensor->status.detect = MAL_SENSOR_RESET;
}

void MAL_SENSOR_LimitTrigger(MAL_SENSOR_Limit_HandleTypeDef *msensor)
{
	msensor->status.new = msensor->status.detect;

	if(msensor->status.new != msensor->status.old)
	{
		msensor->status.old = msensor->status.new;

		msensor->status.f_newEvent = SET;


		MAL_Protocol_Ani_EventSensorDetect(&msensor->setting.id, (uint16_t)msensor->status.new);
	}
}

MAL_SENSOR_LimitDetectionStatus MAL_SENSOR_GetDetection(MAL_SENSOR_Limit_HandleTypeDef *msensor)
{
	 if(HAL_GPIO_ReadPin(msensor->port.GPIOx, msensor->port.GPIO_Pin) == msensor->setting.detectionState)
		 return MAL_SENSOR_SET;
	 else
		 return MAL_SENSOR_RESET;
}

