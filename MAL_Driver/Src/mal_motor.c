/*
 * mal_motor.c
 *
 *  Created on: Mar 11, 2020
 *      Author: shin
 */

#include "main.h"
#include "mal_board_info.h"
#include "mal_motor.h"
#include "mal_motor_acPanasonic.h"
#include "mal_motor_bldcMd.h"

#include "app_pid_init_cmd.h"

#ifdef HAL_MOTOR_MODULE_ENABLED
extern MAL_MOTOR_HandleTypeDef mmotor[MOTOR_AXLE_CNT];
extern MAL_MOTOR_PanasonicHandleTypeDef mpanasonic;//210413

extern MAL_SENSOR_Limit_HandleTypeDef sensor[4];
MAL_MOTOR_ManagerHandleTypeDef motorManager;


extern my_can_id_data_t my_can_id_data;

void MAL_Motor_CallBackInit(MAL_MOTOR_HandleTypeDef *mmotor, uint32_t *ctrHandle, MAL_MOTOR_TypeTypeDef motorType);

/*===================================================================
 * 모터 초기화
 ===================================================================*/
#if 0  //init.c 양식
void MAL_Motor_Init(void)
{
	//모터 구조체 초기화
	//MAL_Motor_CallBackInit(&mmotor[0], MAL_MOTOR_AC_PANASONIC );

	MAL_Motor_CallBackInit(&mmotor[0], (uint32_t *)&muart4, MAL_MOTOR_BLDC_MD );

}
#endif

void MAL_Motor_CallBackInit(MAL_MOTOR_HandleTypeDef *mmotor, uint32_t *ctrHandle, MAL_MOTOR_TypeTypeDef motorType)
{
	mmotor->ctrHandle = ctrHandle;

	switch (motorType)
	{
		case MAL_MOTOR_BLDC_MD:
#ifdef HAL_MOTOR_MLDC_MD_MODULE_ENABLED
			mmotor->mal_motor_setLocationCallBack = MAL_Motor_BldcMd_SetLocation;
			mmotor->mal_motor_setBldcInfoCallBack = MAL_Motor_BldcMd_SetBldcInfo;
			mmotor->mal_motor_setSettingCallBack = MAL_Motor_BldcMd_SetSetting;
			mmotor->mal_motor_getSettingFlagCallBack = MAL_Motor_BldcMd_GetSettingFlag;
			mmotor->mal_motor_setDefaultLocationCallBack = MAL_Motor_BldcMd_SetDefaultLocation;

			mmotor->mal_motor_setJogCounter = MAL_Motor_BldcMd_SetJogCounter;

#endif
			break;
		case MAL_MOTOR_AC_PANASONIC:
#ifdef HAL_MOTOR_AC_MODULE_ENABLED
		mmotor->mal_motor_setLocationCallBack = MAL_Motor_AcPanasonic_SetLocation;
		mmotor->mal_motor_setBldcInfoCallBack = MAL_Motor_AcPanasonic_NullFunction;//빈함수
		mmotor->mal_motor_setSettingCallBack = MAL_Motor_AcPanasonic_SetSetting;
		mmotor->mal_motor_setSetting_AbsoluteCallBack = MAL_Motor_AcPanasonic_SetSetting_Absolute;
		mmotor->mal_motor_getSettingFlagCallBack = MAL_Motor_AcPanasonic_GetSettingFlag;
		mmotor->mal_motor_setDefaultLocationCallBack = MAL_Motor_AcPanasonic_SetDefaultLocation;
		mmotor->mal_motor_setJogCounter = MAL_Motor_AcPanasonic_SetCounter;

		mmotor->mal_motor_setLoadAbsoCnt = MAL_Motor_AcPanasonic_SetLoadAbsoCnt;//20201103
		mmotor->mal_motor_getAbsoStatusCallBack = MAL_Motor_AcPanasonic_GetAbsoStatus;//20201103
		mmotor->mal_motor_getAbsoStatusOkCallBack = MAL_Motor_AcPanasonic_GetAbsoStatusOk;//
		mmotor->mal_motor_getAbsoCountOkCallBack = MAL_Motor_AcPanasonic_GetAbsoCountOk;
#endif
			break;
	}
}

void MAL_Motor_ManagerInit(uint32_t *commHandle)
{
	motorManager.mcan = (MAL_CAN_HandleTypeDef*) commHandle;
}

/*===================================================================
 * 모터 제어
 ===================================================================*/

void MAL_Motor_SetAllLocation(uint16_t location)
{
	uint8_t i = 0;

	for (i = 0; i < MOTOR_AXLE_CNT; i++)
	{
		mmotor[i].mal_motor_setLocationCallBack(mmotor[i].ctrHandle, location);
	}
}
void app_rx_init_sub_pid_move_sensor_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
		return;

	MAL_Motor_AcPanasonic_StartInit(mmotor[0].ctrHandle);

	app_tx_init_sub_pid_move_sensor_rsp(
				num,
				0,
				my_can_id_data.id,
				MASTER_CAN_ID,
				my_can_id_data.sub_id[0],
				0,
				0);
}

//=============================================================================================================
/*void MAL_Motor_SetLocation(uint8_t axleId, uint16_t location)
{
	if (MOTOR_AXLE_CNT <= axleId) return;
	//xxx 변경해야함
	mmotor[axleId].mal_motor_setLocationCallBack(mmotor[axleId].ctrHandle, location);
}*/
void app_rx_motion_sub_pid_adc_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_motion_adc_t *pData)
{
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	uint16_t location;

	prtc_data_ctl_motion_adc_t *temp = (prtc_data_ctl_motion_adc_t *)pData;

	location = (uint16_t)temp->adc_val;

	MAL_Motor_AcPanasonic_SetLocation(mmotor[0].ctrHandle, location);
}
//=============================================================================================================

/*void MAL_Motor_SetBldcInfo(uint8_t axleId, uint16_t count, uint16_t rpm)
{
	if (MOTOR_AXLE_CNT <= axleId) return;
	mmotor[axleId].mal_motor_setBldcInfoCallBack(mmotor[axleId].ctrHandle, count, rpm);

	motorManager.senInitFlag[axleId] = SET;
}*/


//=============================================================================================================
/*void MAL_Motor_SetSetting(uint8_t axleId, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio)
{
	if (MOTOR_AXLE_CNT <= axleId) return;
	mmotor[axleId].mal_motor_setSettingCallBack(mmotor[axleId].ctrHandle, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);

	motorManager.senInitFlag[axleId] = SET;
}*/

void app_rx_init_sub_pid_driver_data1_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_driver_data1_t *pData)
{
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	uint8_t SensorDirection;
	uint16_t OppositeLimit;
	uint16_t DefaultLocation;
	uint16_t ReductionRatio;

	prtc_data_ctl_init_driver_data1_t *temp = (prtc_data_ctl_init_driver_data1_t *)pData;

	SensorDirection = (uint8_t)temp->direction;
	OppositeLimit = (uint16_t)temp->angle;
	DefaultLocation = (uint16_t)temp->init_position;
	ReductionRatio = (uint16_t)temp->reducer_ratio;

	//제거 2022.04.18
	//home sensor 고정
//	if(SensorDirection == MAL_SENSOR_CW)
//	{
//		MAL_Motor_AcPanasonic_SensorLimRegInit(
//					&mpanasonic,
//					&sensor[1],
//					&sensor[0]);
//	}

	//MAL_Motor_AcPanasonic_SetSetting(mmotor[0].ctrHandle, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);
	MAL_Motor_AcPanasonic_SetSetting_Absolute(mmotor[0].ctrHandle, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);

	app_tx_init_sub_pid_driver_data1_rsp(
			num,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			SensorDirection,
			OppositeLimit,
			DefaultLocation,
			ReductionRatio);
}
//=============================================================================================================

//=============================================================================================================
/*uint8_t MAL_Motor_SetSetting_Absolute(uint8_t axleId, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio)
{
	if (MOTOR_AXLE_CNT <= axleId) return 0;
	return mmotor->mal_motor_setSetting_AbsoluteCallBack(mmotor[axleId].ctrHandle, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);

}*/

void app_rx_init_sub_pid_absolute_battery_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_absolute_battery_t *pData)
{
	uint32_t absoData = 0;
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	prtc_data_ctl_init_absolute_battery_t *temp = (prtc_data_ctl_init_absolute_battery_t *)pData;
	absoData = (uint32_t)temp->save_data;

	MAL_Motor_AcPanasonic_SetLoadAbsoCnt(mmotor[0].ctrHandle, absoData);

	app_tx_init_sub_pid_absolute_battery_rsp(
			num,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			absoData);

}

//=============================================================================================================
/*void MAL_Motor_SetDefaultLocation(uint8_t axleId)
{
	if (MOTOR_AXLE_CNT <= axleId) return;
	mmotor[axleId].mal_motor_setDefaultLocationCallBack(mmotor[axleId].ctrHandle);
}*/
void app_rx_init_sub_pid_move_init_position_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	MAL_Motor_AcPanasonic_SetDefaultLocation(mmotor[0].ctrHandle);
/*	app_tx_init_sub_pid_move_sensor_rsp(
			0,
			MAL_Board_ID_GetValue(),
			MASTER_CAN_ID,
			axleId);*/

	app_tx_init_sub_pid_move_init_position_rsp(
			num,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0);
}
void app_rx_init_sub_pid_move_init_position_rqt(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	app_tx_init_sub_pid_move_init_position_rsp(
			num,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0);
}
void app_rx_motion_sub_pid_direction_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_motion_direction_t *pData)
{
	/*void MAL_Motor_SetJogCounter(uint8_t axleId, int16_t counter)
	{
		if (MOTOR_AXLE_CNT <= axleId) return;
		mmotor[0].mal_motor_setJogCounter(mmotor[0].ctrHandle, counter);
	}*/
	int16_t counter;
	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	prtc_data_ctl_motion_direction_t *temp = (prtc_data_ctl_motion_direction_t *)pData;

	if(temp->direction == MOTION_DIRECTION_CCW)
		counter = -temp->val;
	else
	{
		counter = temp->val;
	}


	mmotor[0].mal_motor_setJogCounter(mmotor[0].ctrHandle, counter);
}
//=============================================================================================================





void MAL_Motor_SetAxleEnable(uint8_t axleId, uint8_t flag)
{
	//if (MOTOR_AXLE_CNT <= axleId) return;
	if(my_can_id_data.sub_id[0] != axleId)
		return;


	mmotor[0].axleActive = flag;
}

//20201103
void MAL_Motor_SetLoadAbsoCnt(uint8_t axleId, uint32_t loadAbsoCnt)
{
	//if (MOTOR_AXLE_CNT <= axleId) return;
	if(my_can_id_data.sub_id[0] != axleId)
			return;

	mmotor[0].mal_motor_setLoadAbsoCnt(mmotor[0].ctrHandle, loadAbsoCnt);
}
/*===================================================================
 * 모터 상태
 ===================================================================*/
uint8_t MAL_Motor_GetSettingFlag(uint8_t axleId)
{
//	if (MOTOR_AXLE_CNT <= axleId) return 0;
	if(my_can_id_data.sub_id[0] != axleId)
			return MAL_SEN_DEINIT;

	uint8_t ret = mmotor[0].mal_motor_getSettingFlagCallBack(mmotor[0].ctrHandle);
	return ret;
}

uint8_t MAL_Motor_GetAxleNum(void)
{
	return MOTOR_AXLE_CNT;
}

uint8_t MAL_Motor_GetSensorInitRecvFlag(uint8_t axleId)
{
	//return motorManager.senInitFlag[axleId];
	return motorManager.senInitFlag[0];
}

//20201103
//=============================================================================================================
/*
uint8_t MAL_Motor_GetAbsoStatus(uint8_t axleId)
{
	if (MOTOR_AXLE_CNT <= axleId) return 0;
	uint8_t ret = mmotor[axleId].mal_motor_getAbsoStatusCallBack(mmotor[axleId].ctrHandle);
	return ret;
}
*/


//=============================================================================================================

uint8_t MAL_Motor_GetAbsoStatusOk(uint8_t axleId)
{
	//if (MOTOR_AXLE_CNT <= axleId) return 0;
	if(my_can_id_data.sub_id[0] != axleId)
			return MAL_SEN_DEINIT;
	uint8_t ret = mmotor[0].mal_motor_getAbsoStatusOkCallBack(mmotor[0].ctrHandle);
	return ret;
}

uint32_t MAL_Motor_GetAbsoCountOk(uint8_t axleId)
{
	//if (MOTOR_AXLE_CNT <= axleId) return 0;
	if(my_can_id_data.sub_id[0] != axleId)
			return MAL_SEN_DEINIT;
	uint32_t ret = mmotor[0].mal_motor_getAbsoCountOkCallBack(mmotor[0].ctrHandle);
	return ret;
}
void app_rx_init_sub_pid_absolute_battery_rqt(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{

	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	app_tx_init_sub_pid_absolute_battery_rsp(
			num,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			//MAL_Motor_GetAbsoCountOk(my_can_id_data.sub_id[0]));
			MAL_Motor_GetAbsoCountOk(my_can_id_data.sub_id[0]));//230307 보드 아이디 검사.
}

// event
void MAL_Protocol_Ani_EventBootAlm(void)
{
	app_tx_init_sub_pid_boot_ctl(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0);
}

void MAL_Protocol_Ani_AlmSensorDetection(uint8_t axleId, uint8_t cwSen, uint8_t ccwSen)
{

	app_tx_sensor_sub_pid_detect_ctl(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			cwSen,
			ccwSen);

}
//==========================================================================================================
void MAL_Protocol_Ani_RspSensorInitSuccess(uint8_t axleId,int32_t absoCnt)
{
	app_tx_init_sub_pid_absolute_battery_ctl(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			absoCnt
			);
}
//==========================================================================================================
//1.5
void MAL_Protocol_Ani_RspAcAbsoBatteryOk(uint8_t axleId)
{
	app_tx_init_sub_pid_status_rsp(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			ABSOLUTE_BATTERY,
			MAL_Motor_AcPanasonic_GetAbsoStatusOk(mmotor[axleId].ctrHandle)
			);
}

void MAL_Protocol_Ani_RspDefPosi(uint8_t axleId, uint8_t initFlag)
{
	app_tx_init_sub_pid_status_ctl(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			MOVE_INIT_POSITION,
			initFlag);
}

void MAL_Protocol_Ani_EventSensorDetect(MAL_SENSOR_LimitIDTypeDef *axleId, uint16_t value)
{
	app_tx_sensor_sub_pid_detect_ctl(
			0,
			0,
			my_can_id_data.id,
			MASTER_CAN_ID,
			my_can_id_data.sub_id[0],
			0,
			MAL_SENSOR_GetDetection(mpanasonic.ccwSen),
			MAL_SENSOR_GetDetection(mpanasonic.cwSen)
			//MAL_SENSOR_GetDetection(mpanasonic.cwSen),
			//MAL_SENSOR_GetDetection(mpanasonic.ccwSen)
			);
}

//==========================================================================================================

//
void app_rx_init_sub_pid_status_rqt(uint8_t num, prtc_header_t *pPh, prtc_data_rqt_init_status_t *pData)
{

	uint8_t status = 0;

	if((my_can_id_data.id != pPh->target_id)||(my_can_id_data.sub_id[0] != pPh->target_sub_id))
			return;

	prtc_data_rqt_init_status_t *temp = (prtc_data_rqt_init_status_t *) pData;

	switch(temp->step)
	{
	//init step 1 : 1. vattery check
	case ABSOLUTE_BATTERY:
		app_tx_init_sub_pid_status_rsp(
				num,
				0,
				my_can_id_data.id,
				MASTER_CAN_ID,
				my_can_id_data.sub_id[0],
				0,
				ABSOLUTE_BATTERY,
				MAL_Motor_AcPanasonic_GetAbsoStatus(mmotor[0].ctrHandle));
		break;
	case DRIVER_DATA1:

		break;
	case DRIVER_DATA2:

		break;
	case MOVE_SENSOR:

		status = MAL_Motor_AcPanasonic_GetSettingFlag(mmotor[0].ctrHandle);
		if(status == MAL_SEN_INIT_OK)
			status = 1;//ok
		else if(status == MAL_SEN_DEINIT)
			status = 2;//error
		else if(status == MAL_SEN_EMERGENCY_STOP)
			status = 2;//error
		else
			status = 0;//wait

		app_tx_init_sub_pid_status_rsp(
						num,
						0,
						my_can_id_data.id,
						MASTER_CAN_ID,
						my_can_id_data.sub_id[0],
						0,
						MOVE_SENSOR,
						status);

		break;
	case MOVE_INIT_POSITION:

		status = MAL_Motor_AcPanasonic_GetSettingFlag(mmotor[0].ctrHandle);
		if(status == MAL_SEN_INIT_OK)
			status = 1;//ok
		else if(status == MAL_SEN_DEINIT)
			status = 2;//error
		else if(status == MAL_SEN_EMERGENCY_STOP)
			status = 2;//error
		else
			status = 0;//wait

		//idtest
		app_tx_init_sub_pid_status_rsp(
						num,
						0,
						my_can_id_data.id,
						MASTER_CAN_ID,
						my_can_id_data.sub_id[0],
						0,
						MOVE_INIT_POSITION,
						status);

		break;
	}
}
#endif

