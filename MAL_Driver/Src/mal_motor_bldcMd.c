/*
 * mal_motor_bldcMd.c
 *
 *  Created on: Mar 11, 2020
 *      Author: shin
 */

#include "main.h"
#include "mal_motor.h"
#include "mal_motor_bldcMd.h"
#include "mal_motor_bldcMd_Protocol.h"

#include "mal_sensor_limit.h"
#include "mal_systimer.h"
#include "math.h"

#ifdef HAL_MOTOR_MLDC_MD_MODULE_ENABLED

MAL_MOTOR_BLDCMDHandleTypeDef mbldcMd[2] =
{ 0, };
void MAL_Motor_BldcMd_ProcessSensorAlm(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd);
void MAL_Motor_BldcMd_ProcessSensorInit(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd);

void MAL_Motor_BldcMd_SetPosi(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd);

void MAL_Motor_BldcMd_PID_POSI_SET(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t setPosition);
void MAL_Motor_BldcMd_PID_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t setRpm);

/*===================================================================
 * md init
 ===================================================================*/
void MAL_Motor_BldcMd_UartReg(MAL_UART_HandleTypeDef *muart)
{
	static uint8_t regCnt = 0;
	static uint8_t regNum = 0;

	if (regCnt >= 2) return;

	mbldcMd[regCnt].muart = muart;
	mbldcMd[regCnt].status.axleNum = regNum; //등록된 축 수

	regCnt++;
	regNum++;

	if (regCnt >= 2)
	{
		regCnt = 1;
	}
}

void MAL_Motor_BldcMd_SensorLimRegInit(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, MAL_SENSOR_Limit_HandleTypeDef *cwSen, MAL_SENSOR_Limit_HandleTypeDef *ccwSen)
{
	mbldcMd->cwSen = cwSen;
	mbldcMd->ccwSen = ccwSen;
}

/*===================================================================
 * md process
 ===================================================================*/

void MAL_Motor_BldcMd_Process(void)
{
	MAL_Motor_BldcMd_ProcessSensorInit(&mbldcMd[0]);
	MAL_Motor_BldcMd_ProcessSensorInit(&mbldcMd[1]);

	MAL_Motor_BldcMd_ProcessSensorAlm(&mbldcMd[0]);
	MAL_Motor_BldcMd_ProcessSensorAlm(&mbldcMd[1]);
}

void MAL_Motor_BldcMd_ProcessSensorAlm(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd)
{

	if(mbldcMd->setting.flag == MAL_SEN_EMERGENCY_STOP)
	{
		if(MAL_SysTimer_Elapsed(mbldcMd->setting.t_emergency)>=500)
		{
			mbldcMd->setting.t_emergency = MAL_SysTimer_GetTickCount();
			MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
			MAL_Protocol_Ani_AlmSensorDetection(mbldcMd->status.axleNum, MAL_SENSOR_GetDetection(mbldcMd->cwSen), MAL_SENSOR_GetDetection(mbldcMd->ccwSen));
		}
	}

	if (mbldcMd->setting.flag == MAL_SEN_INIT_OK)
	{

		if ((MAL_SENSOR_GetDetection(mbldcMd->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(mbldcMd->ccwSen) == MAL_SENSOR_SET))
		{
			if ((mbldcMd->cwSen->status.f_newEvent == SET) || (mbldcMd->ccwSen->status.f_newEvent == SET))
			{
				mbldcMd->cwSen->status.f_newEvent = RESET;
				mbldcMd->ccwSen->status.f_newEvent = RESET;

				mbldcMd->setting.flag = MAL_SEN_EMERGENCY_STOP;
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				MAL_Protocol_Ani_AlmSensorDetection(mbldcMd->status.axleNum, MAL_SENSOR_GetDetection(mbldcMd->cwSen), MAL_SENSOR_GetDetection(mbldcMd->ccwSen));
			}
		}
	}
}

int32_t MAL_Motor_BldcMd_CalcInitialPosition(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t digree)
{
	float ratio = 0;
	int32_t ret = 0;

	ratio = (float) mbldcMd->setting.count360 / 360; // 1도 카운터를 알아냄
	ret = ratio * (float) digree;

	return ret;

}

void MAL_Motor_BldcMd_ProcessSensorInit(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd)
{

	if (mbldcMd->setting.flag != MAL_SEN_INIT_ING) return;

	int32_t initRpm = mbldcMd->setting.Rpm;
	initRpm = initRpm / 100;

	if (mbldcMd->setting.f_setStop == SET)
	{
		if (MAL_SysTimer_Elapsed(mbldcMd->setting.t_setPosi) >= 100)
		{
			mbldcMd->setting.initStCnt++;
			mbldcMd->setting.t_setPosi = MAL_SysTimer_GetTickCount();

			switch (mbldcMd->setting.initStCnt)
			{
				case 1:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;

				case 2:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;
				case 3:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;
				case 4:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;

				case 5:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;
				case 6:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;
				case 7:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;

				case 35:
					if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, -5), initRpm * 10);
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, 5), initRpm * 10);
					}
					break;
				case 36:
					if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, -5), initRpm * 10);
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, 5), initRpm * 10);
					}
					break;
				case 37:
					if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, -5), initRpm * 10);
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, 5), initRpm * 10);
					}
					break;
				case 38:
					if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, -5), initRpm * 10);
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, 5), initRpm * 10);
					}
					break;

				case 270:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;
				case 271:
					MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
					break;

				case 360:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;
				case 361:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;
				case 362:
					MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);
					break;
				case 375:
					MAL_Motor_BldcMd_SetPosi(mbldcMd);
					MAL_Protocol_Ani_RspSensorInitSuccess(mbldcMd->status.axleNum,0);
					mbldcMd->setting.f_setStop = RESET;
					mbldcMd->setting.initStCnt = 0;
					return;
					break;
#if 0
			case 1:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;

			case 2:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;
			case 3:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;
			case 4:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;

			case 35:
				if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
				{
					MAL_Motor_BldcMd_PID_INC_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, -5), initRpm);
				}
				else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
				{
					MAL_Motor_BldcMd_PID_INC_POSI_VEL_CMD(mbldcMd, MAL_Motor_BldcMd_CalcInitialPosition(mbldcMd, 5), initRpm);
				}
				break;

			case 270:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;
			case 271:
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로
				break;

			case 360:
				MAL_Motor_BldcMd_SetPosi(mbldcMd);
				break;
			case 361:
				MAL_Motor_BldcMd_SetPosi(mbldcMd);
				break;
			case 362:
				MAL_Motor_BldcMd_SetPosi(mbldcMd);
				break;
			case 375:
				MAL_Motor_BldcMd_SetPosi(mbldcMd);
				MAL_Protocol_Ani_RspSensorInitSuccess(mbldcMd->status.axleNum,0);
				mbldcMd->setting.f_setStop = RESET;
				mbldcMd->setting.initStCnt = 0;
				return;
				break;
#endif
			}
			return;
		}

#if 0
		if (mbldcMd->setting.t_setPosi >= 1000)
		{
			MAL_Motor_BldcMd_SetPosi(mbldcMd);
			//191119
			MAL_Protocol_Ani_RspSensorInitSuccess(mbldcMd->status.axleNum,0);
			mbldcMd->setting.f_setStop = RESET;
		}
#endif
	}

	if (mbldcMd->setting.f_setStop == RESET)
	{
		if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
		{
			if (MAL_SENSOR_GetDetection(mbldcMd->cwSen) == MAL_SENSOR_SET)
			//if ((MAL_SENSOR_GetDetection(mbldcMd->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(mbldcMd->ccwSen) == MAL_SENSOR_SET))
			{
				//0.7
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로

				MAL_Protocol_Ani_RspSensorInitSuccess(mbldcMd->status.axleNum,0);

				mbldcMd->setting.f_setStop = SET;		//체크 세팅
				mbldcMd->setting.initStCnt = 0;
				mbldcMd->setting.t_setPosi = MAL_SysTimer_GetTickCount();		//시간세팅
			}
			else
			{
				if (MAL_SysTimer_Elapsed(mbldcMd->setting.t_senRetry) > 500)
				{
					mbldcMd->setting.t_senRetry = MAL_SysTimer_GetTickCount();
					if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, -initRpm); //속도
						//MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, -initRpm); //속도
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, initRpm); //속도
						//MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, initRpm); //속도
					}
				}
			}
		}
		else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
		{
			if (MAL_SENSOR_GetDetection(mbldcMd->ccwSen) == MAL_SENSOR_SET)
			//if ((MAL_SENSOR_GetDetection(mbldcMd->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(mbldcMd->ccwSen) == MAL_SENSOR_SET))
			{
				//0.7
				MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, 0); //속도를 0으로

				MAL_Protocol_Ani_RspSensorInitSuccess(mbldcMd->status.axleNum,0);

				mbldcMd->setting.f_setStop = SET;		//체크 세팅
				mbldcMd->setting.initStCnt = 0;
				mbldcMd->setting.t_setPosi = MAL_SysTimer_GetTickCount();		//시간세팅
			}
			else
			{
				if (MAL_SysTimer_Elapsed(mbldcMd->setting.t_senRetry) > 500)
				{
					mbldcMd->setting.t_senRetry = MAL_SysTimer_GetTickCount();
					if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
					{
						MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, -initRpm); //속도
						//MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, -initRpm); //속도
					}
					else if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
					{
						MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, initRpm); //속도
						//MAL_Motor_BldcMd_PID_VEL_CMD(mbldcMd, initRpm); //속도
					}
				}
			}
		}
	}
}
/*===================================================================
 * md utility
 ===================================================================*/
int32_t MAL_Motor_BldcMd_pwmConvRatio(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd)
{
	int32_t ret;

	float ratio;
	float temp;

	ratio = (float) mbldcMd->setting.DefaultLocation / 4095; //비율을 계산

	temp = (float) mbldcMd->settingConv.range * ratio;

	//temp = (float)ratio * 1075;

	ret = (int32_t) floor(temp + 0.5);

	//ret += AC[ch].SettingConv.offset;

	return ret;
}

uint32_t MAL_Motor_BldcMd_DegreeToCount(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd)
{
	uint32_t ret = 0;
	float temp = 0;
	float oppLim = 0;

	//AC[ch].Setting.ReductionRatio 감속비
	//AC[ch].Setting.OppositeLimit 한계각도

	oppLim = (float) mbldcMd->setting.OppositeLimit / 100;

	temp = (float)mbldcMd->setting.count360 / 360;

	temp *= oppLim;

	ret = (uint32_t) floor((temp) + 0.5);
	return ret;

}

void MAL_Motor_BldcMd_SetPosi(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd)
{
	MAL_Motor_BldcMd_PID_POSI_SET(mbldcMd, 0);

	if (mbldcMd->setting.SensorDirection == MAL_RO_CW)
	{
		mbldcMd->settingConv.cwLim = 0;
		mbldcMd->settingConv.ccwLim = -MAL_Motor_BldcMd_DegreeToCount(mbldcMd);
		mbldcMd->settingConv.range = mbldcMd->settingConv.ccwLim;
		mbldcMd->setting.flag = MAL_SEN_INIT_OK;
	}
	else if (mbldcMd->setting.SensorDirection == MAL_RO_CCW)
	{
		mbldcMd->settingConv.ccwLim = 0;
		mbldcMd->settingConv.cwLim = MAL_Motor_BldcMd_DegreeToCount(mbldcMd);
		mbldcMd->settingConv.range = mbldcMd->settingConv.cwLim;
		mbldcMd->setting.flag = MAL_SEN_INIT_OK;
	}
	else
	{
		while (1)
			; //error
	}

}

static uint8_t MAL_Motor_BldcMd_GetCheckSum(uint16_t nPacketSize, uint8_t *byArray)
{
	uint8_t byTmp = 0;
	uint16_t i;
	for (i = 0; i < nPacketSize; i++)
		byTmp += *(byArray + i);
	return (~byTmp + 1);
}
/*===================================================================
 * md protocol
 ===================================================================*/
void MAL_Motor_BldcMd_PID_INC_POSI_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t targetPosition, int16_t rpm)
{
	uint8_t pBuff[sizeof(MD_Header_TypeDef) + sizeof(MD_PID_INC_POSI_VEL_TypeDef) + sizeof(MD_CheckSum_TypeDef)] =
	{ 0, };

	MD_Header_TypeDef *hp = (MD_Header_TypeDef*) pBuff;
	MD_PID_INC_POSI_VEL_TypeDef *dp = (MD_PID_INC_POSI_VEL_TypeDef*) hp->payload;
	MD_CheckSum_TypeDef *cp = (MD_CheckSum_TypeDef*) dp->payload;

	hp->rmid = 0xB7;
	hp->tmid = 0xB8;
	hp->mo_id = 0x01;
	hp->pid = PID_INC_POSI_VEL_CMD;
	hp->len = sizeof(MD_PID_POSI_VEL_TypeDef);

	dp->targetPosition = targetPosition;
	dp->rpm = rpm;

	cp->chk = MAL_Motor_BldcMd_GetCheckSum(sizeof(pBuff) - sizeof(MD_CheckSum_TypeDef), pBuff);

	MAL_UART_SendDataStream(mbldcMd->muart, (uint8_t*) pBuff, sizeof(pBuff));
}

void MAL_Motor_BldcMd_PID_POSI_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t targetPosition, int16_t rpm)
{
	uint8_t pBuff[sizeof(MD_Header_TypeDef) + sizeof(MD_PID_POSI_VEL_TypeDef) + sizeof(MD_CheckSum_TypeDef)] =
	{ 0, };

	MD_Header_TypeDef *hp = (MD_Header_TypeDef*) pBuff;
	MD_PID_POSI_VEL_TypeDef *dp = (MD_PID_POSI_VEL_TypeDef*) hp->payload;
	MD_CheckSum_TypeDef *cp = (MD_CheckSum_TypeDef*) dp->payload;

	hp->rmid = 0xB7;
	hp->tmid = 0xB8;
	hp->mo_id = 0x01;
	hp->pid = PID_POSI_VEL_CMD;
	hp->len = sizeof(MD_PID_POSI_VEL_TypeDef);

	dp->targetPosition = targetPosition;
	dp->rpm = rpm;

	cp->chk = MAL_Motor_BldcMd_GetCheckSum(sizeof(pBuff) - sizeof(MD_CheckSum_TypeDef), pBuff);

	MAL_UART_SendDataStream(mbldcMd->muart, (uint8_t*) pBuff, sizeof(pBuff));
}

void MAL_Motor_BldcMd_PID_VEL_CMD(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t setRpm)
{
	uint8_t pBuff[sizeof(MD_Header_TypeDef) + sizeof(MD_PID_VEL_CMD_TypeDef) + sizeof(MD_CheckSum_TypeDef)] =
	{ 0, };

	MD_Header_TypeDef *hp = (MD_Header_TypeDef*) pBuff;
	MD_PID_VEL_CMD_TypeDef *dp = (MD_PID_VEL_CMD_TypeDef*) hp->payload;
	MD_CheckSum_TypeDef *cp = (MD_CheckSum_TypeDef*) dp->payload;

	hp->rmid = 0xB7;
	hp->tmid = 0xB8;
	hp->mo_id = 0x01;
	hp->pid = PID_VEL_CMD;
	hp->len = sizeof(MD_PID_VEL_CMD_TypeDef);

	dp->setRpm = setRpm;

	cp->chk = MAL_Motor_BldcMd_GetCheckSum(sizeof(pBuff) - sizeof(MD_CheckSum_TypeDef), pBuff);

	MAL_UART_SendDataStream(mbldcMd->muart, (uint8_t*) pBuff, sizeof(pBuff));
}

void MAL_Motor_BldcMd_PID_POSI_SET(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, int32_t setPosition)
{
	uint8_t pBuff[sizeof(MD_Header_TypeDef) + sizeof(MD_PID_POSI_SET_TypeDef) + sizeof(MD_CheckSum_TypeDef)] =
	{ 0, };

	MD_Header_TypeDef *hp = (MD_Header_TypeDef*) pBuff;
	MD_PID_POSI_SET_TypeDef *dp = (MD_PID_POSI_SET_TypeDef*) hp->payload;
	MD_CheckSum_TypeDef *cp = (MD_CheckSum_TypeDef*) dp->payload;

	hp->rmid = 0xB7;
	hp->tmid = 0xB8;
	hp->mo_id = 0x01;
	hp->pid = PID_POSI_SET;
	hp->len = sizeof(MD_PID_POSI_SET_TypeDef);

	dp->setPosition = setPosition;

	cp->chk = MAL_Motor_BldcMd_GetCheckSum(sizeof(pBuff) - sizeof(MD_CheckSum_TypeDef), pBuff);

	MAL_UART_SendDataStream(mbldcMd->muart, (uint8_t*) pBuff, sizeof(pBuff));
}

/*===================================================================
 * md callback
 ===================================================================*/
void MAL_Motor_BldcMd_SetBldcInfo(uint32_t *mbldcMd, uint16_t count, uint16_t rpm)
{
	MAL_MOTOR_BLDCMDHandleTypeDef *tempBldcMd = (MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd;

	//각도 수신전엔 리턴한다
	if (tempBldcMd->setting.f_infoRev != SET) return;

	tempBldcMd->setting.count360 = count;

	tempBldcMd->setting.countRatio = count;

	float CountRatio = (float) tempBldcMd->setting.OppositeLimit / 36000;
	tempBldcMd->setting.countRatio = tempBldcMd->setting.countRatio * CountRatio;

	tempBldcMd->setting.Rpm = rpm;

	// 0.0.4v 수정
	int32_t initRpm = rpm;
	initRpm = initRpm / 100;

	if (tempBldcMd->setting.SensorDirection == MAL_RO_CCW)
	{
		if (MAL_SENSOR_GetDetection(tempBldcMd->ccwSen) == MAL_SENSOR_SET)
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, 0); //속도를 0으로
		}
		else
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, -initRpm); //속도
			//MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, -initRpm); //속도
		}

	}
	else if (tempBldcMd->setting.SensorDirection == MAL_RO_CW)
	{
		if (MAL_SENSOR_GetDetection(tempBldcMd->cwSen) == MAL_SENSOR_SET)
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, 0); //속도를 0으로
		}
		else
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, initRpm); //속도
			//MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, initRpm); //속도
		}
	}

	tempBldcMd->setting.f_setStop = RESET;
	tempBldcMd->setting.flag = MAL_SEN_INIT_ING;

	MAL_Protocol_Ani_RspSensorInit(tempBldcMd->status.axleNum);

#if 0
	if ((MAL_SENSOR_GetDetection(tempBldcMd->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(tempBldcMd->ccwSen) == MAL_SENSOR_SET))
	{
		//0.7
		MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, 0); //속도를 0으로
//xxx 센서 찍혔을때 초기화
	}
	else
	{
		int32_t initRpm = rpm;
		initRpm = initRpm/20;

		//센서가 감지되어 있지 않으면 초기화 진행
		if (tempBldcMd->setting.SensorDirection == MAL_RO_CCW)
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, -initRpm); //속도
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, -initRpm); //속도
		}
		else if (tempBldcMd->setting.SensorDirection == MAL_RO_CW)
		{
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, initRpm); //속도
			MAL_Motor_BldcMd_PID_VEL_CMD(tempBldcMd, initRpm); //속도
		}

	}

	tempBldcMd->setting.f_setStop = RESET;
	tempBldcMd->setting.flag = MAL_SEN_INIT_ING;

	MAL_Protocol_Ani_RspSensorInit(tempBldcMd->status.axleNum);
#endif

}

void MAL_Motor_BldcMd_SetSettingVal(MAL_MOTOR_BLDCMDHandleTypeDef *mbldcMd, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation,
		uint16_t ReductionRatio)
{
	mbldcMd->setting.f_infoRev = SET;

	mbldcMd->setting.reductionPuls = 0;

	mbldcMd->setting.SensorDirection = SensorDirection;
	mbldcMd->setting.OppositeLimit = OppositeLimit;
	mbldcMd->setting.DefaultLocation = DefaultLocation;
	mbldcMd->setting.ReductionRatio = ReductionRatio;

	mbldcMd->setting.flag = MAL_SEN_INIT_INGWAIT;

	//mbldcMd->setting.reductionPuls = (float) 27.777777 * mbldcMd->setting.ReductionRatio;

//	//각도 설정
//	float CountRatio = (float)OppositeLimit / 36000;
//	mbldcMd->setting.count360 = mbldcMd->setting.count360 * CountRatio;
#ifdef IWDG_INIT
	HAL_IWDG_Refresh(&hiwdg);
#endif

}

//-------------------------------------------------------------------
void MAL_Motor_BldcMd_SetLocation(uint32_t *mbldcMd, uint16_t location)
{


	MAL_MOTOR_BLDCMDHandleTypeDef *tempBldcMd = (MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd;
	if (tempBldcMd->setting.flag != MAL_SEN_INIT_OK)
			return;

	float ratio = (float) location / 4095;

	int convLocation = tempBldcMd->setting.countRatio * ratio;

	if (tempBldcMd->setting.SensorDirection == MAL_RO_CW)
	{
		convLocation = -convLocation;
	}

	MAL_Motor_BldcMd_PID_POSI_VEL_CMD((MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd, convLocation, tempBldcMd->setting.Rpm);
}

void MAL_Motor_BldcMd_SetDefaultLocation(uint32_t *mbldcMd)
{

	MAL_MOTOR_BLDCMDHandleTypeDef *temp = (MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd;

	MAL_Motor_BldcMd_PID_POSI_VEL_CMD((MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd, MAL_Motor_BldcMd_pwmConvRatio((MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd), (temp->setting.Rpm/50));
	MAL_Protocol_Ani_RspDefPosi(temp->status.axleNum, MAL_SEN_INIT_OK);
	MAL_Protocol_Ani_RspDefPosi(temp->status.axleNum, MAL_SEN_INIT_OK);
}

void MAL_Motor_BldcMd_SetSetting(uint32_t *mbldcMd, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio)
{
	MAL_Motor_BldcMd_SetSettingVal((MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);
}

void MAL_Motor_BldcMd_SetJogCounter(uint32_t *mbldcMd, int16_t counter)
{
	MAL_MOTOR_BLDCMDHandleTypeDef *temp = (MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd;

	MAL_Motor_BldcMd_PID_INC_POSI_VEL_CMD((MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd, counter, temp->setting.Rpm / 10);
}
//get

uint8_t MAL_Motor_BldcMd_GetSettingFlag(uint32_t *mbldcMd)
{
	MAL_MOTOR_BLDCMDHandleTypeDef *ptemp = (MAL_MOTOR_BLDCMDHandleTypeDef*) mbldcMd;

	return ptemp->setting.flag;
}

#endif
