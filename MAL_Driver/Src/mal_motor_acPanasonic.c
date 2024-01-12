/*
 * mal_motor_acPanasonic.c
 *
 *  Created on: Mar 11, 2020
 *      Author: shin
 */

#include "main.h"
#include "mal_motor_acPanasonic.h"
#include "mal_motor.h"
#include "mal_systimer.h"
#include "math.h"

#include "mal_motor_acPana232_v2.h"
#include "mal_motor_acPana232Func.h"


#include "mal_motor_acPanaCurve.h"

#ifdef HAL_MOTOR_AC_MODULE_ENABLED

MAL_MOTOR_PanasonicHandleTypeDef mpanasonic = {0,};
extern MAL_MOTOR_ACPANA232_FuncTypeDef mAc232_Fnc;

float degreePuls = (float) 27.777777;

void MAL_Motor_AcPanasonic_ProcessGetAbsoluteCounter(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic); //20201103
void MAL_Motor_AcPanasonic_ProcessSensorAlm(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);
void MAL_Motor_AcPanasonic_ProcessSensorInit(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);
void MAL_Motor_AcPanasonic_DefulatLocationProcess(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);
void MAL_Motor_AcPanasonic_JogControl(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic);

void memset_hword(uint16_t *target, uint16_t setdata, uint32_t cnt) {

	uint32_t i = 0;

	while (cnt--) {
		target[i++] = setdata;
	}
}
void memset_fword(uint32_t *target, uint32_t setdata, uint32_t cnt) {

	uint32_t i = 0;

	while (cnt--) {
		target[i++] = setdata;
	}
}

#if 0
void timersetting(void)
{
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
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
  Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

/* USER CODE END TIM3_Init 2 */
HAL_TIM_MspPostInit(&htim3);
}
#endif

void MAL_Motor_AcPanasonic_TimerRegInit(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, TIM_HandleTypeDef *cwhtim, uint32_t cwChannel, TIM_HandleTypeDef *ccwhtim, uint32_t ccwChannel,
		TIM_HandleTypeDef *cyclehtim) {
	static uint8_t regNum = 0;

	pmpanasonic->status.axleNum = regNum; //등록된 축 수
	regNum++;

	pmpanasonic->cwTim.htim = cwhtim;
	pmpanasonic->ccwTim.htim = ccwhtim;

	pmpanasonic->cyclehtim = cyclehtim;

	pmpanasonic->cwTim.Channel = cwChannel;
	pmpanasonic->ccwTim.Channel = ccwChannel;

	pmpanasonic->cwTim.inPeriod = PANASONIC_CW_TIM_FREQ;
	pmpanasonic->ccwTim.inPeriod = PANASONIC_CCW_TIM_FREQ;

	memset_hword(pmpanasonic->cwDMA, PANASONIC_1US_PULSE_COUNT, PANASONIC_DMA_MEM_SIZE);
	memset_hword(pmpanasonic->ccwDMA, PANASONIC_1US_PULSE_COUNT, PANASONIC_DMA_MEM_SIZE);

	//timer DMA stop
	HAL_TIM_PWM_Stop_DMA(pmpanasonic->cwTim.htim, pmpanasonic->cwTim.Channel);
	HAL_TIM_PWM_Stop_DMA(pmpanasonic->ccwTim.htim, pmpanasonic->ccwTim.Channel);

}

void MAL_Motor_AcPanasonic_SensorLimRegInit(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, MAL_SENSOR_Limit_HandleTypeDef *cwSen, MAL_SENSOR_Limit_HandleTypeDef *ccwSen) {
	pmpanasonic->cwSen = cwSen;
	pmpanasonic->ccwSen = ccwSen;
}

void MAL_Motor_AcPanasonic_TimerStart(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	HAL_TIM_Base_Start_IT(pmpanasonic->cyclehtim);
}

void MAL_Motor_AcPanasonic_Process(void) {
	MAL_Motor_AcPanasonic_ProcessSensorInit(&mpanasonic);
	MAL_Motor_AcPanasonic_DefulatLocationProcess(&mpanasonic);
	MAL_Motor_AcPanasonic_JogControl(&mpanasonic);
	MAL_Motor_AcPanasonic_ProcessSensorAlm(&mpanasonic);
	MAL_Motor_AcPanasonic_ProcessGetAbsoluteCounter(&mpanasonic);
}

void MAL_Motor_AcPanasonic_ProcessSensorAlm(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {

	if (pmpanasonic->setting.flag == MAL_SEN_EMERGENCY_STOP) {
		if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_emergency) >= 500) {
			pmpanasonic->setting.t_emergency = MAL_SysTimer_GetTickCount();
			MAL_Protocol_Ani_AlmSensorDetection(pmpanasonic->status.axleNum, MAL_SENSOR_GetDetection(pmpanasonic->cwSen), MAL_SENSOR_GetDetection(pmpanasonic->ccwSen));
		}
	}

	if ((pmpanasonic->setting.flag == MAL_SEN_INIT_OK) || (pmpanasonic->setting.flag == MAL_SEN_DEF_LOCATION)) {

		if ((MAL_SENSOR_GetDetection(pmpanasonic->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET)) {
			if ((pmpanasonic->cwSen->status.f_newEvent == SET) || (pmpanasonic->ccwSen->status.f_newEvent == SET)) {
				pmpanasonic->cwSen->status.f_newEvent = RESET;
				pmpanasonic->ccwSen->status.f_newEvent = RESET;

				pmpanasonic->setting.flag = MAL_SEN_EMERGENCY_STOP;
				MAL_Protocol_Ani_AlmSensorDetection(pmpanasonic->status.axleNum, MAL_SENSOR_GetDetection(pmpanasonic->cwSen), MAL_SENSOR_GetDetection(pmpanasonic->ccwSen));
			}
		}
	}
}

void MAL_Motor_AcPanasonic_ProcessGetAbsoluteCounter(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	static uint32_t t_absoRead;
	if (pmpanasonic->setting.absoReadFlag == SET) {

		if (MAL_SysTimer_Elapsed(t_absoRead) >= 500) {
			pmpanasonic->setting.absoRetryCnt++;
			pmpanasonic->setting.absoReadOk = RESET;
			MAL_Motor_AcPanasonic_232_GetAbsoluteCounter();
			t_absoRead = MAL_SysTimer_GetTickCount();
		}

		if (pmpanasonic->setting.absoReadOk == SET) {
			if (mAc232_Fnc.C2Md.status != 0) {
				pmpanasonic->setting.absoStatus = 2; //battery error
				pmpanasonic->setting.absoRetryCnt = 0;
				pmpanasonic->setting.absoReadFlag = RESET;
				MAL_Motor_AcPanasonic_232_SetAbsoluteClear(); //엡솔루트 클리어 요청
				MAL_Motor_AcPanasonic_232_SetAlmClear();
				MAL_Protocol_Ani_RspAcAbsoBatteryOk(pmpanasonic->status.axleNum);
			} else {
				pmpanasonic->setting.absoStatus = 1; //ok
				pmpanasonic->setting.absoRetryCnt = 0;
				pmpanasonic->setting.absoReadFlag = RESET;
				MAL_Protocol_Ani_RspAcAbsoBatteryOk(pmpanasonic->status.axleNum);
			}
		}

		if (pmpanasonic->setting.absoRetryCnt >= 5) {
			pmpanasonic->setting.absoStatus = 3; //timeout
			pmpanasonic->setting.absoRetryCnt = 0;
			pmpanasonic->setting.absoReadFlag = RESET;
			MAL_Protocol_Ani_RspAcAbsoBatteryOk(pmpanasonic->status.axleNum);
		}
	}
}

static uint32_t MAL_Motor_AcPanasonic_CalcDegreeToCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	uint32_t ret = 0;
	double temp;
	double oppLim;

	double degreePuls = (double) 10000 / 360; //   = 10000/360

	oppLim = (double) pmpanasonic->setting.OppositeLimit / 100;

	temp = degreePuls * oppLim;

	temp *= (double) pmpanasonic->setting.ReductionRatio;

	ret = (uint32_t) floor((temp) + 0.5);
	return ret;

}

static int32_t MAL_Motor_AcPanasonic_CalcTargetCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, int32_t target) {
	int32_t ret;

	float ratio;
	float temp;

	ratio = (float) target / PANASONIC_POSI_MAX; //비율을 계산

	temp = (float) pmpanasonic->settingConv.range * ratio;

	ret = (int32_t) floor(temp + 0.5);

	ret += pmpanasonic->settingConv.offset;

	return ret;
}

static uint32_t MAL_Motor_AcPanasonic_CalcCount(uint32_t calcCount) {
	if (calcCount >= 10000) {
		return 10000;
	}
	return calcCount;
}

static void MAL_Motor_AcPanasonic_CaleChangeFreq(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, MAL_MOTOR_PanasonicTimerTypeDef *panaTim) {
#if 0
	panaTim->tempPeriod = 20000 / pmpanasonic->control.setCount;

	panaTim->tempPeriod /= 1000000;

	panaTim->tempCnt = panaTim->tempPeriod / panaTim->inPeriod;

	panaTim->calcPeriod = floor(panaTim->tempCnt + 0.5);

	if (panaTim->calcPeriod > 64800) {
		panaTim->htim->Instance->PSC = 9;
		panaTim->calcPeriod /= 10;

		if(panaTim->oldDuty != (panaTim->calcPeriod - (PANASONIC_1US_PULSE_COUNT/10)))
		{
			panaTim->oldDuty = panaTim->calcPeriod - (PANASONIC_1US_PULSE_COUNT/10);
			if (pmpanasonic->control.location == MAL_PANASONIC_CCW) {
				memset_hword(pmpanasonic->ccwDMA,
						panaTim->oldDuty,
						PANASONIC_DMA_MEM_SIZE);
			} else {
				memset_hword(pmpanasonic->cwDMA, panaTim->oldDuty,
				PANASONIC_DMA_MEM_SIZE);
			}
		}

	} else {
		panaTim->htim->Instance->PSC = 0;

		if (panaTim->oldDuty != (panaTim->calcPeriod - (PANASONIC_1US_PULSE_COUNT))) {
			panaTim->oldDuty = panaTim->calcPeriod - (PANASONIC_1US_PULSE_COUNT);
			if (pmpanasonic->control.location == MAL_PANASONIC_CCW) {
				memset_hword(pmpanasonic->ccwDMA, panaTim->oldDuty,
				PANASONIC_DMA_MEM_SIZE);
			} else {
				memset_hword(pmpanasonic->cwDMA, panaTim->oldDuty,
				PANASONIC_DMA_MEM_SIZE);
			}
		}
	}

	panaTim->htim->Instance->ARR = panaTim->calcPeriod-1;




#else
	panaTim->tempPeriod = 20000 / pmpanasonic->control.setCount;

	panaTim->tempPeriod /= 1000000;

	panaTim->tempCnt = panaTim->tempPeriod / panaTim->inPeriod;

	panaTim->calcPeriod = floor(panaTim->tempCnt + 0.5);

	if (panaTim->calcPeriod > 64800) {
		panaTim->htim->Instance->PSC = 9;
		panaTim->calcPeriod /= 10;
	} else {
		panaTim->htim->Instance->PSC = 0;
	}

	panaTim->htim->Instance->ARR = panaTim->calcPeriod - 1;
#endif
}

static int32_t MAL_Motor_AcPanasonic_CalcRatioToCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint16_t val) {
//int32_t pwmConvRatio(uint8_t ch, uint16_t val) {

	int32_t ret;

	float ratio;
	float temp;

	ratio = (float) val / PANASONIC_POSI_MAX; //비율을 계산

	temp = (float) pmpanasonic->settingConv.range * ratio;

	ret = (int32_t) floor(temp + 0.5);

	//ret += pmpanasonic->settingConv.offset;

	return ret;

//	int32_t ret;
//
//	float ratio;
//	float temp;
//
//	ratio = (float) val / 4095; //비율을 계산
//
//	temp = (float) pmpanasonic->settingConv.range * ratio;
//
//	ret = (int32_t) floor(temp + 0.5);
//
//	ret += pmpanasonic->settingConv.offset;
//
//	return ret;
}

void MAL_Motor_AcPanasonic_SetSettingValTestValue(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	pmpanasonic->setting.reductionPuls = 0;
	pmpanasonic->setting.jogCount = 0;

	pmpanasonic->setting.SensorDirection = MAL_RO_CW;
	pmpanasonic->setting.OppositeLimit = 36000;
	pmpanasonic->setting.DefaultLocation = 0;
	pmpanasonic->setting.DefaultLocationCnt = 0;
	pmpanasonic->setting.ReductionRatio = 1;

	pmpanasonic->setting.reductionPuls = degreePuls * pmpanasonic->setting.ReductionRatio;

	pmpanasonic->setting.jogCount = (int32_t) floor((pmpanasonic->setting.reductionPuls / 1000) + 0.5); 		//1ms 당 펄스

	if (pmpanasonic->setting.jogCount == 0) {
		pmpanasonic->setting.jogCount = 1;
	}

	//센서위치
	if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		pmpanasonic->setting.jogCount = -pmpanasonic->setting.jogCount;
	}
	pmpanasonic->setting.flag = MAL_SEN_INIT_ING;

	pmpanasonic->status.position.now = 0;
	pmpanasonic->status.position.target = 0;

	//센서찍으러 CW로 갔으면 제어는 -값으로 커진다, 센서찍으러 온 반대편으로 진행함 센서 방향 리미트가 0이된다
	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
		pmpanasonic->settingConv.cwLim = 0;
		pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
		pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;
		pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
	} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		pmpanasonic->settingConv.ccwLim = 0;
		pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
		pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;
		pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
	} else {
		while (1)
			; //error
	}
}
//==========================================================================

void MAL_Motor_AcPanasonic_CheckCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	if (pmpanasonic->status.position.now == pmpanasonic->status.position.target) {
		//HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
		pmpanasonic->cwTim.htim->Instance->CNT = 0;
		pmpanasonic->ccwTim.htim->Instance->CNT = 0;
		pmpanasonic->control.runStatus = MAL_PANASONIC_STOP;
		return;
	} else {
		if (pmpanasonic->status.position.now > pmpanasonic->status.position.target) {
			pmpanasonic->control.calcCount = pmpanasonic->status.position.now - pmpanasonic->status.position.target;
			pmpanasonic->control.location = MAL_PANASONIC_CCW;

			pmpanasonic->control.setCount = MAL_Motor_AcPanasonic_CalcCount(pmpanasonic->control.calcCount);

			MAL_Motor_AcPanasonic_CaleChangeFreq(pmpanasonic, &pmpanasonic->ccwTim);
			pmpanasonic->control.runStatus = MAL_PANASONIC_RUN;
			HAL_TIM_PWM_Start_DMA(pmpanasonic->ccwTim.htim, pmpanasonic->ccwTim.Channel, (uint32_t*) pmpanasonic->ccwDMA, pmpanasonic->control.setCount);

			pmpanasonic->status.position.now -= pmpanasonic->control.setCount;

		} else {
			pmpanasonic->control.calcCount = pmpanasonic->status.position.target - pmpanasonic->status.position.now;
			pmpanasonic->control.location = MAL_PANASONIC_CW;

			pmpanasonic->control.setCount = MAL_Motor_AcPanasonic_CalcCount(pmpanasonic->control.calcCount);

			MAL_Motor_AcPanasonic_CaleChangeFreq(pmpanasonic, &pmpanasonic->cwTim);
			pmpanasonic->control.runStatus = MAL_PANASONIC_RUN;
			HAL_TIM_PWM_Start_DMA(pmpanasonic->cwTim.htim, pmpanasonic->cwTim.Channel, (uint32_t*) pmpanasonic->cwDMA, pmpanasonic->control.setCount);

			pmpanasonic->status.position.now += pmpanasonic->control.setCount;

		}
	}
}

void MAL_Motor_AcPanasonic_SetChangeTarget(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, int32_t target) {
	if (pmpanasonic->setting.flag != MAL_SEN_INIT_OK)
		return;

	pmpanasonic->status.position.target = MAL_Motor_AcPanasonic_CalcTargetCount(pmpanasonic, target);
	if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
		MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
	}
}

void MAL_Motor_AcPanasonic_SetSettingVal(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio) {
	pmpanasonic->setting.reductionPuls = 0;
	pmpanasonic->setting.jogCount = 0;

	pmpanasonic->setting.SensorDirection = SensorDirection;
	pmpanasonic->setting.OppositeLimit = OppositeLimit;
	pmpanasonic->setting.DefaultLocation = DefaultLocation;
	pmpanasonic->setting.ReductionRatio = ReductionRatio;

	// 모터 감속전 1도 펄스 * 감속비
	pmpanasonic->setting.reductionPuls = degreePuls * pmpanasonic->setting.ReductionRatio;

	//pmpanasonic->setting.jogCount = (int32_t) floor((pmpanasonic->setting.reductionPuls / 1000) + 0.5); 		//1ms 당 펄스
	pmpanasonic->setting.jogCount = (pmpanasonic->setting.reductionPuls / 1000); 		//1ms 당 펄스
	if (pmpanasonic->setting.jogCount < 1) {
		pmpanasonic->setting.jogCount = 1;
	}

	//20201104
	double tempDefLocCnt = 0;

	tempDefLocCnt = (double) pmpanasonic->setting.OppositeLimit / 36000; 		//감속전 바퀴수
	tempDefLocCnt *= (double) pmpanasonic->setting.ReductionRatio; 		//감속 후 바퀴수
	tempDefLocCnt *= (double) 10000; 		//한바퀴 카운터  = 총 펄스 수
	tempDefLocCnt *= DefaultLocation;
	tempDefLocCnt = tempDefLocCnt / PANASONIC_POSI_MAX;

	pmpanasonic->setting.DefaultLocationCnt = (int32_t) -tempDefLocCnt;

	//센서위치
	if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		pmpanasonic->setting.jogCount = -pmpanasonic->setting.jogCount;

		pmpanasonic->setting.DefaultLocationCnt = -pmpanasonic->setting.DefaultLocationCnt;
	}

	//pmpanasonic->setting.flag = MAL_SEN_INIT_ING;

	mpanasonic.setting.ampZeroCnt = 0;
	pmpanasonic->setting.AbsoOffsetCnt = 0;
	pmpanasonic->setting.AbsoOffsetFlag = RESET;
	mAc232_Fnc.C2M1.readFlag = RESET;
}
void MAL_Motor_AcPanasonic_StartSenPosi(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic)
{
	pmpanasonic->setting.flag = MAL_SEN_INIT_ING;

	mpanasonic.setting.ampZeroCnt = 0;
	pmpanasonic->setting.AbsoOffsetCnt = 0;
	pmpanasonic->setting.AbsoOffsetFlag = RESET;
	mAc232_Fnc.C2M1.readFlag = RESET;
}
//20201104

int32_t MAL_Motor_AcPanasonic_CalcAbsoToCount(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {

	if (mAc232_Fnc.C2Md.status != 0)
		return 0; 		//error

	if (mAc232_Fnc.C2Md.errorCode != 0)
		return 0;

	if (mAc232_Fnc.C2Md.st1_cnt == 0)
		return 0;

	if (pmpanasonic->setting.absoCount == 0)
		return 0;
	int32_t returnCnt = 0;
	int32_t OffsetCnt;
	double OffsetTemp;
	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {

		OffsetCnt = 0x007FFFFF - pmpanasonic->setting.absoCount;

		OffsetTemp = (double) OffsetCnt / 0x007FFFFF * 10000;

		OffsetCnt = (int32_t) OffsetTemp;
	} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		OffsetCnt = (int32_t) pmpanasonic->setting.absoCount;

		OffsetTemp = (double) OffsetCnt / 0x007FFFFF * 10000;

		OffsetCnt = (int32_t) OffsetTemp;
	}

	double nowAbsoSt1Temp = 0;
	double nowAbsoSt1Cnt = (double) mAc232_Fnc.C2Md.st1_cnt;

	double nowAbsuMultCnt = (double)mAc232_Fnc.C2Md.multTurnCnt;
//
//	int32_t nowCalcSt1Cnt = 0; 		//카운트 전환 변수
//	int32_t nowCalcMultCnt = 0;

	double loadAbsoZeroTemp = 0;
	double loadAbsoZeroCnt = 0;

	//int32_t nowCalcCnt = 0;



	nowAbsuMultCnt *= 10000;


	nowAbsoSt1Temp = nowAbsoSt1Cnt;
	nowAbsoSt1Temp *= 10000;
	nowAbsoSt1Temp = nowAbsoSt1Temp / (double) 0x007FFFFF;
	nowAbsoSt1Cnt = (double)nowAbsoSt1Temp;

	loadAbsoZeroTemp = pmpanasonic->setting.absoCount;
	loadAbsoZeroTemp *= 10000;
	loadAbsoZeroTemp = loadAbsoZeroTemp / (double) 0x007FFFFF;
	loadAbsoZeroCnt = (double)loadAbsoZeroTemp;


	returnCnt = nowAbsuMultCnt + nowAbsoSt1Cnt - loadAbsoZeroCnt;


	return returnCnt;

#if 0
	if (nowAbsuMultCnt >= 0) 		//양수  원점기준 cw방향으로 회전되어 있을경우
			{
		nowCalcMultCnt = nowAbsuMultCnt * 10000; 		//한바퀴 카운드 10000

//		nowAbsoSt1Temp = (double)nowAbsoSt1Cnt*10000;
//		nowAbsoSt1Temp = nowAbsoSt1Temp/0x007FFFFF;//한바퀴 10000카운트

		nowAbsoSt1Temp = nowAbsoSt1Cnt - pmpanasonic->setting.absoCount;
		nowAbsoSt1Temp *= 10000;
		nowAbsoSt1Temp = nowAbsoSt1Temp / (double) 0x007FFFFF;

		nowCalcSt1Cnt = (int32_t) floor((nowAbsoSt1Temp) + 0.5);

		returnCnt = nowCalcMultCnt + nowCalcSt1Cnt; 		// - OffsetCnt;
	} else 		//음수  원점기준 ccw 방향으로 회전되어 있을경우
	{
//		nowCalcMultCnt = nowAbsuMultCnt*10000;//한바퀴 카운드 10000
//
//		nowAbsoSt1Temp = (double) nowAbsoSt1Cnt * 10000;
//		nowAbsoSt1Temp = nowAbsoSt1Temp / 0x007FFFFF; 		//한바퀴 10000카운트

		nowCalcMultCnt = nowAbsuMultCnt * 10000; 		//한바퀴 카운드 10000

		/*
		 nowAbsoSt1Cnt = 0x007FFFFF - nowAbsoSt1Cnt;

		 nowAbsoSt1Temp = (double) nowAbsoSt1Cnt * 10000;
		 nowAbsoSt1Temp = nowAbsoSt1Temp / (double)0x007FFFFF; 		//한바퀴 10000카운트
		 */
		//nowAbsoSt1Cnt = 0x007FFFFF - nowAbsoSt1Cnt;
		nowAbsoSt1Temp = nowAbsoSt1Cnt - pmpanasonic->setting.absoCount;
		nowAbsoSt1Temp *= 10000;
		nowAbsoSt1Temp = nowAbsoSt1Temp / (double) 0x007FFFFF;

		nowCalcSt1Cnt = (int32_t) floor((nowAbsoSt1Temp) - 0.5);

		returnCnt = nowCalcMultCnt + nowCalcSt1Cnt; 		// - OffsetCnt;
	}

	return returnCnt;
#endif
}

uint8_t MAL_Motor_AcPanasonic_SetSettingVal_AbsoluteVal(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation,
		uint16_t ReductionRatio) {
	uint8_t ret = 0; 		//실패

	mAc232_Fnc.C2M1.readFlag = RESET;
	pmpanasonic->setting.AbsoOffsetFlag = SET;
	pmpanasonic->setting.AbsoOffsetCnt = 0;
	pmpanasonic->setting.ampZeroCnt = 0;

	pmpanasonic->setting.reductionPuls = 0;
	pmpanasonic->setting.jogCount = 0;

	pmpanasonic->setting.SensorDirection = SensorDirection;
	pmpanasonic->setting.OppositeLimit = OppositeLimit;
	pmpanasonic->setting.DefaultLocation = DefaultLocation;
	pmpanasonic->setting.ReductionRatio = ReductionRatio;

	// 모터 감속전 1도 펄스 * 감속비
	pmpanasonic->setting.reductionPuls = degreePuls * pmpanasonic->setting.ReductionRatio;

	//pmpanasonic->setting.jogCount = (int32_t) floor((pmpanasonic->setting.reductionPuls / 1000) + 0.5); 		//1ms 당 펄스
	pmpanasonic->setting.jogCount = (pmpanasonic->setting.reductionPuls / 1000); 		//1ms 당 펄스
	if (pmpanasonic->setting.jogCount < 1) {
		pmpanasonic->setting.jogCount = 1;
	}

	//20201104
	double tempDefLocCnt = 0;

	tempDefLocCnt = (double) pmpanasonic->setting.OppositeLimit / 36000; 		//감속전 바퀴수
	tempDefLocCnt *= (double) pmpanasonic->setting.ReductionRatio; 		//감속 후 바퀴수
	tempDefLocCnt *= (double) 10000; 		//한바퀴 카운터  = 총 펄스 수
	tempDefLocCnt *= DefaultLocation;
	tempDefLocCnt = tempDefLocCnt / PANASONIC_POSI_MAX;

	pmpanasonic->setting.DefaultLocationCnt = (int32_t) -tempDefLocCnt;
	pmpanasonic->setting.AbsoOffsetCnt = pmpanasonic->setting.DefaultLocationCnt;
	//센서위치
	if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		pmpanasonic->setting.jogCount = -pmpanasonic->setting.jogCount;

		pmpanasonic->setting.DefaultLocationCnt = -pmpanasonic->setting.DefaultLocationCnt;
	}

	//20201104 계산 루틴 시작

	int32_t get_count = MAL_Motor_AcPanasonic_CalcAbsoToCount(pmpanasonic);

	if (pmpanasonic->setting.absoCount == 0)
		return ret;

	pmpanasonic->status.position.now = get_count;	// 모터값 초기화
	pmpanasonic->status.position.target = get_count; //타겟 위치를 일치시켜 모터를 정지시킴

	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {

		pmpanasonic->settingConv.cwLim = 0;
		pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
		pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;
		pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

		double defLocCalc = (double) get_count * PANASONIC_POSI_MAX;
		defLocCalc = defLocCalc / (double) pmpanasonic->settingConv.range;

		pmpanasonic->setting.DefultLocTemp = (int32_t) floor((defLocCalc) + 0.5);

		pmpanasonic->setting.DefultLocTempCnt = get_count;

	} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {

		pmpanasonic->settingConv.ccwLim = 0;
		pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
		pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;
		pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

		double defLocCalc = (double) get_count * PANASONIC_POSI_MAX;
		defLocCalc = defLocCalc / (double) pmpanasonic->settingConv.range;

		pmpanasonic->setting.DefultLocTemp = (int32_t) floor((defLocCalc) + 0.5);

		pmpanasonic->setting.DefultLocTempCnt = get_count;

	}
	ret = 1;
	//pmpanasonic->setting.AbsoOffsetCnt = pmpanasonic->setting.DefultLocTempCnt;
	//pmpanasonic->setting.ampZeroCnt -= pmpanasonic->setting.DefultLocTempCnt;
	//20201104 계산 루틴 끝

	pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

	return ret;
}

/*-----------------------------------------------------------------
 * 초기화 루틴
 *-----------------------------------------------------------------*/

//초기화용 1도 조그 동작
static void MAL_Motor_AcPanasonic_SetJog1degree(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, int32_t count, uint32_t Dspeed) {

	if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_jog) >= 1) {
		pmpanasonic->setting.t_jog = MAL_SysTimer_GetTickCount();

		pmpanasonic->status.position.target += count * Dspeed;
		if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
			MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
		}
	}
}

uint8_t MAL_Motor_AcPanasonic_OffsetMove(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, int32_t offTarget) {
	uint8_t ret = RESET;

	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
		if (pmpanasonic->status.position.target <= offTarget) {
			ret = SET;
			return ret;
		}
	} else {
		if (pmpanasonic->status.position.target >= offTarget) {
			ret = SET;
			return ret;
		}
	}

	pmpanasonic->status.position.target += (-pmpanasonic->setting.jogCount);
	if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
		MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
	}
	return ret;
//200714
#if 0
	uint8_t ret = RESET;

	pmpanasonic->status.position.target += (-pmpanasonic->setting.jogCount);
	if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
		MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
	}

	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW)
	{
		if(pmpanasonic->status.position.target <= offTarget)
		{
			ret = SET;
		}
	}
	else
	{
		if (pmpanasonic->status.position.target >= offTarget)
		{
			ret = SET;
		}
	}
	return ret;
#endif
}

HAL_StatusTypeDef MAL_Motor_AcPanasonic_WaitCntRead(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint32_t timeOut) {
	static uint8_t timeFlag = SET;
	static uint32_t t_timeOut = 0;

	static uint8_t delayFlag = SET;
	static uint32_t t_delay = 0;

	static uint32_t t_retry = 0;

	if (delayFlag == SET) {
		delayFlag = RESET;
		t_delay = MAL_SysTimer_GetTickCount();
	}

	if (MAL_SysTimer_Elapsed(t_delay) <= 3000) {
		return HAL_BUSY;
	}

	//타임아웃 계산
	if (timeFlag == SET) {
		timeFlag = RESET;
		mAc232_Fnc.C2M1.initFlag = RESET;
		mAc232_Fnc.C2M1.readFlag = RESET; //리셋일경우 232에서 리턴

		MAL_Motor_AcPanasonic_232_GetPulseCounter(); //메인루프 1초 주기 요청 + 이벤트 요청

		t_timeOut = MAL_SysTimer_GetTickCount();

	}

	if (MAL_SysTimer_Elapsed(t_timeOut) >= timeOut) {
		timeFlag = SET;
		delayFlag = SET;
		return HAL_TIMEOUT;
	}
	//=====

	if (MAL_SysTimer_Elapsed(t_retry) >= 1000) {
		MAL_Motor_AcPanasonic_232_GetPulseCounter(); //메인루프 1초 주기 요청 + 이벤트 요청
		t_retry = MAL_SysTimer_GetTickCount();
	}

	if (mAc232_Fnc.C2M1.initFlag == SET) {

		mAc232_Fnc.C2M1.readFlag = SET;

		pmpanasonic->setting.ampZeroCnt = mAc232_Fnc.C2M1.readCnt;

		timeFlag = SET;
		delayFlag = SET;
		return HAL_OK;
	}

	return HAL_BUSY;

//	//타임아웃 계산
//	if (timeFlag == SET) {
//		timeFlag = RESET;
//		mAc232_Fnc.C2M1.initFlag = RESET;
//		mAc232_Fnc.C2M1.readFlag = RESET; //리셋일경우 232에서 리턴
//
//		MAL_Motor_AcPanasonic_232_GetPulseCounter(); //메인루프 1초 주기 요청 + 이벤트 요청
//
//		t_timeOut = MAL_SysTimer_GetTickCount();
//	}
//	if (MAL_SysTimer_Elapsed(t_timeOut) >= timeOut) {
//		timeFlag = SET;
//		return HAL_TIMEOUT;
//	}
//	//=====
//
//	if (mAc232_Fnc.C2M1.initFlag == SET) {
//
//		mAc232_Fnc.C2M1.readFlag = SET;
//		pmpanasonic->setting.ampZeroCnt = mAc232_Fnc.C2M1.readCnt;
//		timeFlag = SET;
//
//		return HAL_OK;
//	}
//
//	return HAL_BUSY;

}
HAL_StatusTypeDef MAL_Motor_AcPanasonic_WaitCntRead_Abso(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint32_t timeOut) {
	static uint8_t timeFlag = SET;
	static uint32_t t_timeOut = 0;

	static uint8_t delayFlag = SET;
	static uint32_t t_delay = 0;

	static uint32_t t_retry = 0;

	if(delayFlag == SET)
	{
		delayFlag = RESET;
		t_delay= MAL_SysTimer_GetTickCount();
	}

	if (MAL_SysTimer_Elapsed(t_delay) <= 3000) {
		return HAL_BUSY;
	}

	//타임아웃 계산
	if (timeFlag == SET) {
		timeFlag = RESET;
		mAc232_Fnc.C2M1.initFlag = RESET;
		mAc232_Fnc.C2M1.readFlag = RESET; //리셋일경우 232에서 리턴

		MAL_Motor_AcPanasonic_232_GetPulseCounter(); //메인루프 1초 주기 요청 + 이벤트 요청

		t_timeOut = MAL_SysTimer_GetTickCount();

	}

	if (MAL_SysTimer_Elapsed(t_timeOut) >= timeOut) {
		timeFlag = SET;
		delayFlag = SET;
		return HAL_TIMEOUT;
	}
	//=====

	if (MAL_SysTimer_Elapsed(t_retry) >= 1000) {
		MAL_Motor_AcPanasonic_232_GetPulseCounter(); //메인루프 1초 주기 요청 + 이벤트 요청
		t_retry = MAL_SysTimer_GetTickCount();
	}


	if (mAc232_Fnc.C2M1.initFlag == SET) {

		mAc232_Fnc.C2M1.readFlag = SET;
		if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
			pmpanasonic->setting.ampZeroCnt = mAc232_Fnc.C2M1.readCnt - pmpanasonic->setting.AbsoOffsetCnt;

		} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {

			pmpanasonic->setting.ampZeroCnt = mAc232_Fnc.C2M1.readCnt + pmpanasonic->setting.AbsoOffsetCnt;
		}
		timeFlag = SET;
		delayFlag = SET;
		return HAL_OK;
	}


	return HAL_BUSY;

}

HAL_StatusTypeDef MAL_Motor_AcPanasonic_WaitAbsoRead(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic, uint32_t timeOut) {
	static uint8_t timeFlag = SET;
	static uint32_t t_timeOut = 0;
	static uint32_t t_retry = 0;
	static uint8_t seqNum = 0;

	//타임아웃 계산
	if (timeFlag == SET) {

		timeFlag = RESET;
		seqNum = 0;

		mAc232_Fnc.C9Mb.ctrStatus = SET;
		mAc232_Fnc.C2Md.ctrStatus = SET;
		MAL_Motor_AcPanasonic_232_SetAbsoluteClear(); //엡솔루트 클리어 요청

		t_timeOut = MAL_SysTimer_GetTickCount();
		t_retry = MAL_SysTimer_GetTickCount();
	}

	if (MAL_SysTimer_Elapsed(t_timeOut) >= timeOut) {
		timeFlag = SET;
		seqNum = 0;

		return HAL_TIMEOUT;
	}
	//=====

	switch (seqNum) {
	case 0:
		if (mAc232_Fnc.C9Mb.ctrStatus == RESET) { //클리어 성공하면

			MAL_Motor_AcPanasonic_232_GetAbsoluteCounter(); //엡솔루트 카운터 요청
			seqNum = 1; //다음 시퀀스로
			t_retry = MAL_SysTimer_GetTickCount();
			break;
		}
		if (MAL_SysTimer_Elapsed(t_retry) >= 600) {
			mAc232_Fnc.C9Mb.ctrStatus = SET;
			MAL_Motor_AcPanasonic_232_SetAbsoluteClear(); //엡솔루트 클리어 요청
			t_retry = MAL_SysTimer_GetTickCount();
		}
		break;
	case 1:
		if (MAL_SysTimer_Elapsed(t_retry) >= 3000) {
			seqNum = 2;
			t_retry = MAL_SysTimer_GetTickCount();
		}
		break;
	case 2:
		if (mAc232_Fnc.C2Md.ctrStatus == RESET) { //클리어 성공하면
			pmpanasonic->setting.absoCount = mAc232_Fnc.C2Md.st1_cnt;

			//변수 초기화
			timeFlag = SET;
			seqNum = 0;

			return HAL_OK;
			break;
		}

		if (MAL_SysTimer_Elapsed(t_retry) >= 600) {
			mAc232_Fnc.C2Md.ctrStatus = SET;
			MAL_Motor_AcPanasonic_232_GetAbsoluteCounter(); //엡솔루트 카운터 요청
			t_retry = MAL_SysTimer_GetTickCount();
		}

		break;
	}

	return HAL_BUSY;

}

void MAL_Motor_AcPanasonic_ProcessSensorInit(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {

	static HAL_StatusTypeDef waitCntFlag = HAL_BUSY;
	HAL_StatusTypeDef waitAbsoFlag = HAL_BUSY;

	static uint8_t toggleFlag = RESET;

	if (pmpanasonic->setting.flag != MAL_SEN_INIT_ING)
		return;

	if (pmpanasonic->setting.offsetFlag == SET) {
		//센서 찍고 정지후 딜레이
		if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_offsetDelay) >= 3000) {

			//지정한 시간 마다 1도씩 이동
			if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_offset) >= 1) {
				pmpanasonic->setting.t_offset = MAL_SysTimer_GetTickCount();

				//센서로 움직였던 반대방향으로 움직인다

//			MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, -pmpanasonic->setting.jogCount, 1);

				/*if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
				 MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, -pmpanasonic->setting.reductionPuls, 1);
				 } else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
				 MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, pmpanasonic->setting.reductionPuls, 1);
				 }*/

				//이동이 완료되면 초기화 정보 입력
				if (MAL_Motor_AcPanasonic_OffsetMove(pmpanasonic, pmpanasonic->setting.moveCnt) == SET) {

					if (toggleFlag == RESET) {
						waitCntFlag = MAL_Motor_AcPanasonic_WaitCntRead(pmpanasonic, 7000);
					}

					if ((waitCntFlag == HAL_OK) || (waitCntFlag == HAL_TIMEOUT)) {
						toggleFlag = SET;
						waitAbsoFlag = MAL_Motor_AcPanasonic_WaitAbsoRead(pmpanasonic, 10000);
					}

					if ((waitAbsoFlag == HAL_OK) || (waitAbsoFlag == HAL_TIMEOUT)) {

						if (waitAbsoFlag == HAL_TIMEOUT) {
							pmpanasonic->setting.absoCount = 0;
						}
						toggleFlag = RESET;
						waitCntFlag = HAL_BUSY;

						pmpanasonic->setting.offsetFlag = RESET;
						pmpanasonic->setting.moveCnt = 0;

						if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
							pmpanasonic->status.position.now = 0;	// 모터값 초기화
							pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

							pmpanasonic->settingConv.cwLim = 0;
							pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
							pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;
							pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

							pmpanasonic->setting.DefultLocTemp = 0;
							pmpanasonic->setting.DefultLocTempCnt = 0;

							//20201104 폴링응답하도록 삭제
							MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum,pmpanasonic->setting.absoCount);
						} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
							pmpanasonic->status.position.now = 0;	// 모터값 초기화
							pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

							pmpanasonic->settingConv.ccwLim = 0;
							pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
							pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;
							pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

							pmpanasonic->setting.DefultLocTemp = 0;
							pmpanasonic->setting.DefultLocTempCnt = 0;

							MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum,pmpanasonic->setting.absoCount);
						}
					}
				}
			}
		}
		return;
	}

	if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {
		if (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET) {
			pmpanasonic->status.position.now = 0;	// 모터값 초기화
			pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

			pmpanasonic->settingConv.cwLim = 0;
			pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
			pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;

			pmpanasonic->setting.offsetFlag = SET;
			pmpanasonic->setting.moveCnt = -pmpanasonic->setting.reductionPuls * 5;
			//pmpanasonic->setting.moveCnt = (5*1000)-1;
			pmpanasonic->setting.t_offset = MAL_SysTimer_GetTickCount();
			pmpanasonic->setting.t_offsetDelay = MAL_SysTimer_GetTickCount();
			//pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

			pmpanasonic->setting.DefultLocTemp = 0;
			pmpanasonic->setting.DefultLocTempCnt = 0;

			//MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum);
		} else {
			MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, pmpanasonic->setting.jogCount, 1);
		}
	} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {
		if (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET) {
			pmpanasonic->status.position.now = 0;	// 모터값 초기화
			pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

			pmpanasonic->settingConv.ccwLim = 0;
			pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
			pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;

			pmpanasonic->setting.offsetFlag = SET;
			pmpanasonic->setting.moveCnt = pmpanasonic->setting.reductionPuls * 5;
			//pmpanasonic->setting.moveCnt = (5*1000)-1;
			pmpanasonic->setting.t_offset = MAL_SysTimer_GetTickCount();
			pmpanasonic->setting.t_offsetDelay = MAL_SysTimer_GetTickCount();
			//pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

			pmpanasonic->setting.DefultLocTemp = 0;
			pmpanasonic->setting.DefultLocTempCnt = 0;

			//MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum);
		} else {
			MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, pmpanasonic->setting.jogCount, 1);
		}
	}

	/*
	 if (pmpanasonic->setting.flag != MAL_SEN_INIT_ING)
	 return;


	 if (pmpanasonic->setting.SensorDirection == MAL_RO_CW)
	 {
	 //if(MAL_SENSOR_GetDetection(pmpanasonic->cwSen) == MAL_SENSOR_SET)
	 if((MAL_SENSOR_GetDetection(pmpanasonic->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET))
	 {
	 pmpanasonic->status.position.now = 0;	// 모터값 초기화
	 pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

	 pmpanasonic->settingConv.cwLim = 0;
	 pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
	 pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;
	 pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

	 pmpanasonic->setting.DefultLocTemp = 0;

	 MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum);
	 }
	 else
	 {
	 MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, pmpanasonic->setting.jogCount, 1);
	 }
	 }
	 else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW)
	 {
	 //if (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET)
	 if((MAL_SENSOR_GetDetection(pmpanasonic->cwSen) == MAL_SENSOR_SET) || (MAL_SENSOR_GetDetection(pmpanasonic->ccwSen) == MAL_SENSOR_SET))
	 {
	 pmpanasonic->status.position.now = 0;	// 모터값 초기화
	 pmpanasonic->status.position.target = 0; //타겟 위치를 일치시켜 모터를 정지시킴

	 pmpanasonic->settingConv.ccwLim = 0;
	 pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
	 pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;
	 pmpanasonic->setting.flag = MAL_SEN_INIT_OK;

	 pmpanasonic->setting.DefultLocTemp = 0;

	 MAL_Protocol_Ani_RspSensorInitSuccess(pmpanasonic->status.axleNum);
	 }
	 else
	 {
	 MAL_Motor_AcPanasonic_SetJog1degree(pmpanasonic, pmpanasonic->setting.jogCount, 1);
	 }
	 }*/
}

//======================================================================

void MAL_Motor_AcPanasonic_DefulatLocationProcess(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {

	//int32_t tempCalc;

	static HAL_StatusTypeDef waitCntFlag = HAL_BUSY;

	static uint8_t CurveFlag = SET;

	if (pmpanasonic->setting.flag == MAL_SEN_DEF_LOCATION) {

		if(CurveFlag == SET)
		{


			MAL_Motor_AcPanasonic_Curve_Clear();
			MAL_Motor_AcPanasonic_Curve_Init(
					(float)pmpanasonic->setting.DefultLocTempCnt,
					(float)pmpanasonic->setting.DefaultLocationCnt,
					0,0);
			CurveFlag = RESET;
			return;
		}

		pmpanasonic->setting.absoStatus = 0;
		if (pmpanasonic->setting.DefaultLocationCnt != pmpanasonic->setting.DefultLocTempCnt) {
			if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_defLoc) >= CURVE_TIME_QUANTUM) {

				if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {

					pmpanasonic->status.position.target = MAL_Motor_AcPanasonic_Curve_CalcHermiteY();
					pmpanasonic->setting.DefultLocTempCnt = pmpanasonic->status.position.target;
					MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
				}

				pmpanasonic->setting.t_defLoc = MAL_SysTimer_GetTickCount();
			}
		} else {

			if (pmpanasonic->setting.AbsoOffsetFlag == SET) {
				waitCntFlag = MAL_Motor_AcPanasonic_WaitCntRead_Abso(pmpanasonic, 7000);

				if (waitCntFlag != HAL_BUSY) {
					waitCntFlag = HAL_BUSY;
					pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
					MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);

					CurveFlag = SET;//210413
				}
			} else {
				pmpanasonic->setting.AbsoOffsetFlag = RESET;
				waitCntFlag = HAL_BUSY;
				pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
				MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);

				CurveFlag = SET;//210413
			}
			//값이 같으면 끝남

/*			if (pmpanasonic->setting.SensorDirection == MAL_RO_CW) {

				pmpanasonic->settingConv.cwLim = 0;
				pmpanasonic->settingConv.ccwLim = -MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
				pmpanasonic->settingConv.range = pmpanasonic->settingConv.ccwLim;
				pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
			} else if (pmpanasonic->setting.SensorDirection == MAL_RO_CCW) {

				pmpanasonic->settingConv.ccwLim = 0;
				pmpanasonic->settingConv.cwLim = MAL_Motor_AcPanasonic_CalcDegreeToCount(pmpanasonic);
				pmpanasonic->settingConv.range = pmpanasonic->settingConv.cwLim;
				pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
			}*/


		}
	}
	else
	{
		//CurveFlag = SET;
	}

#if 0
	int32_t tempCalc;

	static HAL_StatusTypeDef waitCntFlag = HAL_BUSY;

	if (pmpanasonic->setting.flag == MAL_SEN_DEF_LOCATION) {
		pmpanasonic->setting.absoStatus = 0;
		if (pmpanasonic->setting.DefaultLocationCnt != pmpanasonic->setting.DefultLocTempCnt) {
			if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_defLoc) >= 2) {

				if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
					if (pmpanasonic->setting.DefaultLocationCnt > pmpanasonic->setting.DefultLocTempCnt) {

						tempCalc = pmpanasonic->setting.DefaultLocationCnt - pmpanasonic->setting.DefultLocTempCnt;

						tempCalc = tempCalc / pmpanasonic->setting.ReductionRatio;

						//if( (pmpanasonic->setting.DefultLocTempCnt + tempCalc) >pmpanasonic->setting.DefaultLocationCnt)
						if (tempCalc == 0) {
							pmpanasonic->status.position.target = pmpanasonic->setting.DefaultLocationCnt;
							pmpanasonic->setting.DefultLocTempCnt = pmpanasonic->setting.DefaultLocationCnt;
						} else {
							pmpanasonic->status.position.target += tempCalc;
							pmpanasonic->setting.DefultLocTempCnt += tempCalc;
						}
					} else {

						tempCalc = pmpanasonic->setting.DefultLocTempCnt - pmpanasonic->setting.DefaultLocationCnt;

						tempCalc = tempCalc / pmpanasonic->setting.ReductionRatio;

						//if ((pmpanasonic->setting.DefultLocTempCnt - tempCalc) < pmpanasonic->setting.DefaultLocationCnt) {
						if (tempCalc == 0) {
							pmpanasonic->status.position.target = pmpanasonic->setting.DefaultLocationCnt;
							pmpanasonic->setting.DefultLocTempCnt = pmpanasonic->setting.DefaultLocationCnt;
						} else {
							pmpanasonic->status.position.target -= tempCalc;
							pmpanasonic->setting.DefultLocTempCnt -= tempCalc;
						}
					}
					MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
				}

				pmpanasonic->setting.t_defLoc = MAL_SysTimer_GetTickCount();
			}
		} else {

			if (pmpanasonic->setting.AbsoOffsetFlag == SET) {
				waitCntFlag = MAL_Motor_AcPanasonic_WaitCntRead_Abso(pmpanasonic, 7000);

				if (waitCntFlag != HAL_BUSY) {
					waitCntFlag = HAL_BUSY;
					pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
					MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);
				}
			} else {
				pmpanasonic->setting.AbsoOffsetFlag = RESET;
				waitCntFlag = HAL_BUSY;
				pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
				MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);
			}
			//값이 같으면 끝남

		}
	}
#endif
//	if (pmpanasonic->setting.flag == MAL_SEN_DEF_LOCATION) {
//
//		if (pmpanasonic->setting.DefaultLocation != pmpanasonic->setting.DefultLocTemp) {
//			if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_defLoc) >= 2) {
//
//				if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
//					if (pmpanasonic->setting.DefaultLocation > pmpanasonic->setting.DefultLocTemp) {
//						pmpanasonic->setting.DefultLocTemp++;
//						pmpanasonic->status.position.target += MAL_Motor_AcPanasonic_CalcRatioToCount(pmpanasonic, 1);
//					} else {
//						pmpanasonic->setting.DefultLocTemp--;
//						pmpanasonic->status.position.target -= MAL_Motor_AcPanasonic_CalcRatioToCount(pmpanasonic, 1);
//					}
//					MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
//				}
//
//				pmpanasonic->setting.t_defLoc = MAL_SysTimer_GetTickCount();
//			}
//		} else {
//			//값이 같으면 끝남
//			pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
//			MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);
//
//		}
//	}

//	if (pmpanasonic->setting.flag == MAL_SEN_DEF_LOCATION) {
//
//		if (pmpanasonic->setting.DefaultLocation != pmpanasonic->setting.DefultLocTemp) {
//			if (MAL_SysTimer_Elapsed(pmpanasonic->setting.t_defLoc) >= 5) {
//
//				if (pmpanasonic->setting.DefultLocTemp < 4095)
//				{
//					pmpanasonic->setting.DefultLocTemp++;
//					pmpanasonic->status.position.target = MAL_Motor_AcPanasonic_CalcRatioToCount(pmpanasonic, pmpanasonic->setting.DefultLocTemp);
//					if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
//						MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
//					}
//				} else {
//					pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
//				}
//				pmpanasonic->setting.t_defLoc = MAL_SysTimer_GetTickCount();
//			}
//		} else {
//			//값이 같으면 끝남
//			pmpanasonic->setting.flag = MAL_SEN_INIT_OK;
//			MAL_Protocol_Ani_RspDefPosi(pmpanasonic->status.axleNum, MAL_SEN_INIT_OK);
//
//		}
//	}

}

void MAL_Motor_AcPanasonic_JogControl(MAL_MOTOR_PanasonicHandleTypeDef *pmpanasonic) {
	if (pmpanasonic->jogCtr.activeflag != SET)
		return;

// jog packet 이 도착한지 200ms 가 지나면 기능을 비활성화 한다.
	if (MAL_SysTimer_Elapsed(pmpanasonic->jogCtr.tJogMove) >= 200) {
		pmpanasonic->jogCtr.activeflag = RESET;
		pmpanasonic->jogCtr.jogCnt = 0;
		return;
	}

	if (pmpanasonic->jogCtr.jogCnt > 0) {
		if ((pmpanasonic->status.position.now + pmpanasonic->jogCtr.jogCnt) < pmpanasonic->status.position.target)
			return;
	} else {
		if ((pmpanasonic->status.position.now + pmpanasonic->jogCtr.jogCnt) > pmpanasonic->status.position.target)
			return;
	}

	pmpanasonic->status.position.target += pmpanasonic->jogCtr.jogCnt;
	if (pmpanasonic->control.runStatus == MAL_PANASONIC_STOP) {
		MAL_Motor_AcPanasonic_CheckCount(pmpanasonic);
	}

}

/*-----------------------------------------------------------------*/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
	MAL_Motor_AcPanasonic_CheckCount(&mpanasonic);
}

/*===================================================================
 * protocol callback
 ===================================================================*/

//set
void MAL_Motor_AcPanasonic_SetLocation(uint32_t *pmpanasonic, uint16_t location) {

	MAL_Motor_AcPanasonic_SetChangeTarget((MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic, location);
}

void MAL_Motor_AcPanasonic_SetSetting(uint32_t *pmpanasonic, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio) {
	MAL_Motor_AcPanasonic_SetSettingVal((MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);

}
void MAL_Motor_AcPanasonic_StartInit(uint32_t *pmpanasonic)
{
	MAL_Motor_AcPanasonic_StartSenPosi((MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic);
}
//20201104
uint8_t MAL_Motor_AcPanasonic_SetSetting_Absolute(uint32_t *pmpanasonic, uint8_t SensorDirection, uint16_t OppositeLimit, uint16_t DefaultLocation, uint16_t ReductionRatio) {
	return MAL_Motor_AcPanasonic_SetSettingVal_AbsoluteVal((MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic, SensorDirection, OppositeLimit, DefaultLocation, ReductionRatio);
}
void MAL_Motor_AcPanasonic_SetDefaultLocation(uint32_t *pmpanasonic) {
	MAL_MOTOR_PanasonicHandleTypeDef *ppmpanasonic = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

	if(ppmpanasonic->setting.flag ==  MAL_SEN_INIT_OK)
	{
		ppmpanasonic->setting.DefultLocTempCnt = ppmpanasonic->status.position.now;
	}
	ppmpanasonic->setting.flag = MAL_SEN_DEF_LOCATION;
}

void MAL_Motor_AcPanasonic_SetCounter(uint32_t *pmpanasonic, int16_t counter) {
	MAL_MOTOR_PanasonicHandleTypeDef *ppmpanasonic = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

	ppmpanasonic->jogCtr.activeflag = SET;
	ppmpanasonic->jogCtr.jogCnt = counter;
	ppmpanasonic->jogCtr.tJogMove = MAL_SysTimer_GetTickCount();
}

void MAL_Motor_AcPanasonic_SetLoadAbsoCnt(uint32_t *pmpanasonic, uint32_t loadAbsoCnt) {
	MAL_MOTOR_PanasonicHandleTypeDef *ppmpanasonic = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

	ppmpanasonic->setting.absoCount = loadAbsoCnt;
//ppmpanasonic->setting.absoCntToCnt = (float)loadAbsoCnt/0x007FFFFF*10000;
}
//get

uint8_t MAL_Motor_AcPanasonic_GetSettingFlag(uint32_t *pmpanasonic) {
	MAL_MOTOR_PanasonicHandleTypeDef *ptemp = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

	return ptemp->setting.flag;
}


//1.5완료
uint8_t MAL_Motor_AcPanasonic_GetAbsoStatus(uint32_t *pmpanasonic) {
	MAL_MOTOR_PanasonicHandleTypeDef *ptemp = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

//앱소 상태 읽기와 232로 읽기 시도한다
	if(ptemp->setting.absoReadFlag == RESET)
	{
		ptemp->setting.absoStatus = 0;
		ptemp->setting.absoReadFlag = SET;
	}
	return ptemp->setting.absoStatus;
}

uint8_t MAL_Motor_AcPanasonic_GetAbsoStatusOk(uint32_t *pmpanasonic) {
	MAL_MOTOR_PanasonicHandleTypeDef *ptemp = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

//앱소 상태 만 읽는다
	return ptemp->setting.absoStatus;
}
uint32_t MAL_Motor_AcPanasonic_GetAbsoCountOk(uint32_t *pmpanasonic) {
	MAL_MOTOR_PanasonicHandleTypeDef *ptemp = (MAL_MOTOR_PanasonicHandleTypeDef*) pmpanasonic;

	if (ptemp->setting.flag == MAL_SEN_INIT_OK) {
		return ptemp->setting.absoCount;
	} else {
		return 0;
	}
}
//null

void MAL_Motor_AcPanasonic_NullFunction(uint32_t *pmpanasonic, uint16_t count, uint16_t rpm) {

}

#endif

