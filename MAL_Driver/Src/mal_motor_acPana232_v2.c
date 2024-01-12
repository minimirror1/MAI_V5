/*
 * mal_motor_acPana232_v2.c
 *
 *  Created on: 2020. 10. 23.
 *      Author: shin
 */


#include "mal_systimer.h"
#include "string.h"

#include "mal_motor_acPana232_v2.h"
#include "mal_motor_acPana232Func.h"

#include "mal_board_info.h"
#include "mal_motor.h"
#include "app_pid_init_cmd.h"

#ifdef HAL_MOTOR_AC_MODULE_ENABLED
#define RS232TIME_OUT 300

MAL_MOTOR_ACPANA232_PacketHandleTypeDef mAc232;
extern MAL_MOTOR_ACPANA232_FuncTypeDef mAc232_Fnc;


HAL_StatusTypeDef MAL_Motor_AcPanasonic_232GetDataProcess(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet, MAL_MOTOR_ACPANA232_DataBundleTypeDef *packet);

//void 최상위 설정 함수(*리턴포인터)
//{
//
//}
//
//void 최상위 읽기 함수(*리턴포인터)
//{
//
//}

void MAL_Motor_AcPanasonic_232_GetDataAddQueue(MAL_MOTOR_ACPANA232_DataBundleTypeDef *data)
{

	mAc232.getQueue.queue[mAc232.getQueue.front].cmd = data->cmd;
	mAc232.getQueue.queue[mAc232.getQueue.front].mode = data->mode;
	mAc232.getQueue.queue[mAc232.getQueue.front].mal_getDataProcess_CallBack = data->mal_getDataProcess_CallBack;

	mAc232.getQueue.front++;

	if (mAc232.getQueue.front >= ACPANA232_QUEUE_SIZE)
		mAc232.getQueue.front = 0;
}

void MAL_Motor_AcPanasonic_232_CheckQueue() {
	if (mAc232.getQueue.front != mAc232.getQueue.rear) {
		if (MAL_Motor_AcPanasonic_232GetDataProcess(&mAc232, &mAc232.getQueue.queue[mAc232.getQueue.rear]) != HAL_BUSY) {
			//busy 가 아니면  error이거나 ok   다음 큐
			mAc232.getQueue.rear++;

			if (mAc232.getQueue.rear >= ACPANA232_QUEUE_SIZE)
				mAc232.getQueue.rear = 0;
		}
	} else if (mAc232.setQueue.front != mAc232.setQueue.rear) {
		//todo : 세팅 커맨드 미구현
	}
}

//210625
void app_rx_error_sub_pid_clear_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_error_clear_t *pData)
{
	MAL_Motor_AcPanasonic_232_SetAbsoluteClear();
	MAL_Motor_AcPanasonic_232_SetAlmClear();
}

//210625
//240108 예전 에러코드 사용안함.
void MAL_Motor_AcPanasonic_232_Alm()
{
	static uint32_t t_Alm = 0;
	if(mAc232_Fnc.C9M0.codeMain != 0x00)
	{
		if(MAL_SysTimer_Elapsed(t_Alm) >= 1000)
		{
			uint32_t errorCode = 0;
			errorCode = mAc232_Fnc.C9M0.codeMain<<16;
			errorCode |= mAc232_Fnc.C9M0.codeSub;

			//app_tx_error_sub_pid_ac_ctl(0,PRIORITY_HIGH, MAL_Board_ID_GetValue(), MASTER_CAN_ID, MOTOR_AXLE_CNT, errorCode);
			//idtest
			app_tx_error_sub_pid_ac_ctl(
					0,
					PRIORITY_HIGH,
					my_can_id_data.id,
					MASTER_CAN_ID,
					my_can_id_data.sub_id[0],
					0,
					errorCode);

			t_Alm = MAL_SysTimer_GetTickCount();
		}
	}
}
void MAL_Motor_AcPanasonic_232_Process(void) {
	MAL_Motor_AcPanasonic_232_CheckQueue();
	//MAL_Motor_AcPanasonic_232_Alm();
}

void MAL_Motor_AcPanasonic_232_RegInit(MAL_UART_HandleTypeDef *muart) {
	mAc232.muart = muart;
}


void MAL_Motor_AcPanasonic_232_DeleteQueue(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	ac232Packet->muart->rxQueue.rear = ac232Packet->muart->rxQueue.front;
}


HAL_StatusTypeDef MAL_Motor_AcPanasonic_232_WaitPacket(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet, uint8_t *pData, uint32_t size, uint32_t timeOut) {
	static uint8_t timeFlag = SET;
	static uint32_t t_timeOut = 0;
	static uint32_t rxCnt = 0;

	//타임아웃 계산
	if (timeFlag == SET) {
		rxCnt = 0;
		timeFlag = RESET;
		t_timeOut = MAL_SysTimer_GetTickCount();
	}
	if (MAL_SysTimer_Elapsed(t_timeOut) >= timeOut) {
		timeFlag = SET;
		rxCnt = 0; // 변수 초기화

		return HAL_TIMEOUT;
	}
	//=====

	if (MAL_UART_GetQueueData_Byte(ac232Packet->muart, &ac232Packet->parser.rxData) == MAL_UART_CONTAIN) {
		pData[rxCnt] = ac232Packet->parser.rxData;
		rxCnt++;

		if (rxCnt >= size) {
			rxCnt = 0;
			timeFlag = SET;

			return HAL_OK;
		}
	}

	return HAL_BUSY;
}

uint8_t MAL_Motor_AcPanasonic_232_CheckSum(uint8_t *pData, uint8_t size) {
	uint8_t chkSum = 0;
	uint32_t ptr = 0;

	while (size--)
		chkSum += pData[ptr++];

	return chkSum;
}

uint8_t MAL_Motor_AcPanasonic_232_CalcCheckSum(uint8_t *pData, uint8_t size) {
	uint8_t tempData = 0;
	uint8_t chkSum = 0;
	uint32_t ptr = 0;

	while (size--)
		tempData += pData[ptr++];

	chkSum = 0xFF - tempData;
	chkSum += 1;

	return chkSum;

}

void MAL_Motor_AcPanasonic_232_TxData(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet, uint8_t nodeId, uint8_t cmd, uint8_t mode, uint8_t *pdata, uint8_t dataLen) {
	MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef *buff = (MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef*) ac232Packet->txBuff;

	uint32_t txSize = sizeof(MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef) + dataLen;

	buff->len = dataLen;
	buff->nodeId = nodeId;
	buff->mode = mode;
	buff->cmd = cmd;

	if (dataLen != 0) {
		memcpy(buff->payload, pdata, dataLen);
	}

	buff->payload[dataLen] = MAL_Motor_AcPanasonic_232_CalcCheckSum(&buff->len, txSize);

	MAL_UART_SendDataStream(ac232Packet->muart, ac232Packet->txBuff, txSize + 1);
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================


void MAL_MAL_Motor_AcPanasonic_232TimeOut(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet)
{
	ac232Packet->timeOutCnt++;

	if(ac232Packet->timeOutCnt >= 3)
	{
		ac232Packet->f_error = SET;
	}
}
//(tx)호스트->노드 : 통신 요청
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_H_ENQ(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	MAL_Motor_AcPanasonic_232_DeleteQueue(ac232Packet);
	MAL_UART_SendByte(ac232Packet->muart, ACPANA232_ENQ);
	ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_N_EOT; //1

	return HAL_BUSY;
}
//(rx)노드 : 수신대기
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_N_EOT(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {

	HAL_StatusTypeDef ret = HAL_BUSY;
	switch (MAL_Motor_AcPanasonic_232_WaitPacket(ac232Packet, ac232Packet->rxBuff, 1, RS232TIME_OUT)) {
	case HAL_OK:
		if (ac232Packet->rxBuff[0] == ACPANA232_EOT) {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_CMD; //2
			ret = HAL_BUSY;
		} else {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
			ac232Packet->errorFlag = SET;
			ac232Packet->f_error = SET;
			ret = HAL_ERROR;
		}
		break;
	case HAL_ERROR:
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	case HAL_BUSY:
		ret = HAL_BUSY;
		break;
	case HAL_TIMEOUT:
		MAL_MAL_Motor_AcPanasonic_232TimeOut(ac232Packet);
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	}
	return ret;
}

//(tx)호스트->노드 : 모드,지령 전송
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_H_CMD(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet, MAL_MOTOR_ACPANA232_DataBundleTypeDef *packet) {

	uint8_t data = 0;
	MAL_Motor_AcPanasonic_232_DeleteQueue(ac232Packet);
	//MAL_Motor_AcPanasonic_232_TxData(ac232Packet, 1, 1, 2, &data, 0);
	MAL_Motor_AcPanasonic_232_TxData(ac232Packet, 1, packet->cmd, packet->mode, &data, 0);
	ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_N_ACK; //3

	return HAL_BUSY;
}
//(rx)노드 : 수신 응답
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_N_ACK(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	HAL_StatusTypeDef ret = HAL_BUSY;

	switch (MAL_Motor_AcPanasonic_232_WaitPacket(ac232Packet, ac232Packet->rxBuff, 1, RS232TIME_OUT)) {
	case HAL_OK:
		if (ac232Packet->rxBuff[0] == ACPANA232_ACK) {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_N_ENQ; //4
			ret = HAL_BUSY;
		} else {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
			ac232Packet->errorFlag = SET;
			ret = HAL_ERROR;
		}
		break;
	case HAL_ERROR:
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	case HAL_BUSY:
		ret = HAL_BUSY;
		break;
	case HAL_TIMEOUT:
		MAL_MAL_Motor_AcPanasonic_232TimeOut(ac232Packet);
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	}
	return ret;
}
//(rx)노드->호스트 : 통신 요청
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_N_ENQ(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	HAL_StatusTypeDef ret = HAL_BUSY;

	switch (MAL_Motor_AcPanasonic_232_WaitPacket(ac232Packet, ac232Packet->rxBuff, 1, RS232TIME_OUT)) {
	case HAL_OK:
		if (ac232Packet->rxBuff[0] == ACPANA232_ENQ) {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_EOT; //5
			ret = HAL_BUSY;
		} else {
			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
			ac232Packet->errorFlag = SET;
			ret = HAL_ERROR;
		}
		break;
	case HAL_ERROR:
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	case HAL_BUSY:
		ret = HAL_BUSY;
		break;
	case HAL_TIMEOUT:
		MAL_MAL_Motor_AcPanasonic_232TimeOut(ac232Packet);
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	}
	return ret;
}
//(tx)호스트 : 수신대기
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_H_EOT(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	MAL_Motor_AcPanasonic_232_DeleteQueue(ac232Packet);
	MAL_UART_SendByte(ac232Packet->muart, ACPANA232_EOT);

	ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_N_DATA; //6

	return HAL_BUSY;
}



//(rx)노드 : 데이터 전송
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_N_DATA(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef *header = (MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef*) &ac232Packet->rxBuff[0];


	HAL_StatusTypeDef ret = HAL_BUSY;

	switch (MAL_Motor_AcPanasonic_232_WaitPacket(ac232Packet, ac232Packet->rxBuff,
			(sizeof(MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef)+MAL_Motor_AcPanasonic_232_GetDataLen(&ac232Packet->getQueue.queue[ac232Packet->getQueue.rear]) + 1),
			RS232TIME_OUT)) {
	case HAL_OK:

		if (MAL_Motor_AcPanasonic_232_CheckSum(ac232Packet->rxBuff, header->len + 4) == 0x00) {


			ac232Packet->getQueue.queue[ac232Packet->getQueue.rear].mal_getDataProcess_CallBack(&ac232Packet->rxBuff[0]);

			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ACK; //7
			ac232Packet->errorFlag = RESET;
			ret = HAL_BUSY;
		} else {
			MAL_UART_SendByte(ac232Packet->muart, ACPANA232_NAK);

			ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
			ac232Packet->errorFlag = SET;
			ret = HAL_ERROR;
		}

		break;
	case HAL_ERROR:
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	case HAL_BUSY:
		break;
	case HAL_TIMEOUT:
		MAL_MAL_Motor_AcPanasonic_232TimeOut(ac232Packet);
		ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
		ac232Packet->errorFlag = SET;
		ret = HAL_ERROR;
		break;
	}

	return ret;
}
//(tx)호스트 : 응답
HAL_StatusTypeDef MAL_Motor_AcPanasonic_232Packet_H_ACK(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet) {
	MAL_Motor_AcPanasonic_232_DeleteQueue(ac232Packet);
	MAL_UART_SendByte(ac232Packet->muart, ACPANA232_ACK);

	ac232Packet->sequnce = MAL_MOTOR_ACPANA232_PARSER_H_ENQ; //0
	ac232Packet->errorFlag = RESET; //에러아님 정상종료

	return HAL_OK;
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================

HAL_StatusTypeDef MAL_Motor_AcPanasonic_232GetDataProcess(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet, MAL_MOTOR_ACPANA232_DataBundleTypeDef *packet) {

	HAL_StatusTypeDef ret = HAL_BUSY;

	if (MAL_SysTimer_Elapsed(ac232Packet->t_runFlag) >= 500) {
		ac232Packet->t_runFlag = MAL_SysTimer_GetTickCount();
		ac232Packet->errorFlag = RESET;

//		if (mpanasonic.control.runStatus == MAL_PANASONIC_STOP) {
//			if (mpanasonic.status.position.now == mpanasonic.status.position.target) {
//
//
//				ac232Packet->startCnt = mpanasonic.status.position.now;
//			}
//
//		}
	}

	if (ac232Packet->errorFlag == RESET) {
		switch (ac232Packet->sequnce) {
		case MAL_MOTOR_ACPANA232_PARSER_H_ENQ: //호스트->노드 : 통신 요청
			ret = MAL_Motor_AcPanasonic_232Packet_H_ENQ(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_N_EOT: //노드 : 수신대기4
			ret = MAL_Motor_AcPanasonic_232Packet_N_EOT(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_H_CMD: //호스트->노드 : 모드,지령 전송
			ret = MAL_Motor_AcPanasonic_232Packet_H_CMD(ac232Packet, packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_N_ACK: //노드 : 수신 응답
			ret = MAL_Motor_AcPanasonic_232Packet_N_ACK(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_N_ENQ: //노드->호스트 : 통신 요청
			ret = MAL_Motor_AcPanasonic_232Packet_N_ENQ(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_H_EOT: //호스트 : 수신대기
			ret = MAL_Motor_AcPanasonic_232Packet_H_EOT(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_N_DATA: //노드 : 데이터 전송
			ret = MAL_Motor_AcPanasonic_232Packet_N_DATA(ac232Packet);
			break;
		case MAL_MOTOR_ACPANA232_PARSER_H_ACK: //호스트 : 응답
			ret = MAL_Motor_AcPanasonic_232Packet_H_ACK(ac232Packet);
			ac232Packet->f_error = RESET;
			if(ac232Packet->timeOutCnt != 0)
			{
				ac232Packet->timeOutCnt--;
			}
			break;
		}
	}
	return ret;
}

#endif

