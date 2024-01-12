/*
 * mal_motor_acPana232Func.h
 *
 *  Created on: 2020. 10. 23.
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_ACPANA232FUNC_H_
#define INC_MAL_MOTOR_ACPANA232FUNC_H_

#include "mal_motor_acPana232_v2.h"

//===============================================================================

//패킷 형식

//=================================================================================================
//=============command 2
//=================================================================================================

//지령펄스 카운터 읽기 cmd : 2, mode : 1-------------------------------------------------------------
//232패킷
#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketCommandPulseCounterTypeDef
{
	int32_t counter;
	uint8_t errorCode;
}MAL_MOTOR_ACPANA232_PacketC2M1TypeDef;
#pragma pack()

//저장 구조체
#pragma pack(1)//지령 펄스 카운터 읽기 cmd:2 mode:1
typedef struct __MAL_MOTOR_ACPANA232_C2M1TypeDef {
	//todo : 값수신하여 저장할 변수 추가
	//todo : 아래 코드는 ac카운터 구조체로 이동
	int32_t startCnt;	//루틴 시작시 모터 카운트

	uint8_t initFlag;	//센서 초기화 완료 카운터 읽기 플래그
	uint8_t readFlag;	//초기위치 읽기 성공 플래그
	int32_t readCnt;	//앰프 카운터
	int32_t calcCnt;	//초기위치 - 앰프카운터 = 범위내 위치
} MAL_MOTOR_ACPANA232_C2M1TypeDef;
#pragma pack()

//---------------------------------------------------------------------------------------------

//앱솔루트 엔코더 읽기 cmd : 2, mode : D-------------------------------------------------------------
#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketAbsoluteCounterTypeDef
{
	uint16_t encoderId;
	uint16_t status;

	uint8_t  st1_cnt_1;
	uint8_t  st1_cnt_2;
	uint8_t  st1_cnt_3;

	uint16_t multTurnCnt;

	uint8_t res;

	uint8_t errorCode;

}MAL_MOTOR_ACPANA232_PacketC2MdTypeDef;
#pragma pack()

//저장 구조체
#pragma pack(1)//앱솔루트 엔코더 읽기 cmd:2 mode:d
typedef struct __MAL_MOTOR_ACPANA232_C2MdTypeDef {
	uint16_t endcoderId;
	uint16_t status;

	uint32_t st1_cnt;

	int16_t multTurnCnt;

	uint8_t errorCode;

	uint8_t ctrStatus;// RESET : 성공,  SET : 요청(타임아웃시 실패의미)

} MAL_MOTOR_ACPANA232_C2MdTypeDef;
#pragma pack()

//---------------------------------------------------------------------------------------------

//=================================================================================================
//=================================================================================================

//=================================================================================================
//=============command 9
//=================================================================================================

//알람클리어 cmd : 9, mode : 0-------------------------------------------------------------
#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketAlmNumberTypeDef
{
	uint8_t codeMain;
	uint8_t codeSub;
	uint8_t errorCode;
}MAL_MOTOR_ACPANA232_PacketC9M0TypeDef;
#pragma pack()

//알람클리어 cmd : 9, mode : 4-------------------------------------------------------------
#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketAlmClearTypeDef
{
	uint8_t errorCode;
}MAL_MOTOR_ACPANA232_PacketC9M4TypeDef;
#pragma pack()

//저장 구조체
#pragma pack(1)//앱솔루트 엔코더 읽기 cmd:9 mode:0
typedef struct __MAL_MOTOR_ACPANA232_C9M0TypeDef {

	uint8_t codeMain;
	uint8_t codeSub;
	uint8_t errorCode;

	uint8_t ctrStatus;// RESET : 성공,  SET : 요청(타임아웃시 실패의미)
} MAL_MOTOR_ACPANA232_C9M0TypeDef;
#pragma pack()

#pragma pack(1)//앱솔루트 엔코더 읽기 cmd:9 mode:4
typedef struct __MAL_MOTOR_ACPANA232_C9M4TypeDef {
	uint8_t errorCode;

	uint8_t ctrStatus;// RESET : 성공,  SET : 요청(타임아웃시 실패의미)
} MAL_MOTOR_ACPANA232_C9M4TypeDef;


#pragma pack()
//---------------------------------------------------------------------------------------------


//앱소클리어 cmd : 9, mode : B-------------------------------------------------------------
#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketAbsoluteClearTypeDef
{
	uint8_t errorCode;
}MAL_MOTOR_ACPANA232_PacketC9MbTypeDef;
#pragma pack()

//저장 구조체
#pragma pack(1)//앱솔루트 엔코더 읽기 cmd:9 mode:B
typedef struct __MAL_MOTOR_ACPANA232_C9MbTypeDef {
	uint8_t errorCode;

	uint8_t ctrStatus;// RESET : 성공,  SET : 요청(타임아웃시 실패의미)
} MAL_MOTOR_ACPANA232_C9MbTypeDef;
#pragma pack()
//---------------------------------------------------------------------------------------------


//=================================================================================================
//=================================================================================================

//===============================================================================

//===============================================================================







#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_FuncTypeDef
{
	//command : 2
	MAL_MOTOR_ACPANA232_C2M1TypeDef C2M1;//지령펄스 카운터 읽기

	MAL_MOTOR_ACPANA232_C2MdTypeDef C2Md;//앱솔루트 엔코더 읽기

	//command : 9
	MAL_MOTOR_ACPANA232_C9M0TypeDef C9M0;//알람 읽기 210625
	MAL_MOTOR_ACPANA232_C9M4TypeDef C9M4;//알람 클리어  20201109
	MAL_MOTOR_ACPANA232_C9MbTypeDef C9Mb;//앱소클리어
}MAL_MOTOR_ACPANA232_FuncTypeDef;
#pragma pack()
//===============================================================================
extern uint8_t MAL_Motor_AcPanasonic_232_ReturnError(uint8_t errorCode);
extern uint8_t MAL_Motor_AcPanasonic_232_GetDataLen(MAL_MOTOR_ACPANA232_DataBundleTypeDef *pData);

extern void MAL_Motor_AcPanasonic_232_GetPulseCounter(void);
extern void MAL_Motor_AcPanasonic_232_GetAbsoluteCounter(void);
extern void MAL_Motor_AcPanasonic_232_GetAlmNumber(void);
extern void MAL_Motor_AcPanasonic_232_SetAbsoluteClear(void);
extern void MAL_Motor_AcPanasonic_232_SetAlmClear(void);


#endif /* INC_MAL_MOTOR_ACPANA232FUNC_H_ */
