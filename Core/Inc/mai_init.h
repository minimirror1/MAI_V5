/*
 * mai_init.h
 *
 *  Created on: 2020. 3. 18.
 *      Author: shin
 */

#ifndef INC_MAI_INIT_H_
#define INC_MAI_INIT_H_

#include "main.h"

#define MAL_VERSION_MAJER 0
#define MAL_VERSION_MINOR 1
#define MAL_VERSION_BUILD 4

//v4
//0.0.1 초기버전
//0.0.2 와치독 활성화
//0.0.3 센서 초기화 후 센서 감지시 상태 알람
//0.0.4 원점 5도 오프셋, 센서 감지시 알람 패킷 송신 500ms 반복
//0.0.5 원점 이동 부드럽게

//0.0.8 초기위치 이동 cmd 0x0C
//0.0.9 센서 감지시 멈춤
//0.0.10 엡소모드 추가

//0.1.0 초기위치 이동 Curve 5초

//0.0.4 1.5차버전   엡소값 읽기 폴링응답

//v5
//0.0.1 하드웨어 테스트
//0.0.2 홈 센서 고정 home = ccw
//0.1.4 엡소 폴링 읽기에서 아이디 검사 0 고정 -> 수신아이디와 내 아이디 같은지 검사
//0.1.5 감속비 입력범위 1byte -> 2byte 변경



//배포시 와치독 활성화 필수!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define IWDG_INIT

#define MAL_TEST_MODE


typedef struct __MAL_BoardFwVerTypeDef
{
	uint8_t Majer;//Majer
	uint8_t Minor;//Minor
	uint8_t Build;//Build
}MAL_BoardFwVerTypeDef;

typedef struct __MAL_BoardManagerTypeDef
{
	uint8_t myCanId;
	MAL_BoardFwVerTypeDef fwVersion;
}MAL_BoardManagerTypeDef;


extern void MAL_MAI_V1_Init(void);

#endif /* INC_MAI_INIT_H_ */
