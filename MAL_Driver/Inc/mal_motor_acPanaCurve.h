/*
 * mal_motor_acPanaCurve.h
 *
 *  Created on: 2020. 11. 11.
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_ACPANACURVE_H_
#define INC_MAL_MOTOR_ACPANACURVE_H_

#include "main.h"


#define CURVE_TIME	5000
#define CURVE_TIME_QUANTUM 20

typedef struct __MAL_MOTOR_ACPANA_CurveTypeDef
{
	unsigned char InitFlag;

	float a;
	float b;
	float c;
	float d;

	float offsetX,offsetY;

	int32_t targetY;

	int32_t TimeRange;//커브 시간
	int32_t TimeQuantum;//커브 키프레임
	int32_t TimeQuantumCnt;//키프레임 수

	int32_t TimeCnt;

}MAL_MOTOR_ACPANA_CurveTypeDef;





extern void MAL_Motor_AcPanasonic_Curve_Init(float y1,float y2,float d1,float d2);
extern void MAL_Motor_AcPanasonic_Curve_Clear(void);
extern void MAL_Motor_AcPanasonic_Curve_Hermite3(float y1,float y2,float d1,float d2);
extern int32_t MAL_Motor_AcPanasonic_Curve_CalcHermiteY(void);
extern int32_t MAL_Motor_AcPanasonic_Curve_CalcHermiteX(float x);


#endif /* INC_MAL_MOTOR_ACPANACURVE_H_ */
