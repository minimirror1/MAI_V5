/*
 * mal_motor_acPanaCurve.c
 *
 *  Created on: 2020. 11. 11.
 *      Author: shin
 */


#include "mal_motor_acPanaCurve.h"



extern MAL_MOTOR_ACPANA_CurveTypeDef panaCurve;


void MAL_Motor_AcPanasonic_Curve_Init(float y1,float y2,float d1,float d2)
{
	if(panaCurve.InitFlag == SET) //Clear 가 선행되고 나서 진입할수 있다.
	{
		panaCurve.TimeRange = CURVE_TIME;
		panaCurve.TimeQuantum = CURVE_TIME_QUANTUM;
		panaCurve.TimeQuantumCnt = panaCurve.TimeRange / panaCurve.TimeQuantum;
		panaCurve.TimeCnt = 0;
		MAL_Motor_AcPanasonic_Curve_Hermite3(y1,y2,d1,d2);

		panaCurve.InitFlag = RESET;
	}
}
void MAL_Motor_AcPanasonic_Curve_Clear(void)
{

	panaCurve.InitFlag = SET;
	panaCurve.a = 0;
	panaCurve.b = 0;
	panaCurve.c = 0;
	panaCurve.d = 0;
	panaCurve.offsetX = 0;
	panaCurve.offsetY = 0;
	panaCurve.targetY = 0;

	panaCurve.TimeCnt = 0;
}

//(float x1,float y1,float x2,float y2,float d1,float d2)
void MAL_Motor_AcPanasonic_Curve_Hermite3(float y1,float y2,float d1,float d2)
{
	float x1, x2;
	x1 = 0;
	x2 = panaCurve.TimeRange;


	panaCurve.targetY = y2;

	panaCurve.offsetX=x1;
	panaCurve.offsetY=y1;

    float x = x2 - x1;
    float y = y2 - y1;

    panaCurve.a = 0;
    panaCurve.b = d1;
    float A = (panaCurve.b + d2)*x - 2*y;
    panaCurve.d = A/(x*x*x);
    panaCurve.c = (d2-panaCurve.b-3*panaCurve.d*x*x)/(2*x);
}

int32_t MAL_Motor_AcPanasonic_Curve_CalcHermiteY(void)
{
	int32_t temp;
    float ret;
    float x1 = (float)panaCurve.TimeCnt;
    float x2 = x1*x1;
    float x3 = x2*x1;

	ret = panaCurve.a;
	ret += (panaCurve.b * x1);
	ret += (panaCurve.c * x2);
	ret += (panaCurve.d * x3);

    ret += panaCurve.offsetY;

    temp = (int32_t) ret;
    if(panaCurve.TimeCnt < panaCurve.TimeRange)
    {
    	panaCurve.TimeCnt += (float)CURVE_TIME_QUANTUM;
    }
    else
    {
    	temp = panaCurve.targetY;
    }


    return temp;
}

int32_t MAL_Motor_AcPanasonic_Curve_CalcHermiteX(float x)
{
    float ret;

    ret = panaCurve.offsetX + x;

    return (int32_t)ret;
}

