/*
 * mal_systimer.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: shin
 */


#include "main.h"
#include "mal_systimer.h"

/**
  * @간략      매개변수 time과 systimer 차이 계산
  * @매개변수 time
  * @반환      tick차이
  */
uint32_t MAL_SysTimer_Elapsed ( uint32_t time )
{
    if (HAL_GetTick() >= time) return (HAL_GetTick() - time               );
    else                             return ((0xffffffff - time)+ HAL_GetTick()+1);
}

/**
  * @간략      매개변수 time1,2 의 차이 계산
  * @매개변수 time1
  * @매개변수 time2
  * @반환      time 차이
  */
uint32_t MAL_SysTimer_GetElapsed (uint32_t time1, uint32_t time2)
{
    if (time1 >= time2) return (time1 - time2);
    else                return ((0xffffffff-time2) + time1+1);
}

/**
  * @간략      현재 tick 반환
  * @반환      tick
  */
uint32_t MAL_SysTimer_GetTickCount(void)
{
    return HAL_GetTick();
}

/**
  * @간략      매개변수 time마다 호출할 function 등록
  * @매개변수 *function 호출할 함수
  * @매개변수 *timer 타임스탬프 변수 주소
  * @매개변수 time 함수 호출 주기시간
  * @반환      없음
  */
void MAL_SysTimer_FunctionCall(void (*function)(), uint32_t *timer, uint32_t time )
{

	if (MAL_SysTimer_Elapsed(*timer) >= time) {
		function();
		*timer = HAL_GetTick();
	}
}
