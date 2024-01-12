/*
 * mal_systimer.h
 *
 *  Created on: 2020. 3. 11.
 *      Author: shin
 */

#ifndef INC_MAL_SYSTIMER_H_
#define INC_MAL_SYSTIMER_H_



#include "mal_hal_header.h"





extern uint32_t MAL_SysTimer_Elapsed ( uint32_t time );
extern uint32_t MAL_SysTimer_GetElapsed (uint32_t time1, uint32_t time2);
extern uint32_t MAL_SysTimer_GetTickCount(void);

extern void MAL_SysTimer_FunctionCall(void (*function)(), uint32_t *timer, uint32_t time );


#endif /* INC_MAL_SYSTIMER_H_ */
