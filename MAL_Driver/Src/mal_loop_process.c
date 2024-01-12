/*
 * mal_loop_process.c
 *
 *  Created on: 2020. 3. 19.
 *      Author: shin
 */





#include "main.h"
#include "mal_loop_process.h"


MAL_LOOP_Process_HandleTypeDef mloopManager = {0,};


void MAL_LOOP_ProcessHandler(void)
{
	if(mloopManager.processCnt == 0)
		return;

	mloopManager.mal_loop_process_callBack[mloopManager.runningNum]();
	mloopManager.runningNum++;
	if(mloopManager.runningNum >= mloopManager.processCnt)
	{
		mloopManager.runningNum = 0;
	}
}



void MAL_LOOP_ProcessAddr(void (*function)())
{
	mloopManager.mal_loop_process_callBack[mloopManager.processCnt] = function;
	mloopManager.processCnt++;
	if(mloopManager.processCnt >= PROCESS_REGIST_SIZE)
	{
		Error_Handler();
	}
}



