/*
 * mal_loop_process.h
 *
 *  Created on: 2020. 3. 19.
 *      Author: shin
 */

#ifndef INC_MAL_LOOP_PROCESS_H_
#define INC_MAL_LOOP_PROCESS_H_



#define PROCESS_REGIST_SIZE 20

typedef struct __MAL_LOOP_Process_HandleTypeDef
{
	/*call back*/
	void (*mal_loop_process_callBack[PROCESS_REGIST_SIZE])(void);

	uint32_t processCnt;

	uint32_t runningNum;
}MAL_LOOP_Process_HandleTypeDef;



extern void MAL_LOOP_ProcessHandler(void);
extern void MAL_LOOP_ProcessAddr(void (*function)());

#endif /* INC_MAL_LOOP_PROCESS_H_ */
