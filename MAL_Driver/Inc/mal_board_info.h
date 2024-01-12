/*
 * mal_board_id.h
 *
 *  Created on: 2020. 3. 23.
 *      Author: shin
 */

#ifndef INC_MAL_BOARD_INFO_H_
#define INC_MAL_BOARD_INFO_H_

#include "main.h"

#define MAL_BOARD_ID_DIGIT 5

//id=================================================================
typedef struct __MAL_BOARD_IdPortTypeDef
{
	uint8_t initFlag;
	GPIO_TypeDef* GPIOx;
	uint16_t Pin;
	GPIO_PinState PinState;
	uint8_t digit;
}MAL_BOARD_IdPortTypeDef;

typedef struct __MAL_BOARD_IdTypeDef
{
	MAL_BOARD_IdPortTypeDef port[MAL_BOARD_ID_DIGIT];
	uint8_t value;
	uint8_t digitCnt;
}MAL_BOARD_IdTypeDef;
//===================================================================


//version=================================================================
typedef struct __MAL_BOARD_versionTypeDef
{
	uint8_t majer;
	uint8_t minor;
	uint8_t build;
}MAL_BOARD_versionTypeDef;

//===================================================================

//board==============================================================
typedef struct __MAL_BOARD_InfoTypeDef
{
	MAL_BOARD_IdTypeDef id;
	MAL_BOARD_versionTypeDef version;
}MAL_BOARD_InfoTypeDef;
//===================================================================



//Init===================================================================
//id--------------------------------------------------------------------
void MAL_Board_ID_InitPort(GPIO_TypeDef* GPIOx, uint16_t Pin, GPIO_PinState PinState);
//-------------------------------------------------------------------
//version--------------------------------------------------------------------
void MAL_Board_Version_init(uint8_t majer, uint8_t minor, uint8_t build);
//-------------------------------------------------------------------
//===================================================================

//GET===================================================================
//id--------------------------------------------------------------------
uint8_t MAL_Board_ID_GetValue(void);
uint8_t MAL_Board_ID_GetReloadValue(void);
//-------------------------------------------------------------------
//version--------------------------------------------------------------------
uint8_t MAL_Board_Version_GetValueMajer(void);
uint8_t MAL_Board_Version_GetValueMinor(void);
uint8_t MAL_Board_Version_GetValueBuild(void);
//-------------------------------------------------------------------
//===================================================================

//SET===================================================================
//id--------------------------------------------------------------------
void MAL_Board_ID_SetReadValue(void);

//-------------------------------------------------------------------

#endif /* INC_MAL_BOARD_INFO_H_ */
