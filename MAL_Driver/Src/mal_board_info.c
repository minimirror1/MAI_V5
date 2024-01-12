/*
 * mal_board_id.c
 *
 *  Created on: 2020. 3. 23.
 *      Author: shin
 */
#include "main.h"
#include "mal_board_info.h"

#include "mal_hal_header.h"

MAL_BOARD_InfoTypeDef mBoradInfo = {0,};

//Init===================================================================
//id--------------------------------------------------------------------
void MAL_Board_ID_InitPort(GPIO_TypeDef* GPIOx, uint16_t Pin, GPIO_PinState PinState)
{
	if(mBoradInfo.id.digitCnt >= MAL_BOARD_ID_DIGIT)
		return;

	mBoradInfo.id.port[mBoradInfo.id.digitCnt].initFlag = SET;
	mBoradInfo.id.port[mBoradInfo.id.digitCnt].GPIOx = GPIOx;
	mBoradInfo.id.port[mBoradInfo.id.digitCnt].Pin = Pin;
	mBoradInfo.id.port[mBoradInfo.id.digitCnt].PinState = PinState;
	mBoradInfo.id.port[mBoradInfo.id.digitCnt].digit = mBoradInfo.id.digitCnt;

	mBoradInfo.id.digitCnt++;

}
//-------------------------------------------------------------------
//version--------------------------------------------------------------------
void MAL_Board_Version_init(uint8_t majer, uint8_t minor, uint8_t build)
{
	mBoradInfo.version.majer = majer;
	mBoradInfo.version.minor = minor;
	mBoradInfo.version.build = build;
}
//-------------------------------------------------------------------
//===================================================================


//function===================================================================
//id--------------------------------------------------------------------
static inline uint8_t MAL_Board_ID_ReadBitValue(MAL_BOARD_IdPortTypeDef *port)
{
	if(port->initFlag != SET)
		return 0;

	if(HAL_GPIO_ReadPin(port->GPIOx, port->Pin) == port->PinState)
		return 1 << port->digit;

	return 0;
}
//-------------------------------------------------------------------
//===================================================================


//GET===================================================================
//id--------------------------------------------------------------------
uint8_t MAL_Board_ID_GetValue(void)
{
	return mBoradInfo.id.value;
}

uint8_t MAL_Board_ID_GetReloadValue(void)
{
	MAL_Board_ID_SetReadValue();
	return mBoradInfo.id.value;
}
//-------------------------------------------------------------------
//version--------------------------------------------------------------------
uint8_t MAL_Board_Version_GetValueMajer(void)
{
	return mBoradInfo.version.majer;
}
uint8_t MAL_Board_Version_GetValueMinor(void)
{
	return mBoradInfo.version.minor;
}
uint8_t MAL_Board_Version_GetValueBuild(void)
{
	return mBoradInfo.version.build;
}
//-------------------------------------------------------------------
//===================================================================


//SET===================================================================
//id--------------------------------------------------------------------
void MAL_Board_ID_SetReadValue(void)
{
	uint8_t temp = 0;
	uint8_t i = 0;
	for(i = 0; i < MAL_BOARD_ID_DIGIT; i++)
	{
		temp |= MAL_Board_ID_ReadBitValue(&mBoradInfo.id.port[i]);
	}
	mBoradInfo.id.value = temp;
}
//-------------------------------------------------------------------
//===================================================================

