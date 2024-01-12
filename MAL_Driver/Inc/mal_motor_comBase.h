/*
 * mal_motor_comBase.h
 *
 *  Created on: Feb 14, 2022
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_COMBASE_H_
#define INC_MAL_MOTOR_COMBASE_H_

#include "main.h"
#include "mal_uart.h"
#include "mal_uart_packet.h"


typedef enum __MAL_MOTOR_COMBASE_ParserSequnce
{
	MAL_MOTOR_COMBASE_PARSER_STX = 0,
	MAL_MOTOR_COMBASE_PARSER_CMD,
	MAL_MOTOR_COMBASE_PARSER_LEN,
	MAL_MOTOR_COMBASE_PARSER_DATA,
	MAL_MOTOR_COMBASE_PARSER_CRC,
	MAL_MOTOR_COMBASE_PARSER_END
}MAL_MOTOR_COMBASE_ParserSequnce;

//parsing-----------------------------------------------------------------------
#define COMBASE_PARSER_STX	'['
#define COMBASE_PARSER_END	']'
//cmd---------------------------------------------------------------------------
#define COMBASE_CMD_IDDEF	'I'

#pragma pack(1)


//==============================================================================
typedef struct __MAL_MOTOR_COMBASE_Packet_Start_TypeDef
{
	uint8_t stx;
	uint8_t payload[0];
}MAL_MOTOR_COMBASE_Packet_Start_TypeDef;
typedef struct __MAL_MOTOR_COMBASE_Packet_Header_TypeDef
{
	uint8_t cmd;
	uint8_t len;
	uint8_t payload[0];
}MAL_MOTOR_COMBASE_Packet_Header_TypeDef;
typedef struct __MAL_MOTOR_COMBASE_Packet_Crc_TypeDef
{
	uint16_t crc;
	uint8_t payload[0];
}MAL_MOTOR_COMBASE_Packet_Crc_TypeDef;
typedef struct __MAL_MOTOR_COMBASE_Packet_End_TypeDef
{
	uint8_t end;
}MAL_MOTOR_COMBASE_Packet_End_TypeDef;
//------------------------------------------------------------------------------


typedef struct __MAL_MOTOR_COMBASE_GroupIdDef_TypeDef
{
	uint8_t groupId;
	uint8_t subId;
	uint8_t payload[0];
}MAL_MOTOR_COMBASE_GroupIdDef_TypeDef;



//==============================================================================

typedef struct __MAL_MOTOR_COMBASE_PacketHandleTypeDef
{
	MAL_UART_HandleTypeDef *muart;				/*hal uart handle*/

	MAL_UART_PacketParserTypeDef parser;

	MAL_MOTOR_COMBASE_ParserSequnce sequnce;


	uint8_t rxBuff[100];
	uint8_t txBuff[100];



	uint8_t errorFlag;
	uint32_t t_runFlag;

	uint32_t timeOutCnt;
	uint8_t f_error;

}MAL_MOTOR_COMBASE_PacketHandleTypeDef;
#pragma pack()



void MAL_Motor_ComBase_RegInit(MAL_UART_HandleTypeDef *muart);
void MAL_Motor_ComBase_Process(void);


#endif /* INC_MAL_MOTOR_COMBASE_H_ */
