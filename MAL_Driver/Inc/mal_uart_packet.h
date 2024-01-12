/*
 * mal_uart_packet.h
 *
 *  Created on: Mar 27, 2020
 *      Author: shin
 */

#ifndef INC_MAL_UART_PACKET_H_
#define INC_MAL_UART_PACKET_H_

#include "mal_uart.h"

#define MAL_UART_PACKET_BUFFER	512

#define MAL_UART_PARSER_STX_CODE 	0x16
#define MAL_UART_PARSER_STX_SIZE	4

#define MAL_UART_PARSER_HEADER_LEN  sizeof(MAL_UART_Packet_LengthTypeDef)
#define MAL_UART_PARSER_HEADER_SIZE	sizeof(MAL_UART_Packet_HeaderTypeDef)

//packet
#define MAL_UART_MSG_TYPE_SET	0
#define MAL_UART_MSG_TYPE_GET	1
#define MAL_UART_MSG_TYPE_RSP	2

typedef enum __MAL_UART_Packet_ParserSequnce
{
	MAL_UART_PACKET_PARSER_STX = 0,
	MAL_UART_PACKET_PARSER_LENGTH,
	MAL_UART_PACKET_PARSER_HEADER,
	MAL_UART_PACKET_PARSER_BODY,
	MAL_UART_PACKET_PARSER_CRC

}MAL_UART_Packet_ParserSequnce;


//*data packet*//
#pragma pack(1)
typedef struct __MAL_UART_Packet_StxTypeDef {
	uint8_t stx[MAL_UART_PARSER_STX_SIZE];
	uint8_t payload[0];
} MAL_UART_Packet_StxTypeDef;
#pragma pack()

#pragma pack(1)
typedef struct MAL_UART_Packet_LengthTypeDef {
	uint16_t len;
	uint8_t payload[0];
} MAL_UART_Packet_LengthTypeDef;
#pragma pack()


#pragma pack(1)
typedef struct __MAL_UART_Packet_HeaderTypeDef{
		uint8_t tarId;
		uint8_t srcId;
		uint8_t cmd;
		uint8_t msgType;
		uint8_t payload[0];
}MAL_UART_Packet_HeaderTypeDef;
#pragma pack()

#pragma pack(1)
typedef struct __MAL_UART_PacketParserCRC32TypeDef
{
	uint8_t crcCnt;
	union{
		uint8_t crcByte[4];
		uint32_t crc32;
	};
}MAL_UART_PacketParserCRC32TypeDef;
#pragma pack()


//*parser*//
#pragma pack(1)
typedef struct __MAL_UART_PacketParserTypeDef
{
	uint8_t rxData;			//수신 1byte

	uint32_t t_disjunction; //패킷 분리 시간

	MAL_UART_Packet_ParserSequnce sequence;		//패킷 해석 순서

	uint32_t len;

	uint32_t seqLen;

	uint8_t buffer[MAL_UART_PACKET_BUFFER];

	MAL_UART_PacketParserCRC32TypeDef crc;

}MAL_UART_PacketParserTypeDef;
#pragma pack()


#pragma pack(1)
typedef struct __MAL_UART_PacketHandleTypeDef
{
	MAL_UART_HandleTypeDef *muart;				/*hal uart handle*/

	MAL_UART_PacketParserTypeDef parser;

	void (*mal_uart_packetReceiveCallBack)(uint32_t *muartPacket);


}MAL_UART_PacketHandleTypeDef;
#pragma pack()

extern void MAL_UART_PacketProcess(void);
extern void MAL_UART_PacketSend(MAL_UART_PacketHandleTypeDef *muartPacket);
extern void MAL_UART_PacketRegist(MAL_UART_HandleTypeDef *muart,void (*function)());


#endif /* INC_MAL_UART_PACKET_H_ */
