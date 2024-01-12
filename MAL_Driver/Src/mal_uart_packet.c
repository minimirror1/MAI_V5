/*
 * mal_uart_protocol.c
 *
 *  Created on: Mar 27, 2020
 *      Author: shin
 */

#include "main.h"
#include "mal_hal_header.h"
#include "mal_uart_packet.h"
#include "mal_uart.h"
#include "mal_systimer.h"


MAL_UART_PacketHandleTypeDef muartPacket;

#ifdef HAL_CRC_MODULE_ENABLED
extern CRC_HandleTypeDef hcrc;
#endif

void MAL_UART_PacketParsingProcess(MAL_UART_PacketHandleTypeDef *muartPacket);

#if 0

uint16_t swaph(uint16_t data)
{
	uint16_t ret = (data << 8);
	ret |= (data >> 8);
	return ret;
}
#endif


void MAL_UART_PacketSend(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	MAL_UART_Packet_StxTypeDef *packetStx = (MAL_UART_Packet_StxTypeDef *)muartPacket->parser.buffer;
	MAL_UART_Packet_LengthTypeDef *lenHeader = (MAL_UART_Packet_LengthTypeDef *)packetStx->payload;
	MAL_UART_Packet_HeaderTypeDef *packetHeader = (MAL_UART_Packet_HeaderTypeDef *)lenHeader->payload;

	uint16_t tempLen = 0;
#ifdef HAL_CRC_MODULE_ENABLED
	muartPacket->parser.crc.crc32 = HAL_CRC_Calculate(&hcrc,(uint32_t *)&packetHeader->tarId,lenHeader->len);
#endif
	tempLen = sizeof(MAL_UART_Packet_StxTypeDef) +sizeof(MAL_UART_Packet_LengthTypeDef) + lenHeader->len;

	MAL_UART_SendDataStream(muartPacket->muart, muartPacket->parser.buffer, tempLen);
	MAL_UART_SendDataStream(muartPacket->muart, (uint8_t *)&muartPacket->parser.crc.crc32, 4);
}

void MAL_UART_PacketProcess(void)
{
	MAL_UART_PacketParsingProcess(&muartPacket);
}

void MAL_UART_PacketRegist(MAL_UART_HandleTypeDef *muart,void (*function)())
{
	muartPacket.muart = muart;
	muartPacket.mal_uart_packetReceiveCallBack = function;
}



void MAL_UART_PacketParser_STX(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	if(muartPacket->parser.rxData == MAL_UART_PARSER_STX_CODE)
	{
		muartPacket->parser.buffer[muartPacket->parser.len] = muartPacket->parser.rxData;
		muartPacket->parser.len++;

		if(muartPacket->parser.len == MAL_UART_PARSER_STX_SIZE)
		{
			muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_LENGTH;
			muartPacket->parser.len = 0;
			muartPacket->parser.crc.crcCnt = 0;
		}
	}
	else
	{
		muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_STX;
		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}
void MAL_UART_PacketParser_LENGTH(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	MAL_UART_Packet_StxTypeDef *packetStx = (MAL_UART_Packet_StxTypeDef *)muartPacket->parser.buffer;
	//MAL_UART_Packet_LengthTypeDef *lenHeader = (MAL_UART_Packet_LengthTypeDef *)packetStx->payload;

	packetStx->payload[muartPacket->parser.len] = muartPacket->parser.rxData;
	muartPacket->parser.len++;
	if (muartPacket->parser.len == MAL_UART_PARSER_HEADER_LEN)
	{
		muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_HEADER;
		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}

void MAL_UART_PacketParser_HEADER(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	MAL_UART_Packet_StxTypeDef *packetStx = (MAL_UART_Packet_StxTypeDef *)muartPacket->parser.buffer;
	MAL_UART_Packet_LengthTypeDef *lenHeader = (MAL_UART_Packet_LengthTypeDef *)packetStx->payload;
	//MAL_UART_Packet_HeaderTypeDef *packetHeader = (MAL_UART_Packet_HeaderTypeDef *)lenHeader->payload;

	lenHeader->payload[muartPacket->parser.len] = muartPacket->parser.rxData;
	muartPacket->parser.len++;

	if (muartPacket->parser.len == MAL_UART_PARSER_HEADER_SIZE) {


		if(lenHeader->len == MAL_UART_PARSER_HEADER_SIZE)
			muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_CRC;
		else
			muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_BODY;

		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}


void MAL_UART_PacketParser_BODY(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	MAL_UART_Packet_StxTypeDef *packetStx = (MAL_UART_Packet_StxTypeDef *)muartPacket->parser.buffer;
	MAL_UART_Packet_LengthTypeDef *lenHeader = (MAL_UART_Packet_LengthTypeDef *)packetStx->payload;
	MAL_UART_Packet_HeaderTypeDef *packetHeader = (MAL_UART_Packet_HeaderTypeDef *)lenHeader->payload;

	packetHeader->payload[muartPacket->parser.len] = muartPacket->parser.rxData;
	muartPacket->parser.len++;

	if (muartPacket->parser.len == lenHeader->len) {
		muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_CRC;
		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}


void MAL_UART_PacketParser_CRC(MAL_UART_PacketHandleTypeDef *muartPacket)
{
	MAL_UART_Packet_StxTypeDef *packetStx = (MAL_UART_Packet_StxTypeDef *)muartPacket->parser.buffer;
	MAL_UART_Packet_LengthTypeDef *lenHeader = (MAL_UART_Packet_LengthTypeDef *)packetStx->payload;
	MAL_UART_Packet_HeaderTypeDef *packetHeader = (MAL_UART_Packet_HeaderTypeDef *)lenHeader->payload;

	muartPacket->parser.crc.crcByte[muartPacket->parser.crc.crcCnt] = muartPacket->parser.rxData;
	muartPacket->parser.crc.crcCnt++;

	if(muartPacket->parser.crc.crcCnt == 4)
	{
/*		muartPacket->parser.crc.crc32 = 0;
		muartPacket->parser.crc.crc32 |= muartPacket->parser.crc.crcByte[0] << 24;
		muartPacket->parser.crc.crc32 |= muartPacket->parser.crc.crcByte[1] << 16;
		muartPacket->parser.crc.crc32 |= muartPacket->parser.crc.crcByte[2] << 8;
		muartPacket->parser.crc.crc32 |= muartPacket->parser.crc.crcByte[3];*/

#ifdef HAL_CRC_MODULE_ENABLED
		//crc32 mpeg-2
		uint32_t rxDataCrc = HAL_CRC_Calculate(&hcrc,(uint32_t *)&packetHeader->tarId,lenHeader->len);

		if(muartPacket->parser.crc.crc32  == rxDataCrc)
		{
			muartPacket->mal_uart_packetReceiveCallBack((uint32_t *)muartPacket);
		}
#endif
		muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_STX;
		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}

void MAL_UART_PacketParsingProcess(MAL_UART_PacketHandleTypeDef *muartPacket) {

	if (MAL_UART_GetQueueData_Byte(muartPacket->muart, &muartPacket->parser.rxData) == MAL_UART_CONTAIN) {
		muartPacket->parser.t_disjunction = MAL_SysTimer_GetTickCount();

		switch (muartPacket->parser.sequence) {
		case MAL_UART_PACKET_PARSER_STX:
			MAL_UART_PacketParser_STX(muartPacket);
			break;
		case MAL_UART_PACKET_PARSER_LENGTH:
			MAL_UART_PacketParser_LENGTH(muartPacket);
			break;
		case MAL_UART_PACKET_PARSER_HEADER:
			MAL_UART_PacketParser_HEADER(muartPacket);
			break;
		case MAL_UART_PACKET_PARSER_BODY:
			MAL_UART_PacketParser_BODY(muartPacket);
			break;
		case MAL_UART_PACKET_PARSER_CRC:
			MAL_UART_PacketParser_CRC(muartPacket);
			break;
		}
	}

	if (MAL_SysTimer_Elapsed(muartPacket->parser.t_disjunction) >= 200) {
		muartPacket->parser.sequence = MAL_UART_PACKET_PARSER_STX;
		muartPacket->parser.len = 0;
		muartPacket->parser.crc.crcCnt = 0;
	}
}
