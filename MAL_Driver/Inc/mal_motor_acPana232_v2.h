/*
 * mal_motor_acPana232_v2.h
 *
 *  Created on: 2020. 10. 23.
 *      Author: shin
 */

#ifndef INC_MAL_MOTOR_ACPANA232_V2_H_
#define INC_MAL_MOTOR_ACPANA232_V2_H_




#include "mal_uart.h"
#include "mal_uart_packet.h"

#define ACPANA232_ENQ 0x05
#define ACPANA232_EOT 0x04
#define ACPANA232_ACK 0x06
#define ACPANA232_NAK 0x15

#define ACPANA232_QUEUE_SIZE 30



typedef enum __MAL_MOTOR_ACPANA232_Packet_ParserSequnce
{
	MAL_MOTOR_ACPANA232_PARSER_H_ENQ = 0, 	//호스트->노드 : 통신 요청
	MAL_MOTOR_ACPANA232_PARSER_N_EOT,	  	//노드 : 수신대기
	MAL_MOTOR_ACPANA232_PARSER_H_CMD,		//호스트->노드 : 모드,지령 전송
	MAL_MOTOR_ACPANA232_PARSER_N_ACK,		//노드 : 스신 응답
	MAL_MOTOR_ACPANA232_PARSER_N_ENQ,		//노드->호스트 : 통신 요청
	MAL_MOTOR_ACPANA232_PARSER_H_EOT,		//호스트 : 수신대기
	MAL_MOTOR_ACPANA232_PARSER_N_DATA,		//노드 : 데이터 전송
	MAL_MOTOR_ACPANA232_PARSER_H_ACK		//호스트 : 응답

}MAL_MOTOR_ACPANA232_Packet_ParserSequnce;


#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef
{
	uint8_t len;
	uint8_t nodeId;
	union{
		struct{
			uint8_t cmd : 4;
			uint8_t mode : 4;
		};
		uint8_t modeCmd;
	};
	uint8_t payload[0];
}MAL_MOTOR_ACPANA232_PacketDataHeaderTypeDef;
#pragma pack()



//==============================================================================

#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_DataBundleTypeDef
{
	uint8_t cmd;
	uint8_t mode;

	void (*mal_getDataProcess_CallBack)(uint8_t *data);//data full packet

}MAL_MOTOR_ACPANA232_DataBundleTypeDef;
#pragma pack()

typedef struct __MAL_MOTOR_ACPANA232_DataBundleQueueTypeDef {
	uint32_t front;
	uint32_t rear;
	MAL_MOTOR_ACPANA232_DataBundleTypeDef queue[ACPANA232_QUEUE_SIZE];

}MAL_MOTOR_ACPANA232_DataBundleQueueTypeDef;
#pragma pack()


#pragma pack(1)
typedef struct __MAL_MOTOR_ACPANA232_PacketHandleTypeDefv2
{
	MAL_UART_HandleTypeDef *muart;				/*hal uart handle*/

	MAL_UART_PacketParserTypeDef parser;

	MAL_MOTOR_ACPANA232_Packet_ParserSequnce sequnce;

	uint8_t rxBuff[100];
	uint8_t txBuff[100];

	MAL_MOTOR_ACPANA232_DataBundleQueueTypeDef getQueue;
	MAL_MOTOR_ACPANA232_DataBundleQueueTypeDef setQueue;



	uint8_t errorFlag;
	uint32_t t_runFlag;

	uint32_t timeOutCnt;
	uint8_t f_error;



}MAL_MOTOR_ACPANA232_PacketHandleTypeDef;
#pragma pack()




extern void MAL_Motor_AcPanasonic_232_RegInit(MAL_UART_HandleTypeDef *muart);
extern void MAL_Motor_AcPanasonic_232PacketParsingProcess(MAL_MOTOR_ACPANA232_PacketHandleTypeDef *ac232Packet);
extern void MAL_Motor_AcPanasonic_232_Process(void);
extern void MAL_Motor_AcPanasonic_232_GetDataAddQueue(MAL_MOTOR_ACPANA232_DataBundleTypeDef *data);

#endif /* INC_MAL_MOTOR_ACPANA232_V2_H_ */
