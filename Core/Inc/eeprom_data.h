/*
 * eeprom_data.h
 *
 *  Created on: 2020. 10. 30.
 *      Author: shin
 */

#ifndef INC_EEPROM_DATA_H_
#define INC_EEPROM_DATA_H_


//최소 uint 16 사용
#pragma pack(1)
typedef struct __EEPemul_Data_TypeDef
{
	uint16_t flag;
	uint32_t encoderSt1_cnt;

}EEPemul_Data_TypeDef;
#pragma pack()
#endif /* INC_EEPROM_DATA_H_ */
