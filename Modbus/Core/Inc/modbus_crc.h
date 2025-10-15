/*
 * modbus_crc.h
 *
 *  Created on: Sep 16, 2022
 *      Author: arunr
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_
#include "stdint.h"

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
uint16_t ModBus_CRC16(uint8_t *data, uint16_t length);


#endif /* INC_MODBUS_CRC_H_ */
