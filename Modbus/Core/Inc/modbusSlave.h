/*
 * modbusSlave.h
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f4xx_hal.h"

#define SLAVE_ID 1

#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03

extern uint8_t Coils_Database[25];
extern uint16_t Holding_Registers_Database[50];
extern uint8_t RxData[256];
extern uint8_t TxData[256];
extern UART_HandleTypeDef huart1;

uint8_t readHoldingRegs (void);
uint8_t readInputRegs (void);

uint8_t readCoils (void);
uint8_t readInputs (void);

uint8_t writeSingleReg (void);
uint8_t writeHoldingRegs (void);

uint8_t writeCoil (void);
uint8_t writeCoils (void);

void modbusException (uint8_t exceptioncode);

#endif /* INC_MODBUSSLAVE_H_ */
