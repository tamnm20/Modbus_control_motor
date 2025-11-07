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
#define RX_BUFF_SIZE 16
#define TX_BUFF_SIZE 256
#define UART_TIMEOUT_MS 5000

extern uint8_t Coils_Database[25];
extern uint16_t Holding_Registers_Database[50];

extern volatile uint8_t RxData[RX_BUFF_SIZE];
extern uint8_t TxData[TX_BUFF_SIZE];
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef *modbus_uart;

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
