/*
 * modbus_task.h
 *
 *  Created on: Oct 21, 2025
 *      Author: TAMRD
 */

#ifndef INC_MODBUS_TASK_H_
#define INC_MODBUS_TASK_H_

#include "stm32f4xx_hal.h"

typedef struct {
    volatile uint8_t frame_ready;
    volatile uint8_t busy;
    volatile uint8_t error;
    volatile uint8_t uart_hung;
    volatile uint16_t rx_size;
    volatile uint32_t last_error;
    volatile uint32_t error_count;
    volatile uint32_t frame_count;
} ModbusStatus_t;

extern volatile ModbusStatus_t modbus_flags;

void Modbus_TaskInit(UART_HandleTypeDef *huart);
void Modbus_TaskUpdate(void);         // Gọi trong main loop
void Modbus_ExecuteCommands(void);    // Gọi mỗi 10ms

#endif /* INC_MODBUS_TASK_H_ */
