/*
 * modbus_task.h
 *
 *  Created on: Oct 21, 2025
 *      Author: TAMRD
 */

#ifndef INC_MODBUS_TASK_H_
#define INC_MODBUS_TASK_H_

#include "stm32f4xx_hal.h"

typedef struct
{
  uint8_t frame_ready; // Có khung dữ liệu mới
  uint8_t busy;        // �?ang xử lý 1 khung
  uint8_t error;       // Có lỗi (CRC hoặc sai địa chỉ)
  uint16_t rx_size;    // Kích thước khung nhận được
} ModbusStatus_t;

extern volatile ModbusStatus_t modbus_flags;

void Modbus_TaskInit(UART_HandleTypeDef *huart);
void Modbus_TaskUpdate(void);

#endif /* INC_MODBUS_TASK_H_ */
