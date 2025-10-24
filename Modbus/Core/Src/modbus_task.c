/*
 * modbus_task.c
 *
 *  Created on: Oct 21, 2025
 *      Author: TAMRD
 */

#include "modbus_task.h"
#include "modbusSlave.h"
#include "motor_ctr.h"
#include "axis_task.h"

uint8_t RxData[BUFF_SIZE];
uint8_t TxData[BUFF_SIZE];
volatile ModbusStatus_t modbus_flags = {0};
UART_HandleTypeDef *modbus_uart;

void Modbus_TaskInit(UART_HandleTypeDef *huart)
{
    modbus_uart = huart;
    HAL_UARTEx_ReceiveToIdle_IT(modbus_uart, RxData, sizeof(RxData));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != modbus_uart) return;

    if (RxData[0] == SLAVE_ID)
    {
        modbus_flags.frame_ready = 1;
        modbus_flags.rx_size = Size;
    }
    else
        modbus_flags.error = 1;

    HAL_UARTEx_ReceiveToIdle_IT(modbus_uart, RxData, sizeof(RxData));
}

void Modbus_TaskUpdate(void)
{
    if (!modbus_flags.frame_ready || modbus_flags.busy) return;

    modbus_flags.busy = 1;
    modbus_flags.frame_ready = 0;

    uint8_t func = RxData[1];
    switch (func)
    {
    case 0x03: readHoldingRegs(); break;
    case 0x04: readInputRegs(); break;
    case 0x01: readCoils(); break;
    case 0x02: readInputs(); break;
    case 0x05: writeCoil(); break;
    case 0x0F: writeCoils(); break;
    case 0x06: writeSingleReg(); break;
    case 0x10: writeHoldingRegs(); break;
    default:   modbusException(ILLEGAL_FUNCTION); break;
    }

    // Xử lý điều khiển từ coil
    uint8_t coils = Coils_Database[0];
    uint8_t coil2 = Coils_Database[1];

#if   (STEPS_PER_MM == 1000u)
    if ((coils >> 3) & 1) Axis_QueueCommand(AXIS_X, LEFT,  100, 2.0f);
    if ((coils >> 4) & 1) Axis_QueueCommand(AXIS_X, RIGHT, 100, 2.0f);
    if ((coils >> 5) & 1) Axis_QueueCommand(AXIS_Y, BACKWARD, 100, 2.0f);
    if ((coils >> 6) & 1) Axis_QueueCommand(AXIS_Y, FORWARD, 100, 2.0f);
#elif (STEPS_PER_MM == 1u)
    if ((coils >> 3) & 1) Axis_QueueCommand(AXIS_X, LEFT,  10, 5000.0f);
    if ((coils >> 4) & 1) Axis_QueueCommand(AXIS_X, RIGHT, 10, 5000.0f);
    if ((coils >> 5) & 1) Axis_QueueCommand(AXIS_Y, BACKWARD, 10, 5000.0f);
    if ((coils >> 6) & 1) Axis_QueueCommand(AXIS_Y, FORWARD, 10, 5000.0f);
#else
  #error "STEPS_PER_MM is 1000u or 100u"
#endif
    if ((coil2 >> 7) & 1)
    {
        Coils_Database[1] &= ~(1 << 7);
        Axis_QueueHomeAll();
    }
    else if ((coils >> 2) & 1)
    {
        Coils_Database[0] = 0;
        Axis_QueueMoveTo(
            Holding_Registers_Database[0],
            Holding_Registers_Database[1],
            50000.0f);
    }
    modbus_flags.busy = 0;
}

