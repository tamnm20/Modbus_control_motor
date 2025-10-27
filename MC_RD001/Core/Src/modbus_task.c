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
#include <string.h>

volatile uint8_t RxData[RX_BUFF_SIZE];
uint8_t TxData[TX_BUFF_SIZE];
uint8_t ProcessBuffer[RX_BUFF_SIZE];
volatile ModbusStatus_t modbus_flags = {0};
UART_HandleTypeDef *modbus_uart;
static uint32_t last_rx_tick = 0;

/* ========== COMMAND FLAGS ========== */
typedef struct {
    uint8_t move_to_pending;
    uint8_t home_pending;
    uint8_t jog_x_left;
    uint8_t jog_x_right;
    uint8_t jog_y_back;
    uint8_t jog_y_forward;
} ModbusCmd_t;

static ModbusCmd_t mb_cmd = {0};

void Modbus_TaskInit(UART_HandleTypeDef *huart)
{
    modbus_uart = huart;
    last_rx_tick = HAL_GetTick();

    // Start reception với buffer size phù hợp
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(
        modbus_uart,
        (uint8_t*)RxData,
        RX_BUFF_SIZE  // ← 16 bytes
    );

    if(status != HAL_OK)
    {
        modbus_flags.error = 1;
        modbus_flags.uart_hung = 1;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != modbus_uart) return;

    modbus_flags.frame_count++;
    last_rx_tick = HAL_GetTick();

    // Validate frame
    if (RxData[0] == SLAVE_ID && Size >= 4 && Size <= RX_BUFF_SIZE)
    {
        // Copy to processing buffer
        memcpy(ProcessBuffer, (void*)RxData, Size);
        modbus_flags.rx_size = Size;
        modbus_flags.frame_ready = 1;
    }
    else
    {
        modbus_flags.error = 1;
    }

    // Restart reception
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(
        modbus_uart,
        (uint8_t*)RxData,
        RX_BUFF_SIZE  // ← 16 bytes
    );

    if(status != HAL_OK)
    {
        modbus_flags.uart_hung = 1;
        HAL_UART_Abort_IT(modbus_uart);
        HAL_Delay(1);
        HAL_UARTEx_ReceiveToIdle_IT(modbus_uart, (uint8_t*)RxData, RX_BUFF_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart != modbus_uart) return;

    modbus_flags.last_error = HAL_UART_GetError(huart);
    modbus_flags.error_count++;

    // Clear error flags (STM32F4 method)
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;  // Read SR to clear NE, FE, PE
    tmp = huart->Instance->DR;  // Read DR to clear ORE
    (void)tmp;

    // Abort & Restart
    HAL_UART_Abort_IT(huart);
    HAL_UARTEx_ReceiveToIdle_IT(modbus_uart, (uint8_t*)RxData, RX_BUFF_SIZE);
}



/* ========== MODBUS TASK ========== */

//void Modbus_TaskUpdate(void)
//{
//    static uint32_t last_update = 0;
////    static uint32_t last_update2 = 0;
//    uint32_t now = HAL_GetTick();
//
//
//    if(now - last_update >= 1)
//    {
//        last_update = now;
//        // 1. Xử lý frame Modbus
//		if (modbus_flags.frame_ready && !modbus_flags.busy)
//        {
//            modbus_flags.busy = 1;
//            modbus_flags.frame_ready = 0;
//
//            uint8_t func = RxData[1];
//            switch (func)
//            {
//            case 0x03: readHoldingRegs(); break;
//            case 0x04: readInputRegs(); break;
//            case 0x01: readCoils(); break;
//            case 0x05: writeCoil(); break;
//            case 0x06: writeSingleReg(); break;
//            case 0x10: writeHoldingRegs(); break;
//            default:   modbusException(ILLEGAL_FUNCTION); break;
//            }
//
//            uint8_t coils = Coils_Database[0];
//            uint8_t coil2 = Coils_Database[1];
//
//            mb_cmd.jog_x_left    = (coils >> 3) & 1;
//            mb_cmd.jog_x_right   = (coils >> 4) & 1;
//            mb_cmd.jog_y_back    = (coils >> 5) & 1;
//            mb_cmd.jog_y_forward = (coils >> 6) & 1;
//            mb_cmd.move_to_pending = (coils >> 2) & 1;
//            mb_cmd.home_pending = (coil2 >> 7) & 1;
//
//            modbus_flags.busy = 0;
//        }
//    }
////    // 2. Cập nhật Modbus registers mỗi 50ms
////    if(now - last_update2 >= 50)
////    {
////        last_update2 = now;
////        Axis_UpdateModbusRegisters();
////    }
//}

void Modbus_TaskUpdate(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t last_update = 0;
    // ===== WATCHDOG TIMEOUT =====
    if(now - last_rx_tick > UART_TIMEOUT_MS)
    {
        modbus_flags.uart_hung = 1;

        // Reset UART
        HAL_UART_Abort_IT(modbus_uart);

        // Clear flags (STM32F4)
        volatile uint32_t tmp;
        tmp = modbus_uart->Instance->SR;
        tmp = modbus_uart->Instance->DR;
        (void)tmp;

        HAL_UARTEx_ReceiveToIdle_IT(modbus_uart, (uint8_t*)RxData, RX_BUFF_SIZE);
        last_rx_tick = now;
    }
    if(now - last_update >= 10)
    {
        last_update = now;
        // ===== PROCESS FRAME =====
		if (!modbus_flags.frame_ready || modbus_flags.busy)
			return;

		modbus_flags.busy = 1;
		modbus_flags.frame_ready = 0;

		// Process from ProcessBuffer (copy of RxData)
		uint8_t func = ProcessBuffer[1];

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
		uint8_t coils = Coils_Database[0];
		uint8_t coil2 = Coils_Database[1];

		mb_cmd.jog_x_left    = (coils >> 3) & 1;
		mb_cmd.jog_x_right   = (coils >> 4) & 1;
		mb_cmd.jog_y_back    = (coils >> 5) & 1;
		mb_cmd.jog_y_forward = (coils >> 6) & 1;
		mb_cmd.move_to_pending = (coils >> 2) & 1;
		mb_cmd.home_pending = (coil2 >> 7) & 1;
		modbus_flags.busy = 0;
    }

}

/* Gọi trong main loop */
void Modbus_ExecuteCommands(void)
{
    // Home
    if(mb_cmd.home_pending)
    {
        mb_cmd.home_pending = 0;
        Coils_Database[1] &= ~(1 << 7);
        Axis_MoveTo(0, 0, 10000.0f);
        return;
    }

    // Move To
    if(mb_cmd.move_to_pending)
    {
        mb_cmd.move_to_pending = 0;
        Coils_Database[0] &= ~(1 << 2);

        float x = (float)Holding_Registers_Database[0];
        float y = (float)Holding_Registers_Database[1];
        Axis_MoveTo(x, y, 5000.0f);
        return;
    }

    // Jog commands
    if(mb_cmd.jog_x_left)
    {
        mb_cmd.jog_x_left = 0;
        Axis_Jog(AXIS_X, LEFT, 1000, 5000.0f);
    }

    if(mb_cmd.jog_x_right)
    {
        mb_cmd.jog_x_right = 0;
        Axis_Jog(AXIS_X, RIGHT, 1000, 5000.0f);
    }

    if(mb_cmd.jog_y_back)
    {
        mb_cmd.jog_y_back = 0;
        Axis_Jog(AXIS_Y, BACKWARD, 100, 5000.0f);
    }

    if(mb_cmd.jog_y_forward)
    {
        mb_cmd.jog_y_forward = 0;
        Axis_Jog(AXIS_Y, FORWARD, 100, 5000.0f);
    }
}
