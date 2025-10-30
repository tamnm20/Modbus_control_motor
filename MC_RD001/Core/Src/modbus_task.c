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
typedef void (*MotorHandler_t)(void);

/* ========== COMMAND FLAGS ========== */
typedef struct {
    uint8_t home_pending;
    uint8_t jog_z_down;
} ModbusCmd_t;

typedef union {
    struct {
        uint8_t reserved : 2;
        uint8_t Set      : 1;
        uint8_t Left     : 1;
        uint8_t Right    : 1;
        uint8_t In      : 1;
        uint8_t Out       : 1;
        uint8_t Up       : 1;
    } bits;
    uint8_t all;
} Control_motor_t;

typedef struct {
    uint8_t bitMask;
    MotorHandler_t handler;
} MotorActionMap_t;

MotorActionMap_t motorActionTable[] = {
    { 1 << 2, Handle_Set   },
    { 1 << 3, Handle_Left  },
    { 1 << 4, Handle_Right },
    { 1 << 5, Handle_In    },
    { 1 << 6, Handle_Out   },
    { 1 << 7, Handle_Up    },
};

Control_motor_t* Control_motor = (Control_motor_t*)&Coils_Database[0];

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

void Modbus_TaskUpdate(void)
{
    uint32_t now = HAL_GetTick();
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
//		uint8_t coils = Coils_Database[0];
		uint8_t coil2 = Coils_Database[1];
//
		mb_cmd.jog_z_down    = (coil2) & 1;
//		mb_cmd.jog_x_right   = (coils >> 4) & 1;
//		mb_cmd.jog_y_back    = (coils >> 5) & 1;
//		mb_cmd.jog_y_forward = (coils >> 6) & 1;
//		mb_cmd.move_to_pending = (coils >> 2) & 1;
		mb_cmd.home_pending = (coil2 >> 7) & 1;
		modbus_flags.busy = 0;

}

/* Gọi trong main loop */
void Modbus_ExecuteCommands(void)
{
    // Home
    if(mb_cmd.home_pending)
    {
        mb_cmd.home_pending = 0;
        Coils_Database[1] &= ~(1 << 7);
        Axis_MoveTo(0, 0, 0, 10000.0f);
        return;
    }
//
//    // Move To
//    if(mb_cmd.move_to_pending)
//    {
//        mb_cmd.move_to_pending = 0;
//        Coils_Database[0] &= ~(1 << 2);
//
//        float x = (float)Holding_Registers_Database[0];
//        float y = (float)Holding_Registers_Database[1];
//        Axis_MoveTo(x, y, 10000.0f);
//        return;
//    }
//
//    // Jog commands
//    if(mb_cmd.jog_x_left)
//    {
//        mb_cmd.jog_x_left = 0;
//        Axis_Jog(AXIS_X, LEFT, 100, 5000.0f);
//    }
//
//    if(mb_cmd.jog_x_right)
//    {
//        mb_cmd.jog_x_right = 0;
//        Axis_Jog(AXIS_X, RIGHT, 100, 5000.0f);
//    }
//
//    if(mb_cmd.jog_y_back)
//    {
//        mb_cmd.jog_y_back = 0;
//        Axis_Jog(AXIS_Y, BACKWARD, 100, 5000.0f);
//    }
//
//    if(mb_cmd.jog_y_forward)
//    {
//        mb_cmd.jog_y_forward = 0;
//        Axis_Jog(AXIS_Y, FORWARD, 100, 5000.0f);
//    }
    if(mb_cmd.jog_z_down)
    {
        mb_cmd.jog_z_down = 0;
        Axis_Jog(AXIS_Z, DOWN, 100, 2000.0f);
    }
    uint8_t current = Control_motor->all;

	for (int i = 0; i < sizeof(motorActionTable)/sizeof(MotorActionMap_t); i++)
	{
		if (current & motorActionTable[i].bitMask)
		{
			motorActionTable[i].handler();
			//break;
		}
	}
}

void Handle_Set(void)
{
    Coils_Database[0] &= ~(1 << 2);
    float x = (float)Holding_Registers_Database[0];
    float y = (float)Holding_Registers_Database[1];
    float z = (float)Holding_Registers_Database[2];
    Axis_MoveTo(x, y, z, 10000.0f);
}
void Handle_Left(void)
{
	Axis_Jog(AXIS_X, LEFT, 10, 5000.0f);
}
void Handle_Right(void)
{
	Axis_Jog(AXIS_X, RIGHT, 10, 5000.0f);
}
void Handle_In(void)
{
    Axis_Jog(AXIS_Y, BACKWARD, 10, 5000.0f);
}
void Handle_Out(void)
{
    Axis_Jog(AXIS_Y, FORWARD, 10, 5000.0f);
}
void Handle_Up(void)
{
	Axis_Jog(AXIS_Z, UP, 100, 2000.0f);
}
void Handle_Down(void)
{
	Axis_Jog(AXIS_Z, DOWN, 100, 2000.0f);
}
