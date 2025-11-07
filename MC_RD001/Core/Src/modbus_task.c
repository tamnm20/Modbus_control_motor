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
#include "flash.h"
#include <string.h>

volatile uint8_t RxData[RX_BUFF_SIZE];
uint8_t TxData[TX_BUFF_SIZE];
uint8_t ProcessBuffer[RX_BUFF_SIZE];
volatile ModbusStatus_t modbus_flags = {0};
static uint32_t last_rx_tick = 0;

/* ========== COMMAND FLAGS ========== */
ActionMap_t motorActionTable[] = {
	{ 1 << 0, Handle_Left   },
	{ 1 << 1, Handle_Right  },
    { 1 << 2, Handle_In   	},
    { 1 << 3, Handle_Out  	},
    { 1 << 4, Handle_Up 	},
    { 1 << 5, Handle_Down   },
    { 1 << 6, Handle_Set    },
    { 1 << 7, Handle_Home   },
};
ActionMap_t saveActionTable[] = {
	{ 1 << 0, Handle_GL1   	},
	{ 1 << 1, Handle_GL2  	},
    { 1 << 2, Handle_GL3   	},
    { 1 << 3, Handle_CV1 	},
    { 1 << 4, Handle_CV2   	},
    { 1 << 5, Handle_CV3    },
    { 1 << 6, Handle_GL_save},
    { 1 << 7, Handle_CV_save},
};
Tab_Control_t* Tab = (Tab_Control_t*)&Coils_Database[0];
Control_motor_t* Control_motor = (Control_motor_t*)&Coils_Database[1];
Save_point_t* Save_point = (Save_point_t *)&Coils_Database[4];

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
		modbus_flags.busy = 0;

}

/* Gọi trong main loop */
void Modbus_ExecuteCommands(void)
{
	if(Tab->bits.Engine){
	    uint8_t current = Control_motor->all;

		for (int i = 0; i < sizeof(motorActionTable)/sizeof(ActionMap_t); i++)
		{
			if (current & motorActionTable[i].bitMask)
			{
				motorActionTable[i].handler();
				break;
			}
		}
		current = Save_point->all;
		for (int i = 0; i < sizeof(saveActionTable)/sizeof(ActionMap_t); i++)
		{
			if (current & saveActionTable[i].bitMask)
			{
				saveActionTable[i].handler();
				break;
			}
		}
	}
	else if(Tab->bits.Home){

	}
}

void Handle_Set(void)
{
    Coils_Database[1] &= ~(1 << 6);
    float x = (float)Holding_Registers_Database[0];
    float y = (float)Holding_Registers_Database[1];
    float z = (float)Holding_Registers_Database[2];
    Axis_MoveTo(x, y, z, 10000.0f);
}
void Handle_Home(void)
{
    Coils_Database[1] &= ~(1 << 7);
    Axis_MoveTo(0, 0, 0, 10000.0f);
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
