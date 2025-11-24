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


typedef union {
    struct {
        uint8_t Left     	: 1;
        uint8_t Right    	: 1;
        uint8_t In      	: 1;
        uint8_t Out       	: 1;
        uint8_t Up       	: 1;
        uint8_t Down		: 1;
        uint8_t Set      	: 1;
        uint8_t Home		: 1;
    } bits;
    uint8_t all;
} Control_motor_t;

typedef union {
    struct {
        uint8_t GL1     	: 1;
        uint8_t GL2	    	: 1;
        uint8_t GL3      	: 1;
        uint8_t CV1       	: 1;
        uint8_t CV2 		: 1;
        uint8_t CV3     	: 1;
        uint8_t GL_save    	: 1;
        uint8_t CV_save		: 1;
    } bits;
    uint8_t all;
} Save_point_t;

typedef union {
    struct {
    	uint8_t Home :1;
    	uint8_t Engine :1;
//    	uint8_t Start_state:1;
//    	uint8_t Start:1;
//    	uint8_t Scan_state:1;
//    	uint8_t Scan:1;
        uint8_t reserved : 6;
    } bits;
    uint8_t all;
} Tab_Control_t;

typedef union {
    struct {
    	uint8_t Tray1 :1;
    	uint8_t Tray2 :1;
    	uint8_t Tray3 :1;
    	uint8_t Tray4 :1;
    	uint8_t Start:1;
    	uint8_t Scan:1;
    	uint8_t Reset_state:1;
        uint8_t reserved : 1;
    } bits;
    uint8_t all;
} GL_tray_t;

typedef union {
    struct {
        uint8_t LedG     	: 1;
        uint8_t LedR    	: 1;
        uint8_t Tray1      	: 1;
        uint8_t Tray2      	: 1;
        uint8_t Tray3      	: 1;
        uint8_t Tray4		: 1;
        uint8_t Start      	: 1;
        uint8_t Scan		: 1;
    } bits;
    uint8_t all;
} Home_state_t;

typedef void (*Handler_t)(void);
typedef struct {
    uint8_t bitMask;
    Handler_t handler;
} ActionMap_t;

extern volatile ModbusStatus_t modbus_flags;

void Modbus_TaskInit(UART_HandleTypeDef *huart);
void Modbus_TaskUpdate(void);
void Modbus_ExecuteCommands(void);

void Handle_Home(void);
void Handle_Set(void);
void Handle_Left(void);
void Handle_Right(void);
void Handle_In(void);
void Handle_Out(void);
void Handle_Up(void);
void Handle_Down(void);
void Handle_Tray1(void);
void Handle_Tray2(void);
void Handle_Tray3(void);
void Handle_Tray4(void);
void Handle_Start(void);
void Handle_Scan(void);
void Handle_Reset_state(void);

#endif /* INC_MODBUS_TASK_H_ */
