/*
 * RS232.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"
#include "My_Lib/modbusSlave.h"
#ifndef INC_RS232_H_
#define INC_RS232_H_

typedef enum {
	EngineerSetting,
	WorkerRunning,

} MachineState;

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
typedef union {
    struct {
    	uint8_t Home :1;
    	uint8_t Engineer :1;
        uint8_t reserved : 6;

    } bits;
    uint8_t all;
} Tab_Control_t;
typedef union {
    struct {
    	uint8_t Home :1;
        uint8_t reserved : 7;

    } bits;
    uint8_t all;
} GoHome_t;


extern GoHome_t* Home;
extern Control_motor_t* Control_motor;
extern Tab_Control_t* Tab;

void HMI_Init(void);

//void uart2_receive_IDLE_DMA();
#endif /* INC_RS232_H_ */
