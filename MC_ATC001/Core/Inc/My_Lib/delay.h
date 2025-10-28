/*
 * delay.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

extern uint32_t Tick;



void Delay_Init(void);
void delay_ms(uint32_t milisecond);
void delay_us(uint32_t microsecond);

#endif /* INC_DELAY_H_ */
