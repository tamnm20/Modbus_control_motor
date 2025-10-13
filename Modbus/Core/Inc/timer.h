/*
 * timer.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32f4xx_hal.h"

extern volatile uint32_t x_steps_rem;
extern volatile uint32_t y_steps_rem;

void tim2_init(void);
void tim3_init(void);

#endif /* INC_TIMER_H_ */
