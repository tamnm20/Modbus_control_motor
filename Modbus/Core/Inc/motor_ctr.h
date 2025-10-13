/*
 * motor_ctr.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */

#ifndef INC_MOTOR_CTR_H_
#define INC_MOTOR_CTR_H_

#include "stm32f4xx_hal.h"
#include <math.h>
#include "gpio.h"
#include "timer.h"

#define STEPS_PER_MM 100u
#define FREQ_MAX     50000u // Hz (50 kHz)

static inline void Set_Dir_X(uint8_t dir)
{
    if (!dir)
        gpio_set(GPIOA, 4);   // ✅ đổi sang PA4
    else
        gpio_clr(GPIOA, 4);
}

static inline void Set_Dir_Y(uint8_t dir)
{
    if (!dir)
        gpio_set(GPIOA, 3);
    else
        gpio_clr(GPIOA, 3);
}
void Toggle_Dir_X(void);
void Toggle_Dir_Y(void);
void X_SetFeed_mm_s(float feed_mm_s);
void Y_SetFeed_mm_s(float feed_mm_s);
void X_StartSteps(uint32_t steps);
void Y_StartSteps(uint32_t steps);
void move_to_mm(float x_mm, float y_mm, float feed_mm_s);
void home_all(void);

#endif /* INC_MOTOR_CTR_H_ */
