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

#define LEFT 1
#define RIGHT 0
#define FORWARD 0
#define BACKWARD 1
#define UP 1
#define DOWN 0


typedef enum {
    MOTOR_IDLE,
    MOTOR_RUN,
    MOTOR_BUSY,
    MOTOR_RETURN_HOME
} MotorState_t;

typedef struct {
    float position;
    MotorState_t state;
    uint32_t pulse_freq;
    float velocity;
    uint8_t isHomed;
    uint8_t direction;
} ServoMotor_t;

typedef struct {
    ServoMotor_t X;
    ServoMotor_t Y;
    ServoMotor_t Z;
} AxisSystem_t;

// -------------------------------
extern AxisSystem_t Axis;


static inline void Set_Dir_X(uint8_t dir)
{
    if (!dir)
        gpio_set(GPIOA, 4);
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
