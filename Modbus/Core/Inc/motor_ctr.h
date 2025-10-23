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

// ===== CONFIG =====
#define STEPS_PER_MM 100u
#define FREQ_MAX     50000u // Hz
#define EPS_MM 0.0005f  // float guard for zero
#define X_MAX_MM  500.0f
#define Y_MAX_MM  250.0f
#define Z_MAX_MM     100.0f
// ===== DIRECTION =====
#define LEFT       1
#define RIGHT      0
#define FORWARD    0
#define BACKWARD   1
#define UP         1
#define DOWN       0

// ===== ENUM =====
typedef enum {
    MOTOR_IDLE,
    MOTOR_RUN,
    MOTOR_BUSY,
    MOTOR_RETURN_HOME
} MotorState_t;

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z
} AxisName_t;

// ===== STRUCT =====
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

extern AxisSystem_t Axis;
extern volatile uint32_t x_steps_rem;
extern volatile uint32_t y_steps_rem;

#define Motor_SetFeed_X(feed)   Motor_SetFeed(AXIS_X, feed)
#define Motor_SetFeed_Y(feed)   Motor_SetFeed(AXIS_Y, feed)
#define Motor_SetFeed_Z(feed)   Motor_SetFeed(AXIS_Z, feed)

#define Motor_Start_X(steps)    Motor_Start(AXIS_X, steps)
#define Motor_Start_Y(steps)    Motor_Start(AXIS_Y, steps)
#define Motor_Start_Z(steps)    Motor_Start(AXIS_Z, steps)

#define Axis_IsHomed_X()   (Axis.X.isHomed)
#define Axis_IsHomed_Y()   (Axis.Y.isHomed)
#define Axis_IsHomed_Z()   (Axis.Z.isHomed)

#define Axis_IsBusy_X()    (Axis.X.state == MOTOR_BUSY)
#define Axis_IsBusy_Y()    (Axis.Y.state == MOTOR_BUSY)
#define Axis_IsBusy_Z()    (Axis.Z.state == MOTOR_BUSY)

// ===== INLINE DIR CONTROL =====
static inline void Set_Dir_X(uint8_t dir) {
    if (dir != RIGHT) gpio_set(GPIOA, 4);
    else gpio_clr(GPIOA, 4);
}

static inline void Set_Dir_Y(uint8_t dir) {
    if (dir != FORWARD) gpio_set(GPIOA, 3);
    else gpio_clr(GPIOA, 3);
}

static inline void Set_Dir_Z(uint8_t dir) {
    if (dir == DOWN) gpio_set(GPIOA, 2);
    else gpio_clr(GPIOA, 2);
}

// ===== PROTOTYPES =====
void Axis_Init(uint16_t *modbus_regs);
void Axis_Home(void);
void Axis_Process(void);

void Motor_SetFeed(AxisName_t axis, float feed_mm_s);
void Motor_Start(AxisName_t axis, uint32_t steps);
void Motor_Stop(AxisName_t axis);

void Move_To(float x_mm, float y_mm, float feed_mm_s);
void Home_All(void);
void Axis_UpdateState(AxisName_t axis, uint8_t dir, float feed_mm_s);
void Axis_MoveStep(AxisName_t axis, uint8_t dir, uint32_t steps, float feed_mm_s);

#endif /* INC_MOTOR_CTR_H_ */
