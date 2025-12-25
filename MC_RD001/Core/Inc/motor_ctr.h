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

#define HOME_X_Pin GPIO_PIN_13
#define HOME_X_GPIO_Port GPIOE
#define HOME_X_EXTI_IRQn EXTI15_10_IRQn
#define HOME_Y_Pin GPIO_PIN_14
#define HOME_Y_GPIO_Port GPIOE
#define HOME_Y_EXTI_IRQn EXTI15_10_IRQn
#define HOME_Z_Pin GPIO_PIN_15
#define HOME_Z_GPIO_Port GPIOE
#define HOME_Z_EXTI_IRQn EXTI15_10_IRQn
#define Pulse_X_Pin GPIO_PIN_6
#define Pulse_X_GPIO_Port GPIOC
#define Dir_X_Pin GPIO_PIN_7
#define Dir_X_GPIO_Port GPIOC
#define Pulse_Z_Pin GPIO_PIN_8
#define Pulse_Z_GPIO_Port GPIOC
#define Dir_Z_Pin GPIO_PIN_9
#define Dir_Z_GPIO_Port GPIOC
#define Pulse_Y_Pin GPIO_PIN_8
#define Pulse_Y_GPIO_Port GPIOA
#define Dir_Y_Pin GPIO_PIN_9
#define Dir_Y_GPIO_Port GPIOA


// ===== CONFIG =====
#define STEPS_PER_MM 1u
#define FREQ_MAX     50000u // Hz
#define EPS_MM 		0.0005f  // float guard for zero
#define X_MAX_MM  50000.0f
#define Y_MAX_MM  28000.0f
#define Z_MAX_MM  8000.0f
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
    HOMING_IDLE,
    HOMING_INITIAL_CHECK,
    HOMING_FAST_APPROACH,
    HOMING_BACKOFF,
    HOMING_SLOW_APPROACH,
    HOMING_COMPLETE
} HomingState_t;

// Homing parameters
#define HOMING_BACKOFF_STEPS    1000
#define HOMING_FAST_FEED        2000.0f
#define HOMING_SLOW_FEED        1000.0f

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z
} AxisName_t;

// ===== STRUCT =====
typedef struct {
    float position;
    float start_position;
    float target_position;

    MotorState_t state;
    HomingState_t homing_state;
    uint8_t direction;
    volatile uint8_t sensor_triggered;

    uint32_t total_steps;
    uint32_t pulse_freq;
} ServoMotor_t;

typedef struct {
    ServoMotor_t X;
    ServoMotor_t Y;
    ServoMotor_t Z;
} AxisSystem_t;

extern AxisSystem_t Axis;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

#define Motor_SetFeed_X(feed)   Motor_SetFeed(AXIS_X, feed)
#define Motor_SetFeed_Y(feed)   Motor_SetFeed(AXIS_Y, feed)
#define Motor_SetFeed_Z(feed)   Motor_SetFeed(AXIS_Z, feed)

#define Motor_Start_X(steps)    Motor_Start(AXIS_X, steps)
#define Motor_Start_Y(steps)    Motor_Start(AXIS_Y, steps)
#define Motor_Start_Z(steps)    Motor_Start(AXIS_Z, steps)

static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin) { port->BSRR = (1u << pin); }
static inline void gpio_clr(GPIO_TypeDef *port, uint8_t pin) { port->BSRR = (1u << (pin + 16)); }

// ===== INLINE DIR CONTROL =====
static inline void Set_Dir_X(uint8_t dir) {
    Axis.X.direction = dir;
    if (dir != RIGHT) gpio_set(Dir_X_GPIO_Port, 7);
    else gpio_clr(Dir_X_GPIO_Port, 7);
}

static inline void Set_Dir_Y(uint8_t dir) {
    Axis.Y.direction = dir;
    if (dir != FORWARD) gpio_set(Dir_Y_GPIO_Port, 9);
    else gpio_clr(Dir_Y_GPIO_Port, 9);
}

static inline void Set_Dir_Z(uint8_t dir) {
    Axis.Z.direction = dir;
    if (dir != DOWN) gpio_set(Dir_Z_GPIO_Port, 9);
    else gpio_clr(Dir_Z_GPIO_Port, 9);
}

// ===== PROTOTYPES =====
void Axis_Init(void);
void Axis_Home(void);

void Motor_SetFeed(AxisName_t axis, float feed_mm_s);
void Motor_Start(AxisName_t axis, uint32_t steps);
void Motor_Stop(AxisName_t axis);

void Home_All(void);

//void Axis_UpdateState(AxisName_t axis, uint8_t dir, float feed_mm_s);
//void Axis_MoveStep(AxisName_t axis, uint8_t dir, uint32_t steps, float feed_mm_s);

#endif /* INC_MOTOR_CTR_H_ */
