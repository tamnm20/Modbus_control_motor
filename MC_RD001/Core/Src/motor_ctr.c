/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"

AxisSystem_t Axis;

// ================== INIT ==================
void Axis_Init(void)
{
    Axis.X = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, RIGHT};
    Axis.Y = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, FORWARD};
    Axis.Z = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, DOWN};
}

// ================== HOME RESET ==================
void Axis_Home(void)
{
    Axis.X.position = 0.0f;
    Axis.Y.position = 0.0f;
    Axis.Z.position = 0.0f;
    Axis.X.isHomed = Axis.Y.isHomed = Axis.Z.isHomed = 1;
}
//f_PWM = TIMCLK / ((PSC+1)·(ARR+1))
//Duty (%) ≈ (CCR1 / (ARR+1)) · 100
// ================== MOTOR FEEDRATE ==================
void Motor_SetFeed(AxisName_t axis, float feed_mm_s)
{
    uint32_t f_hz = (uint32_t)roundf(feed_mm_s * STEPS_PER_MM);
    if (f_hz > FREQ_MAX) f_hz = FREQ_MAX;
    if (f_hz == 0) f_hz = 1;

    uint32_t arr = (1000000u / f_hz) - 1;
    if (arr < 2) arr = 2;

    switch (axis)
    {
        case AXIS_X:
            TIM1->ARR  = arr;
            TIM1->CCR1 = (arr + 1) / 2;
            Axis.X.pulse_freq = f_hz;
            break;

        case AXIS_Y:
            TIM8->ARR  = arr;
            TIM8->CCR1 = (arr + 1) / 2;
            Axis.Y.pulse_freq = f_hz;
            break;

        case AXIS_Z:
            TIM3->ARR  = arr;
            TIM3->CCR3 = (arr + 1) / 2;
            Axis.Z.pulse_freq = f_hz;
            break;
    }
}

// ================== MOTOR START ==================

void Motor_Start(AxisName_t axis, uint32_t steps)
{
    switch (axis)
    {
        case AXIS_X:
        	if (Axis.X.state != MOTOR_IDLE && Axis.X.state != MOTOR_RETURN_HOME)
        	    return;
        	if(steps == 0) return;
            Axis.X.state = MOTOR_BUSY;                // mark as busy
        	htim2.Instance->ARR = steps-1;
        	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        	HAL_TIM_Base_Start_IT(&htim2);
            break;

        case AXIS_Y:
        	if (Axis.Y.state != MOTOR_IDLE && Axis.Y.state != MOTOR_RETURN_HOME)
        	    return;
        	if(steps == 0) return;
            Axis.Y.state = MOTOR_BUSY;
        	htim5.Instance->ARR = steps-1;
        	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        	HAL_TIM_Base_Start_IT(&htim5);
            break;

        case AXIS_Z:
        	if (Axis.Z.state != MOTOR_IDLE && Axis.Z.state != MOTOR_RETURN_HOME)
        	    return;
        	if(steps == 0) return;
            Axis.Z.state = MOTOR_BUSY;
        	htim9.Instance->ARR = steps-1;
        	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        	HAL_TIM_Base_Start_IT(&htim9);
            break;
    }
}

// ================== MOTOR STOP ==================
void Motor_Stop(AxisName_t axis)
{
    switch (axis)
    {
        case AXIS_X:
		    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_Base_Stop_IT(&htim2);
            TIM2->CNT = 0;
            TIM2->SR = 0;
		    Axis.X.state = MOTOR_IDLE;
            break;

        case AXIS_Y:
        	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
            HAL_TIM_Base_Stop_IT(&htim5);
            TIM5->CNT = 0;
            TIM5->SR = 0;
        	Axis.Y.state = MOTOR_IDLE;
            break;

        case AXIS_Z:
        	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            HAL_TIM_Base_Stop_IT(&htim9);
            TIM9->CNT = 0;
            TIM9->SR = 0;
		  	Axis.Z.state = MOTOR_IDLE;
            break;
    }
}

// ================== HOME ALL ==================
void Home_All(void)
{
    Axis.X.state = MOTOR_RETURN_HOME;
    Axis.Y.state = MOTOR_RETURN_HOME;
    Axis.Z.state = MOTOR_RETURN_HOME;
    Axis.X.isHomed = 0;
    Axis.Y.isHomed = 0;
    Axis.Z.isHomed = 0;

    Set_Dir_X(LEFT);
    Set_Dir_Y(BACKWARD);
    Set_Dir_Z(UP);
    HAL_Delay(10);

    Motor_SetFeed(AXIS_X, 2000.0f);
    Motor_SetFeed(AXIS_Y, 2000.0f);
    Motor_SetFeed(AXIS_Z, 2000.0f);

    Motor_Start(AXIS_X, 0xFFFFFFFF);
    Motor_Start(AXIS_Y, 0xFFFFFFFF);
    Motor_Start(AXIS_Z, 0xFFFFFFFF);

    while (!(Axis.X.isHomed && Axis.Y.isHomed && Axis.Z.isHomed)) {
    	HAL_Delay(1);
    }

    Axis_Home();
    HAL_Delay(1);

}

// ================== UPDATE AXIS STATE ==================
void Axis_UpdateState(AxisName_t axis, uint8_t dir, float feed_mm_s)
{
    ServoMotor_t *m = NULL;

    switch (axis)
    {
        case AXIS_X: m = &Axis.X; break;
        case AXIS_Y: m = &Axis.Y; break;
        case AXIS_Z: m = &Axis.Z; break;
        default: return;
    }

    // Save parameters
    m->direction  = dir;
    m->velocity   = feed_mm_s;
}
