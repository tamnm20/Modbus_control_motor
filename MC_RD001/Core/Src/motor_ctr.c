/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"

AxisSystem_t Axis;
/* ===== ISR - MINIMAL & FAST ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // CRITICAL: Stop PWM immediately
    if(GPIO_Pin == HOME_X_Pin)
    {
    	Motor_Stop(AXIS_X);
//	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//        HAL_TIM_Base_Stop_IT(&htim2);
    	Axis.X.position = 0.0f;
        Axis.X.sensor_triggered = 1;
    }
    else if(GPIO_Pin == HOME_Y_Pin)
    {
    	Motor_Stop(AXIS_Y);
//    	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//        HAL_TIM_Base_Stop_IT(&htim5);
    	Axis.Y.position = 0.0f;
        Axis.Y.sensor_triggered = 1;
    }
    else if(GPIO_Pin == HOME_Z_Pin)
    {
    	Motor_Stop(AXIS_Z);
//    	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
//        HAL_TIM_Base_Stop_IT(&htim9);
    	Axis.Z.position = 0.0f;
        Axis.Z.sensor_triggered = 1;
    }
}
// ================== INIT ==================
//void Axis_Init(void)
//{
//    Axis.X = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, RIGHT};
//    Axis.Y = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, FORWARD};
//    Axis.Z = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, DOWN};
//}

void Axis_Init(void)
{
    // ===== AXIS X =====
    Axis.X = (ServoMotor_t){
        .state = MOTOR_IDLE,
        .homing_state = HOMING_INITIAL_CHECK,
        .direction = RIGHT
    };

    // ===== AXIS Y =====
    Axis.Y = (ServoMotor_t){
        .state = MOTOR_IDLE,
        .homing_state = HOMING_INITIAL_CHECK,
        .direction = FORWARD
    };

    // ===== AXIS Z =====
    Axis.Z = (ServoMotor_t){
        .state = MOTOR_IDLE,
        .homing_state = HOMING_INITIAL_CHECK,
        .direction = DOWN
    };
}

// ================== HOME RESET ==================
void Axis_Home(void)
{
    Axis.X.position = 0.0f;
    Axis.Y.position = 0.0f;
    Axis.Z.position = 0.0f;
    Axis.X.homing_state = Axis.Y.homing_state = Axis.Z.homing_state = HOMING_COMPLETE;
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

static void Home_ProcessSensor(AxisName_t axis)
{
    ServoMotor_t *m;

    switch(axis)
    {
        case AXIS_X:
            m = &Axis.X;
            break;
        case AXIS_Y:
            m = &Axis.Y;
            break;
        case AXIS_Z:
            m = &Axis.Z;
            break;
        default:
            return;
    }

    if(!m->sensor_triggered)
        return;

    m->sensor_triggered = 0;

    // Cleanup counters (PWM đã stop trong ISR)
//    switch(axis)
//    {
//        case AXIS_X:
//            TIM2->CR1 &= ~TIM_CR1_CEN;
//            TIM2->CNT = 0;
//            TIM2->SR = 0;
//            break;
//        case AXIS_Y:
//            TIM5->CR1 &= ~TIM_CR1_CEN;
//            TIM5->CNT = 0;
//            TIM5->SR = 0;
//            break;
//        case AXIS_Z:
//            TIM9->CR1 &= ~TIM_CR1_CEN;
//            TIM9->CNT = 0;
//            TIM9->SR = 0;
//            break;
//    }

//    m->state = MOTOR_IDLE;

    // Handle states
    switch(m->homing_state)
    {
        case HOMING_FAST_APPROACH:
            //HAL_Delay(10);

            if(axis == AXIS_X) Set_Dir_X(RIGHT);
            else if(axis == AXIS_Y) Set_Dir_Y(FORWARD);
            else if(axis == AXIS_Z) Set_Dir_Z(DOWN);
            HAL_Delay(1);

            Motor_SetFeed(axis, HOMING_FAST_FEED);
            Motor_Start(axis, HOMING_BACKOFF_STEPS);

            m->homing_state = HOMING_BACKOFF;
            break;

        case HOMING_SLOW_APPROACH:
            //m->isHomed = 1;
            m->homing_state = HOMING_COMPLETE;
            m->position = 0.0f;
            break;

        default:
            break;
    }
}

static void Home_StateMachine(AxisName_t axis)
{
    ServoMotor_t *m;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;

    switch(axis)
    {
        case AXIS_X:
            m = &Axis.X;
            gpio_port = HOME_X_GPIO_Port;
            gpio_pin = HOME_X_Pin;
            break;
        case AXIS_Y:
            m = &Axis.Y;
            gpio_port = HOME_Y_GPIO_Port;
            gpio_pin = HOME_Y_Pin;
            break;
        case AXIS_Z:
            m = &Axis.Z;
            gpio_port = HOME_Z_GPIO_Port;
            gpio_pin = HOME_Z_Pin;
            break;
        default:
            return;
    }

    switch(m->homing_state)
    {
        case HOMING_INITIAL_CHECK:
            if(HAL_GPIO_ReadPin(gpio_port, gpio_pin) == GPIO_PIN_RESET)
            {
                if(axis == AXIS_X) Set_Dir_X(RIGHT);
                else if(axis == AXIS_Y) Set_Dir_Y(FORWARD);
                else if(axis == AXIS_Z) Set_Dir_Z(DOWN);
                HAL_Delay(1);

                Motor_SetFeed(axis, HOMING_FAST_FEED);  // 2kHz
                Motor_Start(axis, HOMING_BACKOFF_STEPS); // 1000 steps

                m->homing_state = HOMING_BACKOFF;
            }
            else
            {
                if(axis == AXIS_X) Set_Dir_X(LEFT);
                else if(axis == AXIS_Y) Set_Dir_Y(BACKWARD);
                else if(axis == AXIS_Z) Set_Dir_Z(UP);
                HAL_Delay(1);

                Motor_SetFeed(axis, HOMING_FAST_FEED);
                Motor_Start(axis, 0xFFFFFFFF);

                m->homing_state = HOMING_FAST_APPROACH;
            }
            break;
        case HOMING_BACKOFF:
            if(m->state == MOTOR_IDLE)
            {
                if(axis == AXIS_X) Set_Dir_X(LEFT);
                else if(axis == AXIS_Y) Set_Dir_Y(BACKWARD);
                else if(axis == AXIS_Z) Set_Dir_Z(UP);
                HAL_Delay(1);

                Motor_SetFeed(axis, HOMING_SLOW_FEED);
                Motor_Start(axis, 0xFFFFFFFF);

                m->homing_state = HOMING_SLOW_APPROACH;
            }
            break;

        case HOMING_FAST_APPROACH:
        case HOMING_SLOW_APPROACH:
        case HOMING_COMPLETE:
            break;

        default:
            break;
    }
}

void Home_All(void)
{
    // ===== INIT =====
    Axis.X.state = MOTOR_RETURN_HOME;
    Axis.Y.state = MOTOR_RETURN_HOME;
    Axis.Z.state = MOTOR_RETURN_HOME;

    Axis.X.sensor_triggered = 0;
    Axis.Y.sensor_triggered = 0;
    Axis.Z.sensor_triggered = 0;

    while(!(Axis.X.homing_state == HOMING_COMPLETE &&
            Axis.Y.homing_state == HOMING_COMPLETE &&
            Axis.Z.homing_state == HOMING_COMPLETE))
    {
        // State machines
        Home_StateMachine(AXIS_X);
        Home_StateMachine(AXIS_Y);
        Home_StateMachine(AXIS_Z);
        // sensor triggers (ISR)
        Home_ProcessSensor(AXIS_X);
        Home_ProcessSensor(AXIS_Y);
        Home_ProcessSensor(AXIS_Z);

        HAL_Delay(1);
    }

//    // ===== FINALIZE =====
    Axis.X.position = 0.0f;
    Axis.Y.position = 0.0f;
    Axis.Z.position = 0.0f;

    Axis.X.sensor_triggered = 0;
    Axis.Y.sensor_triggered = 0;
    Axis.Z.sensor_triggered = 0;
    HAL_Delay(10);
}
