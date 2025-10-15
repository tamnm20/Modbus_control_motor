/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"
#include "gpio.h"

static uint16_t *g_modbus_regs = NULL; // pointer to Modbus holding registers

AxisSystem_t Axis;
float cur_x_mm = 0.0f;
float cur_y_mm = 0.0f;

// ================== INIT ==================
void Axis_Init(uint16_t *modbus_regs)
{
	g_modbus_regs = modbus_regs;
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
            TIM2->ARR  = arr;
            TIM2->CCR1 = (arr + 1) / 2;
            TIM2->EGR  = TIM_EGR_UG;
            TIM2->SR  &= ~TIM_SR_UIF;
            Axis.X.pulse_freq = f_hz;
            break;

        case AXIS_Y:
            TIM3->ARR  = arr;
            TIM3->CCR1 = (arr + 1) / 2;
            TIM3->EGR  = TIM_EGR_UG;
            TIM3->SR  &= ~TIM_SR_UIF;
            Axis.Y.pulse_freq = f_hz;
            break;

        case AXIS_Z:
            // Add timer config if Z uses TIM4 or other
            break;
    }
}

// ================== MOTOR START ==================
extern volatile uint32_t x_steps_rem;
extern volatile uint32_t y_steps_rem;

//void Motor_Start(AxisName_t axis, uint32_t steps)
//{
//    switch (axis)
//    {
//        case AXIS_X:
//            x_steps_rem = steps;
//            TIM2->CNT = 0;
//            TIM2->SR &= ~TIM_SR_UIF;
//            TIM2->CCER |= TIM_CCER_CC1E;
//            TIM2->CR1  |= TIM_CR1_CEN;
//            Axis.X.state = MOTOR_RUN;
//            break;
//
//        case AXIS_Y:
//            y_steps_rem = steps;
//            TIM3->CNT = 0;
//            TIM3->SR &= ~TIM_SR_UIF;
//            TIM3->CCER |= TIM_CCER_CC1E;
//            TIM3->CR1  |= TIM_CR1_CEN;
//            Axis.Y.state = MOTOR_RUN;
//            break;
//
//        case AXIS_Z:
//            // Add when Z implemented
//            break;
//    }
//}

void Motor_Start(AxisName_t axis, uint32_t steps)
{
    switch (axis)
    {
        case AXIS_X:
            //if (Axis.X.state != MOTOR_IDLE) return;   // ignore if busy
        	if (Axis.X.state != MOTOR_IDLE && Axis.X.state != MOTOR_RETURN_HOME)
        	    return;
            Axis.X.state = MOTOR_BUSY;                // mark as busy

            x_steps_rem = steps;
            TIM2->CNT = 0;
            TIM2->SR &= ~TIM_SR_UIF;
            TIM2->CCER |= TIM_CCER_CC1E;
            TIM2->CR1  |= TIM_CR1_CEN;
            break;

        case AXIS_Y:
            //if (Axis.Y.state != MOTOR_IDLE) return;
        	if (Axis.Y.state != MOTOR_IDLE && Axis.Y.state != MOTOR_RETURN_HOME)
        	    return;
            Axis.Y.state = MOTOR_BUSY;

            y_steps_rem = steps;
            TIM3->CNT = 0;
            TIM3->SR &= ~TIM_SR_UIF;
            TIM3->CCER |= TIM_CCER_CC1E;
            TIM3->CR1  |= TIM_CR1_CEN;
            break;

        case AXIS_Z:
            // same idea if you add Z later
            break;
    }
}

// ================== MOTOR STOP ==================
//void Motor_Stop(AxisName_t axis)
//{
//    switch (axis)
//    {
//        case AXIS_X:
//            TIM2->CCER &= ~TIM_CCER_CC1E;
//            TIM2->CR1  &= ~TIM_CR1_CEN;
//            Axis.X.state = MOTOR_IDLE;
//            break;
//
//        case AXIS_Y:
//            TIM3->CCER &= ~TIM_CCER_CC1E;
//            TIM3->CR1  &= ~TIM_CR1_CEN;
//            Axis.Y.state = MOTOR_IDLE;
//            break;
//
//        case AXIS_Z:
//            // Add stop for Z if needed
//            break;
//    }
//}

void Motor_Stop(AxisName_t axis)
{
    switch (axis)
    {
        case AXIS_X:
            TIM2->CCER &= ~TIM_CCER_CC1E;
            TIM2->CR1  &= ~TIM_CR1_CEN;
            Axis.X.state = MOTOR_IDLE;
            break;

        case AXIS_Y:
            TIM3->CCER &= ~TIM_CCER_CC1E;
            TIM3->CR1  &= ~TIM_CR1_CEN;
            Axis.Y.state = MOTOR_IDLE;
            break;

        case AXIS_Z:
            // same for Z
            break;
    }
}


// ================== MOVE TO ==================
void Move_To(float x_mm, float y_mm, float feed_mm_s)
{
    int32_t dx_steps = lroundf((x_mm - cur_x_mm) * STEPS_PER_MM);
    int32_t dy_steps = lroundf((y_mm - cur_y_mm) * STEPS_PER_MM);

    uint32_t nx = (dx_steps >= 0) ? dx_steps : -dx_steps;
    uint32_t ny = (dy_steps >= 0) ? dy_steps : -dy_steps;

    if (nx == 0 && ny == 0) return;

    // Set direction
    uint8_t dir_x = (dx_steps >= 0) ? RIGHT : LEFT;
    uint8_t dir_y = (dy_steps >= 0) ? FORWARD : BACKWARD;
    Set_Dir_X(dir_x);
    Set_Dir_Y(dir_y);
    Axis.X.direction = dir_x;
    Axis.Y.direction = dir_y;
    HAL_Delay(5);

    // Calculate total time
    float dist_mm = sqrtf((float)(nx*nx + ny*ny)) / STEPS_PER_MM;
    float T = dist_mm / feed_mm_s;
    if (T <= 0) T = 0.001f;

    uint32_t f_x = (uint32_t)roundf(nx / T);
    uint32_t f_y = (uint32_t)roundf(ny / T);
    if (f_x > FREQ_MAX) f_x = FREQ_MAX;
    if (f_y > FREQ_MAX) f_y = FREQ_MAX;

    // Apply feed and start
    Motor_SetFeed(AXIS_X, (float)f_x / STEPS_PER_MM);
    Motor_SetFeed(AXIS_Y, (float)f_y / STEPS_PER_MM);
    Motor_Start(AXIS_X, nx);
    Motor_Start(AXIS_Y, ny);

    while (x_steps_rem || y_steps_rem);

    cur_x_mm = x_mm;
    cur_y_mm = y_mm;
    Axis.X.position = x_mm;
    Axis.Y.position = y_mm;

    Motor_Stop(AXIS_X);
    Motor_Stop(AXIS_Y);
}

// ================== HOME ALL ==================
//void Home_All(void)
//{
//    Axis.X.isHomed = 0;
//    Axis.Y.isHomed = 0;
//
//    exti_init();
//
//    Set_Dir_X(LEFT);
//    Set_Dir_Y(BACKWARD);
//    HAL_Delay(10);
//
//    Motor_SetFeed(AXIS_X, 20.0f);
//    Motor_SetFeed(AXIS_Y, 20.0f);
//
//    x_steps_rem = 0xFFFFFFFF;
//    y_steps_rem = 0xFFFFFFFF;
//
//    Motor_Start(AXIS_X, 0xFFFFFFFF);
//    Motor_Start(AXIS_Y, 0xFFFFFFFF);
//
//    while (!(Axis.X.isHomed && Axis.Y.isHomed)) {}
//
//    Axis_Home();
//
//    Motor_Stop(AXIS_X);
//    Motor_Stop(AXIS_Y);
//}

void Home_All(void)
{
    Axis.X.state = MOTOR_RETURN_HOME;
    Axis.Y.state = MOTOR_RETURN_HOME;
    Axis.X.isHomed = 0;
    Axis.Y.isHomed = 0;

    exti_init();
    Set_Dir_X(LEFT);
    Set_Dir_Y(BACKWARD);
    HAL_Delay(10);

    Motor_SetFeed(AXIS_X, 20.0f);
    Motor_SetFeed(AXIS_Y, 20.0f);

    x_steps_rem = 0xFFFFFFFF;
    y_steps_rem = 0xFFFFFFFF;

    Motor_Start(AXIS_X, 0xFFFFFFFF);
    Motor_Start(AXIS_Y, 0xFFFFFFFF);

    while (!(Axis.X.isHomed && Axis.Y.isHomed)) {}

    Axis_Home();

    Motor_Stop(AXIS_X);
    Motor_Stop(AXIS_Y);

    Axis.X.state = MOTOR_IDLE;
    Axis.Y.state = MOTOR_IDLE;
}

// ================== UPDATE AXIS STATE ==================
void Axis_UpdateState(AxisName_t axis, uint8_t dir, float feed_mm_s, uint32_t steps)
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
    m->pulse_freq = (uint32_t)(feed_mm_s * STEPS_PER_MM);
    m->state      = MOTOR_RUN;

    // Calculate estimated position change
    float delta_mm = (float)steps / (float)STEPS_PER_MM;
    if (dir == LEFT || dir == BACKWARD || dir == UP)
        m->position -= delta_mm;
    else
        m->position += delta_mm;

    // End motion
    m->state = MOTOR_IDLE;
}

void Axis_MoveStep(AxisName_t axis, uint8_t dir, uint32_t steps, float feed_mm_s)
{
    ServoMotor_t *m = NULL;
    float max_limit = 0.0f;

    // Select axis
    switch (axis)
    {
        case AXIS_X:
            if (Axis.X.isHomed == 0 || Axis.X.state == MOTOR_BUSY) return;
            m = &Axis.X;
            max_limit = X_MAX_MM;
            break;

        case AXIS_Y:
            if (Axis.Y.isHomed == 0 || Axis.Y.state == MOTOR_BUSY) return;
            m = &Axis.Y;
            max_limit = Y_MAX_MM;
            break;

        case AXIS_Z:
            if (Axis.Z.isHomed == 0 || Axis.Z.state == MOTOR_BUSY) return;
            m = &Axis.Z;
            max_limit = Z_MAX_MM;
            break;

        default: return;
    }

    uint8_t neg = (dir == LEFT || dir == BACKWARD || dir == UP);

    // Boundary check
    if (neg && m->position <= EPS_MM) return;
    if (!neg && m->position >= (max_limit - EPS_MM)) return;

    // Trim steps if exceeding limits
    if (neg) {
        float mm_to_zero = m->position;
        uint32_t max_steps = (uint32_t)floorf(mm_to_zero * STEPS_PER_MM + 0.5f);
        if (steps > max_steps) steps = max_steps;
        if (steps == 0) return;
    } else {
        float mm_to_max = max_limit - m->position;
        uint32_t max_steps = (uint32_t)floorf(mm_to_max * STEPS_PER_MM + 0.5f);
        if (steps > max_steps) steps = max_steps;
        if (steps == 0) return;
    }

    // Set direction
    if (axis == AXIS_X) Set_Dir_X(dir);
    else if (axis == AXIS_Y) Set_Dir_Y(dir);
    HAL_Delay(5);

    // Run
    Motor_SetFeed(axis, feed_mm_s);
    Motor_Start(axis, steps);
    Axis_UpdateState(axis, dir, feed_mm_s, steps);

    // Update position
    float delta_mm = (float)steps / (float)STEPS_PER_MM;
    if (neg) {
        m->position -= delta_mm;
        if (m->position < 0) m->position = 0;
    } else {
        m->position += delta_mm;
        if (m->position > max_limit) m->position = max_limit;
    }

    // Update Modbus register if available
    if (g_modbus_regs != NULL)
    {
        switch (axis)
        {
            case AXIS_X: g_modbus_regs[0] = (uint16_t)m->position; break;
            case AXIS_Y: g_modbus_regs[1] = (uint16_t)m->position; break;
            case AXIS_Z: g_modbus_regs[2] = (uint16_t)m->position; break;
            default: break;
        }
    }
}

// ================== EXTI ISR ==================
//void EXTI9_5_IRQHandler(void)
//{
//    if (EXTI->PR & (1u << 5))
//    {
//        EXTI->PR = (1u << 5);
//        Axis.X.isHomed = 1;
//        Motor_Stop(AXIS_X);
//    }
//}
//
//void EXTI15_10_IRQHandler(void)
//{
//    if (EXTI->PR & (1u << 13))
//    {
//        EXTI->PR = (1u << 13);
//        Axis.Y.isHomed = 1;
//        Motor_Stop(AXIS_Y);
//    }
//}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1u << 5))
    {
        EXTI->PR = (1u << 5);
        Axis.X.isHomed = 1;
        Motor_Stop(AXIS_X);
        Axis.X.state = MOTOR_IDLE;
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 13))
    {
        EXTI->PR = (1u << 13);
        Axis.Y.isHomed = 1;
        Motor_Stop(AXIS_Y);
        Axis.Y.state = MOTOR_IDLE;
    }
}

