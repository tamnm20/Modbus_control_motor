/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"
//#include "gpio.h"

static uint16_t *g_modbus_regs = NULL; // pointer to Modbus holding registers

AxisSystem_t Axis;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM5){
	  Motor_Stop(AXIS_Y);
    }
  if(htim->Instance == TIM2){
	  Motor_Stop(AXIS_X);
      }
  if(htim->Instance == TIM9){
	  Motor_Stop(AXIS_Z);
	}
}

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

    while (!(Axis.X.isHomed && Axis.Y.isHomed && Axis.Z.isHomed)) {}

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

void Move_To_Start(float x_mm, float y_mm, float feed_mm_s)
{
    if (Axis.X.state == MOTOR_BUSY || Axis.Y.state == MOTOR_BUSY)
        return;

    int32_t dx_steps = lroundf((x_mm - Axis.X.position) * STEPS_PER_MM);
    int32_t dy_steps = lroundf((y_mm - Axis.Y.position) * STEPS_PER_MM);

    uint32_t nx = (dx_steps >= 0) ? dx_steps : -dx_steps;
    uint32_t ny = (dy_steps >= 0) ? dy_steps : -dy_steps;
    if (nx == 0 && ny == 0) return;

    // --- Set hướng ---
    uint8_t dir_x = (dx_steps >= 0) ? RIGHT : LEFT;
    uint8_t dir_y = (dy_steps >= 0) ? FORWARD : BACKWARD;
    Set_Dir_X(dir_x);
    Set_Dir_Y(dir_y);
    Axis.X.direction = dir_x;
    Axis.Y.direction = dir_y;
    HAL_Delay(1);

    // --- Tính toán tốc độ ---
    float dist_mm = sqrtf((float)(nx*nx + ny*ny)) / STEPS_PER_MM;
    float T = dist_mm / feed_mm_s;
    if (T <= 0.001f) T = 0.001f;

    uint32_t f_x = (uint32_t)roundf(nx / T);
    uint32_t f_y = (uint32_t)roundf(ny / T);
    if (f_x > FREQ_MAX) f_x = FREQ_MAX;
    if (f_y > FREQ_MAX) f_y = FREQ_MAX;

    // --- Áp dụng và khởi động ---
    Motor_SetFeed(AXIS_X, (float)f_x / STEPS_PER_MM);
    Motor_SetFeed(AXIS_Y, (float)f_y / STEPS_PER_MM);
    Motor_Start(AXIS_X, nx);
    Motor_Start(AXIS_Y, ny);

    moveTask.state = MOVE_RUNNING;
    moveTask.x_target = x_mm;
    moveTask.y_target = y_mm;
    moveTask.feed_target = feed_mm_s;
    moveTask.x_steps = nx;
    moveTask.y_steps = ny;
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
    HAL_Delay(1);

    // Run
    Motor_SetFeed(axis, feed_mm_s);
    Motor_Start(axis, steps);
    Axis_UpdateState(axis, dir, feed_mm_s);

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


