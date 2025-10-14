/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"
#include "gpio.h"

AxisSystem_t Axis;

void Axis_Init(void)
{
    Axis.X = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, RIGHT};
    Axis.Y = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, FORWARD};
    Axis.Z = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 0, DOWN};
}

void Axis_Home(void)
{
    Axis.X = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 1, RIGHT};
    Axis.Y = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 1, FORWARD};
    Axis.Z = (ServoMotor_t){0, MOTOR_IDLE, 0, 0.0f, 1, UP};
}

// ================== DIR ==================

void Toggle_Dir_X(void)
{
    if (GPIOA->ODR & (1 << 4))
        gpio_clr(GPIOA, 4);
    else
        gpio_set(GPIOA, 4);
}

void Toggle_Dir_Y(void)
{
    if (GPIOA->ODR & (1 << 3))
        gpio_clr(GPIOA, 3);
    else
        gpio_set(GPIOA, 3);
}

// ================== Set Feedrate ==================
void X_SetFeed_mm_s(float feed_mm_s)
{
    uint32_t f_hz = (uint32_t)roundf(feed_mm_s * STEPS_PER_MM);
    if (f_hz > FREQ_MAX) f_hz = FREQ_MAX;
    if (f_hz == 0) { TIM2->CCER &= ~TIM_CCER_CC1E; return; }

    uint32_t arr = (1000000u / f_hz) - 1;
    if (arr < 2) arr = 2;

    TIM2->ARR  = arr;
    TIM2->CCR1 = (arr + 1) / 2;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;
}

void Y_SetFeed_mm_s(float feed_mm_s)
{
    uint32_t f_hz = (uint32_t)roundf(feed_mm_s * STEPS_PER_MM);
    if (f_hz > FREQ_MAX) f_hz = FREQ_MAX;
    if (f_hz == 0) { TIM3->CCER &= ~TIM_CCER_CC1E; return; }

    uint32_t arr = (1000000u / f_hz) - 1;
    if (arr < 2) arr = 2;

    TIM3->ARR  = arr;
    TIM3->CCR1 = (arr + 1) / 2;
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->SR  &= ~TIM_SR_UIF;
}

// ================== Start Steps ==================
void X_StartSteps(uint32_t steps)
{
    x_steps_rem = steps;
    TIM2->CNT = 0;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CCER |= TIM_CCER_CC1E; // bật output
    TIM2->CR1  |= TIM_CR1_CEN;   // bắt đầu đếm
}

void Y_StartSteps(uint32_t steps)
{
    y_steps_rem = steps;
    TIM3->CNT = 0;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1  |= TIM_CR1_CEN;
}

// ================== Move position ==================
//static float cur_x_mm = 0.0f;
//static float cur_y_mm = 0.0f;

float cur_x_mm = 0.0f;
float cur_y_mm = 0.0f;

void move_to_mm(float x_mm, float y_mm, float feed_mm_s)
{
    int32_t dx_steps = lroundf((x_mm - cur_x_mm) * STEPS_PER_MM);
    int32_t dy_steps = lroundf((y_mm - cur_y_mm) * STEPS_PER_MM);

    uint32_t nx = (dx_steps >= 0) ? dx_steps : -dx_steps;
    uint32_t ny = (dy_steps >= 0) ? dy_steps : -dy_steps;

    if (nx == 0) TIM2->CCER &= ~TIM_CCER_CC1E;
    if (ny == 0) TIM3->CCER &= ~TIM_CCER_CC1E;
    if (nx == 0 && ny == 0) return;

    // --- 1�?⃣ Set hướng trước ---
    Set_Dir_X(dx_steps >= 0);
    Set_Dir_Y(dy_steps >= 0);

    // --- 2�?⃣ Ch�? 10ms ổn định DIR ---
    //delay_ms(10);
    HAL_Delay(10);

    // --- 3�?⃣ Tính th�?i gian chạy ---
    float dist_mm = sqrtf((float)(nx*nx + ny*ny)) / STEPS_PER_MM;
    float T = dist_mm / feed_mm_s;
    if (T <= 0) T = 0.001f;

    // --- 4�?⃣ Tính tần số ---
    uint32_t f_x = (uint32_t)roundf(nx / T);
    uint32_t f_y = (uint32_t)roundf(ny / T);
    if (f_x > FREQ_MAX) f_x = FREQ_MAX;
    if (f_y > FREQ_MAX) f_y = FREQ_MAX;

    // --- 5�?⃣ Chạy ---
    if (nx > 0) { X_SetFeed_mm_s((float)f_x / STEPS_PER_MM); X_StartSteps(nx); }
    else        { TIM2->CCER &= ~TIM_CCER_CC1E; }

    if (ny > 0) { Y_SetFeed_mm_s((float)f_y / STEPS_PER_MM); Y_StartSteps(ny); }
    else        { TIM3->CCER &= ~TIM_CCER_CC1E; }

    while (x_steps_rem || y_steps_rem);
    cur_x_mm = x_mm;
    cur_y_mm = y_mm;
//    Set_Dir_X(1);
//    Set_Dir_Y(1);
}

void home_all(void)
{
//    x_home_done = 0;
//    y_home_done = 0;
    Axis.X.isHomed = 0;
    Axis.Y.isHomed = 0;
    exti_init();

    Set_Dir_X(LEFT);
    Set_Dir_Y(BACKWARD);

    //delay_ms(10);
    HAL_Delay(10);

    X_SetFeed_mm_s(20.0f);
    Y_SetFeed_mm_s(20.0f);

    x_steps_rem = 0xFFFFFFFF;
    y_steps_rem = 0xFFFFFFFF;

    // Reset counter & flags
    TIM2->CNT = 0;
    TIM3->CNT = 0;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM3->SR &= ~TIM_SR_UIF;

    // Bật timer **sau khi DIR ổn định**
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM2->CR1  |= TIM_CR1_CEN;
    TIM3->CR1  |= TIM_CR1_CEN;

    while (!(Axis.X.isHomed && Axis.X.isHomed))
    {

    }

//    Axis.X.position = 0.0f;
//    Axis.Y.position = 0.0f;
//    cur_x_mm = 0.0f;
//    cur_y_mm = 0.0f;
    Axis_Home();

    // 7️⃣ Dừng hoàn toàn Timer
    TIM2->CCER &= ~TIM_CCER_CC1E;
    TIM3->CCER &= ~TIM_CCER_CC1E;
    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM3->CR1  &= ~TIM_CR1_CEN;
}

//__attribute__((used))
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1u << 5))
    {
        EXTI->PR = (1u << 5);
        //x_home_done = 1;
        Axis.X.isHomed = 1;
        TIM2->CCER &= ~TIM_CCER_CC1E; // tắt output
        TIM2->CR1  &= ~TIM_CR1_CEN;   // dừng timer
    }
}
//__attribute__((used))
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 13))
    {
        EXTI->PR = (1u << 13);
        Axis.Y.isHomed = 1;
        TIM3->CCER &= ~TIM_CCER_CC1E;
        TIM3->CR1  &= ~TIM_CR1_CEN;
    }
}

