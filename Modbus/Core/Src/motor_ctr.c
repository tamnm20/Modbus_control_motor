/*
 * motor_ctr.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "motor_ctr.h"
#include "gpio.h"

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
    x_home_done = 0;
    y_home_done = 0;

    exti_init();

    // 1️⃣ Đặt hướng âm
    Set_Dir_X(0);
    Set_Dir_Y(0);

    // 2️⃣ Chờ 10ms cho DIR ổn định
    //delay_ms(10);
    HAL_Delay(10);

    // 3️⃣ Đặt tốc độ về home
    X_SetFeed_mm_s(20.0f);
    Y_SetFeed_mm_s(20.0f);

    // 4️⃣ Chạy không giới hạn
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

    // 5️⃣ Đợi đến khi cả hai cảm biến đều báo home
    while (!(x_home_done && y_home_done))
    {
        // Nếu 1 trục về trước, ISR sẽ tắt riêng trục đó
    }

    // 6️⃣ Đặt lại tọa độ gốc
    cur_x_mm = 0.0f;
    cur_y_mm = 0.0f;

    // 7️⃣ Dừng hoàn toàn Timer
    TIM2->CCER &= ~TIM_CCER_CC1E;
    TIM3->CCER &= ~TIM_CCER_CC1E;
    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM3->CR1  &= ~TIM_CR1_CEN;
}
