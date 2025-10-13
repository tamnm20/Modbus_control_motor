/*
 * timer.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */

#include "timer.h"
// ================== Gvar ==================
volatile uint32_t x_steps_rem = 0;
volatile uint32_t y_steps_rem = 0;
// ================== Timer Init ==================
void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;                      // đảm bảo timer tắt
    TIM2->CCER &= ~TIM_CCER_CC1E;       // tắt output channel 1 trước

    TIM2->PSC = 84 - 1;                 // 1 MHz tick (với APB1 timer clock = 84 MHz)
    TIM2->ARR = 1000 - 1;               // 1 kHz mặc định
    TIM2->CCR1 = (TIM2->ARR + 1) / 2;   // duty 50%

    TIM2->CCMR1 &= ~(7u << 4);
    TIM2->CCMR1 |= (6u << 4);           // PWM1 mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM2->CCER &= ~TIM_CCER_CC1P;       // active high
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);

    // ❌ KHÔNG bật CC1E và KHÔNG bật CEN ở đây ❌
}

void tim3_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 = 0;
    TIM3->CCER &= ~TIM_CCER_CC1E;

    TIM3->PSC = 84 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->CCR1 = (TIM3->ARR + 1) / 2;

    TIM3->CCMR1 &= ~(7u << 4);
    TIM3->CCMR1 |= (6u << 4); // PWM1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM3->CCER &= ~TIM_CCER_CC1P;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR  &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);

    // ❌ KHÔNG bật CC1E, KHÔNG bật CEN
}

// ================== IRQ Handler ==================
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        if (x_steps_rem && (--x_steps_rem == 0))
        {
            TIM2->CCER &= ~TIM_CCER_CC1E; // tắt output
            TIM2->CR1  &= ~TIM_CR1_CEN;   // dừng timer
        }
            //TIM2->CCER &= ~TIM_CCER_CC1E;
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        if (y_steps_rem && (--y_steps_rem == 0))
        {
            TIM3->CCER &= ~TIM_CCER_CC1E; // tắt output
            TIM3->CR1  &= ~TIM_CR1_CEN;   // dừng timer
        }
           // TIM3->CCER &= ~TIM_CCER_CC1E;
    }
}
