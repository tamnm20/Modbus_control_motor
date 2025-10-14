/*
 * gpio.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */


#include "gpio.h"

volatile uint8_t x_home_done = 0;
volatile uint8_t y_home_done = 0;
// ================== IO Init ==================
void io_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // --- PA0: TIM2_CH1 (AF1) ---
    GPIOA->MODER &= ~(3u << (0 * 2));
    GPIOA->MODER |=  (2u << (0 * 2));  // AF mode
    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] |=  (1u << (0 * 4)); // AF1 (TIM2)

    // --- PA6: TIM3_CH1 (AF2) ---
    GPIOA->MODER &= ~(3u << (6 * 2));
    GPIOA->MODER |=  (2u << (6 * 2));  // AF mode
    GPIOA->AFR[0] &= ~(0xF << (6 * 4));
    GPIOA->AFR[0] |=  (2u << (6 * 4)); // AF2 (TIM3)

    // --- PA4: DIR_X output push-pull ---
    GPIOA->MODER &= ~(3u << (4 * 2));
    GPIOA->MODER |=  (1u << (4 * 2));

    // --- PA3: DIR_Y output push-pull ---
    GPIOA->MODER &= ~(3u << (3 * 2));
    GPIOA->MODER |=  (1u << (3 * 2));

    GPIOA->BSRR = (1u << (4 + 16)) | (1u << (3 + 16));
}

void exti_init(void)
{
    // Bật clock cho SYSCFG và GPIO
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // --- PA5: Input Pull-Down (Home X) ---
    GPIOA->MODER &= ~(3u << (5 * 2));
    GPIOA->PUPDR &= ~(3u << (5 * 2));
    GPIOA->PUPDR |=  (2u << (5 * 2));  // Pull-down

    // --- PC13: Input Pull-Down (Home Y) ---
    GPIOC->MODER &= ~(3u << (13 * 2));
    GPIOC->PUPDR &= ~(3u << (13 * 2));
    GPIOC->PUPDR |=  (2u << (13 * 2)); // Pull-down

    // --- Gán chân vào EXTI ---
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5;           // EXTI5 ← PA5
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;       // EXTI13 ← PC13

    // --- Cấu hình ngắt sườn lên ---
    EXTI->RTSR |= (1u << 5) | (1u << 13);                 // Rising edge
    EXTI->FTSR &= ~((1u << 5) | (1u << 13));              // Không bắt sườn xuống

    // --- Cho phép mask ---
    EXTI->IMR |= (1u << 5) | (1u << 13);

    // --- Ưu tiên và bật NVIC ---
    NVIC_SetPriority(EXTI9_5_IRQn, 1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

//// ================== Delay ms ==================
//void delay_ms(uint32_t ms)
//{
//    for (uint32_t i = 0; i < ms; i++)
//        for (volatile uint32_t j = 0; j < 42000; j++); // ~1ms @168MHz
//}
