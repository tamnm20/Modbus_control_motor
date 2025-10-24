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
volatile uint32_t g_tick_ms = 0;
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

void Global_Timer_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 168 - 1;     // 1 MHz
    TIM1->ARR = 1000 - 1;    // 1 ms
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1  |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;
        g_tick_ms++;
    }
}

/* Scheduler mềm đơn giản */
uint8_t Task_RunEvery(uint16_t period_ms)
{
    static uint32_t last_run = 0;
    if ((uint32_t)(g_tick_ms - last_run) >= period_ms)
    {
        last_run = g_tick_ms;
        return 1;
    }
    return 0;
}

uint32_t millis(void) { return g_tick_ms; }

void TIM6_Init_1ms(void)
{
    // Bật clock cho TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    /*
     * F_APB1 = 84 MHz
     * TIM6 counter clock = 84 MHz
     * Chọn PSC để còn 84 000 Hz → ARR = 83 -> 1 ms
     */
    TIM6->PSC = 84 - 1;     // 84 MHz / 84 = 1 MHz (1 tick = 1 µs)
    TIM6->ARR = 1000 - 1;   // 1000 tick = 1 ms

    TIM6->CNT = 0;
    TIM6->SR  &= ~TIM_SR_UIF;     // clear flag
    TIM6->DIER |= TIM_DIER_UIE;   // enable interrupt
    TIM6->CR1  |= TIM_CR1_CEN;    // start counter

    NVIC_SetPriority(TIM6_DAC_IRQn, 6);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* === ISR TIM6 mỗi 1ms === */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &= ~TIM_SR_UIF;
        Timer6_Callback_1ms();
    }
}

//#ifndef TIM1CLK_HZ
//#define TIM1CLK_HZ (168000000UL) /* TIM1 clock thực tế */
//#endif
//
///* ==================== Biến toàn cục ==================== */
//tMsg_Global_Tick g_Global_Tick_Msg;
//tMsg_Time_s      ga_tCAN_Time_Msg[TID_COUNT];
//
///* ==================== Tiện ích nội bộ ==================== */
//static inline uint8_t _timer_elapsed(U32 start, U32 delay_ms, U32 now)
//{
//    /* Kiểm tra (now - start) >= delay_ms với wrap 32-bit an toàn */
//    return (uint32_t)(now - start) >= delay_ms ? 1u : 0u;
//}
//
///* ======================================================================
// * Khởi tạo TIM1: 1 kHz update interrupt
// * ====================================================================== */
//void Timer1_Init(void)
//{
//    /* Bật clock cho TIM1 (APB2) */
//    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//
//    /* Dừng TIM1 trước khi cấu hình */
//    TIM1->CR1 = 0;
//
//    /* Tính PSC/ARR cho 1kHz (1ms) theo giả định TIM1CLK_HZ */
//    /* Ở cấu hình mặc định F407: TIM1CLK_HZ = 168 MHz */
//    TIM1->PSC = (uint16_t)(168 - 1);   /* 167 => chia 168 */
//    TIM1->ARR = (uint16_t)(1000 - 1);  /* 999  => đếm 1000 lần */
//
//    /* Clear cờ Update và enable ngắt Update */
//    TIM1->SR   = ~(TIM_SR_UIF);
//    TIM1->DIER = TIM_DIER_UIE;
//
//    /* Generate update event để load PSC/ARR ngay */
//    TIM1->EGR = TIM_EGR_UG;
//
//    /* Bật NVIC cho TIM1 Update (TIM1_UP_TIM10_IRQn) */
//    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5);
//    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
//
//    /* Enable counter */
//    TIM1->CR1 |= TIM_CR1_CEN;
//}
//
///* ======================================================================
// * ISR: TIM1 Update (1ms/tick)
// * - Tăng g_Global_Tick_Msg.Tick_1ms
// * - Không cần xử lý gì thêm ở đây; logic timer mềm xử lý ở Delay_Time_Get
// * ====================================================================== */
//void TIMER1_TickISR(void)
//{
//    g_Global_Tick_Msg.Tick_1ms++;
//}
//
///* Weak IRQ handler: gọi sang hook TIMER1_TickISR() */
//void TIM1_UP_TIM10_IRQHandler(void)
//{
//    if (TIM1->SR & TIM_SR_UIF) {
//        TIM1->SR = (uint16_t)~TIM_SR_UIF; /* clear UIF */
//        TIMER1_TickISR();
//    }
//}
//
///* ======================================================================
// * Global init: clear biến + bật TIM1
// * ====================================================================== */
//void Global_Timer_Init(void)
//{
//    for (U8 i = 0U; i < TID_COUNT; i++) {
//        ga_tCAN_Time_Msg[i].Set        = (U8)0U;
//        ga_tCAN_Time_Msg[i].Delay_Time = (U32)0U;
//        ga_tCAN_Time_Msg[i].Cur_Time   = (U32)0U;
//        ga_tCAN_Time_Msg[i].End_Time   = (U32)0U;
//    }
//
//    g_Global_Tick_Msg.Tick_1ms = 0U;
//    g_Global_Tick_Msg.Limit    = 0U;
//    g_Global_Tick_Msg.Over_Set = 0U;
//
//    Timer1_Init();
//}
//
///* ======================================================================
// * Busy-wait ms dựa trên Tick_1ms (không phụ thuộc HAL_Delay)
// * ====================================================================== */
//void Wait_ms(U16 ms)
//{
//    U32 start = g_Global_Tick_Msg.Tick_1ms;
//    while ((U32)(g_Global_Tick_Msg.Tick_1ms - start) < (U32)ms) {
//        /* bận rộn: có thể chèn __WFI() nếu muốn tiết kiệm điện */
//        __NOP();
//    }
//}
//
///* ======================================================================
// * API “timer mềm” theo ID
// * ====================================================================== */
//
//void Delay_Time_Expire(U8 ID)
//{
//    if (ID >= TID_COUNT) return;
//    ga_tCAN_Time_Msg[ID].Set        = (U8)0U;
//    ga_tCAN_Time_Msg[ID].Delay_Time = (U32)0U;
//    ga_tCAN_Time_Msg[ID].Cur_Time   = (U32)0U;
//    ga_tCAN_Time_Msg[ID].End_Time   = (U32)0U;
//}
//
//void Delay_Time_Set(U8 ID, U16 Delay_Time_ms)
//{
//    if (ID >= TID_COUNT) return;
//    U32 now = g_Global_Tick_Msg.Tick_1ms;
//
//    ga_tCAN_Time_Msg[ID].Cur_Time   = now;
//    ga_tCAN_Time_Msg[ID].Delay_Time = (U32)Delay_Time_ms;
//    ga_tCAN_Time_Msg[ID].Set        = TRUE;
//    ga_tCAN_Time_Msg[ID].End_Time   = now + (U32)Delay_Time_ms; /* tham khảo */
//}
//
///* Trả về:
// * 0: timer chưa Set
// * 1: đã hết hạn (và tự reload lại chu kỳ)
// * 2: đang đếm (chưa hết hạn)
// */
//U8 Delay_Time_Get(U8 ID)
//{
//    if (ID >= TID_COUNT) return 0U;
//
//    if (ga_tCAN_Time_Msg[ID].Set != TRUE) {
//        return 0U;
//    }
//
//    U32 now   = g_Global_Tick_Msg.Tick_1ms;
//    U32 start = ga_tCAN_Time_Msg[ID].Cur_Time;
//    U32 dly   = ga_tCAN_Time_Msg[ID].Delay_Time;
//
//    if (_timer_elapsed(start, dly, now)) {
//        /* Auto-reload để giữ chu kỳ đều */
//        ga_tCAN_Time_Msg[ID].Cur_Time = now;            /* chốt lại mốc mới */
//        ga_tCAN_Time_Msg[ID].End_Time = now + dly;
//        return 1U;
//    } else {
//        return 2U;
//    }
//}
//
//U16 Get_Time(void)
//{
//    return (U16)(g_Global_Tick_Msg.Tick_1ms & 0xFFFFu);
//}
//
//U8 Get_Time_Set(U8 ID)
//{
//    if (ID >= TID_COUNT) return 0U;
//    return ga_tCAN_Time_Msg[ID].Set;
//}
