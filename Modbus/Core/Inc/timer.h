/*
 * timer.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32f4xx_hal.h"

extern volatile uint32_t x_steps_rem;
extern volatile uint32_t y_steps_rem;

void tim2_init(void);
void tim3_init(void);

/* ================= Global Define (theo ví dụ C8051) ================= */

#define TID_FRAME_VALIDE            (0x01U)
#define TID_FRAME_COUNT             (0x02U)
#define TID_OVERLAY_GUIDELINE       (0x03U)
#define TID_SENSOR_IDLE_PERIOD      (0x04U)
#define TID_I2C_COMM_CHECK          (0x05U)
#define TID_MODBUS                  (0x06U)
#define TID_DTC_WRITE               (0x07U)
#define TID_DIAG_ECU_RESET          (0x08U)
#define TID_FIRME_UPDATE            (0x09U)
#define TID_SENSOR_RESET            (0x0AU)
#define TID_CHECKSTATUS_LOOP        (0x0BU)
#define TID_ADC_CHANGE_TIMER        (0x0CU)
#define TID_I2C_BUSY_CHECK          (0x0DU)
#define TID_DOORBELL_CHECK          (0x0EU)
#define TID_GEAR_CHATTERING         (0x0FU)
#define TID_SAS_TIMEOUT             (0x10U)
#define TID_MAIN_TIMER              (0x11U)
#define TID_BLIND_TIME              (0x12U)
#define TID_3000_CALI_TIME          (0x13U)
#define TID_HEV_GEAR_CHATTERING     (0x14U)
#define TID_5000_CALI_TIME          (0x15U)
#define TID_TOPVIEW_SW              (0x16U)
#define TID_USM_RESET               (0x17U)
#define TID_RVM_SW                  (0x18U)
#define TID_COUNT                   (0x19U)

/* ---------- TIMER DELAY (ms) ---------- */
#define DT_FRAME_VALIDE             (100U)
#define DT_FRAME_COUNT              (500U)
#define DT_MODBUS                   (1U)
#define DT_OVERLAY_GUIDELINE        (33U)
#define DT_DTC_WRITE                (50U)
#define DT_DIAG_ECU_RESET           (100U)
#define DT_FIRME_UPDATE             (30U)
#define DT_SENSOR_RESET             (500U)
#define DT_SENSOR_IDLE_PERIOD       (300U)
#define DT_CHECKSTATUS_LOOP         (100U)
#define DT_ADC_CHANGE_TIMER         (10U)
#define DT_I2C_COMM_CHECK           (100U)
#define DT_I2C_BUSY_CHECK           (10U)
#define DT_DOORBELL_CHECK           (10U)
#define DT_GEAR_CHATTERING          (350U)
#define DT_SAS_TIMEOUT              (500U)
#define DT_MAIN_TIMER               (1U)
#define DT_BLIND_TIME               (500U)
#define DT_BLIND_150                (150U)
#define DT_BLIND_10                 (10U)
#define DT_3000_CALI_TIME           (3000U)
#define DT_HEV_GEAR_CHATTERING      (350U)
#define DT_5000_CALI_TIME           (5000U)
#define DT_TOPVIEW_SW               (150U)
#define DT_USM_RESET                (150U)
#define DT_RVM_SW                   (1000U)

/* ================= Kiểu dữ liệu tương thích ================= */
typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;

#ifndef TRUE
#  define TRUE  (1u)
#  define FALSE (0u)
#endif

typedef struct {
    U8  Set;           /* = TRUE khi timer đang chạy (chu kỳ) */
    U32 Delay_Time;    /* ms của chu kỳ */
    U32 Cur_Time;      /* tick hiện tại (ms) lần gần nhất check/set */
    U32 End_Time;      /* Cur_Time + Delay_Time (tham khảo) */
} tMsg_Time_s;

typedef struct {
    U32 Tick_1ms;      /* counter ms global (overflow tự nhiên 32-bit) */
    U32 Limit;         /* không dùng nhưng giữ để tương thích */
    U8  Over_Set;      /* không dùng nhưng giữ để tương thích */
} tMsg_Global_Tick;

/* ================= API ================= */
#ifdef __cplusplus
extern "C" {
#endif

/* Khởi tạo toàn cục (clear biến + bật TIM1 1kHz) */
void Global_Timer_Init(void);

/* Chỉ khởi tạo Timer1 1kHz (nếu cần tự gọi riêng) */
void Timer1_Init(void);

/* Delay bận rộn theo ms, dùng Tick_1ms (không dùng HAL_Delay) */
void Wait_ms(U16 ms);

/* Bộ API “timer mềm” theo ID */
void Delay_Time_Set(U8 ID, U16 Delay_Time_ms);
U8   Delay_Time_Get(U8 ID);     /* 0: not set, 1: timeout (đã tự reload), 2: đang đếm */
void Delay_Time_Expire(U8 ID);  /* Hủy / reset timer ID */

U16  Get_Time(void);            /* trả về (U16)Tick_1ms - tương thích C8051 */
U8   Get_Time_Set(U8 ID);       /* trả về trạng thái Set của timer ID */

/* Biến dùng chung (extern) */
extern tMsg_Global_Tick g_Global_Tick_Msg;
extern tMsg_Time_s      ga_tCAN_Time_Msg[TID_COUNT];

/* Hàm ISR hook (nếu không dùng weak IRQ mặc định) */
void TIMER1_TickISR(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIMER_H_ */
