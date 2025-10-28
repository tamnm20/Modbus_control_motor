/*
 * axis_task.c
 *
 *  Created on: Oct 23, 2025
 *      Author: TAMRD
 */


#include "axis_task.h"
#include "motor_ctr.h"
#include "modbusSlave.h"
#include <stdlib.h>
#include "math.h"

typedef enum {
    TASK_IDLE,
    TASK_MOVING,
    TASK_HOMING
} TaskState_t;

typedef struct {
    TaskState_t state;
//    float target_x, target_y;
} MoveTask_t;

static MoveTask_t task = {TASK_IDLE};

/* ========== HELPER FUNCTIONS ========== */

uint32_t Axis_GetStepsDone(AxisName_t axis)
{
    switch(axis)
    {
        case AXIS_X: return TIM2->CNT;
        case AXIS_Y: return TIM5->CNT;
        case AXIS_Z: return TIM9->CNT;
    }
    return 0;
}

float Axis_GetCurrentPosition(AxisName_t axis)
{
    ServoMotor_t *m;
    uint32_t steps_done;

    switch(axis)
    {
        case AXIS_X: m = &Axis.X; steps_done = TIM2->CNT; break;
        case AXIS_Y: m = &Axis.Y; steps_done = TIM5->CNT; break;
        case AXIS_Z: m = &Axis.Z; steps_done = TIM9->CNT; break;
        default: return 0;
    }

    if(m->state != MOTOR_BUSY)
        return m->position;

    float delta_mm = (float)steps_done / STEPS_PER_MM;

    if(m->direction == RIGHT || m->direction == FORWARD || m->direction == DOWN)
        return m->start_position + delta_mm;
    else
        return m->start_position - delta_mm;
}

/* ========== MODBUS UPDATE (Gọi mỗi 50ms) ========== */

void Axis_UpdateModbusRegisters(void)
{
    if(Axis.X.state == MOTOR_IDLE &&
       Axis.Y.state == MOTOR_IDLE &&
       Axis.Z.state == MOTOR_IDLE)
    {
        Holding_Registers_Database[0] = (uint16_t)roundf(Axis.X.position);
        Holding_Registers_Database[1] = (uint16_t)roundf(Axis.Y.position);
        Holding_Registers_Database[2] = (uint16_t)roundf(Axis.Z.position);
    }
}

/* ========== MOVE TO ========== */

void Axis_MoveTo(float x_mm, float y_mm, float feed_mm_s)
{
    // Kiểm tra bận
    if (Axis.X.state == MOTOR_BUSY || Axis.Y.state == MOTOR_BUSY)
        return;

    int32_t dx_steps = lroundf((x_mm - Axis.X.position) * STEPS_PER_MM);
    int32_t dy_steps = lroundf((y_mm - Axis.Y.position) * STEPS_PER_MM);

    uint32_t nx = abs(dx_steps);
    uint32_t ny = abs(dy_steps);

    if (nx == 0 && ny == 0) return;

    // Lưu vị trí bắt đầu
    Axis.X.start_position = Axis.X.position;
    Axis.Y.start_position = Axis.Y.position;
    Axis.X.target_position = x_mm;
    Axis.Y.target_position = y_mm;

    // Set hướng
    uint8_t dir_x = (dx_steps >= 0) ? RIGHT : LEFT;
    uint8_t dir_y = (dy_steps >= 0) ? FORWARD : BACKWARD;
    Set_Dir_X(dir_x);
    Set_Dir_Y(dir_y);
    Axis.X.direction = dir_x;
    Axis.Y.direction = dir_y;
    //HAL_Delay(1);

    // Tính tốc độ
    //float dist_mm = sqrtf((float)(dx_steps*dx_steps + dy_steps*dy_steps)) / STEPS_PER_MM;
    float dist_mm = sqrtf((float)nx * (float)nx + (float)ny * (float)ny) / STEPS_PER_MM;
    float T = dist_mm / feed_mm_s;
    if (T <= 0.001f) T = 0.001f;

    uint32_t f_x = (nx > 0) ? (uint32_t)roundf(nx / T) : 0;
    uint32_t f_y = (ny > 0) ? (uint32_t)roundf(ny / T) : 0;

    if (f_x > FREQ_MAX) f_x = FREQ_MAX;
    if (f_y > FREQ_MAX) f_y = FREQ_MAX;

    // Lưu tổng steps
    Axis.X.total_steps = nx;
    Axis.Y.total_steps = ny;

    // Khởi động
    if(nx > 0)
    {
        Motor_SetFeed(AXIS_X, (float)f_x / STEPS_PER_MM);
        Motor_Start(AXIS_X, nx);
    }

    if(ny > 0)
    {
        Motor_SetFeed(AXIS_Y, (float)f_y / STEPS_PER_MM);
        Motor_Start(AXIS_Y, ny);
    }

    task.state = TASK_MOVING;
//    task.target_x = x_mm;
//    task.target_y = y_mm;
}

/* ========== JOG  ========== */

void Axis_Jog(AxisName_t axis, uint8_t dir, uint32_t steps, float feed_mm_s)
{
    ServoMotor_t *m = NULL;
    float max_limit = 0.0f;

    // ===== 1. SELECT AXIS & CHECK STATE =====
    switch (axis)
    {
        case AXIS_X:
            if (Axis.X.isHomed == 0 || Axis.X.state == MOTOR_BUSY)
                return;
            m = &Axis.X;
            max_limit = X_MAX_MM;
            break;

        case AXIS_Y:
            if (Axis.Y.isHomed == 0 || Axis.Y.state == MOTOR_BUSY)
                return;
            m = &Axis.Y;
            max_limit = Y_MAX_MM;
            break;

        case AXIS_Z:
            if (Axis.Z.isHomed == 0 || Axis.Z.state == MOTOR_BUSY)
                return;
            m = &Axis.Z;
            max_limit = Z_MAX_MM;
            break;

        default:
            return;
    }

    // ===== 2. DETERMINE DIRECTION =====
    // neg = moving towards zero (home)
    uint8_t neg = (dir == LEFT || dir == BACKWARD || dir == UP);

    // ===== 3. BOUNDARY CHECK - Hard Limit =====
    if (neg && m->position <= EPS_MM)
    {
        return;
    }

    if (!neg && m->position >= (max_limit - EPS_MM))
    {
        return;
    }

    // ===== 4. TRIM STEPS - Soft Limit =====
    if (neg)
    {
        float mm_to_zero = m->position;
        uint32_t max_steps = (uint32_t)floorf(mm_to_zero * STEPS_PER_MM + 0.5f);

        if (steps > max_steps)
            steps = max_steps;

        if (steps == 0)
            return;
    }
    else
    {
        float mm_to_max = max_limit - m->position;
        uint32_t max_steps = (uint32_t)floorf(mm_to_max * STEPS_PER_MM + 0.5f);

        if (steps > max_steps)
            steps = max_steps;

        if (steps == 0)
            return;
    }

    // ===== 5. SET DIRECTION =====
    switch (axis)
    {
        case AXIS_X:
            Set_Dir_X(dir);
            break;
        case AXIS_Y:
            Set_Dir_Y(dir);
            break;
        case AXIS_Z:
            Set_Dir_Z(dir);
            break;
    }
    //HAL_Delay(1);

    // ===== 6. SAVE START POSITION =====
    m->start_position = m->position;
    m->direction = dir;
    m->total_steps = steps;

    // ===== 7. CALCULATE TARGET POSITION =====
    float delta_mm = (float)steps / STEPS_PER_MM;

    if (neg)
    {
        m->target_position = m->position - delta_mm;
        if (m->target_position < 0.0f)
            m->target_position = 0.0f;
    }
    else
    {
        m->target_position = m->position + delta_mm;
        if (m->target_position > max_limit)
            m->target_position = max_limit;
    }

    // ===== 8. START MOTOR =====
    Motor_SetFeed(axis, feed_mm_s);
    Motor_Start(axis, steps);
}

/* ========== TASK UPDATE ========== */

//void Axis_TaskUpdate(void)
//{
//    // Kiểm tra hoàn thành
//    if(task.state == TASK_MOVING)
//    {
//        if(Axis.X.state == MOTOR_IDLE && Axis.Y.state == MOTOR_IDLE)
//        {
//            Axis.X.position = task.target_x;
//            Axis.Y.position = task.target_y;
//            Holding_Registers_Database[0] = (uint16_t)roundf(Axis.X.position);
//            Holding_Registers_Database[1] = (uint16_t)roundf(Axis.Y.position);
//            task.state = TASK_IDLE;
//        }
//    }
//
//    if(Axis.X.state == MOTOR_IDLE && Axis.X.total_steps > 0)
//    {
//        Axis.X.position = Axis.X.target_position;
//        Holding_Registers_Database[0] = (uint16_t)roundf(Axis.X.position);
//        Axis.X.total_steps = 0;
//    }
//
//    if(Axis.Y.state == MOTOR_IDLE && Axis.Y.total_steps > 0)
//    {
//        Axis.Y.position = Axis.Y.target_position;
//        Holding_Registers_Database[1] = (uint16_t)roundf(Axis.Y.position);
//        Axis.Y.total_steps = 0;
//    }
//
//    if(Axis.Z.state == MOTOR_IDLE && Axis.Z.total_steps > 0)
//    {
//        Axis.Z.position = Axis.Z.target_position;
//        Holding_Registers_Database[2] = (uint16_t)roundf(Axis.Z.position);
//        Axis.Z.total_steps = 0;
//    }
//}

void Axis_TaskUpdate(void)
{
    // Real-time position update khi MOVING
    if(task.state == TASK_MOVING)
    {
        // Đọc steps đã hoàn thành
        uint32_t done_x = TIM2->CNT;
        uint32_t done_y = TIM5->CNT;

        // Tính quãng đường đã đi (mm)
        float moved_x = (float)done_x / STEPS_PER_MM;
        float moved_y = (float)done_y / STEPS_PER_MM;

        // Tính toán vị trí hiện tại dựa trên hướng di chuyển
        float cur_x = (Axis.X.direction == RIGHT)
                      ? Axis.X.start_position + moved_x
                      : Axis.X.start_position - moved_x;

        float cur_y = (Axis.Y.direction == FORWARD)
                      ? Axis.Y.start_position + moved_y
                      : Axis.Y.start_position - moved_y;

        // Theo dõi và cập nhật vị trí theo từng mm
        static float last_update_x = 0.0;
        static float last_update_y = 0.0;

        // Kiểm tra và cập nhật vị trí X
        if(fabsf(cur_x - last_update_x) >= 1.0f)
        {
            last_update_x = cur_x;
            Axis.X.position = cur_x;
            Holding_Registers_Database[0] = (uint16_t)roundf(cur_x);
        }

        // Kiểm tra và cập nhật vị trí Y
        if(fabsf(cur_y - last_update_y) >= 1.0f)
        {
            last_update_y = cur_y;
            Axis.Y.position = cur_y;
            Holding_Registers_Database[1] = (uint16_t)roundf(cur_y);
        }

        // Xác nhận hoàn thành nhiệm vụ di chuyển
        if(Axis.X.state == MOTOR_IDLE && Axis.Y.state == MOTOR_IDLE)
        {
            // Cập nhật vị trí cuối cùng của trục
            Axis.X.position = Axis.X.target_position;
            Axis.Y.position = Axis.Y.target_position;
            Holding_Registers_Database[0] = (uint16_t)roundf(Axis.X.position);
            Holding_Registers_Database[1] = (uint16_t)roundf(Axis.Y.position);

//            // Đặt lại điểm cập nhật cuối
//            last_update_x = Axis.X.target_position;
//            last_update_y = Axis.Y.target_position;

            // Chuyển trạng thái nhiệm vụ về không hoạt động
            task.state = TASK_IDLE;
        }
    }
    if(Axis.X.state == MOTOR_IDLE && Axis.X.total_steps > 0)
    {
        Axis.X.position = Axis.X.target_position;
        Holding_Registers_Database[0] = (uint16_t)roundf(Axis.X.position);
        Axis.X.total_steps = 0;
    }

    if(Axis.Y.state == MOTOR_IDLE && Axis.Y.total_steps > 0)
    {
        Axis.Y.position = Axis.Y.target_position;
        Holding_Registers_Database[1] = (uint16_t)roundf(Axis.Y.position);
        Axis.Y.total_steps = 0;
    }

    if(Axis.Z.state == MOTOR_IDLE && Axis.Z.total_steps > 0)
    {
        Axis.Z.position = Axis.Z.target_position;
        Holding_Registers_Database[2] = (uint16_t)roundf(Axis.Z.position);
        Axis.Z.total_steps = 0;
    }
}

