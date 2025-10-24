/*
 * axis_task.c
 *
 *  Created on: Oct 23, 2025
 *      Author: TAMRD
 */


#include "axis_task.h"
#include "motor_ctr.h"
#include "modbusSlave.h"
#include "math.h"

#define CMD_QUEUE_SIZE 100

typedef struct {
    AxisCmdType_t type;
    AxisName_t axis;
    float x, y, feed;
    uint8_t dir;
    uint32_t steps;
} AxisCommand_t;

typedef enum {
    MOVE_IDLE,
    MOVE_RUNNING
} MoveState_t;

typedef struct {
    MoveState_t state;
    float x_target;
    float y_target;
    float feed_target;
    uint32_t x_steps;
    uint32_t y_steps;
} MoveTask_t;

static MoveTask_t moveTask = { MOVE_IDLE };

static AxisCommand_t queue[CMD_QUEUE_SIZE];
static uint8_t head = 0, tail = 0;

void Axis_TaskInit(void)
{
    Axis_Home();
}

static uint8_t queue_full(void)
{
    return ((head + 1) % CMD_QUEUE_SIZE) == tail;
}

void Axis_QueueCommand(AxisName_t axis, uint8_t dir, uint32_t steps, float feed)
{
    if (queue_full()) return;
    queue[head] = (AxisCommand_t){CMD_MOVE_STEP, axis, 0,0,feed,dir,steps};
    head = (head + 1) % CMD_QUEUE_SIZE;
}

void Axis_QueueMoveTo(float x, float y, float feed)
{
    if (queue_full()) return;
    queue[head] = (AxisCommand_t){CMD_MOVE_TO, 0, x, y, feed,0,0};
    head = (head + 1) % CMD_QUEUE_SIZE;
}

void Axis_QueueHomeAll(void)
{
    if (queue_full()) return;
    queue[head] = (AxisCommand_t){CMD_HOME_ALL,0,0,0,0,0,0};
    head = (head + 1) % CMD_QUEUE_SIZE;
}

/* Xử lý tuần tự từng lệnh */
void Axis_TaskUpdate(void)
{
    Move_To_Task();

    if (Axis.X.state == MOTOR_BUSY || Axis.Y.state == MOTOR_BUSY)
        return;

    if (head == tail)
        return;

    AxisCommand_t *cmd = &queue[tail];
    tail = (tail + 1) % CMD_QUEUE_SIZE;

    switch (cmd->type)
    {
    case CMD_MOVE_STEP:
        Axis_MoveStep(cmd->axis, cmd->dir, cmd->steps, cmd->feed);
        break;

    case CMD_MOVE_TO:
        Move_To_Start(cmd->x, cmd->y, cmd->feed);
        break;

    case CMD_HOME_ALL:
        //Home_All();
    	Move_To_Start(0, 0, 10000.0f);
        break;
    }
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

void Move_To_Task(void)
{
    if (moveTask.state == MOVE_IDLE)
        return;
    /* --- Tính số bước đã thực hiện --- */
    uint32_t done_x = moveTask.x_steps - x_steps_rem;
    uint32_t done_y = moveTask.y_steps - y_steps_rem;

    /* --- Quãng đường tương ứng theo mm --- */
    float moved_x = (float)done_x / STEPS_PER_MM;
    float moved_y = (float)done_y / STEPS_PER_MM;

    /* --- Vị trí hiện tại tạm thời (so với gốc di chuyển) --- */
    float cur_x = (Axis.X.direction == RIGHT)
                    ? Axis.X.position + moved_x
                    : Axis.X.position - moved_x;

    float cur_y = (Axis.Y.direction == FORWARD)
                    ? Axis.Y.position + moved_y
                    : Axis.Y.position - moved_y;

    /* --- Cập nhật mỗi khi vượt qua 1 mm --- */
    static float last_update_x = 0.0f;
    static float last_update_y = 0.0f;

    if (fabsf(cur_x - last_update_x) >= 1.0f)
    {
        last_update_x = cur_x;
        Holding_Registers_Database[0] = (uint16_t)cur_x;
    }

    if (fabsf(cur_y - last_update_y) >= 1.0f)
    {
        last_update_y = cur_y;
        Holding_Registers_Database[1] = (uint16_t)cur_y;
    }
    /* --- Nếu cả hai trục đã xong thì hoàn tất --- */
    if (Axis.X.state == MOTOR_IDLE && Axis.Y.state == MOTOR_IDLE)
    {
        Axis.X.position = moveTask.x_target;
        Axis.Y.position = moveTask.y_target;
        Holding_Registers_Database[0] = Axis.X.position;
        Holding_Registers_Database[1] = Axis.Y.position;
        moveTask.state = MOVE_IDLE;
        return;
    }
}

