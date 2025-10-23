/*
 * axis_task.h
 *
 *  Created on: Oct 23, 2025
 *      Author: TAMRD
 */

#ifndef INC_AXIS_TASK_H_
#define INC_AXIS_TASK_H_

#include "stm32f4xx_hal.h"
#include "motor_ctr.h"

typedef enum {
    CMD_MOVE_STEP = 0,
    CMD_MOVE_TO,
    CMD_HOME_ALL
} AxisCmdType_t;

void Axis_TaskInit(void);
void Axis_TaskUpdate(void);

void Axis_QueueCommand(AxisName_t axis, uint8_t dir, uint32_t steps, float feed);
void Axis_QueueMoveTo(float x, float y, float feed);
void Axis_QueueHomeAll(void);
void Move_To_Start(float x_mm, float y_mm, float feed_mm_s);
void Move_To_Task(void);

#endif /* INC_AXIS_TASK_H_ */
