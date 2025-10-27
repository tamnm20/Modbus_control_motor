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

void Axis_TaskInit(void);
void Axis_TaskUpdate(void);

void Axis_UpdateModbusRegisters(void); // Gọi mỗi 50ms

// Command functions
void Axis_MoveTo(float x_mm, float y_mm, float feed_mm_s);
void Axis_Jog(AxisName_t axis, uint8_t dir, uint32_t steps, float feed_mm_s);

// Helper functions
uint32_t Axis_GetStepsDone(AxisName_t axis);
float Axis_GetCurrentPosition(AxisName_t axis);

#endif /* INC_AXIS_TASK_H_ */
