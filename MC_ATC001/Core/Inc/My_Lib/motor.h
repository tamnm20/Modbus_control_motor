/*
 * motor.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#define F_high	20000
#define Pulse_x_max 50000
#define Pulse_y_max 50000
#define Pulse_z_max 50000
typedef enum{
	Left,
	Right,
	In,
	Out,
	Up,
	Down,
	Unknow
} Sign;
typedef enum {
	OK,
	NG,
	Emty,
	Begin,
	End,
	OK_Begin,
	OK_End
} ItemState;
typedef struct {
	uint32_t x;
	uint32_t y;
	ItemState state;
}Item;
typedef enum{
	Run_by_Hand,
	Run_Auto,
	Go_to_Home,
	Stop
} MotorState;
typedef struct{
	Sign dir;
	MotorState state;
	uint16_t t0;
}Motor_t;
typedef struct{
	uint16_t Old_Position_x;
	uint16_t Old_Position_y;
	uint16_t Old_Position_z;
	uint16_t* Cur_x;
	uint16_t* Cur_y;
	uint16_t* Cur_z;
	uint16_t Taget_x;
	uint16_t Taget_y;
	uint16_t Taget_z;
}Handler_t;

extern Motor_t Motor_x;
extern Motor_t Motor_y;
extern Motor_t Motor_z;
extern Handler_t Handler;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

void Move_x(Sign dir, uint16_t Frequency);
void Move_y(Sign dir, uint16_t Frequency);
void Move_z(Sign dir, uint16_t Frequency);
void Move_x_2_target(uint32_t pulse,Sign dir);
void Move_y_2_target(uint32_t pulse,Sign dir);
void Move_z_2_target(uint32_t pulse,Sign dir);
void Move_Handle_2_point(Item newitem);
void Set_Frequency_Motor(uint16_t f, uint16_t f_max);

void Stop_x();
void Stop_y();
void Stop_z();



void Tim2_Init(void);
void Tim5_Init(void);
void Tim9_Init(void);
void Tim1_Init(void);
void Tim3_Init(void);
void Tim8_Init(void);
void Motor_x_Init(void);
void Motor_y_Init(void);
void Motor_z_Init(void);
#endif /* INC_MOTOR_H_ */
