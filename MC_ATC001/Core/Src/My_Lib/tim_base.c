/*
 * tim_base.c
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include <stdlib.h>
#include "My_Lib/tim_base.h"
#include "My_Lib/WDT.h"
#include "My_Lib/motor.h"
#include "My_Lib/RS232.h"
#include "My_Lib/delay.h"
//#define time 1000 //1ms
#define Time 1000
TIM_HandleTypeDef htim7;
uint8_t Emergency = 0;


typedef void (*MotorHandler_t)(void);
typedef struct {
    uint8_t bitMask;
    MotorHandler_t handler;
} MotorActionMap_t;

MotorActionMap_t motorActionTable[] = {
    { 1 << 2, Handle_Set   },
    { 1 << 3, Handle_Left  },
    { 1 << 4, Handle_Right },
    { 1 << 5, Handle_In    },
    { 1 << 6, Handle_Out   },
    { 1 << 7, Handle_Up    },
};
void scan_HMI(){
	if(Emergency!=0) return;
	uint8_t current = Control_motor->all;

	for (int i = 0; i < sizeof(motorActionTable)/sizeof(MotorActionMap_t); i++)
	{
		if (current & motorActionTable[i].bitMask)
		{
			motorActionTable[i].handler();
			break;
		}
	}

	if(Motor_x.state != Stop){
		*Handler.Cur_x = Motor_x.dir == Left? Handler.Old_Position_x - htim2.Instance -> CNT: Handler.Old_Position_x + htim2.Instance -> CNT;
		if(Motor_x.state == Run_by_Hand){
			if(*Handler.Cur_x > Pulse_x_max) {
				Emergency = 1;
				Stop_x();
			}
			Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_x.t0 +100 , 4000);
			if(Control_motor->bits.Left == 0 && Control_motor->bits.Right == 0)
			{
				Stop_x();
			}
		}
	}
	if(Motor_y.state != Stop){
		*Handler.Cur_y = Motor_y.dir == In? Handler.Old_Position_y - htim5.Instance -> CNT: Handler.Old_Position_y + htim5.Instance -> CNT;
		if(Motor_y.state == Run_by_Hand){
			if(*Handler.Cur_y > Pulse_y_max) {
				Emergency = 1;
				Stop_y();
			}
			Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_y.t0 +100 , 4000);
			if(Control_motor->bits.In == 0 && Control_motor->bits.Out == 0)
				Stop_y();
		}
	}

	if(Motor_z.state != Stop){
		*Handler.Cur_z = Motor_z.dir == Up? Handler.Old_Position_z - htim9.Instance -> CNT : Handler.Old_Position_z + htim9.Instance -> CNT;
		if(Motor_z.state == Run_by_Hand){
			if(*Handler.Cur_z > Pulse_z_max) {
				Emergency = 1;
				Stop_z();
			}
			Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_z.t0 +100 , 4000);
			if(Control_motor->bits.Up == 0)
			{
				Stop_z();
			}
		}
	}

	if(Home->bits.Home == 1){
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim5);
		HAL_TIM_Base_Stop_IT(&htim9);
		Move_x(Left, 100);
		Move_y(In, 100);
		Move_z(Up, 100);
	}


//	if(Emergency != 0) return;
//
////	if(Tab->bits.Engineer == 1){
//
//		if(Motor_x.state == Stop && Motor_y.state == Stop && Motor_z.state == Stop && Control_motor->bits.Set == 1){
//			Handler.Taget_x = *Handler.Cur_x;
//			int32_t delta_x = Handler.Taget_x - Handler.Old_Position_x;
//			Move_x_2_target(abs(delta_x), delta_x < 0? Left:Right);
//
//			Handler.Taget_y = *Handler.Cur_y;
//			int32_t delta_y = Handler.Taget_y - Handler.Old_Position_y;
//			Move_y_2_target(abs(delta_y), delta_y < 0? In:Out);
//
//			Handler.Taget_z = *Handler.Cur_z;
//			int32_t delta_z = Handler.Taget_z - Handler.Old_Position_z;
//			Move_z_2_target(abs(delta_z), delta_z < 0? Up:Down);
//
//			Control_motor->bits.Set = 0;
//		}
//
//		// control and scan position for x handler
//		if(Motor_x.state == Stop){
//			if(Control_motor->bits.Left == 1)
//				Move_x(Left,100);
//			if(Control_motor->bits.Right == 1)
//				Move_x(Right,100);
//			Motor_x.t0 = Tick/1000;
//		}
//		else{
//			*Handler.Cur_x = Motor_x.dir == Left? Handler.Old_Position_x - htim2.Instance -> CNT: Handler.Old_Position_x + htim2.Instance -> CNT;
//			if(*Handler.Cur_x > Pulse_x_max) {
//				Emergency = 1;
//				Stop_x();
//			}
//			if(Motor_x.state == Run_by_Hand){
//				Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_x.t0 +100 , 4000);
//				if(Control_motor->bits.Left == 0 && Control_motor->bits.Right == 0)
//				{
//					Stop_x();
//				}
//			}
//		}
//
//		// control and scan position for y handler
//		if(Motor_y.state == Stop){
//			if(Control_motor->bits.In == 1)
//				Move_y(In,100);
//			if(Control_motor->bits.Out == 1)
//				Move_y(Out,100);
//			Motor_y.t0 = Tick/1000;
//		}
//		else
//		{
//			*Handler.Cur_y = Motor_y.dir == In? Handler.Old_Position_y - htim5.Instance -> CNT: Handler.Old_Position_y + htim5.Instance -> CNT;
//			if(*Handler.Cur_y > Pulse_y_max) {
//				Emergency = 1;
//				Stop_y();
//			}
//			if(Motor_y.state == Run_by_Hand){
//				Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_y.t0 +100 , 4000);
//				if(Control_motor->bits.In == 0 && Control_motor->bits.Out == 0)
//					Stop_y();
//			}
//
//		}
//
//		// control and scan position for z handler
//		if(Motor_z.state == Stop){
//				if(Control_motor->bits.Up == 1)
//					Move_z(Up,100);
//		//		if(Control_motor->bits.Down == 1)
//		//			Move_x(Down);
//		}
//		else
//		{
//			*Handler.Cur_z = Motor_z.dir == Up? Handler.Old_Position_z - htim9.Instance -> CNT : Handler.Old_Position_z + htim9.Instance -> CNT;
//			if(*Handler.Cur_y > Pulse_y_max) {
//				Emergency = 1;
//				Stop_z();
//			}
//			if(Motor_z.state == Run_by_Hand){
//				Set_Frequency_Motor((uint16_t)(Tick/1000)-Motor_y.t0 +100 , 4000);
//				if(Control_motor->bits.Up == 0) Stop_z();
//			}
//		}
//		if(Home->bits.Home == 1){
//			HAL_TIM_Base_Stop_IT(&htim2);
//			HAL_TIM_Base_Stop_IT(&htim5);
//			HAL_TIM_Base_Stop_IT(&htim9);
//			Move_x(Left, 100);
//			Move_y(In, 100);
//			Move_z(Up, 100);
//		}
////	}
////	else if(Tab->bits.Home == 1){
////
////	}

}

void Handle_Set(void)
{
	if(Motor_x.state == Stop && Motor_y.state == Stop && Motor_z.state == Stop){
		Handler.Taget_x = *Handler.Cur_x;
		int32_t delta_x = Handler.Taget_x - Handler.Old_Position_x;
		Move_x_2_target(abs(delta_x), delta_x < 0? Left:Right);

		Handler.Taget_y = *Handler.Cur_y;
		int32_t delta_y = Handler.Taget_y - Handler.Old_Position_y;
		Move_y_2_target(abs(delta_y), delta_y < 0? In:Out);

		Handler.Taget_z = *Handler.Cur_z;
		int32_t delta_z = Handler.Taget_z - Handler.Old_Position_z;
		Move_z_2_target(abs(delta_z), delta_z < 0? Up:Down);

		Control_motor->bits.Set = 0;
	}
}
void Handle_Left(void)
{
	if(Motor_x.state == Stop){
		Move_x(Left,100);
		Motor_x.t0 = Tick/1000;
	}
}
void Handle_Right(void)
{
	if(Motor_x.state == Stop){
		Move_x(Right,100);
		Motor_x.t0 = Tick/1000;
	}
}
void Handle_In(void)
{
	if(Motor_y.state == Stop){
		Move_y(In,100);
		Motor_y.t0 = Tick/1000;
	}
}
void Handle_Out(void)
{
	if(Motor_y.state == Stop){
		Move_y(Out,100);
		Motor_y.t0 = Tick/1000;
	}

}
void Handle_Up(void)
{
	if(Motor_z.state == Stop){
		Move_z(Up,100);
		Motor_z.t0 = Tick/1000;
	}

}












void Tim_Base_Init(){
	MX_TIM7_Init();
	HAL_TIM_Base_Start_IT(&htim7);
}
void Set_Time(uint16_t time){
	htim7.Instance->ARR = time -1;
}
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  scan_HMI();
  Refresh_WDT();
  /* USER CODE END TIM7_IRQn 1 */
}
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}
