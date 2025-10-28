/*
 * tim_base.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"
#ifndef INC_TIM_BASE_H_
#define INC_TIM_BASE_H_
void scan_HMI(void);
void Tim_Base_Init(void);
void Set_Time(uint16_t time);
void MX_TIM7_Init(void);

void Handle_Set(void);
void Handle_Left(void);
void Handle_Right(void);
void Handle_In(void);
void Handle_Out(void);
void Handle_Up(void);



#endif /* INC_TIM_BASE_H_ */
