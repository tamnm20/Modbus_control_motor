/*
 * GPIO.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"
#ifndef INC_MY_LIB_GPIO_H_
#define INC_MY_LIB_GPIO_H_

#define O13_Pin GPIO_PIN_2
#define O13_GPIO_Port GPIOE
#define O14_Pin GPIO_PIN_3
#define O14_GPIO_Port GPIOE
#define O15_Pin GPIO_PIN_4
#define O15_GPIO_Port GPIOE
#define O16_Pin GPIO_PIN_5
#define O16_GPIO_Port GPIOE
#define O17_Pin GPIO_PIN_6
#define O17_GPIO_Port GPIOE
#define O18_Pin GPIO_PIN_13
#define O18_GPIO_Port GPIOC
#define I1_Pin GPIO_PIN_0
#define I1_GPIO_Port GPIOC
#define I2_Pin GPIO_PIN_1
#define I2_GPIO_Port GPIOC
#define I3_Pin GPIO_PIN_2
#define I3_GPIO_Port GPIOC
#define I4_Pin GPIO_PIN_3
#define I4_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define I5_Pin GPIO_PIN_4
#define I5_GPIO_Port GPIOC
#define I6_Pin GPIO_PIN_5
#define I6_GPIO_Port GPIOC
#define I7_Pin GPIO_PIN_0
#define I7_GPIO_Port GPIOB
#define I8_Pin GPIO_PIN_1
#define I8_GPIO_Port GPIOB
#define I9_Pin GPIO_PIN_7
#define I9_GPIO_Port GPIOE
#define I10_Pin GPIO_PIN_8
#define I10_GPIO_Port GPIOE
#define I11_Pin GPIO_PIN_9
#define I11_GPIO_Port GPIOE
#define I18_Pin GPIO_PIN_12
#define I18_GPIO_Port GPIOB
#define DIR2_CH1_Pin GPIO_PIN_8
#define DIR2_CH1_GPIO_Port GPIOD
#define DIR2_CH2_Pin GPIO_PIN_9
#define DIR2_CH2_GPIO_Port GPIOD
#define DIR2_CH3_Pin GPIO_PIN_10
#define DIR2_CH3_GPIO_Port GPIOD
#define DIR2_CH4_Pin GPIO_PIN_11
#define DIR2_CH4_GPIO_Port GPIOD
#define DIR1_CH1_Pin GPIO_PIN_7
#define DIR1_CH1_GPIO_Port GPIOC
#define DIR1_CH2_Pin GPIO_PIN_9
#define DIR1_CH2_GPIO_Port GPIOC
#define DIR1_CH3_Pin GPIO_PIN_9
#define DIR1_CH3_GPIO_Port GPIOA
#define DIR1_CH4_Pin GPIO_PIN_15
#define DIR1_CH4_GPIO_Port GPIOA
#define O1_Pin GPIO_PIN_3
#define O1_GPIO_Port GPIOD
#define O2_Pin GPIO_PIN_4
#define O2_GPIO_Port GPIOD
#define O3_Pin GPIO_PIN_5
#define O3_GPIO_Port GPIOD
#define O4_Pin GPIO_PIN_6
#define O4_GPIO_Port GPIOD
#define O5_Pin GPIO_PIN_7
#define O5_GPIO_Port GPIOD
#define O6_Pin GPIO_PIN_3
#define O6_GPIO_Port GPIOB
#define O7_Pin GPIO_PIN_4
#define O7_GPIO_Port GPIOB
#define O8_Pin GPIO_PIN_5
#define O8_GPIO_Port GPIOB
#define O9_Pin GPIO_PIN_8
#define O9_GPIO_Port GPIOB
#define O10_Pin GPIO_PIN_9
#define O10_GPIO_Port GPIOB
#define O11_Pin GPIO_PIN_0
#define O11_GPIO_Port GPIOE
#define O12_Pin GPIO_PIN_1
#define O12_GPIO_Port GPIOE

void MX_GPIO_Init(void);

#endif /* INC_MY_LIB_GPIO_H_ */
