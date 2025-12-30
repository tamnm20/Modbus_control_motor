/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "flash.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* HSE_VALUE */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I1_Color_Pin GPIO_PIN_0
#define I1_Color_GPIO_Port GPIOC
#define I2_Vaccum1_Pin GPIO_PIN_1
#define I2_Vaccum1_GPIO_Port GPIOC
#define I3_Vaccum2_Pin GPIO_PIN_2
#define I3_Vaccum2_GPIO_Port GPIOC
#define I4_ALM_X_Pin GPIO_PIN_3
#define I4_ALM_X_GPIO_Port GPIOC
#define I5_ALM_Y_Pin GPIO_PIN_4
#define I5_ALM_Y_GPIO_Port GPIOC
#define I6_Pressure_Pin GPIO_PIN_5
#define I6_Pressure_GPIO_Port GPIOC
#define I7_L_Door_Pin GPIO_PIN_0
#define I7_L_Door_GPIO_Port GPIOB
#define I8_R_Door_Pin GPIO_PIN_1
#define I8_R_Door_GPIO_Port GPIOB
#define I10_Start_Pin GPIO_PIN_8
#define I10_Start_GPIO_Port GPIOE
#define I11_Stop_Pin GPIO_PIN_9
#define I11_Stop_GPIO_Port GPIOE
#define I12_Rsset_Pin GPIO_PIN_10
#define I12_Rsset_GPIO_Port GPIOE
#define HOME_X_Pin GPIO_PIN_13
#define HOME_X_GPIO_Port GPIOE
#define HOME_X_EXTI_IRQn EXTI15_10_IRQn
#define HOME_Y_Pin GPIO_PIN_14
#define HOME_Y_GPIO_Port GPIOE
#define HOME_Y_EXTI_IRQn EXTI15_10_IRQn
#define HOME_Z_Pin GPIO_PIN_15
#define HOME_Z_GPIO_Port GPIOE
#define HOME_Z_EXTI_IRQn EXTI15_10_IRQn
#define I18_Emergency_Pin GPIO_PIN_12
#define I18_Emergency_GPIO_Port GPIOB
#define I18_Emergency_EXTI_IRQn EXTI15_10_IRQn
#define PWM_EN_Pin GPIO_PIN_15
#define PWM_EN_GPIO_Port GPIOB
#define Pulse_X_Pin GPIO_PIN_6
#define Pulse_X_GPIO_Port GPIOC
#define Dir_X_Pin GPIO_PIN_7
#define Dir_X_GPIO_Port GPIOC
#define Pulse_Z_Pin GPIO_PIN_8
#define Pulse_Z_GPIO_Port GPIOC
#define Dir_Z_Pin GPIO_PIN_9
#define Dir_Z_GPIO_Port GPIOC
#define Pulse_Y_Pin GPIO_PIN_8
#define Pulse_Y_GPIO_Port GPIOA
#define Dir_Y_Pin GPIO_PIN_9
#define Dir_Y_GPIO_Port GPIOA
#define O1_Xilanh1_Pin GPIO_PIN_3
#define O1_Xilanh1_GPIO_Port GPIOD
#define O2_Xilanh2_Pin GPIO_PIN_4
#define O2_Xilanh2_GPIO_Port GPIOD
#define O10_LED_G_Pin GPIO_PIN_9
#define O10_LED_G_GPIO_Port GPIOB
#define O11_LED_R_Pin GPIO_PIN_0
#define O11_LED_R_GPIO_Port GPIOE
#define O12_BUZZER_Pin GPIO_PIN_1
#define O12_BUZZER_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
