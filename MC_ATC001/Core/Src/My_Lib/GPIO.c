/*
 * GPIO.c
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
#include "My_Lib/GPIO.h"
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, O13_Pin|O14_Pin|O15_Pin|O16_Pin
                          |O17_Pin|O11_Pin|O12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, O18_Pin|DIR1_CH1_Pin|DIR1_CH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|DIR1_CH3_Pin|DIR1_CH4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|O6_Pin|O7_Pin|O8_Pin
                          |O9_Pin|O10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR2_CH1_Pin|DIR2_CH2_Pin|DIR2_CH3_Pin|DIR2_CH4_Pin
                          |O1_Pin|O2_Pin|O3_Pin|O4_Pin
                          |O5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : O13_Pin O14_Pin O15_Pin O16_Pin
                           O17_Pin O11_Pin O12_Pin */
  GPIO_InitStruct.Pin = O13_Pin|O14_Pin|O15_Pin|O16_Pin
                          |O17_Pin|O11_Pin|O12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : O18_Pin DIR1_CH1_Pin DIR1_CH2_Pin */
  GPIO_InitStruct.Pin = O18_Pin|DIR1_CH1_Pin|DIR1_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I1_Pin I2_Pin I3_Pin I4_Pin
                           I5_Pin I6_Pin */
  GPIO_InitStruct.Pin = I1_Pin|I2_Pin|I3_Pin|I4_Pin
                          |I5_Pin|I6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin DIR1_CH3_Pin DIR1_CH4_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DIR1_CH3_Pin|DIR1_CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I7_Pin I8_Pin I18_Pin */
  GPIO_InitStruct.Pin = I7_Pin|I8_Pin|I18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I9_Pin I10_Pin I11_Pin */
  GPIO_InitStruct.Pin = I9_Pin|I10_Pin|I11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 O6_Pin O7_Pin O8_Pin
                           O9_Pin O10_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|O6_Pin|O7_Pin|O8_Pin
                          |O9_Pin|O10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2_CH1_Pin DIR2_CH2_Pin DIR2_CH3_Pin DIR2_CH4_Pin
                           O1_Pin O2_Pin O3_Pin O4_Pin
                           O5_Pin */
  GPIO_InitStruct.Pin = DIR2_CH1_Pin|DIR2_CH2_Pin|DIR2_CH3_Pin|DIR2_CH4_Pin
                          |O1_Pin|O2_Pin|O3_Pin|O4_Pin
                          |O5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

