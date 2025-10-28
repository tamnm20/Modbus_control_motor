/*
 * WDT.c
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "My_Lib/WDT.h"
IWDG_HandleTypeDef hiwdg;

void Refresh_WDT(){
	hiwdg.Instance->KR |= 0x0000AAAAu; //reset IWDT
}
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 79;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
