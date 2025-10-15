/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t Coils_Database[25];
extern uint16_t Holding_Registers_Database[50];
extern float cur_x_mm;
extern float cur_y_mm;
uint8_t RxData[256];
uint8_t TxData[256];

//extern AxisSystem_t Axis;

typedef struct {
    uint8_t  frame_ready;     // Có khung dữ liệu mới
    uint8_t  busy;            // �?ang xử lý 1 khung
    uint8_t  error;           // Có lỗi (CRC hoặc sai địa chỉ)
    uint16_t rx_size;         // Kích thước khung nhận được
} ModbusStatus_t;
volatile ModbusStatus_t modbus_flags = {0};
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        // Nếu nhận được đúng Slave ID → báo có khung hợp lệ
        if (RxData[0] == SLAVE_ID)
        {
            modbus_flags.frame_ready = 1;
            modbus_flags.rx_size = Size;
        }
        else
        {
            modbus_flags.error = 1;
        }
    }

    // enable DMA receive
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if (RxData[0] == SLAVE_ID)
//	{
//		switch (RxData[1]){
//		case 0x03:
//			readHoldingRegs();
//			break;
//		case 0x04:
//			readInputRegs();
//			break;
//		case 0x01:
//			readCoils();
//			break;
//		case 0x02:
//			readInputs();
//			break;
//		case 0x05:
//			writeCoil();
//			break;
//		case 0x0F:
//			writeCoils();
//			break;
//		case 0x06:
//			writeSingleReg();
//			break;
//		case 0x10:
//			writeHoldingRegs();
//			break;
//		default:
//			modbusException(ILLEGAL_FUNCTION);
//			break;
//		}
//	}
//
//	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
//}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  io_init();
  tim2_init();
  tim3_init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
  Global_Timer_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Axis_Init(Holding_Registers_Database);
  HAL_Delay(1000);
  Home_All();
  U8 On_Time = FALSE;
  Delay_Time_Set(TID_MODBUS,DT_MODBUS);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (modbus_flags.frame_ready && !modbus_flags.busy)
	  {
		  modbus_flags.busy = 1;
		  modbus_flags.frame_ready = 0;

		  uint8_t func = RxData[1];

		  switch (func)
		  {
			  case 0x03: readHoldingRegs(); break;
			  case 0x04: readInputRegs(); break;
			  case 0x01: readCoils(); break;
			  case 0x02: readInputs(); break;
			  case 0x05: writeCoil(); break;
			  case 0x0F: writeCoils(); break;
			  case 0x06: writeSingleReg(); break;
			  case 0x10: writeHoldingRegs(); break;
			  default:   modbusException(ILLEGAL_FUNCTION); break;
		  }

		  modbus_flags.busy = 0; // done
	  }

	  // ===== error? =====
	  if (modbus_flags.error)
	  {
		  //log, reset UART, v.v.
		  modbus_flags.error = 0;
	  }

	On_Time = Delay_Time_Get(TID_MODBUS);
	if (On_Time == TRUE)
	{
		uint8_t coils = Coils_Database[0];
		uint8_t coil2 = Coils_Database[1];

//		// ---- X- (chỉ chạy khi pos > 0) ----
//		if ((coils >> 3) & 0x01) {
//		    if (Axis_IsHomed_X() && !Axis_IsBusy_X() && Axis.X.position > EPS_MM) {
//		        Set_Dir_X(LEFT);
//		        HAL_Delay(5);
//		        Motor_SetFeed_X(200.0f);
//		        Motor_Start_X(5000);
//		        Axis_UpdateState(AXIS_X, LEFT, 200.0f, 5000);
//
//		        Axis.X.position -= (5000.0f / STEPS_PER_MM);
//		        if (Axis.X.position < 0) Axis.X.position = 0;  // clamp to 0
//		        Holding_Registers_Database[0] = (uint16_t)Axis.X.position;
//		    }
//		}
//
//		// ---- X+ (giới hạn 500 mm) ----
//		else if ((coils >> 4) & 0x01) {
//		    if (Axis_IsHomed_X() && !Axis_IsBusy_X() && Axis.X.position < (X_MAX_MM - EPS_MM)) {
//		        Set_Dir_X(RIGHT);
//		        HAL_Delay(5);
//		        Motor_SetFeed_X(200.0f);
//		        Motor_Start_X(5000);
//		        Axis_UpdateState(AXIS_X, RIGHT, 200.0f, 5000);
//
//		        Axis.X.position += (5000.0f / STEPS_PER_MM);
//		        if (Axis.X.position > X_MAX_MM) Axis.X.position = X_MAX_MM;  // clamp to max
//		        Holding_Registers_Database[0] = (uint16_t)Axis.X.position;
//		    }
//		}
//
//		// ---- Y- (chỉ chạy khi pos > 0) ----
//		else if ((coils >> 5) & 0x01) {
//		    if (Axis_IsHomed_Y() && !Axis_IsBusy_Y() && Axis.Y.position > EPS_MM) {
//		        Set_Dir_Y(BACKWARD);
//		        HAL_Delay(5);
//		        Motor_SetFeed_Y(200.0f);
//		        Motor_Start_Y(5000);
//		        Axis_UpdateState(AXIS_Y, BACKWARD, 200.0f, 5000);
//
//		        Axis.Y.position -= (5000.0f / STEPS_PER_MM);
//		        if (Axis.Y.position < 0) Axis.Y.position = 0;
//		        Holding_Registers_Database[1] = (uint16_t)Axis.Y.position;
//		    }
//		}
//
//		// ---- Y+ (giới hạn 400 mm) ----
//		else if ((coils >> 6) & 0x01) {
//		    if (Axis_IsHomed_Y() && !Axis_IsBusy_Y() && Axis.Y.position < (Y_MAX_MM - EPS_MM)) {
//		        Set_Dir_Y(FORWARD);
//		        HAL_Delay(5);
//		        Motor_SetFeed_Y(200.0f);
//		        Motor_Start_Y(5000);
//		        Axis_UpdateState(AXIS_Y, FORWARD, 200.0f, 5000);
//
//		        Axis.Y.position += (5000.0f / STEPS_PER_MM);
//		        if (Axis.Y.position > Y_MAX_MM) Axis.Y.position = Y_MAX_MM;
//		        Holding_Registers_Database[1] = (uint16_t)Axis.Y.position;
//		    }
//		}
		if ((coils >> 3) & 0x01)      Axis_MoveStep(AXIS_X, LEFT,     2500, 20.0f);
		else if ((coils >> 4) & 0x01) Axis_MoveStep(AXIS_X, RIGHT,    2500, 20.0f);
		else if ((coils >> 5) & 0x01) Axis_MoveStep(AXIS_Y, BACKWARD, 2500, 20.0f);
		else if ((coils >> 6) & 0x01) Axis_MoveStep(AXIS_Y, FORWARD,  2500, 20.0f);
		if((coil2 >> 7) & 0x01){
			Holding_Registers_Database[0]=0;
			Holding_Registers_Database[1]=0;
			Holding_Registers_Database[2]=0;
			//HAL_Delay(1000);
			Home_All();
			Coils_Database[1]= Coils_Database[1] & ~(1u<<7);
		}
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#ifdef Tuan_board
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
#endif
#ifdef Chuc_board
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
#endif
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
