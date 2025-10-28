/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "My_Lib/delay.h"
#include "My_Lib/tim_base.h"
#include "My_Lib/RS232.h"
#include "My_Lib/modbusSlave.h"
#include "My_Lib/motor.h"
#include "My_Lib/WDT.h"
#include "My_Lib/DMA.h"
#include "My_Lib/GPIO.h"

//#include "My_Lib/CAN.h"
//#include "My_Lib/I2C.h"
//#include "My_Lib/SPI.h"
//#include "My_Lib/RTC.h"
//#include "usb_device.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define RX_BUF_SIZE 256




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

volatile uint16_t anpha = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);



/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




void check_reset_cause(void) {
    uint32_t flags = RCC->CSR;

    if (flags & RCC_CSR_WWDGRSTF) {

    } else if (flags & RCC_CSR_IWDGRSTF) {

    } else if (flags & RCC_CSR_SFTRSTF) {

    } else if (flags & RCC_CSR_PORRSTF) {

    } else if (flags & RCC_CSR_PINRSTF) {

    }else if(flags & RCC_CSR_LPWRRSTF){

    }

    // Xóa cờ để lần sau đọc lại chính xác
    RCC->CSR |= RCC_CSR_RMVF;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  Motor_x_Init();
  Motor_y_Init();
  Motor_z_Init();
  Delay_Init();
  Tim_Base_Init();
  HMI_Init();



  Handler.Cur_x = &Holding_Registers_Database[0];
  Handler.Cur_y = &Holding_Registers_Database[1];
  Handler.Cur_z = &Holding_Registers_Database[2];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  int32_t delta_x = 1000;
//	  Move_x_2_target(abs(delta_x), delta_x < 0? Left:Right);
//	  Move_x_2_target(50,Left);
//	  Move_y_2_target(10,In);
//	  Move_z_2_target(15,Up);

	  delay_ms(200);

    /* USER CODE END WHILE */

  }
}


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
