/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "lcd5110.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SystemCoreClockInMHz (SystemCoreClock/1000000)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// TIM6

volatile uint32_t tim6_overflows = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if( htim->Instance == TIM6 ) {
		++tim6_overflows;
	}
}

inline void TIM6_reinit() {
	HAL_TIM_Base_Stop(&htim6);
	__HAL_TIM_SET_PRESCALER( &htim6, (SystemCoreClockInMHz-1) );
	__HAL_TIM_SET_COUNTER( &htim6, 0 );
	tim6_overflows = 0;
	HAL_TIM_Base_Start_IT(&htim6);
}

inline uint32_t get_tim6_us() {
	__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
	//__disable_irq();
	uint32_t res = tim6_overflows * 10000 + __HAL_TIM_GET_COUNTER(&htim6);
	//__enable_irq();
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
	return res;
}

inline void udelay_TIM6(uint32_t useconds) {
	uint32_t before = get_tim6_us();
	while( get_tim6_us() < before+useconds){}
}

//

//extern void initialise_monitor_handles(void);
void BSP_ACCELERO_GetXYZ(int16_t *pDataXYZ);
void BSP_GYRO_GetXYZ(float* pfData);
uint8_t BSP_ACCELERO_Init();
uint8_t BSP_GYRO_Init();
LCD5110_display lcd1;

// Calculations

void cleanAccBuffer(int16_t* buffer, double* clean) {
	clean[0] = ((double)(buffer[0]) / 16 + 15) / 1000;
	clean[1] = ((double)(buffer[1]) / 1633 + 2.06) / 1000;
	clean[2] = ((double)(buffer[2]) / 1633) / 1000;
}

void cleanGyroBuffer(float* buffer, double* clean) {
	clean[0] = ((double)(buffer[0]) + 439.88) / 1000;
	clean[1] = ((double)(buffer[1]) - 91.8575) / 1000;
	clean[2] = ((double)(buffer[2]) - 808.3075) / 1000;
}

void multiplyBySeconds(double* vector, int time) {
	double seconds = (double)time / 1000000;
	vector[0] *= seconds;
	vector[1] *= seconds;
	vector[2] *= seconds;
}

void addVectors(double* v1, double* v2) {
	v1[0] += v2[0];
	v1[1] += v2[1];
	v1[2] += v2[2];
}

void resetAll(double* currentPosition, double* currentSpeed, double* currentAngle) {
	currentPosition[0] = 0;
	currentPosition[1] = 0;
	currentPosition[2] = 0;
	currentSpeed[0] = 0;
	currentSpeed[1] = 0;
	currentSpeed[2] = 0;
	currentAngle[0] = 0;
	currentAngle[1] = 0;
	currentAngle[2] = 0;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  // Inits
  TIM6_reinit();
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  lcd1.hw_conf.spi_handle = &hspi2;
  lcd1.hw_conf.spi_cs_pin =  LCD1_CS_Pin;
  lcd1.hw_conf.spi_cs_port = LCD1_CS_GPIO_Port;
  lcd1.hw_conf.rst_pin =  LCD1_RST_Pin;
  lcd1.hw_conf.rst_port = LCD1_RST_GPIO_Port;
  lcd1.hw_conf.dc_pin =  LCD1_DC_Pin;
  lcd1.hw_conf.dc_port = LCD1_DC_GPIO_Port;
  lcd1.def_scr = lcd5110_def_scr;
  LCD5110_init(&lcd1.hw_conf, LCD5110_NORMAL_MODE, 0x40, 2, 3);

  //

  float gyroBuffer[3] = {0};
  double gyroClean[3] = {0};
  int16_t accBuffer[3] = {0};
  double accClean[3] = {0};

  double currentPosition[3] = {0, 0, 0};
  double currentSpeed[3] = {0, 0, 0};
  double currentAngle[3] = {0, 0, 0};
  double gravity[3] = {0, 0, -9.81};

  int lastTime = get_tim6_us();

  int iter = 0;
  double maxSpeed = 0;

  /* USER CODE END 2 */


  // clk   din   dc   ce   rst
  // pb13  pb15  pb12 pb14 pb11

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BSP_GYRO_GetXYZ(gyroBuffer);
	  BSP_ACCELERO_GetXYZ(accBuffer);

	  int deltaTime = get_tim6_us() - lastTime;
	  lastTime = get_tim6_us();

	  cleanAccBuffer(accBuffer, accClean);
	  cleanGyroBuffer(gyroBuffer, gyroClean);

	  addVectors(accClean, gravity);

	  multiplyBySeconds(accClean, deltaTime);
	  multiplyBySeconds(gyroClean, deltaTime);

	  addVectors(currentSpeed, accClean);

	  multiplyBySeconds(currentSpeed, deltaTime);
	  addVectors(currentPosition, currentSpeed);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
