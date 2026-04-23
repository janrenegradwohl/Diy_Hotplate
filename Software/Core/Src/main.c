/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <pid.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float ntc_get_temperature(uint32_t adc_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ADC1_RES     4095.0    // Auflösung (12 Bit: 2^12 - 1)
#define R_Seri_T       6800.0	//
#define A_COEFF_T     0.0924917  // a in Ohm
#define B_COEFF_T     4135.86     // b in Kelvin 4135.86
#define BUFFER_SIZE 64

uint32_t PWM_Duty_T2C3 = 1;

int pispeed[5] = { 1, 10, 20, 100, 500 };
volatile uint32_t ADC1_Reading[2] = { 0, 0 };
volatile uint32_t temp_data = 1;
volatile uint32_t m_tim1_set_value = 0;
const float t_kp = 0.08;
const float t_ki = 0.002;
const float t_kd = 0.0001;
volatile float t_imem;
volatile float t_previous_measurement = 0;
volatile uint32_t t_value;

//uint8_t HeatMode = 0b00000000;
float TempSetPoint = 0;

char UART_pData_TX[BUFFER_SIZE];
char UART_pData_RX[BUFFER_SIZE];
char DMA_BUFFER[BUFFER_SIZE];

volatile uint16_t globalTime = 0;
volatile uint16_t globalLastTime = 0;
volatile uint16_t globalElapsedTime = 0;
volatile uint16_t lastTime = 0;

//uint32_t tempAddLastTime = 0;
//uint32_t tempAddTimer = 10; 		//temperature add timer
//uint16_t tempSetValue = 20;			//20°C
//uint16_t tempMaxValue = 150;	//defines max temperature value, init with 150°C

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


	PID_f temp_pid;
	pid_begin(&temp_pid, t_kp, t_ki, t_kd, 255, 0, t_previous_measurement, t_imem);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim6); // runs on 1000Hz update rate
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_Reading, 2);

	//temp_pi.setpoint = pispeed[2];
	temp_pid.setpoint = 100;
	TIM2->CCR3 = PWM_Duty_T2C3;

	while (1) {
		globalLastTime = __HAL_TIM_GET_COUNTER(&htim6);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

		t_value = ADC1_Reading[0];
		temp_pid.measurement = ntc_get_temperature(t_value);
		sprintf(DMA_BUFFER, "Reg: %i, Temp [°C]; %i, ElapsedTime [us]; %i, PWM [DC]; %i\n\r",
				(int) t_value, (int) ntc_get_temperature(t_value),(int) globalElapsedTime, (int) temp_pid.output);

		pid_parallel_t(&temp_pid, (float) globalElapsedTime/1000.0f); //globalElapsedTime/1000.0f
		TIM2->CCR3 = (uint32_t) temp_pid.output;

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*) DMA_BUFFER,
				strlen(DMA_BUFFER));
		HAL_Delay(1);

		globalTime = __HAL_TIM_GET_COUNTER(&htim6);
		globalElapsedTime = globalTime - globalLastTime;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM15|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float ntc_get_temperature(uint32_t adc_value) {

        float V = (adc_value / ADC1_RES) * 3.3;
        float R_ntc = R_Seri_T * (V / (3.3 - V));

        float lnR = logf(R_ntc);
        float lnA = logf(A_COEFF_T);

        float T_K = B_COEFF_T / (lnR - lnA);  // Temperatur in Kelvin
        float T_C = T_K - 273.15;           // in °C umrechnen

        return T_C;
    }
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
