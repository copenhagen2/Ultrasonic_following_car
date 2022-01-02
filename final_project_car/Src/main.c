/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
		float rec[10];
		float max_v;
		float min_v;
		float vpp;
		uint8_t count;
} VOL;

typedef struct {
		float kp, ki, kd;
		float pmax, imax, dmax;
		float sum;
		float this_error;
		float last_error;
		float output;
} PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VCC 3.3
#define max(a,b) ((a>b)? a:b)
#define min(a,b) ((a>b)? b:a)
#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float right = 0;
float left = 0;
float delta = 0;
float speed1 = 0;
float speed2 = 0;
uint8_t state = 0;
uint8_t cmd = 0;
VOL sonic_left = {
		.rec = {0},
		.max_v = 0,
		.min_v = VCC,
		.count = 0,
		.vpp = 0,
};

VOL sonic_right = {
		.rec = {0},
		.max_v = 0,
		.min_v = VCC,
		.count = 0,
		.vpp = 0,
};

PID pid = {
		.kp = 900,
		.ki = 0,
		.kd = 100,
		.pmax = 700,
		.imax = 0,
		.dmax = 600,
		.sum = 0,
		.this_error = 0,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SetChassisSpeed(int16_t left, int16_t right);
void ReadSonicVoltage(VOL *feedback, float value);
float Calculate(PID *pid, float error); 
void MainControlLoop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)&("Power On\n"), sizeof("Power On\n"));
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_TIM_Base_Start_IT(&htim1);
	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&cmd, sizeof(cmd));
		//MainControlLoop();
		//SetChassisSpeed(speed1, speed2);
		if (cmd == 'n' || cmd == 'N') {
			SetChassisSpeed(0,0);
		}
		else if (cmd == 'y' || cmd == 'Y') {
			MainControlLoop();
		}
		//MainControlLoop();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
*** The IT callback functions
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		if (huart->Instance == huart1.Instance) {
				HAL_UART_Transmit_IT(&huart1, (uint8_t *)&("OK"), sizeof("OK"));
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim->Instance == htim1.Instance) { //400kHz used to measure the voltage of sonic sensor
				ReadSonicVoltage(&sonic_left, left);
				ReadSonicVoltage(&sonic_right, right);
			//MainControlLoop();
		}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
		if (hadc->Instance == hadc1.Instance) {
				left = (float)HAL_ADC_GetValue(&hadc1) / 4096.0f * VCC; //12-digits adc
				HAL_ADC_Start_IT(&hadc1);
		}
		if (hadc->Instance == hadc2.Instance) {
				right = (float)HAL_ADC_GetValue(&hadc2) / 4096.0f * VCC; //12-digits adc
				HAL_ADC_Start_IT(&hadc2);
		}
}
/** 
*** Functions for chassis-controlling
*/
void SetChassisSpeed(int16_t left, int16_t right) {
		// left motor
		if (left > 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, left);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		}
		else if (left == 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		}
		else if (left < 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, -left);
		}
		
		// right motor
		if (right > 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, right);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
		}
		else if (right == 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
		}
		else if (right < 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, -right);
		}
}
/**
*** Read sonic value
*/
void ReadSonicVoltage(VOL *feedback, float value) {
		feedback->rec[feedback->count] = value;
		feedback->count++;
		if (feedback->count == 10) {
				for (int i = 0; i < 10; i++) {
						feedback->max_v = max(feedback->max_v, feedback->rec[i]);
						feedback->min_v = min(feedback->min_v, feedback->rec[i]);
				}
				feedback->vpp = feedback->max_v - feedback->min_v;
				feedback->count = 0;
				feedback->max_v = 0;
				feedback->min_v = VCC;
		}
}
/**
*** pid regulater
*/
float Calculate(PID *pid, float error) {
		pid->last_error = pid->this_error;
		pid->this_error = error;
		float pcom = pid->kp * pid->this_error;
		float dcom = pid->kd * (pid->this_error - pid->last_error);
		pid->sum += pid->this_error;
		INRANGE(pcom, -pid->pmax, pid->pmax)
		INRANGE(dcom, -pid->dmax, pid->dmax)
		//NRANGE(pid->sum, -pid->imax, pid->imax)
		pid->output = pcom + dcom + pid->sum * pid->ki;
		return pid->output;
}
/**
*** main control loop
*/
void MainControlLoop() {
		//stop
	
		if (sonic_left.vpp > 1.2f || sonic_right.vpp > 1.20f) {
				SetChassisSpeed(0,0);
				state = 0;
				return;
		}
		//forward
		if (fabs(sonic_left.vpp - sonic_right.vpp) < 0.4) {
				SetChassisSpeed(550,550);
				state = 1;
				return;
		}
	
		//turn
		else {
			delta = sonic_left.vpp - sonic_right.vpp;
			float out = Calculate(&pid, delta);
			SetChassisSpeed(-out, out);
  


		}			
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
