/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../HCSR04/HCSR04.h"
#include "../SERVO/SERVO.h"
#include "../DCMotor/DCMotor.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04_SENSOR1  0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t LeftIrCounter;
volatile uint8_t RightIrCounter;
volatile uint8_t LeftIrWarnCoutner;
volatile uint8_t RightIrWarnCoutner;
uint16_t Buffer_ASCII_TO_INT;
uint8_t Buffer[4];
uint8_t Car_Current_Mode      = CAR_DEFAULT_MODE  ;
uint8_t Car_Current_Direction = CAR_STOP          ;
uint8_t Car_Current_Status    = CAR_RUNNING       ;
uint8_t Car_Current_Speed     = CAR_DEFAULT_SPEED ;
uint8_t GUI_TRANSMIT_INSTANT  = GUI_NO_TRANSMIT   ;
uint8_t Car_LaneAssist_Enable = MODE_DISABLE	  ;
uint8_t Car_BlindSpot_Enable  = MODE_DISABLE	  ;
uint8_t GUI_Send_Speed        = CAR_DEFAULT_SPEED ;

uint8_t Buffer_GUI[GUI_ARRAY_SIZE] = {"0000000000000"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
	MX_TIM5_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM11_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/* Initialize two Ultrasonic sensors */
	HCSR04_Init(HCSR04_SENSOR1, &htim2);
	HCSR04_Init(HCSR04_SENSOR2, &htim2);
	/* Initialize two Servo motors */
	SERVO_Init(SERVO_MOTOR1);
	SERVO_Init(SERVO_MOTOR2);
	/* Start PWM on Motor channels */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	/* Start PWM on Servo channel */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


	/* UART Receive */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)Buffer, 4);

	/**********************************************/
	//  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	//HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	/**********************************************/


	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* Ultrasonic Input Capture Unit ISR handler*/
	HCSR04_TMR_IC_ISR(htim);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Buffer_ASCII_TO_INT = atoi((char*)Buffer);
	/* Set Vehicle mode (Normal, Adaptive Cruise Control or Self Driving Mode) */
	if((Buffer_ASCII_TO_INT == NORMAL_MODE)||(Buffer_ASCII_TO_INT == ACC_MODE) || (Buffer_ASCII_TO_INT == SELF_DRIVING_MODE))
	{
		Car_Current_Mode = Buffer_ASCII_TO_INT;
		Car_Current_Status = CAR_RUNNING ;

	}
	/* Set Vehicle direction (Forward, Backward, Leftwards or Rightwards) */
	else if((Buffer_ASCII_TO_INT == MOVE_FORWARD)||(Buffer_ASCII_TO_INT == MOVE_BACKWORD)||(Buffer_ASCII_TO_INT == MOVE_RIGHT)||(Buffer_ASCII_TO_INT == MOVE_LEFT))
	{
		Car_Current_Direction = Buffer_ASCII_TO_INT;
	}
	/* Set Vehicle mode (stop mode) */
	else if (Buffer_ASCII_TO_INT == STOP_MOTOR)
	{
		Car_Current_Speed  = CAR_STOP ;
		GUI_Send_Speed     = CAR_STOP ;
		Car_Current_Status = CAR_STOP ;
	}
	/* Set Lane Assist mode */
	else if (Buffer_ASCII_TO_INT == LANEASSIST_ON || Buffer_ASCII_TO_INT == LANEASSIST_OFF)
	{
		Car_LaneAssist_Enable = (Buffer_ASCII_TO_INT == LANEASSIST_ON);
	}
	/* Set Blindspot mode */
	else if (Buffer_ASCII_TO_INT == BLINDSPOT_ON || Buffer_ASCII_TO_INT == BLINDSPOT_OFF)
	{
		Car_BlindSpot_Enable = (Buffer_ASCII_TO_INT == BLINDSPOT_ON);
	}
	/* Set Vehicle speed */
	else
	{
		if(Car_Current_Speed != (Buffer_ASCII_TO_INT - CAR_SPEED_OFFSET))
		{
			Car_Current_Speed = Buffer_ASCII_TO_INT - CAR_SPEED_OFFSET ;
			GUI_Send_Speed    = Car_Current_Speed ;
			DCMotor_changeSpeed(Car_Current_Speed);
		}
		Car_Current_Status = CAR_RUNNING ;

	}
	GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;
}
/* EXTI interrupt callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static Trigger TriggerState = FALLING;
	if (TriggerState == FALLING)
	{
		TriggerState = RISING;
		DCMotor_stop();
	}
	else
	{
		TriggerState = FALLING;
	}
	switch(GPIO_Pin)
	{
	case LEFT_IR_Pin:
		if(Car_Current_Mode == SELF_DRIVING_MODE)
		{
			LeftIrWarnCoutner++;
		}
		else
		{
			LeftIrCounter++;
		}

		break;
	case RIGHT_IR_Pin:
		if(Car_Current_Mode == SELF_DRIVING_MODE)
		{
			RightIrWarnCoutner++;
		}
		else
		{
			RightIrCounter++;
		}

		break;
	}
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM10) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	HCSR04_TMR_OVF_ISR(htim);
	/* USER CODE END Callback 1 */
}

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
