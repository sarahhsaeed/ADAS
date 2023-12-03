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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned short u8;

#define  IN1   4
#define  IN2   3
#define  EN1   0
#define  IN3   2
#define  IN4   1
#define  EN2   6


typedef enum
{
	M_CW,
	M_CCW,
	M_STOP
}Direction_status;
typedef enum
{
	M1,
	M2
}MOTOR_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile u8 counter1,counter2;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* Definitions for LKA_TASK */
osThreadId_t LKA_TASKHandle;
const osThreadAttr_t LKA_TASK_attributes = {
  .name = "LKA_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LDW_TASK */
osThreadId_t LDW_TASKHandle;
const osThreadAttr_t LDW_TASK_attributes = {
  .name = "LDW_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/* EXTI interrupt callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case IR_LEFT_Pin:
		counter1++;
		break;
	case IR_RIGHT_Pin:
		counter2++;
		break;
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void LaneKeepAssist(void *argument);
void LaneDetectionWarning(void *argument);
void MOTOR_Speed_dir(MOTOR_type motor,u8 speed , Direction_status status);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**********************************************************************************************/
/* Operate all car motors in CW direction */
static void MoveForward(u8 Copy_u8Speed)
{
	MOTOR_Speed_dir(M1, Copy_u8Speed , M_CW);
	MOTOR_Speed_dir(M2, Copy_u8Speed , M_CW);

}
/**********************************************************************************************/
/* Operate all car motors in CCW direction */
static void MoveBackword(u8 Copy_u8Speed)
{

	MOTOR_Speed_dir(M1, Copy_u8Speed , M_CCW);
	MOTOR_Speed_dir(M2, Copy_u8Speed , M_CCW);
}
/**********************************************************************************************/
/* Operate two right car motors in CW direction
 * Operate two left car motors in CCW direction */
static void MoveRight(u8 Copy_u8Speed)
{
	MOTOR_Speed_dir(M1, Copy_u8Speed , M_CW);
	MOTOR_Speed_dir(M2, Copy_u8Speed , M_CCW);
}
/**********************************************************************************************/
/* Operate two right car motors in CCW direction
 * Operate two left car motors in CW direction */
static void MoveLeft(u8 Copy_u8Speed)
{
	MOTOR_Speed_dir(M1, Copy_u8Speed , M_CCW);
	MOTOR_Speed_dir(M2, Copy_u8Speed , M_CW);

}
/**********************************************************************************************/
/* Stop all car motors */
void MOTOR_Speed_dir(MOTOR_type motor,u8 speed , Direction_status status)
{

	switch (motor)
	{
	case M1:
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, speed);
		switch (status)
		{
		case M_CW:
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

			break;
		case M_CCW:
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			break;
		case M_STOP:
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			break;
		}
		break;

		case M2:
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1, speed);
			switch (status)
			{
			case M_CW:
				HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
				HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);

				break;
			case M_CCW:
				HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
				HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
				break;
			case M_STOP:
				HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
				HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
				break;
			}
			break;
	}


}

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LKA_TASK */
  LKA_TASKHandle = osThreadNew(LaneKeepAssist, NULL, &LKA_TASK_attributes);

  /* creation of LDW_TASK */
  LDW_TASKHandle = osThreadNew(LaneDetectionWarning, NULL, &LDW_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 6399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN4_Pin|IN3_Pin|IN2_Pin|IN1_Pin
                          |RIGHT_IR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port, LEFT_IR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN4_Pin IN3_Pin IN2_Pin IN1_Pin
                           RIGHT_IR_LED_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin|IN2_Pin|IN1_Pin
                          |RIGHT_IR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_LEFT_Pin IR_RIGHT_Pin */
  GPIO_InitStruct.Pin = IR_LEFT_Pin|IR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_IR_LED_Pin */
  GPIO_InitStruct.Pin = LEFT_IR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEFT_IR_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_LaneKeepAssist */
/**
  * @brief  Function implementing the LKA_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LaneKeepAssist */
void LaneKeepAssist(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	/* Infinite loop */
	for(;;)
	{
		if(counter1==1 && counter2==0)//left
		{
			MoveRight(50);
			osDelay(200);
			MoveForward(60);


		}
		else if(counter2==1 && counter1==0)
		{
			MoveLeft(50);
			osDelay(200);
			MoveForward(60);

		}
		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LaneDetectionWarning */
	/**
	 * @brief Function implementing the LDW_TASK thread.
	 * @param argument: Not used
	 * @retval None
	 */
/* USER CODE END Header_LaneDetectionWarning */
void LaneDetectionWarning(void *argument)
{
  /* USER CODE BEGIN LaneDetectionWarning */
		/* Infinite loop */
		for(;;)
		{
			if(counter1>2)
			{
				counter1=0;
			}
			if(counter2>2)
			{
				counter2=0;
			}
			if(counter1 == 1 && counter2 == 0)
			{
				// Activate left lane warning
				HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,1);
				osDelay(500);
			}
			else if(counter1 == 2 )
			{
				// Deactivate left lane warning
				counter1 = 0;
				counter2 = 0;
				HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
				osDelay(500);
			}
			else if(counter2 == 1 && counter1 == 0)
			{
				// Activate right lane warning
				HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,1);
				osDelay(500);
			}
			else if(counter2 == 2)
			{
				// Deactivate right lane warning
				counter1 = 0;
				counter2 = 0;
				HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
				osDelay(500);
			}
			else if((counter2 == 1 && counter1 == 1))
			{
				counter1 = 0;
				counter2 = 0;
				HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
				HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
				osDelay(500);
			}
			else
			{
				// No lane departure warning
				osDelay(300);
			}
		}
  /* USER CODE END LaneDetectionWarning */
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
