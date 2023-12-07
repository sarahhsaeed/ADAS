/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../HCSR04/HCSR04.h"
#include "../DCMotor/DCMotor.h"
#include "../SERVO/SERVO.h"
#include "../Blindspot/blindspot_assist.h"
#include "tim.h"
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
/* USER CODE BEGIN Variables */
extern uint8_t Buffer[4];
extern uint16_t Buffer_ASCII_TO_INT;
extern uint8_t Car_Current_Mode;
extern uint8_t Car_Current_Direction;
extern uint8_t Car_Current_Status;
extern uint8_t Car_Current_Speed;
extern uint8_t Buffer_GUI[GUI_ARRAY_SIZE];
extern uint8_t GUI_TRANSMIT_INSTANT;
extern volatile uint8_t LeftIrCounter;
extern volatile uint8_t RightIrCounter;

float Distance = 0.0;
float Distance_Right = 0.0;
float Distance_Left = 0.0;

/* USER CODE END Variables */
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ACCTask */
osThreadId_t ACCTaskHandle;
const osThreadAttr_t ACCTask_attributes = {
  .name = "ACCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for NormalModeTask */
osThreadId_t NormalModeTaskHandle;
const osThreadAttr_t NormalModeTask_attributes = {
  .name = "NormalModeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for GUI_UpdateTask */
osThreadId_t GUI_UpdateTaskHandle;
const osThreadAttr_t GUI_UpdateTask_attributes = {
  .name = "GUI_UpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SelfDrivingTask */
osThreadId_t SelfDrivingTaskHandle;
const osThreadAttr_t SelfDrivingTask_attributes = {
  .name = "SelfDrivingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for LDW_TASK */
osThreadId_t LDW_TASKHandle;
const osThreadAttr_t LDW_TASK_attributes = {
  .name = "LDW_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RainDetect_TASK */
osThreadId_t RainDetect_TASKHandle;
const osThreadAttr_t RainDetect_TASK_attributes = {
  .name = "RainDetect_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LKA_TASK */
osThreadId_t LKA_TASKHandle;
const osThreadAttr_t LKA_TASK_attributes = {
  .name = "LKA_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Blindspot */
osThreadId_t BlindspotHandle;
const osThreadAttr_t Blindspot_attributes = {
  .name = "Blindspot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorQueue */
osMessageQueueId_t motorQueueHandle;
const osMessageQueueAttr_t motorQueue_attributes = {
  .name = "motorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern UART_HandleTypeDef huart2;
void SelfDrivingCheck_side(void);

/* USER CODE END FunctionPrototypes */

void StartmotorTask(void *argument);
void StartACCTask(void *argument);
void StartNormalMode(void *argument);
void StartGUI_UpdateTask(void *argument);
void StartSelfDrivingTask(void *argument);
void LaneDepartureWarning(void *argument);
void RainDetection(void *argument);
void LaneKeepAssist(void *argument);
void StartBlindspot(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorQueue */
  motorQueueHandle = osMessageQueueNew (16, sizeof(motorControl_t), &motorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartmotorTask, NULL, &motorTask_attributes);

  /* creation of ACCTask */
  ACCTaskHandle = osThreadNew(StartACCTask, NULL, &ACCTask_attributes);

  /* creation of NormalModeTask */
  NormalModeTaskHandle = osThreadNew(StartNormalMode, NULL, &NormalModeTask_attributes);

  /* creation of GUI_UpdateTask */
  GUI_UpdateTaskHandle = osThreadNew(StartGUI_UpdateTask, NULL, &GUI_UpdateTask_attributes);

  /* creation of SelfDrivingTask */
  SelfDrivingTaskHandle = osThreadNew(StartSelfDrivingTask, NULL, &SelfDrivingTask_attributes);

  /* creation of LDW_TASK */
  LDW_TASKHandle = osThreadNew(LaneDepartureWarning, NULL, &LDW_TASK_attributes);

  /* creation of RainDetect_TASK */
  RainDetect_TASKHandle = osThreadNew(RainDetection, NULL, &RainDetect_TASK_attributes);

  /* creation of LKA_TASK */
  LKA_TASKHandle = osThreadNew(LaneKeepAssist, NULL, &LKA_TASK_attributes);

  /* creation of Blindspot */
  BlindspotHandle = osThreadNew(StartBlindspot, NULL, &Blindspot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartmotorTask */
/**
 * @brief  Function implementing the motorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartmotorTask */
void StartmotorTask(void *argument)
{
  /* USER CODE BEGIN StartmotorTask */
	/* Infinite loop */
	for(;;)
	{
		motorControl_t motortask;
		uint8_t motortask_prio;
		if(osMessageQueueGet(motorQueueHandle, &motortask, &motortask_prio, 10) == osOK)
		{
			/*if(motortask.motors[0].speed > 0 && motortask.motors[0].speed < 40)
		  {
			  Motor1_SetSpeed(70);
			  Motor2_SetSpeed(70);
			  HAL_Delay(5);
		  }*/
			double prev_speeds[2];
			Motors_GetSpeeds(&prev_speeds[0], &prev_speeds[1]);
			uint8_t flag = 0;
			for(uint8_t i = 0; i < 2; i++)
			{
				if(motortask.motors[i].control == MOTOR_OFF)
				{
					flag = 1;
					break;
				}
				if((motortask.motors[i].modify & MOTOR_MODIFY_SPEED))
				{
					if(motortask.motors[i].speed < prev_speeds[i])
					{
						flag = 1;
						break;
					}
				}
			}
			if(flag)
			{
				HAL_GPIO_WritePin(RED_LIGHT_1_GPIO_Port, RED_LIGHT_1_Pin, 1);
				HAL_GPIO_WritePin(RED_LIGHT_2_GPIO_Port, RED_LIGHT_2_Pin, 1);
			}
			else
			{
				HAL_GPIO_WritePin(RED_LIGHT_1_GPIO_Port, RED_LIGHT_1_Pin, 0);
				HAL_GPIO_WritePin(RED_LIGHT_2_GPIO_Port, RED_LIGHT_2_Pin, 0);
			}
			DCMotor_handleRequest(&motortask);
		}
		osDelay(10);
	}
  /* USER CODE END StartmotorTask */
}

/* USER CODE BEGIN Header_StartACCTask */
/**
 * @brief Function implementing the ACCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartACCTask */
void StartACCTask(void *argument)
{
  /* USER CODE BEGIN StartACCTask */
	static uint8_t TRIG_Ticks = 0;
	static double prev_distance;
	static double prev_distances[2];
	/* Infinite loop */
	for(;;)
	{
		if(Car_Current_Mode == ACC_MODE)
		{
			Distance = HCSR04_Read(HCSR04_SENSOR1);
			/*if(Distance == prev_distances[0] && Distance == prev_distances[1])
	  {
		  Distance = 9999.0;
	  }*/
			TRIG_Ticks++;
			if(TRIG_Ticks >= 5)
			{
				HCSR04_Trigger(HCSR04_SENSOR1);
				TRIG_Ticks = 0;
				// MOTOR TEST:
				/*
		  static uint8_t lmao = 0;
		  osMessageQueuePut(motorQueueHandle, &lmao, 0, 1);
		  lmao = (lmao+1) % 8;
				 */
#if 0
				static uint8_t lmao = 0;
				motorControl_t payload = {0};
				switch(lmao)
				{
				case 0: // STOP ALL
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 0;
						payload.motors[i].control = MOTOR_OFF;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 1: // MOVE FORWARD
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 100;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 2: // MOVE BACKWARD
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 100;
						payload.motors[i].control = MOTOR_REV;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 3: // 30%
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 30;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 4: // 40%
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 40;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 5: // 50%
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 50;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 6: // 60%
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 60;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				case 7: // 70%
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = 70;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				}
				lmao = (lmao+1) % 8;
#endif
			}
#if 1
			// ACC START
#define DISTANCE_1	80
#define DISTANCE_2	65
#define DISTANCE_3	50
#define DISTANCE_4	35
#define DISTANCE_5	20
#define SPEED_1	100
#define SPEED_2	80
#define SPEED_3	60
#define SPEED_4	40
#define SPEED_5	20
#if 0
			static enum
			{
				CRUISE_CONTROL_FULL,
				CRUISE_CONTROL_SLOWDOWN,
				CRUISE_CONTROL_SPEEDUP,
				CRUISE_CONTROL_STOP
			} cruiseControl_state = CRUISE_CONTROL_FULL;
			double motorSpeeds[2];
			Motors_GetSpeeds(&motorSpeeds[0], &motorSpeeds[1]);
			double delta_distance = Distance - prev_distance;
			switch(cruiseControl_state)
			{
			case CRUISE_CONTROL_FULL:
			{
				if(Distance >= DISTANCE_1 && ((motorSpeeds[0] - motorSpeeds[1]) != 0 || motorSpeeds[0] < SPEED_1))
				{
					motorControl_t payload = {0};
					for(uint8_t i = 0; i < 2; i++)
					{
						payload.motors[i].modify = 1;
						payload.motors[i].speed = SPEED_1;
						payload.motors[i].control = MOTOR_FWD;
					}
					osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
					break;
				}

				if(delta_distance < 0)
				{
					cruiseControl_state = CRUISE_CONTROL_SLOWDOWN;
				}
				else if(delta_distance > 0)
				{
					cruiseControl_state = CRUISE_CONTROL_SPEEDUP;
				}
				else break;
				//break;
			}
			case CRUISE_CONTROL_SLOWDOWN:
			{
				if(delta_distance < 0)
				{
					if(delta_distance/-10 > 1)
					{
						// Apply speed loss based on distance loss
						double distance_loss = delta_distance/Distance*-1;
						double speed_loss = motorSpeeds[0] * distance_loss;
						if(speed_loss < 10)
						{
							speed_loss = 0;
							cruiseControl_state = CRUISE_CONTROL_STOP;
						}
						motorControl_t payload = {0};
						for(uint8_t i = 0; i < 2; i++)
						{
							payload.motors[i].modify = 1;
							payload.motors[i].speed = speed_loss;
							payload.motors[i].control = MOTOR_FWD;
						}
						osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
						cruiseControl_state = CRUISE_CONTROL_SLOWDOWN;
					}
					else
					{
						cruiseControl_state = CRUISE_CONTROL_SLOWDOWN;
					}
				}
				else if(delta_distance == 0)
				{
					cruiseControl_state = CRUISE_CONTROL_FULL;
				}
				else if(delta_distance > 0)
				{
					cruiseControl_state = CRUISE_CONTROL_SPEEDUP;
				}

				break;
			}
			case CRUISE_CONTROL_SPEEDUP:
			{
				double distance_gain = delta_distance/Distance + 1;
				double speed_gain = motorSpeeds[0] * distance_gain;
				if(delta_distance == 0 && motorSpeeds[0] < SPEED_5)
				{
					speed_gain = SPEED_5;
				}
				if(delta_distance > 0)
				{
					if(delta_distance/10 > 1)
					{
						// Apply speed gain based on distance gain

						if(speed_gain > SPEED_1)
							speed_gain = SPEED_1;
						motorControl_t payload = {0};
						for(uint8_t i = 0; i < 2; i++)
						{
							payload.motors[i].modify = 1;
							payload.motors[i].speed = motorSpeeds[i] * distance_gain;
							payload.motors[i].control = MOTOR_FWD;
						}
						osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
						cruiseControl_state = CRUISE_CONTROL_SPEEDUP;
					}
					else
					{
						cruiseControl_state = CRUISE_CONTROL_SLOWDOWN;
					}
				}
				break;
			}
			case CRUISE_CONTROL_STOP:
			{
				motorControl_t payload = {0};
				for(uint8_t i = 0; i < 2; i++)
				{
					payload.motors[i].modify = 1;
					payload.motors[i].speed = 0;
					payload.motors[i].control = MOTOR_FWD;
				}
				osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
				if(delta_distance > 0)
					cruiseControl_state = CRUISE_CONTROL_SPEEDUP;
				break;
			}
			}
#endif
#if 1
			if(Distance == 9999.0) continue;
			if(Distance > DISTANCE_1)
			{
				DCMotor_moveForward(SPEED_1);
			}
			else if(Distance < DISTANCE_1 && Distance > DISTANCE_2)
			{
				DCMotor_moveForward(SPEED_2);
			}
			else if(Distance < DISTANCE_2 && Distance > DISTANCE_3)
			{
				DCMotor_moveForward(SPEED_3);
			}
			else if(Distance < DISTANCE_3 && Distance > DISTANCE_4)
			{
				DCMotor_moveForward(SPEED_4);
			}
			else if(Distance < DISTANCE_4 && Distance > DISTANCE_5)
			{
				DCMotor_moveForward(SPEED_5);
			}
			else if(Distance < DISTANCE_5)
			{
				DCMotor_stop();
			}
#endif
			prev_distance = Distance;
			prev_distances[1] = prev_distances[0];
			prev_distances[0] = Distance;
			// ACC END
#endif
		}
		osDelay(50);
	}
  /* USER CODE END StartACCTask */
}

/* USER CODE BEGIN Header_StartNormalMode */
/**
 * @brief Function implementing the NormalModeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartNormalMode */
void StartNormalMode(void *argument)
{
  /* USER CODE BEGIN StartNormalMode */
	/* Infinite loop */
	for(;;)
	{
		if(Car_Current_Mode == NORMAL_MODE)
		{
			if(Car_Current_Status == CAR_RUNNING)
			{
				switch (Car_Current_Direction)
				{
				case MOVE_FORWARD:
					DCMotor_moveForward(Car_Current_Speed);
					break;

				case MOVE_BACKWORD:
					DCMotor_moveBackward(Car_Current_Speed);
					break;
				case MOVE_RIGHT:
					DCMotor_moveRight(Car_Current_Speed);
					osDelay(100);         // TODO :: TEST
					DCMotor_moveForward(Car_Current_Speed);
					Car_Current_Direction = MOVE_FORWARD;
					break;
				case MOVE_LEFT:
					DCMotor_moveLeft(Car_Current_Speed);
					osDelay(100);        // TODO :: TEST
					DCMotor_moveForward(Car_Current_Speed);
					Car_Current_Direction = MOVE_FORWARD;
					break;
				default:
					break;
				}
			}
			else if (Car_Current_Status == CAR_STOP)
			{
				DCMotor_stop();
			}
			else
			{
				// RETURN ERROR //
			}
		}
		osDelay(10);
	}
  /* USER CODE END StartNormalMode */
}

/* USER CODE BEGIN Header_StartGUI_UpdateTask */
/**
 * @brief Function implementing the GUI_UpdateTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGUI_UpdateTask */
void StartGUI_UpdateTask(void *argument)
{
  /* USER CODE BEGIN StartGUI_UpdateTask */
	/* Infinite loop */
	for(;;)
	{
		switch(Car_Current_Mode)
		{
		case NORMAL_MODE:
			Buffer_GUI[MODE_DIG_1_IDx] = CHARACTER_ZERO;
			Buffer_GUI[MODE_DIG_2_IDx] = CHARACTER_ZERO;
			break;
		case ACC_MODE:
			Buffer_GUI[MODE_DIG_1_IDx] = CHARACTER_ZERO;
			Buffer_GUI[MODE_DIG_2_IDx] = CHARACTER_ONE;
			break;
		case SELF_DRIVING_MODE:
			Buffer_GUI[MODE_DIG_1_IDx] = CHARACTER_ONE;
			Buffer_GUI[MODE_DIG_2_IDx] = CHARACTER_ZERO;
			break;
		}

		Buffer_GUI[SPEED_DIG_1_IDx] = ((Car_Current_Speed  * 2) / 100) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_2_IDx] = (((Car_Current_Speed * 2) / 10) % 10) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_3_IDx] = ((Car_Current_Speed  * 2) % 10) + CHARACTER_ZERO;
		//if(GUI_TRANSMIT_INSTANT == 1 )
		{
			HAL_UART_Transmit(&huart2, Buffer_GUI, 14, 20);
			/********* To Protect Global Variable "GUI_TRANSMIT_INSTANT" *********/
			//		HAL_NVIC_DisableIRQ(USART2_IRQn);
			GUI_TRANSMIT_INSTANT = 0 ;   //TODO:: Disable/Enable EXTI - IR
			//		HAL_NVIC_EnableIRQ(USART2_IRQn);
		}
		osDelay(100);
	}
  /* USER CODE END StartGUI_UpdateTask */
}

/* USER CODE BEGIN Header_StartSelfDrivingTask */
/**
 * @brief Function implementing the SelfDrivingTask thread.
 * @param argument: Not used
 * @\ None
 */
/* USER CODE END Header_StartSelfDrivingTask */
void StartSelfDrivingTask(void *argument)
{
  /* USER CODE BEGIN StartSelfDrivingTask */
	static uint8_t TRIG_Ticks = 0;
	/* Infinite loop */
	for(;;)
	{
		if(Car_Current_Mode == SELF_DRIVING_MODE)
		{
			Distance = HCSR04_Read(HCSR04_SENSOR1);
			/*if(Distance == prev_distances[0] && Distance == prev_distances[1])
		  	  {
		  		  Distance = 9999.0;
		  	  }*/
			TRIG_Ticks++;
			if(TRIG_Ticks >= 1)
			{
				HCSR04_Trigger(HCSR04_SENSOR1);
				TRIG_Ticks = 0;
			}
			if (Distance <= SELF_DRIVING_CRITICAL_RANGE)
			{
				uint8_t last_speed = Car_Current_Speed;
				const uint8_t turn_speed = 80;
				SelfDrivingCheck_side(); /* Check both sides - Left and Right - to get a decision for diversion */
				if(Distance_Left > Distance_Right)
				{
					/*DCMotor_moveLeft(turn_speed);
					osDelay(500);
					DCMotor_moveForward(last_speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveRight(turn_speed);
					osDelay(500);
					DCMotor_moveForward(last_speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveRight(turn_speed);
					osDelay(400);
					DCMotor_moveForward(last_speed);
					osDelay(500);
					DCMotor_moveLeft(turn_speed);
					osDelay(400);
					DCMotor_moveForward(last_speed);*/
					DCMotor_moveLeft(turn_speed);
					osDelay(10);
					uint8_t tick = 0;
					while(1)
					{
						HCSR04_Trigger(HCSR04_SENSOR1);
						osDelay(50);
						double d = HCSR04_Read(HCSR04_SENSOR1);
						tick++;
						if(d >= Distance_Left)
							break;
					}
					DCMotor_stop();
					SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_RIGHT);
					osDelay(100);
					DCMotor_moveForward(last_speed);
					osDelay(10);
					while(1)
					{
						HCSR04_Trigger(HCSR04_SENSOR1);
						osDelay(50);
						double d = HCSR04_Read(HCSR04_SENSOR1);
						if(d >= SELF_DRIVING_CRITICAL_RANGE)
							break;
					}
					DCMotor_stop();
					SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_CENTER);
					osDelay(100);

					DCMotor_moveLeft(turn_speed);
					osDelay(10);
					while(--tick)
					{
						osDelay(50);
					}
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveForward(last_speed);
					osDelay(10);
				}
				else
				{
					/*DCMotor_moveRight(turn_speed);
					osDelay(500);
					DCMotor_moveForward(last_speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveLeft(turn_speed);
					osDelay(500);
					DCMotor_moveForward(last_speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveLeft(turn_speed);
					osDelay(400);
					DCMotor_moveForward(last_speed);
					osDelay(500);
					DCMotor_moveRight(turn_speed);
					osDelay(400);
					DCMotor_moveForward(last_speed);*/
					DCMotor_moveRight(turn_speed);
					osDelay(10);
					uint8_t tick = 0;
					while(1)
					{
						HCSR04_Trigger(HCSR04_SENSOR1);
						osDelay(50);
						double d = HCSR04_Read(HCSR04_SENSOR1);
						tick++;
						if(d >= Distance_Right)
							break;
					}
					DCMotor_stop();
					SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_LEFT);
					osDelay(100);
					DCMotor_moveForward(last_speed);
					osDelay(10);
					while(1)
					{
						HCSR04_Trigger(HCSR04_SENSOR1);
						osDelay(50);
						double d = HCSR04_Read(HCSR04_SENSOR1);
						if(d >= SELF_DRIVING_CRITICAL_RANGE)
							break;
					}
					DCMotor_stop();
					SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_CENTER);
					osDelay(100);

					DCMotor_moveRight(turn_speed);
					osDelay(10);
					while(--tick)
					{
						osDelay(50);
					}
					DCMotor_stop();
					osDelay(100);
					DCMotor_moveForward(last_speed);
					osDelay(10);
				}

			}

			else
			{
				DCMotor_moveForward(Car_Current_Speed);  //Just keeping forward if there is no obstacles in front of the car
			}
		}
		osDelay(30);
	}
  /* USER CODE END StartSelfDrivingTask */
}

/* USER CODE BEGIN Header_LaneDepartureWarning */
/**
 * @brief Function implementing the LDW_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LaneDepartureWarning */
void LaneDepartureWarning(void *argument)
{
  /* USER CODE BEGIN LaneDepartureWarning */
	/* Infinite loop */
	for(;;)
	{
		if(LeftIrCounter>2)
		{
			LeftIrCounter=0;
		}
		if(RightIrCounter>2)
		{
			RightIrCounter=0;
		}
		if(LeftIrCounter == 1 && RightIrCounter == 0)
		{
			// Activate left lane warning
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,1);
			Buffer_GUI[LANE_DIG_1_IDx] = 1;
			osDelay(500);
		}
		else if(LeftIrCounter == 2 )
		{
			// Deactivate left lane warning
			LeftIrCounter = 0;
			RightIrCounter = 0;
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_1_IDx] = 0;
			osDelay(500);
		}
		else if(RightIrCounter == 1 && LeftIrCounter == 0)
		{
			// Activate right lane warning
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,1);
			Buffer_GUI[LANE_DIG_2_IDx] = 1;
			osDelay(500);
		}
		else if(RightIrCounter == 2)
		{
			// Deactivate right lane warning
			LeftIrCounter = 0;
			RightIrCounter = 0;
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_2_IDx] = 0;
			osDelay(500);
		}
		else if((RightIrCounter == 1 && LeftIrCounter == 1))
		{
			LeftIrCounter = 0;
			RightIrCounter = 0;
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_1_IDx] = 0;
			Buffer_GUI[LANE_DIG_2_IDx] = 0;
			osDelay(500);
		}
		else
		{
			// No lane departure warning
			osDelay(500);
		}
	}
  /* USER CODE END LaneDepartureWarning */
}

/* USER CODE BEGIN Header_RainDetection */
/**
 * @brief Function implementing the RainDetect_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RainDetection */
void RainDetection(void *argument)
{
  /* USER CODE BEGIN RainDetection */
	/* Infinite loop */
	/* Infinite loop */
	uint8_t RainDetectFlag = 0;
	/* Infinite loop */
	for(;;)
	{
		if(HAL_GPIO_ReadPin(RAIN_SENSOR_GPIO_Port, RAIN_SENSOR_Pin)==1)
		{
			HAL_GPIO_WritePin(RAIN_LED_GPIO_Port, RAIN_LED_Pin, 1);
			Buffer_GUI[RAIN_DIG_1_IDx] = 1;
			if(RainDetectFlag==0)
			{
				//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 100);
				SERVO_MoveTo(SERVO_MOTOR2,SERVO_ANGLE_FULL_LEFT);
				RainDetectFlag=1;
			}
			else if(RainDetectFlag==1)
			{
				//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 2000);
				SERVO_MoveTo(SERVO_MOTOR2,SERVO_ANGLE_FULL_RIGHT);
				RainDetectFlag=0;

			}
		}
		else
		{
			HAL_GPIO_WritePin(RAIN_LED_GPIO_Port, RAIN_LED_Pin, 0);
			Buffer_GUI[RAIN_DIG_1_IDx] = 0;
		}
		osDelay(500);
	}
  /* USER CODE END RainDetection */
}

/* USER CODE BEGIN Header_LaneKeepAssist */
/**
 * @brief Function implementing the LKA_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LaneKeepAssist */
void LaneKeepAssist(void *argument)
{
  /* USER CODE BEGIN LaneKeepAssist */
	uint8_t laneKeepFlag=0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(1000);continue; // temp disable
		if(LeftIrCounter==1)
		{
			laneKeepFlag=1;
			DCMotor_moveRight(70);
			osDelay(200);

		}
		if(laneKeepFlag==1 && LeftIrCounter==0)
		{
			laneKeepFlag=0;
			DCMotor_moveForward(Car_Current_Speed);
			osDelay(200);
		}
		if(RightIrCounter==1)
		{
			laneKeepFlag=1;
			DCMotor_moveLeft(70);
			osDelay(200);

		}
		if(laneKeepFlag==1 && RightIrCounter==0)
		{
			laneKeepFlag=0;
			DCMotor_moveForward(Car_Current_Speed);
			osDelay(200);
		}
		osDelay(100);
	}
  /* USER CODE END LaneKeepAssist */
}

/* USER CODE BEGIN Header_StartBlindspot */
/**
* @brief Function implementing the Blindspot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlindspot */
void StartBlindspot(void *argument)
{
  /* USER CODE BEGIN StartBlindspot */
  /* Infinite loop */
  for(;;)
  {
	static uint8_t TRIG_Ticks = 0;
	TRIG_Ticks++;
	if(TRIG_Ticks >= 1)
	{
		HCSR04_Trigger(HCSR04_SENSOR2);
		TRIG_Ticks = 0;
		osDelay(50);
	}
	uint8_t blind_check = blindspot_isObjectDetected();
	if(blind_check == 1)
	{
		// Toggle warning LED
		HAL_GPIO_WritePin(BLIND_LED_GPIO_Port, BLIND_LED_Pin, 1);
		Buffer_GUI[B_SPOT_DIG1_IDx] = Buffer_GUI[B_SPOT_DIG2_IDx] = 1;
		osDelay(1000);
	}
	else
	{
		HAL_GPIO_WritePin(BLIND_LED_GPIO_Port, BLIND_LED_Pin, 0);
		Buffer_GUI[B_SPOT_DIG1_IDx] = Buffer_GUI[B_SPOT_DIG2_IDx] = 0;
		osDelay(100);
	}
  }
  /* USER CODE END StartBlindspot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void SelfDrivingCheck_side(void)
{

	DCMotor_stop();
	DCMotor_moveBackward(Car_Current_Speed);
	while(1)
	{
		HCSR04_Trigger(HCSR04_SENSOR1);
		osDelay(50);
		double d = HCSR04_Read(HCSR04_SENSOR1);
		if(d > (SELF_DRIVING_CRITICAL_RANGE+10))
			break;
		osDelay(10);
	}
	DCMotor_stop();
	osDelay(10);
	/* Servo turn to Left (150) then read distance*/
	SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_LEFT);
	HCSR04_Trigger(HCSR04_SENSOR1);
	osDelay(800);
	Distance_Left = HCSR04_Read(HCSR04_SENSOR1);

	/* Servo turn to Right (50) then read distance*/
	SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_RIGHT);
	HCSR04_Trigger(HCSR04_SENSOR1);
	osDelay(800);
	Distance_Right = HCSR04_Read(HCSR04_SENSOR1);
	/* Servo turn to origin (100) then read distance*/
	SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_CENTER);
	osDelay(200);
}
/* USER CODE END Application */

