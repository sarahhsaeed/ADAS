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
#include "tim.h"
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
/* USER CODE BEGIN Variables */
extern uint8_t Buffer[4];
extern uint16_t Buffer_ASCII_TO_INT;
extern uint8_t Car_Current_Mode;
extern uint8_t Car_Current_Direction;
extern uint8_t Car_Current_Status;
extern uint8_t Car_Current_Speed;
extern uint8_t Buffer_GUI[GUI_ARRAY_SIZE];



float Distance = 0.0;
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
/* Definitions for motorQueue */
osMessageQueueId_t motorQueueHandle;
const osMessageQueueAttr_t motorQueue_attributes = {
  .name = "motorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern UART_HandleTypeDef huart2;
/* USER CODE END FunctionPrototypes */

void StartmotorTask(void *argument);
void StartACCTask(void *argument);
void StartNormalMode(void *argument);

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
	  osDelay(20);
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
		Buffer_GUI[MODE_DIG_1_IDx] = CHARACTER_ZERO;
		Buffer_GUI[MODE_DIG_2_IDx] = CHARACTER_ZERO;

		Buffer_GUI[SPEED_DIG_1_IDx] = ((Car_Current_Speed  * 2) / 100) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_2_IDx] = (((Car_Current_Speed * 2) / 10) % 10) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_3_IDx] = ((Car_Current_Speed  * 2) % 10) + CHARACTER_ZERO;

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
					osDelay(5);         // TODO :: TEST
					DCMotor_moveForward(Car_Current_Speed);
					Car_Current_Direction = MOVE_FORWARD;
					break;
				case MOVE_LEFT:
					DCMotor_moveLeft(Car_Current_Speed);
					osDelay(5);        // TODO :: TEST
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

