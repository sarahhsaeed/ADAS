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
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	struct motorControl_entity
	{
		uint8_t modify;
		uint8_t speed;
		enum
		{
			MOTOR_OFF = 0x00,
			MOTOR_FWD = 0x02,
			MOTOR_REV = 0x04
		} control;
	} motors[2];
} motorControl_t;
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
volatile uint8_t Static_u8UART_Recieve[4];
volatile uint8_t Global_u8CarMode;
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
/* Definitions for UART_RX_TASK */
osThreadId_t UART_RX_TASKHandle;
const osThreadAttr_t UART_RX_TASK_attributes = {
  .name = "UART_RX_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
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
void UART_RX(void *argument);

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

  /* creation of UART_RX_TASK */
  UART_RX_TASKHandle = osThreadNew(UART_RX, NULL, &UART_RX_TASK_attributes);

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
		  if(motortask.motors[0].speed > 0 && motortask.motors[0].speed < 40)
		  {
			  Motor1_SetSpeed(70);
			  Motor2_SetSpeed(70);
			  HAL_Delay(5);
		  }
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  uint16_t INx1_pin, INx2_pin;
			  GPIO_TypeDef *INx1_port, *INx2_port;
			  if(motortask.motors[i].modify == 0) continue;
			  if(i == 0){
				  Motor1_SetSpeed(motortask.motors[i].speed);
				  INx1_port = MOTOR_IN1_GPIO_Port;
				  INx2_port = MOTOR_IN2_GPIO_Port;
				  INx1_pin = MOTOR_IN1_Pin;
				  INx2_pin = MOTOR_IN2_Pin;
			  }
			  else if(i == 1){
				  Motor2_SetSpeed(motortask.motors[i].speed);
				  INx1_port = MOTOR_IN3_GPIO_Port;
				  INx2_port = MOTOR_IN4_GPIO_Port;
				  INx1_pin = MOTOR_IN3_Pin;
				  INx2_pin = MOTOR_IN4_Pin;
			  }
		  switch(motortask.motors[i].control)
		  {
		  case MOTOR_OFF:
			  HAL_GPIO_WritePin(INx1_port, INx1_pin, 0);
			  HAL_GPIO_WritePin(INx2_port, INx2_pin, 0);
			  break;
		  case MOTOR_FWD:
			  HAL_GPIO_WritePin(INx1_port, INx1_pin, 1);
			  HAL_GPIO_WritePin(INx2_port, INx2_pin, 0);
			  break;
		  case MOTOR_REV:
			  HAL_GPIO_WritePin(INx1_port, INx1_pin, 0);
			  HAL_GPIO_WritePin(INx2_port, INx2_pin, 1);
			  break;
		  }
		  }
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
  /* Infinite loop */
  for(;;)
  {
	  Distance = HCSR04_Read(HCSR04_SENSOR1);
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
	 #define DISTANCE_1	50
	 #define DISTANCE_2	35
	 #define DISTANCE_3	25
	 #define DISTANCE_4	15
 	 #define DISTANCE_5	5
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
	  if(Distance > DISTANCE_1)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = SPEED_1;
			  payload.motors[i].control = MOTOR_FWD;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
	  else if(Distance < DISTANCE_1 && Distance > DISTANCE_2)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = SPEED_2;
			  payload.motors[i].control = MOTOR_FWD;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
	  else if(Distance < DISTANCE_2 && Distance > DISTANCE_3)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = SPEED_3;
			  payload.motors[i].control = MOTOR_FWD;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
	  else if(Distance < DISTANCE_3 && Distance > DISTANCE_4)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = SPEED_4;
			  payload.motors[i].control = MOTOR_FWD;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
	  else if(Distance < DISTANCE_4 && Distance > DISTANCE_5)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = SPEED_5;
			  payload.motors[i].control = MOTOR_FWD;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
	  else if(Distance < DISTANCE_5)
	  {
		  motorControl_t payload = {0};
		  for(uint8_t i = 0; i < 2; i++)
		  {
			  payload.motors[i].modify = 1;
			  payload.motors[i].speed = 0;
			  payload.motors[i].control = MOTOR_OFF;
		  }
		  osMessageQueuePut(motorQueueHandle, &payload, 0, 1);
	  }
#endif
	  prev_distance = Distance;
	  // ACC END
	 #endif
	  osDelay(20);
  }
  /* USER CODE END StartACCTask */
}

/* USER CODE BEGIN Header_UART_RX */
/**
* @brief Function implementing the UART_RX_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX */
void UART_RX(void *argument)
{
  /* USER CODE BEGIN UART_RX */
  /* Infinite loop */
  for(;;)
  {
   HAL_UART_Receive(&huart2,Static_u8UART_Recieve,4,10);
    osDelay(5);
  }
  /* USER CODE END UART_RX */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void USART2_IRQHandler(void)
{
	 /* USER CODE BEGIN USART1_IRQn 0 */
	  /* USER CODE END USART1_IRQn 0 */
	  HAL_UART_IRQHandler(&huart2);
	  /* USER CODE BEGIN USART1_IRQn 1 */
	  Global_u8CarMode=atoi((char*)Static_u8UART_Recieve);
	  /* USER CODE END USART1_IRQn 1 */
}
/* USER CODE END Application */

