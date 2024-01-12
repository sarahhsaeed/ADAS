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
extern uint8_t GUI_Send_Speed;

extern uint8_t Buffer_GUI[GUI_ARRAY_SIZE];
extern uint8_t GUI_TRANSMIT_INSTANT;
extern volatile uint8_t LeftIrCounter;
extern volatile uint8_t RightIrCounter;
extern uint8_t Car_LaneAssist_Enable;
extern uint8_t Car_BlindSpot_Enable;
extern volatile uint8_t LeftIrWarnCoutner;
extern volatile uint8_t RightIrWarnCoutner;

float Distance = 0.0;
float Distance_Right = 0.0;
float Distance_Left = 0.0;

uint8_t Red_Light_Flag = 0;
uint8_t Left_Light_Flag = 0;
uint8_t Right_Light_Flag = 0;

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
/* Definitions for RearLightsTask */
osThreadId_t RearLightsTaskHandle;
const osThreadAttr_t RearLightsTask_attributes = {
  .name = "RearLightsTask",
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
void StartRearLightsTask(void *argument);

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

  /* creation of RearLightsTask */
  RearLightsTaskHandle = osThreadNew(StartRearLightsTask, NULL, &RearLightsTask_attributes);

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
		/* Fetch motor command request from FreeRTOS queue */
		if(osMessageQueueGet(motorQueueHandle, &motortask, &motortask_prio, 10) == osOK)
		{
			/*if(motortask.motors[0].speed > 0 && motortask.motors[0].speed < 40)
		  {
			  Motor1_SetSpeed(70);
			  Motor2_SetSpeed(70);
			  HAL_Delay(5);
		  }*/
			/* Get previous speeds */
			double prev_speeds[2];
			Motors_GetSpeeds(&prev_speeds[0], &prev_speeds[1]);
			uint8_t flag = 0;
			for(uint8_t i = 0; i < 2; i++)
			{
				/* Raise flag to detect vehicle stop or speed reduction in order to trigger the rear red LEDs */
				if(motortask.motors[i].control == MOTOR_OFF)
				{
					flag = 1;
					break;
				}
				if((motortask.motors[i].modify & MOTOR_MODIFY_SPEED))
				{
					if(motortask.motors[i].speed < prev_speeds[i] || motortask.motors[i].speed == 0)
					{
						flag = 1;
						break;
					}
				}
			}
			if(flag) /* If the flag was raised, flag the red LEDs to get turned on by its own task */
			{
				Red_Light_Flag = 1;
			}
			else /* Otherwise, reset the flag */
			{
				Red_Light_Flag = 0;
			}
			/* If the vehicle is turning right or left, raise the turning indicator LED flag*/
			// LEFT
			if(motortask.motors[0].control == MOTOR_FWD && motortask.motors[1].control == MOTOR_REV)
			{
				Left_Light_Flag = 1;
				Right_Light_Flag = 0;
			}
			else
			// RIGHT
			if(motortask.motors[0].control == MOTOR_REV && motortask.motors[1].control == MOTOR_FWD)
			{
				Left_Light_Flag = 0;
				Right_Light_Flag = 1;
			}
			else // NEITHER
			{
				Left_Light_Flag = 0;
				Right_Light_Flag = 0;
			}
			/* Handle the DC Motor request by setting PWM duty cycle. */
			DCMotor_handleRequest(&motortask);
		}
		/* Send the task to BLOCKING state for 10 ticks (10 ms) */
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
			TRIG_Ticks++;
			if(TRIG_Ticks >= 5)
			{
				HCSR04_Trigger(HCSR04_SENSOR1);
				TRIG_Ticks = 0;
			}
			// ACC START

			if(Distance > ACC_DISTANCE_1)
			{
				DCMotor_moveForward(ACC_SPEED_1);
				GUI_Send_Speed = ACC_SPEED_1;
			}
			else if(Distance < ACC_DISTANCE_1 && Distance > ACC_DISTANCE_2)
			{
				DCMotor_moveForward(ACC_SPEED_2);
				GUI_Send_Speed = ACC_SPEED_2;
			}
			else if(Distance < ACC_DISTANCE_2 && Distance > ACC_DISTANCE_3)
			{
				DCMotor_moveForward(ACC_SPEED_3);
				GUI_Send_Speed = ACC_SPEED_3;
			}
			else if(Distance < ACC_DISTANCE_3 && Distance > ACC_DISTANCE_4)
			{
				DCMotor_moveForward(ACC_SPEED_4);
				GUI_Send_Speed = ACC_SPEED_4;
			}
			else if(Distance < ACC_DISTANCE_4 && Distance > ACC_DISTANCE_5)
			{
				DCMotor_moveForward(ACC_SPEED_5);
				GUI_Send_Speed = ACC_SPEED_5;
			}
			else if(Distance < ACC_DISTANCE_5)
			{
				DCMotor_stop();
				GUI_Send_Speed = CAR_STOP;
			}
			GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

			prev_distance = Distance;
			prev_distances[1] = prev_distances[0];
			prev_distances[0] = Distance;
			// ACC END
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
		if((Car_Current_Mode == NORMAL_MODE) && ( (Car_LaneAssist_Enable==0) || ((LeftIrCounter == 0) && (RightIrCounter == 0))))
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
					osDelay(100);
					DCMotor_moveForward(Car_Current_Speed);
					Car_Current_Direction = MOVE_FORWARD;
					break;
				case MOVE_LEFT:
					DCMotor_moveLeft(Car_Current_Speed);
					osDelay(100);
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

		Buffer_GUI[SPEED_DIG_1_IDx] = ((GUI_Send_Speed  * 2) / 100) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_2_IDx] = (((GUI_Send_Speed * 2) / 10) % 10) + CHARACTER_ZERO;
		Buffer_GUI[SPEED_DIG_3_IDx] = ((GUI_Send_Speed  * 2) % 10) + CHARACTER_ZERO;

		if(GUI_TRANSMIT_INSTANT == GUI_TRANSMIT )
		{
			HAL_UART_Transmit(&huart2, Buffer_GUI, 14, 20);
			GUI_TRANSMIT_INSTANT = GUI_NO_TRANSMIT ;
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
	//static uint8_t mode_first_called = 0;
	//	Car_Current_Speed = 55;
	SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_CENTER);
	/* Infinite loop */
	for(;;)
	{
		if(Car_Current_Mode == SELF_DRIVING_MODE)
		{
			// Distance = HCSR04_Read(HCSR04_SENSOR1);
			HCSR04_Trigger(HCSR04_SENSOR1);
			osDelay(100);
			Distance = HCSR04_Read(HCSR04_SENSOR1);

			if (Distance <= SELF_DRIVING_CRITICAL_RANGE)
			{

				DCMotor_stop();

				GUI_Send_Speed = CAR_STOP ;
				GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;
				osDelay(500);
				DCMotor_moveBackward(Car_self_Speed);
				GUI_Send_Speed = Car_self_Speed ;
				GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
				osDelay(500); /* 500 */
				DCMotor_stop();
				GUI_Send_Speed = CAR_STOP ;
				GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
				osDelay(500); /* 15 */
				/* Servo turn to Right (120) then read distance*/
				SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_RIGHT);
				osDelay(500);
				HCSR04_Trigger(HCSR04_SENSOR1);
				osDelay(500);
				Distance_Right = HCSR04_Read(HCSR04_SENSOR1);
				SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_CENTER);
				osDelay(500);
				/* Servo turn to Left (60) then read distance*/
				SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_LEFT);
				osDelay(500);
				HCSR04_Trigger(HCSR04_SENSOR1);
				osDelay(500);
				Distance_Left = HCSR04_Read(HCSR04_SENSOR1);
				/* Servo turn to origin (115) then read distance*/
				SERVO_MoveTo(SERVO_MOTOR1,SERVO_ANGLE_CENTER);


				if(Distance_Left >= Distance_Right)

				{


					DCMotor_moveLeft(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(600);


					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(1400);


					DCMotor_moveForward(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(700);
					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(1200);
					DCMotor_moveRight(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(600);
					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
					osDelay(1200);

					GUI_Send_Speed = Car_Current_Speed;


					/* DCMotor_moveLeft(Car_self_Speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(1200);
					DCMotor_moveForward(Car_self_Speed);
					osDelay(800);
					DCMotor_stop();
					osDelay(1200);
					DCMotor_moveRight(Car_self_Speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(1200);
					DCMotor_moveForward(Car_self_Speed); // parralle  to origin
					osDelay(1000);
					DCMotor_stop();
					osDelay(1200);
					DCMotor_moveRight(Car_self_Speed);
					osDelay(400);
					DCMotor_stop();
					osDelay(1200);
					DCMotor_moveForward(Car_self_Speed);
					osDelay(800);
					DCMotor_stop();
					osDelay(600);
					for(uint8_t i=0;i<10;i++)
					{
						DCMotor_moveLeft(Car_self_Speed);
						osDelay(50);
						DCMotor_moveForward(Car_self_Speed);
						osDelay(5);
					} */
				}
				else if (Distance_Right >= Distance_Left)
				{
					DCMotor_moveRight(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(600);
					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(1400);
					DCMotor_moveForward(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;

					osDelay(700);
					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
					osDelay(1200);
					DCMotor_moveLeft(Car_self_Speed);
					GUI_Send_Speed = Car_self_Speed;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
					osDelay(600);
					DCMotor_stop();
					GUI_Send_Speed = CAR_STOP;
					GUI_TRANSMIT_INSTANT =  GUI_TRANSMIT ;
					osDelay(1200);

					GUI_Send_Speed = Car_Current_Speed;

				/*	DCMotor_moveForward(Car_self_Speed); // Moving Parrallel to origin

					osDelay(1000);
					DCMotor_moveLeft(Car_self_Speed);
					osDelay(600);
					DCMotor_moveForward(Car_self_Speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(2000);
					DCMotor_moveRight(Car_self_Speed);
					osDelay(600);
					DCMotor_stop();
					osDelay(2000);

					*/

				}

			}

			else
			{
				DCMotor_moveForward(Car_Current_Speed);  // Just keeping forward if there is no obstacles in front of the car
			}
		}
		osDelay(100);
	}

//		static uint8_t TRIG_Ticks = 0;
//		/* Infinite loop */
//		for(;;)
//		{
//			if(Car_Current_Mode == SELF_DRIVING_MODE)
//			{
//				Distance = HCSR04_Read(HCSR04_SENSOR1);
//				/*if(Distance == prev_distances[0] && Distance == prev_distances[1])
//			  	  {
//			  		  Distance = 9999.0;
//			  	  }*/
//				TRIG_Ticks++;
//				if(TRIG_Ticks >= 1)
//				{
//					HCSR04_Trigger(HCSR04_SENSOR1);
//					TRIG_Ticks = 0;
//				}
//				if (Distance <= SELF_DRIVING_CRITICAL_RANGE)
//				{
//					uint8_t last_speed = Car_Current_Speed;
//					const uint8_t turn_speed = 80;
//					SelfDrivingCheck_side(); /* Check both sides - Left and Right - to get a decision for diversion */
//					if(Distance_Left > Distance_Right)
//					{
//						/*DCMotor_moveLeft(turn_speed);
//						osDelay(500);
//						DCMotor_moveForward(last_speed);
//						osDelay(600);
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveRight(turn_speed);
//						osDelay(500);
//						DCMotor_moveForward(last_speed);
//						osDelay(600);
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveRight(turn_speed);
//						osDelay(400);
//						DCMotor_moveForward(last_speed);
//						osDelay(500);
//						DCMotor_moveLeft(turn_speed);
//						osDelay(400);
//						DCMotor_moveForward(last_speed);*/
//						DCMotor_moveLeft(turn_speed);
//						osDelay(10);
//						uint32_t tick = 0;
//						while(1)
//						{
//							HCSR04_Trigger(HCSR04_SENSOR1);
//							osDelay(50);
//							double d = HCSR04_Read(HCSR04_SENSOR1);
//							tick++;
//							if(d >= Distance_Left)
//								break;
//						}
//						DCMotor_stop();
//						SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_RIGHT);
//						osDelay(100);
//						DCMotor_moveForward(last_speed);
//						osDelay(10);
//						while(1)
//						{
//							HCSR04_Trigger(HCSR04_SENSOR1);
//							osDelay(50);
//							double d = HCSR04_Read(HCSR04_SENSOR1);
//							if(d >= SELF_DRIVING_CRITICAL_RANGE)
//								break;
//						}
//						DCMotor_stop();
//						SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_CENTER);
//						osDelay(100);
//
//						DCMotor_moveRight(turn_speed);   // SHOULD BE TURN RIGHT :: Last State : Left
//						osDelay(10);
//						while(--tick)
//						{
//							osDelay(50);
//						}
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveForward(last_speed);
//						osDelay(10);
//					}
//					else
//					{
//						/*DCMotor_moveRight(turn_speed);
//						osDelay(500);
//						DCMotor_moveForward(last_speed);
//						osDelay(600);
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveLeft(turn_speed);
//						osDelay(500);
//						DCMotor_moveForward(last_speed);
//						osDelay(600);
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveLeft(turn_speed);
//						osDelay(400);
//						DCMotor_moveForward(last_speed);
//						osDelay(500);
//						DCMotor_moveRight(turn_speed);
//						osDelay(400);
//						DCMotor_moveForward(last_speed);*/
//						DCMotor_moveRight(turn_speed);
//						osDelay(10);
//						uint32_t tick = 0;
//						while(1)
//						{
//							HCSR04_Trigger(HCSR04_SENSOR1);
//							osDelay(50);
//							double d = HCSR04_Read(HCSR04_SENSOR1);
//							tick++;
//							if(d >= Distance_Right)
//								break;
//						}
//						DCMotor_stop();
//						SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_LEFT);
//						osDelay(100);
//						DCMotor_moveForward(last_speed);
//						osDelay(10);
//						while(1)
//						{
//							HCSR04_Trigger(HCSR04_SENSOR1);
//							osDelay(50);
//							double d = HCSR04_Read(HCSR04_SENSOR1);
//							if(d >= SELF_DRIVING_CRITICAL_RANGE)
//								break;
//						}
//						DCMotor_stop();
//						SERVO_MoveTo(SERVO_MOTOR1, SERVO_ANGLE_CENTER);
//						osDelay(100);
//
//						DCMotor_moveLeft(turn_speed);    // Should be Left  :: Last State : Right
//						osDelay(10);
//						while(--tick)
//						{
//							osDelay(50);
//						}
//						DCMotor_stop();
//						osDelay(100);
//						DCMotor_moveForward(last_speed);
//						osDelay(10);
//					}
//
//				}
//
//				else
//				{
//					DCMotor_moveForward(Car_Current_Speed);  //Just keeping forward if there is no obstacles in front of the car
//				}
//			}
//			osDelay(30);
//		}
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
		if(LeftIrWarnCoutner>LDW_THIRD_DETECTION)
		{
			LeftIrWarnCoutner=LDW_NO_DETECTION;
		}
		if(RightIrWarnCoutner>LDW_THIRD_DETECTION)
		{
			RightIrWarnCoutner=LDW_NO_DETECTION;
		}


		if(LeftIrWarnCoutner == LDW_FIRST_DETECTION && RightIrCounter == LDW_NO_DETECTION)
		{
			// Activate left lane warning
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,1);
			Buffer_GUI[LANE_DIG_1_IDx] = CHARACTER_ONE;
			Buffer_GUI[LANE_DIG_2_IDx] = CHARACTER_ZERO;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;
			osDelay(150);
		}
		else if(LeftIrWarnCoutner == LDW_THIRD_DETECTION )
		{
			// Deactivate left lane warning
			LeftIrWarnCoutner = LDW_NO_DETECTION;
			RightIrWarnCoutner = LDW_NO_DETECTION;
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_1_IDx] = CHARACTER_ZERO;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			osDelay(150);
		}
		else if(RightIrWarnCoutner == LDW_FIRST_DETECTION && LeftIrWarnCoutner == LDW_NO_DETECTION)
		{
			// Activate right lane warning
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,1);
			Buffer_GUI[LANE_DIG_1_IDx] = CHARACTER_ZERO;
			Buffer_GUI[LANE_DIG_2_IDx] = CHARACTER_ONE;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			osDelay(150);
		}
		else if(RightIrCounter == LDW_THIRD_DETECTION)
		{
			// Deactivate right lane warning
			LeftIrWarnCoutner = LDW_NO_DETECTION;
			RightIrWarnCoutner = LDW_NO_DETECTION;
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_2_IDx] = CHARACTER_ZERO;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			osDelay(150);
		}

		else if((RightIrWarnCoutner == LDW_FIRST_DETECTION && LeftIrWarnCoutner == LDW_FIRST_DETECTION))
		{
			LeftIrWarnCoutner = LDW_NO_DETECTION;
			RightIrWarnCoutner = LDW_NO_DETECTION;
			HAL_GPIO_WritePin(LEFT_IR_LED_GPIO_Port,LEFT_IR_LED_Pin,0);
			HAL_GPIO_WritePin(RIGHT_IR_LED_GPIO_Port,RIGHT_IR_LED_Pin,0);
			Buffer_GUI[LANE_DIG_1_IDx] = CHARACTER_ZERO;
			Buffer_GUI[LANE_DIG_2_IDx] = CHARACTER_ZERO;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			osDelay(150);
		}
		else
		{
			// No lane departure warning
			osDelay(50);
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
			Buffer_GUI[RAIN_DIG_1_IDx] = CHARACTER_ONE;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			if(RainDetectFlag==0)
			{
				SERVO_MoveTo(SERVO_MOTOR2,SERVO_ANGLE_FULL_LEFT);
				RainDetectFlag=1;
			}
			else if(RainDetectFlag==1)
			{
				SERVO_MoveTo(SERVO_MOTOR2,SERVO_ANGLE_FULL_RIGHT);
				RainDetectFlag=0;

			}
		}
		else
		{
			//HAL_GPIO_WritePin(RAIN_LED_GPIO_Port, RAIN_LED_Pin, 0);
			Buffer_GUI[RAIN_DIG_1_IDx] = CHARACTER_ZERO;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

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
	//	static uint8_t laneKeepFlag = 0;
	//	uint8_t i = 0;
		/* Infinite loop */
		for(;;)
		{
			//		osDelay(1000); continue; // Temp disable
			if(!Car_LaneAssist_Enable)
			{
				osDelay(1000);
				continue;
			}
			if(LeftIrCounter > 0)
			{
				while(LeftIrCounter != 2)
				{
					DCMotor_moveRight(70);
					osDelay(300); /* Check */
					//DCMotor_moveForward(Car_Current_Speed); /* Check */
					//osDelay(100);
//					DCMotor_stop();
//					osDelay(1000); /* To be modified */

				}

				LeftIrCounter = 0;

			}

			else if(RightIrCounter > 0)
			{
				while(RightIrCounter != 2)
				{
					DCMotor_moveLeft(70);
					osDelay(300);
					//DCMotor_moveForward(Car_Current_Speed);
					//osDelay(100);
//					DCMotor_stop();
//					osDelay(1000); /* To be modified */

				}

				RightIrCounter = 0;
			}

	//		if(laneKeepFlag == 1 && RightIrCounter == 0 && LeftIrCounter == 0)
	//		{
	//			laneKeepFlag = 0;
	//			DCMotor_moveLeft(70);
	//			osDelay(100);
	//			DCMotor_moveForward(70);
	//			osDelay(100);
	//		}
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
		if(!Car_BlindSpot_Enable)
		{
			osDelay(1000);
			continue;
		}
//		osDelay(1000); continue;
		static uint8_t TRIG_Ticks = 0;
//		if(!Car_BlindSpot_Enable)
//		{
//					osDelay(1000);
//					osThreadYield();
//					continue;
//		}
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

			Buffer_GUI[B_SPOT_DIG1_IDx] = CHARACTER_ZERO;
			Buffer_GUI[B_SPOT_DIG2_IDx] = CHARACTER_ONE;
			GUI_TRANSMIT_INSTANT = GUI_TRANSMIT ;

			osDelay(1000);
		}
		else
		{
			HAL_GPIO_WritePin(BLIND_LED_GPIO_Port, BLIND_LED_Pin, 0);
			Buffer_GUI[B_SPOT_DIG1_IDx] = Buffer_GUI[B_SPOT_DIG2_IDx] = CHARACTER_ZERO;
			osDelay(100);
		}
	}
  /* USER CODE END StartBlindspot */
}

/* USER CODE BEGIN Header_StartRearLightsTask */
/**
* @brief Function implementing the RearLightsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRearLightsTask */
void StartRearLightsTask(void *argument)
{
  /* USER CODE BEGIN StartRearLightsTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(RED_LIGHT_GPIO_Port, RED_LIGHT_Pin, Red_Light_Flag);
	  if(Left_Light_Flag)
	  {
		  for(uint8_t t = 0; t < 16; t++) {
		  HAL_GPIO_TogglePin(TURN_LEFT_LIGHT_GPIO_Port, TURN_LEFT_LIGHT_Pin);
		  osDelay(100);
		  }
	  }
	  else
	  if(Right_Light_Flag)
	  {
		  for(uint8_t t = 0; t < 16; t++) {
		  HAL_GPIO_TogglePin(TURN_RIGHT_LIGHT_GPIO_Port, TURN_RIGHT_LIGHT_Pin);
		  osDelay(100);
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(TURN_LEFT_LIGHT_GPIO_Port, TURN_LEFT_LIGHT_Pin, 0);
		  HAL_GPIO_WritePin(TURN_RIGHT_LIGHT_GPIO_Port, TURN_RIGHT_LIGHT_Pin, 0);
	  }
    osDelay(100);
  }
  /* USER CODE END StartRearLightsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

