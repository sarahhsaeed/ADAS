/*
 * DCMotor.c
 *
 *  Created on: Dec 3, 2023
 *      Author: Kirollos
 */


#include "main.h"
#include "DCMotor.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "tim.h"

extern osMessageQueueId_t motorQueueHandle;

void DCMotor_stop(void)
{
	motorControl_t payload = {0};
	for(uint8_t i = 0; i < 2; i++)
	{
	  payload.motors[i].modify = MOTOR_MODIFY_ALL;
	  payload.motors[i].speed = 0;
	  payload.motors[i].control = MOTOR_OFF;
	}
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_moveForward(uint8_t speed)
{
	motorControl_t payload = {0};
	for(uint8_t i = 0; i < 2; i++)
	{
	  payload.motors[i].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	  payload.motors[i].speed = speed;
	  payload.motors[i].control = MOTOR_FWD;
	}
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_moveBackward(uint8_t speed)
{
	motorControl_t payload = {0};
	for(uint8_t i = 0; i < 2; i++)
	{
	  payload.motors[i].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	  payload.motors[i].speed = speed;
	  payload.motors[i].control = MOTOR_REV;
	}
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_moveLeft(uint8_t speed)
{
	motorControl_t payload = {0};

	payload.motors[0].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	payload.motors[0].speed = speed;
	payload.motors[0].control = MOTOR_FWD;

	payload.motors[1].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	payload.motors[1].speed = speed;
	payload.motors[1].control = MOTOR_REV;
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_moveRight(uint8_t speed)
{
	motorControl_t payload = {0};

	payload.motors[0].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	payload.motors[0].speed = speed;
	//payload.motors[0].modify = MOTOR_MODIFY_ALL;
	//payload.motors[0].speed = MOTOR_TURN_SPEED;
	payload.motors[0].control = MOTOR_REV;

	payload.motors[1].modify = speed > 100 ? MOTOR_MODIFY_DIR : MOTOR_MODIFY_ALL;
	payload.motors[1].speed = speed;
	//payload.motors[1].modify = MOTOR_MODIFY_ALL;
	//payload.motors[1].speed = MOTOR_TURN_SPEED;
	payload.motors[1].control = MOTOR_FWD;
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_changeSpeed(uint8_t speed)
{
	motorControl_t payload = {0};
	for(uint8_t i = 0; i < 2; i++)
	{
	  payload.motors[i].modify = speed > 100 ? MOTOR_MODIFY_NONE : MOTOR_MODIFY_SPEED;
	  payload.motors[i].speed = speed;
	}
	osMessageQueuePut(motorQueueHandle, &payload, 0, 0);
}

void DCMotor_handleRequest(motorControl_t* motorRequest)
{
	for(uint8_t i = 0; i < 2; i++)
	{
	  uint16_t INx1_pin, INx2_pin;
	  GPIO_TypeDef *INx1_port, *INx2_port;
	  if(motorRequest->motors[i].modify == 0) continue;
	  if(i == 0){
		  if(motorRequest->motors[i].modify & MOTOR_MODIFY_SPEED)
			  Motor1_SetSpeed(motorRequest->motors[i].speed);
		  INx1_port = MOTOR_IN1_GPIO_Port;
		  INx2_port = MOTOR_IN2_GPIO_Port;
		  INx1_pin = MOTOR_IN1_Pin;
		  INx2_pin = MOTOR_IN2_Pin;
	  }
	  else if(i == 1){
		  if(motorRequest->motors[i].modify & MOTOR_MODIFY_SPEED)
			  Motor2_SetSpeed(motorRequest->motors[i].speed);
		  INx1_port = MOTOR_IN3_GPIO_Port;
		  INx2_port = MOTOR_IN4_GPIO_Port;
		  INx1_pin = MOTOR_IN3_Pin;
		  INx2_pin = MOTOR_IN4_Pin;
	  }
	if((motorRequest->motors[i].modify & MOTOR_MODIFY_DIR) == 0) continue;
	switch(motorRequest->motors[i].control)
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
