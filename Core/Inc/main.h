/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define GUI_ARRAY_SIZE                14
#define MODE_DIG_1_IDx                0
#define MODE_DIG_2_IDx                1
#define DISTANCE_DIG_1_IDx            2
#define DISTANCE_DIG_2_IDx            3
#define DISTANCE_DIG_3_IDx            4
#define SPEED_DIG_1_IDx               5
#define SPEED_DIG_2_IDx               6
#define SPEED_DIG_3_IDx               7
#define RAIN_DIG_1_IDx                8
#define LANE_DIG_1_IDx                9
#define LANE_DIG_2_IDx                10
#define B_SPOT_DIG1_IDx               11
#define B_SPOT_DIG2_IDx               12

#define MODE_DISABLE	(0)
#define MODE_ENABLE		(1)

#define CHARACTER_ZERO                '0'
#define CHARACTER_ONE                 '1'

#define CAR_DEFAULT_SPEED             50
#define CAR_DEFAULT_MODE              0

#define CAR_SPEED_OFFSET              1000
#define Car_self_Speed                90
#define SELF_DRIVING_CRITICAL_RANGE   30
#define SERVO_MOTOR1    0
#define SERVO_MOTOR2    1

// ACC
#define ACC_DISTANCE_1	80
#define ACC_DISTANCE_2	65
#define ACC_DISTANCE_3	50
#define ACC_DISTANCE_4	35
#define ACC_DISTANCE_5	20
#define ACC_SPEED_1	100
#define ACC_SPEED_2	80
#define ACC_SPEED_3	60
#define ACC_SPEED_4	40
#define ACC_SPEED_5	20
// ---

// LDW
#define LDW_NO_DETECTION		0
#define LDW_FIRST_DETECTION		1
#define LDW_SECOND_DETECTION	2
#define LDW_THIRD_DETECTION		2
// ---


typedef enum
{
	NORMAL_MODE         = 6 ,
	ACC_MODE            = 9 ,     // TODO:: In Mobile APP
	SELF_DRIVING_MODE   = 7 ,
	MOVE_RIGHT          = 3 ,
	MOVE_LEFT           = 1 ,
	MOVE_FORWARD   		= 2 ,
	MOVE_BACKWORD  		= 4 ,
	STOP_MOTOR     		= 5 ,
	STOP_MODE     		= 8 ,    // TODO:: In Mobile APP,
	LANEASSIST_ON		= 10,
	LANEASSIST_OFF		= 11,
	BLINDSPOT_ON		= 12,
	BLINDSPOT_OFF		= 13,

}MOBILE_BYTES_t;

typedef enum
{
	CAR_STOP = 0 ,
	CAR_RUNNING
}CAR_STATUS_t;

typedef enum
{
	GUI_NO_TRANSMIT = 0 ,
	GUI_TRANSMIT
}GUI_TX_t;



typedef enum
{
	FALLING = 0 ,
	RISING
}Trigger;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TURN_LEFT_LIGHT_Pin GPIO_PIN_13
#define TURN_LEFT_LIGHT_GPIO_Port GPIOC
#define TURN_RIGHT_LIGHT_Pin GPIO_PIN_14
#define TURN_RIGHT_LIGHT_GPIO_Port GPIOC
#define RED_LIGHT_Pin GPIO_PIN_15
#define RED_LIGHT_GPIO_Port GPIOC
#define MOTOR_EN1_Pin GPIO_PIN_0
#define MOTOR_EN1_GPIO_Port GPIOA
#define MOTOR_EN2_Pin GPIO_PIN_1
#define MOTOR_EN2_GPIO_Port GPIOA
#define LEFT_IR_LED_Pin GPIO_PIN_4
#define LEFT_IR_LED_GPIO_Port GPIOA
#define RIGHT_IR_LED_Pin GPIO_PIN_7
#define RIGHT_IR_LED_GPIO_Port GPIOA
#define LEFT_IR_Pin GPIO_PIN_0
#define LEFT_IR_GPIO_Port GPIOB
#define LEFT_IR_EXTI_IRQn EXTI0_IRQn
#define RIGHT_IR_Pin GPIO_PIN_1
#define RIGHT_IR_GPIO_Port GPIOB
#define RIGHT_IR_EXTI_IRQn EXTI1_IRQn
#define RAIN_SENSOR_Pin GPIO_PIN_2
#define RAIN_SENSOR_GPIO_Port GPIOB
#define BLIND_LED_Pin GPIO_PIN_9
#define BLIND_LED_GPIO_Port GPIOA
#define MOTOR_IN4_Pin GPIO_PIN_10
#define MOTOR_IN4_GPIO_Port GPIOA
#define MOTOR_IN3_Pin GPIO_PIN_5
#define MOTOR_IN3_GPIO_Port GPIOB
#define MOTOR_IN2_Pin GPIO_PIN_6
#define MOTOR_IN2_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_7
#define MOTOR_IN1_GPIO_Port GPIOB
#define SERVO_MOTOR_Pin GPIO_PIN_9
#define SERVO_MOTOR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
