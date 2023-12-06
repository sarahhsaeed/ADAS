
#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
		// Servo Motor 1 Configurations
		{
				GPIOB,
				GPIO_PIN_9,
				TIM11,
				&TIM11->CCR1,
				TIM_CHANNEL_1,
				16000000,
				0.65,
				2.3
		},
		// Servo Motor 2 Configurations
		{
				GPIOA,
				GPIO_PIN_6,
				TIM3,
				&TIM3->CCR1,
				TIM_CHANNEL_1,
				16000000,
				0.65,
				2.3
		}
};
