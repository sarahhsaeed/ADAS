#include "HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOB,
		GPIO_PIN_14,
		TIM2,
		TIM_CHANNEL_1,
		16
	},
	{
		GPIOB,
		GPIO_PIN_13,
		TIM2,
		TIM_CHANNEL_3,
		16
	}
};
