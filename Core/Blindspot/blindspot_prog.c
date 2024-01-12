/*
 * blindspot_prog.c
 *
 *  Created on: Dec 7, 2023
 *      Author: Kirollos
 */

#include "main.h"
#include "blindspot_assist.h"
#include "../HCSR04/HCSR04.h"

uint8_t blindspot_isObjectDetected(void) {

	// Read distance from the ultrasonic sensor
    double distance = HCSR04_Read(HCSR04_SENSOR2);


    // Check if an object is within the blind spot range
    if (distance <= BLIND_SPOT_RANGE_CM)
	{
        return 1;  // Object detected

    }

	else
	{
        // No object in the blind spot
        return 0;  // No object detected
    }
}
