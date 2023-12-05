/*
 * DCMotor.h
 *
 *  Created on: Dec 3, 2023
 *      Author: Kirollos
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

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

void DCMotor_stop(void);
void DCMotor_moveForward(uint8_t speed);
void DCMotor_moveBackward(uint8_t speed);
void DCMotor_moveLeft(uint8_t speed);
void DCMotor_moveRight(uint8_t speed);

void DCMotor_handleRequest(motorControl_t* motorRequest);

#endif /* DCMOTOR_H_ */
