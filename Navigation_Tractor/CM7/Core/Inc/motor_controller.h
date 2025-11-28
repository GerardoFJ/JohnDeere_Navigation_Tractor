/*
 * motor_controller.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Gerardo Fregoso
 */

#ifndef INC_MOTOR_CONTROLLER_H_
#define INC_MOTOR_CONTROLLER_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define INITIAL_STATE 1500
#define SERVO_STATE 1589

void startMotor(TIM_HandleTypeDef *timnum);
void setMotorStep(int step);
void startServo(TIM_HandleTypeDef *tim_servo);
void setServo(int step);

#endif /* INC_MOTOR_CONTROLLER_H_ */
