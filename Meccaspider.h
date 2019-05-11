/*
 * Meccaspider.h
 *
 *  Created on: 09/05/2019
 *      Author: best buy
 */

#ifndef MECCASPIDER_H_
#define MECCASPIDER_H_

#include "MK64F12.h" /* include peripheral declarations */
#include "GPIO.h"
#include "NVIC.h"
#include "stdint.h"
#include "Bits.h"
#include "PIT.h"
#include "FlexTimer.h"

#define MECCASPIDER_ONE_STEP_DELAY_L 1.94688F
#define MECCASPIDER_ONE_STEP_DELAY_R 1.959F
#define CLOCK_RATE 21000000U
#define MECCASPIDER_RIGHT_MOTOR_POS bit_17
#define MECCASPIDER_RIGHT_MOTOR_NEG bit_16
#define MECCASPIDER_LEFT_MOTOR_POS bit_11
#define MECCASPIDER_LEFT_MOTOR_NEG bit_10
#define MECCASPIDER_RIGHT_MOTOR_CALIBRATION bit_10
#define MECCASPIDER_LEFT_MOTOR_CALIBRATION bit_11

typedef enum
{
	FORWARD, BACKWARD, LEFT, RIGHT
}Meccaspider_direction_t;

void Meccaspider_init(void);

void Meccaspider_left_motor_one_step(void);

void Meccaspider_right_motor_one_step(void);

void Meccaspider_move(Meccaspider_direction_t direction);

#endif /* MECCASPIDER_H_ */
