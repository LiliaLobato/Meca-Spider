/*
 * RGB_external.h
 *
 *  Created on: 31/03/2019
 *      Author: best buy
 */

#ifndef RGB_EXTERNAL_H_
#define RGB_EXTERNAL_H_

#include "FlexTimer.h"
#define MIN 0
#define MAX (255U)

typedef enum {RGB_RED, RGB_GREEN, RGB_BLUE} RGB_LEDS;

void RGB_external_change_color(uint8_t intensity, RGB_LEDS channel);

void RGB_external_LEDS_off(void); //Turn off all leds

void RGB_external_BLUE_on(void); //Blue LED on

void RGB_external_RED_on(void); // RED led on

void RGB_external_GREEN_on(void); //GREEN LED on

void RGB_external_YELLOW_on(void); //Yellow on

void RGB_external_PURPLE_on(void); //Purple on

void RGB_external_WHITE_on(void);  //ALL LEDS on


#endif /* RGB_EXTERNAL_H_ */
