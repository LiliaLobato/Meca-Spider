/*
 * RGB_external.h
 *
 *  Created on: 31/03/2019
 *      Author: best buy
 */

#ifndef RGB_EXTERNAL_H_
#define RGB_EXTERNAL_H_

#include "FlexTimer.h"
#include "Bits.h"
#include "GPIO.h"
#define MIN 0
#define MAX (255U)
#define RGB_EXTERNAL_RED bit_5
#define RGB_EXTERNAL_GREEN bit_7
#define RGB_EXTERNAL_BLUE bit_0
#define RGB_EXTERNAL_LED1 bit_9
#define RGB_EXTERNAL_LED2 bit_8
#define RGB_EXTERNAL_LED3 bit_19
#define RGB_EXTERNAL_LED4 bit_18

typedef enum {RGB_RED, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_CYAN, RGB_PURPLE,RGB_WHITE,RGB_BLACK} RGB_LEDS;

typedef enum {LED_1, LED_2, LED_3, LED_4} RGB_LED_NUMBER;

void RGB_external_change_color(uint8_t intensity, RGB_LEDS channel);

void RGB_external_LEDS_off(void); //Turn off all leds

void RGB_external_BLUE_on(void); //Blue LED on

void RGB_external_RED_on(void); // RED led on

void RGB_external_GREEN_on(void); //GREEN LED on

void RGB_external_YELLOW_on(void); //Yellow on

void RGB_external_PURPLE_on(void); //Purple on

void RGB_external_WHITE_on(void);  //ALL LEDS on

void RGB_external_LED_init(void);

void RGB_external_set_LED(RGB_LEDS color, RGB_LED_NUMBER led_num);

void RGB_external_set_color(RGB_LEDS color);

void RGB_external_clear_LEDs();


#endif /* RGB_EXTERNAL_H_ */
