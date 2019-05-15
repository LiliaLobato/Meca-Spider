/*
 * RGB_external.c
 *
 *  Created on: 31/03/2019
 *      Author: best buy
 */
#include "RGB_external.h"

void RGB_external_change_color(uint8_t intensity, RGB_LEDS color)
{
	FlexTimer_PWM_Modify_Duty_Cycle(intensity, color);
}

void RGB_external_LEDS_off(void)
{
	RGB_external_change_color(MIN, RGB_RED);
	RGB_external_change_color(MIN, RGB_GREEN);
	RGB_external_change_color(MIN, RGB_BLUE);
}

void RGB_external_BLUE_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_BLUE);
}

void RGB_external_RED_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_RED);
}

void RGB_external_GREEN_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_GREEN);
}

void RGB_external_YELLOW_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_GREEN);
	RGB_external_change_color(MAX, RGB_RED);
}

void RGB_external_PURPLE_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(204, RGB_BLUE); //RGB(102,0,204) para generar el color moradp
	RGB_external_change_color(102, RGB_RED);
}

void RGB_external_WHITE_on(void)
{
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_BLUE);
	RGB_external_change_color(MAX, RGB_RED);
	RGB_external_change_color(MAX, RGB_GREEN);
}
