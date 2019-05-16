/*
 * RGB_external.c
 *
 *  Created on: 31/03/2019
 *      Author: best buy
 */
#include "RGB_external.h"

void RGB_external_change_color(uint8_t intensity, RGB_LEDS color) {
	FlexTimer_PWM_Modify_Duty_Cycle(intensity, color);
}

void RGB_external_LED_init(void) {
	GPIO_clock_gating(GPIO_C);
	GPIO_clock_gating(GPIO_B);

	gpio_pin_control_register_t output_motor_control = GPIO_MUX1; //Configuración del GPIO

	GPIO_pin_control_register(GPIO_C, RGB_EXTERNAL_RED, &output_motor_control); //PTC5 as GPIO
	GPIO_pin_control_register(GPIO_C, RGB_EXTERNAL_GREEN,
			&output_motor_control); //PTC7 as GPIO
	GPIO_pin_control_register(GPIO_C, RGB_EXTERNAL_BLUE, &output_motor_control); //PTC0 as GPIO
	GPIO_pin_control_register(GPIO_C, RGB_EXTERNAL_LED1, &output_motor_control); //PTC9 as GPIO
	GPIO_pin_control_register(GPIO_C, RGB_EXTERNAL_LED2, &output_motor_control); //PTC8 as GPIO
	GPIO_pin_control_register(GPIO_B, RGB_EXTERNAL_LED3, &output_motor_control); //PTB19 as GPIO
	GPIO_pin_control_register(GPIO_B, RGB_EXTERNAL_LED4, &output_motor_control); //PTB18 as GPIO

	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, RGB_EXTERNAL_RED);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, RGB_EXTERNAL_GREEN);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, RGB_EXTERNAL_BLUE);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, RGB_EXTERNAL_LED1);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, RGB_EXTERNAL_LED2);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, RGB_EXTERNAL_LED3);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, RGB_EXTERNAL_LED4);

}

void RGB_external_set_LED(RGB_LEDS color, RGB_LED_NUMBER led_num) {
	switch (led_num) {
	case LED_1:
		RGB_external_set_color(color);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_LED1);
		break;
	case LED_2:
		RGB_external_set_color(color);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_LED2);
		break;
	case LED_3:
		RGB_external_set_color(color);
		GPIO_set_pin(GPIO_B, RGB_EXTERNAL_LED3);
		break;
	case LED_4:
		RGB_external_set_color(color);
		GPIO_set_pin(GPIO_B, RGB_EXTERNAL_LED4);
		break;
	}
}

void RGB_external_set_color(RGB_LEDS color) {
	//RGB_RED, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_CYAN, RGB_PURPLE,RGB_WHITE,RGB_BLACK
	switch (color) {
	case RGB_RED:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_RED); //RED color ON
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_GREEN:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_GREEN); //GREEN color ON
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_RED);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_BLUE:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_BLUE); //BLUE color ON
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_RED);
		break;
	case RGB_YELLOW:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_RED); //YELLOW color ON
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_CYAN:
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_RED); //CYAN color ON
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_PURPLE:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_RED); //PURPLE color ON
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_WHITE:
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_RED); //WHITE color ON
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	case RGB_BLACK:
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_RED); //BLACK color ON
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_GREEN);
		GPIO_set_pin(GPIO_C, RGB_EXTERNAL_BLUE);
		break;
	}
}

void RGB_external_clear_LEDs() {
	GPIO_set_pin(GPIO_C, RGB_EXTERNAL_RED);
	GPIO_set_pin(GPIO_C, RGB_EXTERNAL_GREEN);
	GPIO_set_pin(GPIO_C, RGB_EXTERNAL_BLUE);

	GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_LED1); //BLUE color ON
	GPIO_clear_pin(GPIO_C, RGB_EXTERNAL_LED2); //BLUE color ON
	GPIO_clear_pin(GPIO_B, RGB_EXTERNAL_LED3); //BLUE color ON
	GPIO_clear_pin(GPIO_B, RGB_EXTERNAL_LED4); //BLUE color ON

}

void RGB_external_LEDS_off(void) {
	RGB_external_change_color(MIN, RGB_RED);
	RGB_external_change_color(MIN, RGB_GREEN);
	RGB_external_change_color(MIN, RGB_BLUE);
}

void RGB_external_BLUE_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_BLUE);
}

void RGB_external_RED_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_RED);
}

void RGB_external_GREEN_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_GREEN);
}

void RGB_external_YELLOW_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_GREEN);
	RGB_external_change_color(MAX, RGB_RED);
}

void RGB_external_PURPLE_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(204, RGB_BLUE); //RGB(102,0,204) para generar el color moradp
	RGB_external_change_color(102, RGB_RED);
}

void RGB_external_WHITE_on(void) {
	RGB_external_LEDS_off();
	RGB_external_change_color(MAX, RGB_BLUE);
	RGB_external_change_color(MAX, RGB_RED);
	RGB_external_change_color(MAX, RGB_GREEN);
}
