/*
 * MecaTeclado.c
 *
 *  Created on: 08/05/2019
 *      Author: Lilia Lobato
 */
#include "Bits.h"
#include "PIT.h"
#include "GPIO.h"
#include "MecaTeclado.h"
#include "NVIC.h"
#include "PushButton.h"
#include <stdio.h>

uint8_t k_teclado;
gpio_pin_control_register_t output_intr_config = GPIO_MUX1;
gpio_pin_control_register_t input_ext_configu = GPIO_MUX1 | GPIO_PE | GPIO_PS
		| INTR_FALLING_EDGE | GPIO_PFE; //Configuración del GPIO Externo

uint8_t MecaTeclado_get_k_teclado(void) {
	return k_teclado;
}

void MecaTeclado_Senales_clean() {
	GPIO_clear_pin(GPIO_B, 2);
	GPIO_clear_pin(GPIO_B, 3);
	GPIO_clear_pin(GPIO_B, 10);
}

void MecaTeclado_Senales() {
	if (TRUE == PIT_get1_interrupt_flag_status()) {
		switch (k_teclado) {

		case K01:
			GPIO_set_pin(GPIO_B, 2);
			GPIO_clear_pin(GPIO_B, 3);
			GPIO_clear_pin(GPIO_B, 10);
			k_teclado++;
			break;
		case K02:
			GPIO_clear_pin(GPIO_B, 2);
			GPIO_set_pin(GPIO_B, 3);
			GPIO_clear_pin(GPIO_B, 10);
			k_teclado++;
			break;
		case K03:
			GPIO_clear_pin(GPIO_B, 2);
			GPIO_clear_pin(GPIO_B, 3);
			GPIO_set_pin(GPIO_B, 10);
			k_teclado = K01;
			break;
		}
		PIT_clear1_interrupt_flag();
	}
}

void MecaTeclado_init() {
	//configuracion de salidas
	GPIO_clock_gating(GPIO_B);
	GPIO_pin_control_register(GPIO_B, 2, &output_intr_config);
	GPIO_pin_control_register(GPIO_B, 3, &output_intr_config);
	GPIO_pin_control_register(GPIO_B, 10, &output_intr_config);
	GPIO_clear_pin(GPIO_B, 2);
	GPIO_clear_pin(GPIO_B, 3);
	GPIO_clear_pin(GPIO_B, 10);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, 2);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, 3);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, 10);
	k_teclado = K01;

	//Configuracion de entradas
	GPIO_clock_gating(GPIO_D);
	GPIO_pin_control_register(GPIO_D, bit_1, &input_ext_configu);
	GPIO_data_direction_pin(GPIO_D, GPIO_INPUT, bit_1);
	GPIO_pin_control_register(GPIO_D, bit_2, &input_ext_configu);
	GPIO_data_direction_pin(GPIO_D, GPIO_INPUT, bit_2);
	GPIO_pin_control_register(GPIO_D, bit_3, &input_ext_configu);
	GPIO_data_direction_pin(GPIO_D, GPIO_INPUT, bit_3);

	NVIC_set_basepri_threshold(PRIORITY_12);
	GPIO_callback_init(GPIO_D, &PushButton_external_handler);
	NVIC_enable_interrupt_and_priotity(PORTD_IRQ, PRIORITY_4);
	//Configuraion PIT
	PIT_clock_gating();
	PIT_enable();
	NVIC_enable_interrupt_and_priotity(PIT_CH1_IRQ, PRIORITY_5);
	NVIC_global_enable_interrupts;
	PIT_delay(PIT_1, SYSTEM_CLOCK, DELAYBARRIDO);
}
