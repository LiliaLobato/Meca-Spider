/*
 * PushButton.c
 *
 *  Created on: 24/02/2019
 *      Author: Lilia Lobato
 */

#include "PushButton.h"
#include "MK64F12.h"
#include "GPIO.h"
#include "Delay.h"
#include "MecaTeclado.h"
#include "PIT.h"

#define ARRMAX 5

uint8_t pb_flag_g = FALSE;
uint8_t pb_change_flag_g = FALSE;
static uint8_t K11_state;
static uint8_t K12_state;
static uint8_t K13_state;
static uint8_t interrupt_num = FALSE;
static uint8_t interrupt_arr[ARRMAX];

gpio_pin_control_register_t dma_intr_config = GPIO_MUX1 | GPIO_PE | GPIO_PS
		| DMA_FALLING_EDGE; /* GPIO configured to trigger the DMA*/
gpio_pin_control_register_t input_intr_config = GPIO_MUX1 | GPIO_PE | GPIO_PS
		| INTR_FALLING_EDGE | GPIO_PFE;	//Configuración del GPIO
gpio_pin_control_register_t input_intr_ext_config = GPIO_MUX1 | GPIO_PE
		| GPIO_PS | INTR_FALLING_EDGE | GPIO_PFE;//Configuración del GPIO Externo

void PushButton_sw2_config(void) {
	GPIO_clock_gating(GPIO_C);								//Activa el puerto C
	GPIO_pin_control_register(GPIO_C, 6, &dma_intr_config); //Configuracion del GPIO C para el sw2
	GPIO_data_direction_port(GPIO_C, ~(SW2)); //Configura el puerto del sw2 del GPIO C como Input
}

void PushButton_sw3_config(void) {
	GPIO_clock_gating(GPIO_A);								//Activa el puerto A
	GPIO_pin_control_register(GPIO_A, 4, &input_intr_config); //Configuracion del GPIO A para el sw3
	PORTA->PCR[4] = (GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE);
	GPIO_data_direction_port(GPIO_A, ~(SW3)); //Configura el puerto del sw3 del GPIO A como Input

}

void PushButton_external_config(gpio_port_name_t GPIOx, BitsType pin,
		uint32_t MASK) {
	GPIO_clock_gating(GPIOx);								//Activa el puerto x
	GPIO_pin_control_register(GPIO_B, pin, &input_intr_ext_config); //Configuracion del GPIO x para el pin n
	GPIO_data_direction_port(GPIOx, MASK); //Configura el puerto del GPIO C como Input
}

uint32_t PushButton_read(PushButton_SW_name sw) {
	uint32_t push_value = 0;
	switch (sw) {
	case PB_SW2:/** PB_SW2 is selected*/
		push_value = GPIO_read_pin(GPIO_C, SW2);
		break;
	case PB_SW3:/** PB_SW3 is selected*/
		push_value = GPIO_read_pin(GPIO_A, SW3);
		break;
	default:/**If doesn't exist the option*/
		push_value = 0x00000000;
		break;
	}
	return push_value;
}

void PushButton_external_handler(void) {
	uint8_t k_teclado_ext = MecaTeclado_get_k_teclado();
	K13_state = GPIO_read_pin(GPIO_D, bit_3); //Obtiene el estado del B0
	K12_state = GPIO_read_pin(GPIO_D, bit_2); //Obtiene el estado del B1
	K11_state = GPIO_read_pin(GPIO_D, bit_1); //Obtiene el estado del B2

	if (ARRMAX == interrupt_num) {
		PIT_dissable();
		float promedio = FALSE;
		for (uint8_t i = FALSE; i < ARRMAX; i++) {
			promedio = promedio + interrupt_arr[i];
			interrupt_arr[i] = FALSE;
		}
		promedio = (promedio / ARRMAX);
		k_teclado_ext = (uint8_t) (
				promedio < 0 ? (promedio - 0.5) : (promedio + 0.5));
		interrupt_num = FALSE;

		/**PB has logic 0 when its pressed*/
		if ((FALSE == K11_state)) //Pregunta quien fue el que interrumpio, para saber que bandera activar
		{
			switch (k_teclado_ext) {
			case K01:
				pb_flag_g = CIRCULO;
				break;
			case K02:
				pb_flag_g = ADELANTE;
				break;
			case K03:
				pb_flag_g = DADOS;
				break;
			}
		} else

		if ((FALSE == K12_state)) //Pregunta quien fue el que interrumpio, para saber que bandera activar
		{
			switch (k_teclado_ext) {
			case K01:
				pb_flag_g = TRIANGULO;
				break;
			case K02:
				pb_flag_g = DERECHA;
				break;
			case K03:
				pb_flag_g = CENTRO;
				break;
			}
		} else

		if ((FALSE == K13_state)) //Pregunta quien fue el que interrumpio, para saber que bandera activar
		{
			switch (k_teclado_ext) {
			case K01:
				pb_flag_g = ATRAS;
				break;
			case K02:
				pb_flag_g = IZQUIERDA;
				break;
			case K03:
				pb_flag_g = ESCUDO;
				break;
			}
		}

		//Lee 5 veces e hizo el promedio
		//El Pit está apagado, manda las señales a 0,0,0
		MecaTeclado_Senales_clean();
		//Mantiene este estado por 1 seg
		delay(900000);
		//Enciende el pit al terminar
		PIT_enable();
		//set bandera cambio state machine
		set_pbn_change_flag();
	} else {
		interrupt_arr[interrupt_num] = k_teclado_ext;
		interrupt_num++;
	}

}

uint8_t get_pbn_flag() {
	return (pb_flag_g);
}

void set_pbn_flag(EXTERNAL_PushButton_SW_name state) {
	pb_flag_g = state;
}

uint8_t get_pbn_change_flag() {
	return (pb_change_flag_g);
}

void clear_pbn_change_flag() {
	pb_change_flag_g = FALSE;
}
void set_pbn_change_flag() {
	pb_change_flag_g = TRUE;
}
