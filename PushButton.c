/*
 * PushButton.c
 *
 *  Created on: 24/02/2019
 *      Author: Lilia Lobato
 */

#include "PushButton.h"
#include "MK64F12.h"
#include "GPIO.h"

uint8_t pb0_flag_g = FALSE;
uint8_t pb1_flag_g = FALSE;
uint8_t pb2_flag_g = FALSE;
uint8_t pb3_flag_g = FALSE;
uint8_t pb4_flag_g = FALSE;
uint8_t pb5_flag_g = FALSE;
uint8_t pb6_flag_g = FALSE;

gpio_pin_control_register_t input_intr_config = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;	//Configuración del GPIO

void PushButton_sw2_config(void) {
	GPIO_clock_gating(GPIO_C);								  //Activa el puerto C
	GPIO_pin_control_register(GPIO_C, 6, &input_intr_config); //Configuracion del GPIO C para el sw2
	GPIO_write_port(GPIO_C, ~(SW2));						  //Escribe un valor seguro
	GPIO_data_direction_port(GPIO_C, ~(SW2));				  //Configura el puerto del sw2 del GPIO C como Input
}

void PushButton_sw3_config(void) {
	GPIO_clock_gating(GPIO_A);								  //Activa el puerto A
	GPIO_pin_control_register(GPIO_A, 4, &input_intr_config); //Configuracion del GPIO A para el sw3
	PORTA->PCR[4] = (GPIO_MUX1 | GPIO_PE | GPIO_PS |INTR_FALLING_EDGE);
	GPIO_write_port(GPIO_A, ~(SW3));						  //Escribe un valor seguro
	GPIO_data_direction_port(GPIO_A, ~(SW3));				  //Configura el puerto del sw3 del GPIO A como Input
}

void PushButton_external_config(gpio_port_name_t GPIOx, BitsType pin, uint32_t MASK)
{
GPIO_clock_gating(GPIOx);								  //Activa el puerto x
GPIO_pin_control_register(GPIO_B, pin, &input_intr_config); //Configuracion del GPIO x para el pin n
GPIO_write_port(GPIOx, ~(MASK));						  //Escribe un valor seguro pin n
GPIO_data_direction_port(GPIOx, ~(MASK));				  //Configura el puerto del GPIO C como Input
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

void PushButton_external_handler(void)
{
	uint8_t pb0_state = GPIO_read_pin(GPIO_B, bit_2); //Obtiene el estado del B0
	uint8_t pb1_state = GPIO_read_pin(GPIO_B, bit_3); //Obtiene el estado del B1
	uint8_t pb2_state = GPIO_read_pin(GPIO_B, bit_10); //Obtiene el estado del B2
	uint8_t pb3_state = GPIO_read_pin(GPIO_B, bit_11); //Obtiene el estado del B3
	uint8_t pb4_state = GPIO_read_pin(GPIO_B, bit_9); //Obtiene el estado del B4
	uint8_t pb5_state = GPIO_read_pin(GPIO_B, bit_19); //Obtiene el estado del B5
	uint8_t pb6_state = GPIO_read_pin(GPIO_B, bit_18); //Obtiene el estado del B6

		/**PB has logic 0 when its pressed*/
	if((FALSE == pb0_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb0_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb1_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb1_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb2_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb2_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb3_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb3_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb4_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb4_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb5_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb5_flag_g = TRUE; //activa la bandera correspondiente
	}

	if((FALSE == pb6_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		pb6_flag_g = TRUE; //activa la bandera correspondiente
	}
}

