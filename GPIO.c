/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    GPIO.c
 * @brief   Application entry point.
 */
/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#ifndef debug

#include "MK64F12.h"
#include "GPIO.h"
#include "bits.h"

static void (*gpio_C_callback)(void) = 0;
static void (*gpio_A_callback)(void) = 0;
static void (*gpio_B_callback)(void) = 0;
static void (*gpio_D_callback)(void) = 0;
uint8_t sw2_flag_g;
uint8_t sw3_flag_g;

uint8_t get_sw2_flag()
{
	return (sw2_flag_g);
}

void clear_sw2_flag()
{
	sw2_flag_g = FALSE;
}


uint8_t get_sw3_flag()
{
	return (sw3_flag_g);
}

void clear_sw3_flag()
{
	sw3_flag_g = FALSE;
}

void GPIO_callback_init(gpio_port_name_t port_name,void (*handler)(void))
{
	switch(port_name)
	{
	case GPIO_A:
		gpio_A_callback = handler;
		break;
	case GPIO_B:
		gpio_B_callback = handler;
		break;
	case GPIO_C:
		gpio_C_callback = handler;
		break;
	case GPIO_D:
		gpio_D_callback = handler;
		break;
	default:
		break;
	}
}

void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)
				{
				case GPIO_A:/** GPIO A is selected*/
					PORTA->ISFR = 0xFFFFFFFF; /**(Interrupt Status Flag Register) set logic 1 to clear the flag*/
					break;
				case GPIO_B:/** GPIO B is selected*/
					PORTB->ISFR = 0xFFFFFFFF; /**(Interrupt Status Flag Register) set logic 1 to clear the flag*/
					break;
				case GPIO_C:/** GPIO C is selected*/
					PORTC->ISFR = 0xFFFFFFFF; /**(Interrupt Status Flag Register) set logic 1 to clear the flag*/
					break;
				case GPIO_D:/** GPIO D is selected*/
					PORTD->ISFR = 0xFFFFFFFF; /**(Interrupt Status Flag Register) set logic 1 to clear the flag*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					PORTE->ISFR = 0xFFFFFFFF; /**(Interrupt Status Flag Register) set logic 1 to clear the flag*/
					break;
				default:/**If doesn't exist the option*/

				break;
				}
}

void PORTC_IRQHandler() /**Entra cuando ocurre una interrupcion en el puerto C*/
{
	if(gpio_C_callback)
	{
		gpio_C_callback();
	}


	GPIO_clear_interrupt(GPIO_C); //reinicia la interrupción

}

void PORTD_IRQHandler() /**Entra cuando ocurre una interrupcion en el puerto C*/
{
	//PIT_dissable();
	if(gpio_D_callback)
	{
		gpio_D_callback();
	}


	GPIO_clear_interrupt(GPIO_D); //reinicia la interrupción

}

void PORTA_IRQHandler(void)
{
	uint8_t sw3_state = GPIO_read_pin(GPIO_A, bit_4); //Obtiene el estado del sw2

		/**SW2 has logic 0 when its pressed*/
	if((FALSE == sw3_state))//Pregunta quien fue el que interrumpio, para saber que bandera activar
	{
		sw3_flag_g = TRUE; //activa la bandera correspondiente
	}

	if(gpio_A_callback)
	{
		gpio_A_callback();
	}

	GPIO_clear_interrupt(GPIO_A);
}

void PORTB_IRQHandler() /**Entra cuando ocurre una interrupcion en el puerto C*/
{


	if(gpio_B_callback)
	{
		gpio_B_callback();
	}


	GPIO_clear_interrupt(GPIO_B); //reinicia la interrupción

}

uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin,const gpio_pin_control_register_t*  pin_control_register)
{

	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] |= *pin_control_register;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] |= *pin_control_register;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] |= *pin_control_register;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] |= *pin_control_register;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin] |= *pin_control_register;
			break;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t portName, uint32_t data)
{
	switch(portName)
				{
				case GPIO_A:/** GPIO A is selected*/
					GPIOA->PDOR |= data; /**assigns a safe value to PORT A*/
					break;
				case GPIO_B:/** GPIO B is selected*/
					GPIOB->PDOR |= data; /**assigns a safe value to PORT B*/
					break;
				case GPIO_C:/** GPIO C is selected*/
					GPIOC->PDOR |= data; /**assigns a safe value to PORT C*/
					break;
				case GPIO_D:/** GPIO D is selected*/
					GPIOD->PDOR |= data; /**assigns a safe value to PORT D*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					GPIOE->PDOR |= data; /**assigns a safe value to PORT E*/
					break;
				default:/**If doesn't exist the option*/

				break;
				}
}

uint8_t GPIO_read_pin(gpio_port_name_t portName, uint8_t pin)
{
	uint8_t input_value = 0;
		switch(portName)
					{
						case GPIO_A:/** GPIO A is selected*/
							input_value	= ((GPIOA->PDIR) >> pin) & TRUE; /**assigns to input_value the value of pin x of PORT A*/
							break;
						case GPIO_B:/** GPIO B is selected*/
							input_value	= (GPIOB->PDIR >> pin) & TRUE; /**assigns to input_value the value of pin x of PORT B*/
							break;
						case GPIO_C:/** GPIO C is selected*/
							input_value	= ((GPIOC->PDIR) >> pin) & TRUE; /**assigns to input_value the value of pin x of PORT C*/
							break;
						case GPIO_D:/** GPIO D is selected*/
							input_value	= (GPIOD->PDIR >> pin) & TRUE; /**assigns to input_value the value of pin x of PORT D*/
							break;
						case GPIO_E: /** GPIO E is selected*/
							input_value	= (GPIOE->PDIR >> pin) & TRUE; /**assigns to input_value the value of pin x of PORT E*/
							break;
						default:/**If doesn't exist the option*/
							input_value = 0x00000000;
						break;
					}
		return input_value;
}

void GPIO_set_pin(gpio_port_name_t portName, uint8_t pin) /**¿uint8 -> uint32? De lo contrario se necesitaria shifts para abarcar los 4 bytes*/
{
	uint32_t pin32 = shifting(pin);
	switch(portName)
				{
				case GPIO_A:/** GPIO A is selected*/
					GPIOA->PSOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to logic 1*/
					break;
				case GPIO_B:/** GPIO B is selected*/
					GPIOB->PSOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to logic 1*/
					break;
				case GPIO_C:/** GPIO C is selected*/
					GPIOC->PSOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to logic 1*/
					break;
				case GPIO_D:/** GPIO D is selected*/
					GPIOD->PSOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to logic 1*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					GPIOE->PSOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to logic 1*/
					break;
				default:/**If doesn't exist the option*/

				break;
				}
}

void GPIO_clear_pin(gpio_port_name_t portName, uint8_t pin)
{
	uint32_t pin32 = shifting(pin);
	switch(portName)
					{
					case GPIO_A:/** GPIO A is selected*/
						GPIOA->PCOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
						break;
					case GPIO_B:/** GPIO B is selected*/
						GPIOB->PCOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
						break;
					case GPIO_C:/** GPIO C is selected*/
						GPIOC->PCOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
						break;
					case GPIO_D:/** GPIO D is selected*/
						GPIOD->PCOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
						break;
					case GPIO_E: /** GPIO E is selected*/
						GPIOE->PCOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
						break;
					default:/**If doesn't exist the option*/

					break;
					}
}

void GPIO_toogle_pin(gpio_port_name_t portName, uint8_t pin)
{
	uint32_t pin32 = shifting(pin);
	switch(portName)
						{
						case GPIO_A:/** GPIO A is selected*/
							GPIOA->PTOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is set to the inverse of its existing logic state*/
							break;
						case GPIO_B:/** GPIO B is selected*/
							GPIOB->PTOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
							break;
						case GPIO_C:/** GPIO C is selected*/
							GPIOC->PTOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
							break;
						case GPIO_D:/** GPIO D is selected*/
							GPIOD->PTOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
							break;
						case GPIO_E: /** GPIO E is selected*/
							GPIOE->PTOR |= pin32; /**if the pin is logic 1, corresponding bit in PDORn is cleared to logic 0*/
							break;
						default:/**If doesn't exist the option*/

						break;
						}
}

void GPIO_data_direction_port(gpio_port_name_t port_name ,uint32_t direction)
{
	switch(port_name)
			{
			case GPIO_A:/** GPIO A is selected*/
				GPIOA->PDDR |= direction; /**assigns direction to PORT A*/
				break;
			case GPIO_B:/** GPIO B is selected*/
				GPIOB->PDDR |= direction;/**assigns direction to PORT B*/
				break;
			case GPIO_C:/** GPIO C is selected*/
				GPIOC->PDDR |= direction;/**assigns direction to PORT C*/
				break;
			case GPIO_D:/** GPIO D is selected*/
				GPIOD->PDDR |= direction;/**assigns direction to PORT D*/
				break;
			case GPIO_E: /** GPIO E is selected*/
				GPIOE->PDDR |= direction;/**assigns direction to PORT E*/
				break;
			default:/**If doesn't exist the option*/

			break;
			}
}
#endif
void GPIO_data_direction_pin(gpio_port_name_t portName, uint8_t state, uint8_t pin)
{
	uint32_t pin32 = shifting(pin); /**Puts pin´s bits on the last 8 bits (24-31)*/

	if(GPIO_OUTPUT == state)
	{
		switch(portName)
		{
			case GPIO_A:/** GPIO A is selected*/
				GPIOA->PDDR |= pin32; /**assigns INPUT direction to PORT A*/
				break;
			case GPIO_B:/** GPIO B is selected*/
				GPIOB->PDDR |= pin32;/**assigns INPUT direction to PORT B*/
				break;
			case GPIO_C:/** GPIO C is selected*/
				GPIOC->PDDR |= pin32;/**assigns INPUT direction to PORT C*/
				break;
			case GPIO_D:/** GPIO D is selected*/
				GPIOD->PDDR |= pin32;/**assigns INPUT direction to PORT D*/
				break;
			case GPIO_E: /** GPIO E is selected*/
				GPIOE->PDDR |= pin32;/**assigns INPUT direction to PORT E*/
				break;
			default:/**If doesn't exist the option*/
			break;

		}
	}
	else
	{
		switch(portName)
		{
			case GPIO_A:/** GPIO A is selected*/
				GPIOA->PDDR &= ~pin32; /**assigns OUTPUT direction to PORT A*/
				break;
			case GPIO_B:/** GPIO B is selected*/
				GPIOB->PDDR &= ~pin32;/**assigns OUTPUT direction to PORT B*/
				break;
			case GPIO_C:/** GPIO C is selected*/
				GPIOC->PDDR &= ~pin32;/**assigns OUTPUT direction to PORT C*/
				break;
			case GPIO_D:/** GPIO D is selected*/
				GPIOD->PDDR &= ~pin32;/**assigns OUTPUT direction to PORT D*/
				break;
			case GPIO_E: /** GPIO E is selected*/
				GPIOE->PDDR &= ~pin32;/**assigns OUTPUT direction to PORT E*/
				break;
			default:/**If doesn't exist the option*/
			break;
		}
	}


}

uint32_t shifting(uint8_t pin)
{
	uint32_t pin32 = 0x1; /**pin32 with LSB with logical 1*/
	pin32 = pin32 << pin; /**pin32 = 1 shifted pin´s value.*/

	return pin32;
}
