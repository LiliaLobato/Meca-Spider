/*
 * UART.c
 *
 *  Created on: 07/04/2019
 *      Author: best buy
 */

#include "UART.h"
#include "GPIO.h"
#include "Delay.h"
#include "NVIC.h"
#include "PushButton.h"

gpio_pin_control_register_t input_configu = GPIO_MUX1;
uart_mail_box_t g_mail_box_uart_0 = { 0, 0 };
uart_mail_box_t g_mail_box_uart_1 = { 0, 0 };
uart_mail_box_t g_mail_box_uart_2 = { 0, 0 };
uart_mail_box_t g_mail_box_uart_3 = { 0, 0 };
uart_mail_box_t g_mail_box_uart_4 = { 0, 0 };
uart_mail_box_t g_mail_box_uart_5 = { 0, 0 };

void UART_bluetooth_init() {
	/**Enables the clock of PortB in order to configures TX and RX of UART peripheral*/
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	/**Configures the pin control register of pin16 in PortB as UART RX*/
	PORTC->PCR[14] = PORT_PCR_MUX(3);
	/**Configures the pin control register of pin16 in PortB as UART TX*/
	PORTC->PCR[15] = PORT_PCR_MUX(3);
	/**Configures UART 0 to transmit/receive at 11520 bauds with a 21 MHz of clock core*/
	UART_init(UART_4, 21000000, BD_9600);
	/**Enables the UART 0 interrupt*/
	UART_interrupt_enable(UART_4);
	/**Enables the UART 0 interrupt in the NVIC*/
	NVIC_enable_interrupt_and_priotity(UART4_IRQ, PRIORITY_5);
}

void UART_init(uart_channel_t uart_channel, uint32_t system_clk,
		uart_baud_rate_t baud_rate) {
	//Variables para Baud Rate
	uint8_t SBRH = 0;
	uint8_t SBRL = 0;
	uint8_t BRFA = 0;
	uint8_t brfa = 0;

	//Cálculo de SBR
	float SBR = system_clk / (16.0 * ((float) (baud_rate)));
	float BFRD = SBR;

	//Decimal part
	while (1 < BFRD) {
		BFRD = BFRD - 0.1F;
	}

	//BFRA
	BRFA = (uint8_t) (BFRD * 32.0F);

	//High and Low SBR
	SBRL = (uint8_t) (((uint16_t) SBR) & 0x007FU);
	SBRH = (uint8_t) ((((uint16_t) SBR) & 0x1F00U) >> 7);

	switch (uart_channel) {
	case UART_0:
		SIM->SCGC4 |= SIM_SCGC4_UART0_MASK; //PIN 10 OF SCGC4 IS SET
		UART0->C2 &= ~UART_C2_TE_MASK; //TRANSMITTER DISABLED
		UART0->C2 &= ~UART_C2_RE_MASK; //RECIEVER DISABLED

		UART0->BDH = SBRH & UART_BDH_SBR_MASK; //Copiar los bits UART baud rate [12:8] a los bits SRB del registro UART0_BDH
		UART0->BDL = SBRL; //Copiar los bits UART baud rate [7:0] al registro UART0_BDL

		UART0->C4 |= BRFA & UART_C4_BRFA_MASK; //Copiar brfa al registro UART0_C4[4:0], los cuales corresponden al campo brfa

		UART0->C2 |= UART_C2_TE_MASK; //TRANSMITTER ENABLED
		UART0->C2 |= UART_C2_RE_MASK; //RECIEVER ENABLED
		break;
	case UART_1:
		SIM->SCGC4 |= SIM_SCGC4_UART1_MASK; //PIN 10 OF SCGC4 IS SET
		UART1->C2 &= ~UART_C2_TE_MASK; //TRANSMITTER DISABLED
		UART1->C2 &= ~UART_C2_RE_MASK; //RECIEVER DISABLED

		UART1->BDH = SBRH & UART_BDH_SBR_MASK; //Copiar los bits UART baud rate [12:8] a los bits SRB del registro UART0_BDH
		UART1->BDL = SBRL; //Copiar los bits UART baud rate [7:0] al registro UART0_BDL

		UART1->C4 |= BRFA & UART_C4_BRFA_MASK; //Copiar brfa al registro UART0_C4[4:0], los cuales corresponden al campo brfa

		UART1->C2 |= UART_C2_TE_MASK; //TRANSMITTER ENABLED
		UART1->C2 |= UART_C2_RE_MASK; //RECIEVER ENABLED
		break;
	case UART_2:
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //PIN 10 OF SCGC4 IS SET
		UART2->C2 &= ~UART_C2_TE_MASK; //TRANSMITTER DISABLED
		UART2->C2 &= ~UART_C2_RE_MASK; //RECIEVER DISABLED

		UART2->BDH = SBRH & UART_BDH_SBR_MASK; //Copiar los bits UART baud rate [12:8] a los bits SRB del registro UART0_BDH
		UART2->BDL = SBRL; //Copiar los bits UART baud rate [7:0] al registro UART0_BDL

		UART2->C4 |= BRFA & UART_C4_BRFA_MASK; //Copiar brfa al registro UART0_C4[4:0], los cuales corresponden al campo brfa

		UART2->C2 |= UART_C2_TE_MASK; //TRANSMITTER ENABLED
		UART2->C2 |= UART_C2_RE_MASK; //RECIEVER ENABLED
		break;
	case UART_3:
		SIM->SCGC4 |= SIM_SCGC4_UART3_MASK; //PIN 10 OF SCGC4 IS SET
		UART3->C2 &= ~UART_C2_TE_MASK; //TRANSMITTER DISABLED
		UART3->C2 &= ~UART_C2_RE_MASK; //RECIEVER DISABLED

		UART3->BDH = SBRH & UART_BDH_SBR_MASK; //Copiar los bits UART baud rate [12:8] a los bits SRB del registro UART0_BDH
		UART3->BDL = SBRL; //Copiar los bits UART baud rate [7:0] al registro UART0_BDL

		UART3->C4 |= BRFA & UART_C4_BRFA_MASK; //Copiar brfa al registro UART0_C4[4:0], los cuales corresponden al campo brfa

		UART3->C2 |= UART_C2_TE_MASK; //TRANSMITTER ENABLED
		UART3->C2 |= UART_C2_RE_MASK; //RECIEVER ENABLED
		break;
	case UART_4:
		GPIO_clock_gating(GPIO_A);
		GPIO_pin_control_register(GPIO_A, bit_1, &input_configu);
		GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_1);

		baud_rate = (uint16_t) ((system_clk) / (baud_rate * 16));
		brfa = (uint8_t) (2 * ((system_clk) / (baud_rate)) - (baud_rate * 32));
		brfa &= 0x1F;

		SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
		UART4->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		//UART4_C1 = 0;
		UART4->BDH |= (((baud_rate & 0x1F00) >> 8));
		UART4->BDL = (uint8_t) (baud_rate & 0x00FF);
		UART4->C4 |= brfa;
		UART4->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_5:
		SIM->SCGC1 |= SIM_SCGC1_UART5_MASK; //PIN 10 OF SCGC4 IS SET
		UART5->C2 &= ~UART_C2_TE_MASK; //TRANSMITTER DISABLED
		UART5->C2 &= ~UART_C2_RE_MASK; //RECIEVER DISABLED

		UART5->BDH = SBRH & UART_BDH_SBR_MASK; //Copiar los bits UART baud rate [12:8] a los bits SRB del registro UART0_BDH
		UART5->BDL = SBRL; //Copiar los bits UART baud rate [7:0] al registro UART0_BDL

		UART5->C4 |= BRFA & UART_C4_BRFA_MASK; //Copiar brfa al registro UART0_C4[4:0], los cuales corresponden al campo brfa

		UART5->C2 |= UART_C2_TE_MASK; //TRANSMITTER ENABLED
		UART5->C2 |= UART_C2_RE_MASK; //RECIEVER ENABLED
		break;
	}

}

void UART_interrupt_enable(uart_channel_t uart_channel) {
	switch (uart_channel) {
	case UART_0:
		UART0->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_1:
		UART1->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_2:
		UART2->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_3:
		UART3->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_4:
		UART4->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_5:
		UART5->C2 |= UART_C2_RIE_MASK;
		break;
	}
}
/**
 UART_put_char
 La trasmisión de un carácter se realiza al escribir directamente en el registro UART0_D, pero antes de realizar esta
 acción hay que asegurarse que la UART no este trasmitiendo, esto se realiza al verificar que la bandera TDRE del
 registro de UART_S1 este en cero.
 */
void UART_put_char(uart_channel_t uart_channel, uint8_t character) {
	switch (uart_channel) {
	case UART_0:
		while (!(UART0->S1 & UART_S1_TDRE_MASK))
			;
		UART0->D = character;
		break;
	case UART_1:
		while (!(UART1->S1 & UART_S1_TDRE_MASK))
			;
		UART1->D = character;
		break;
	case UART_2:
		while (!(UART2->S1 & UART_S1_TDRE_MASK))
			;
		UART2->D = character;
		break;
	case UART_3:
		while (!(UART3->S1 & UART_S1_TDRE_MASK))
			;
		UART3->D = character;
		break;
	case UART_4:
		while (!(UART4->S1 & UART_S1_TDRE_MASK))
			;
		UART4->D = character;
		break;
	case UART_5:
		while (!(UART5->S1 & UART_S1_TDRE_MASK))
			;
		UART5->D = character;
		break;
	}

}
/**
 UART_put_char
 La trasmisión de un carácter se realiza al escribir directamente en el registro UART0_D, pero antes de realizar esta
 acción hay que asegurarse que la UART no este trasmitiendo, esto se realiza al verificar que la bandera TDRE del
 registro de UART_S1 este en cero.
 */

void UART_put_string(uart_channel_t uart_channel, int8_t* string) {
	while ('\0' != *string)
		UART_put_char(uart_channel, *string++);
}

/**
 UART0_RX_TX_IRQHandler
 Por seguridad hay que verificar que primero se terminó de recibir la información por el puesto serie, esto se realiza
 chekando que la bandera RDRF sea cero, después de esto, se captura el valor recibido en UART0_D en el mailbox, y
 se activa la bandera de este.
 */

void UART0_RX_TX_IRQHandler(uart_channel_t uart_channel) {

	while (!(UART0->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_0.flag = TRUE;
	g_mail_box_uart_0.mailBox = UART0->D;
//revisar que RDRF sea cero
	//caputar el valor recibido en UART_D
	//activar la bandera
}

void UART1_RX_TX_IRQHandler(uart_channel_t uart_channel) {
	while (!(UART1->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_1.flag = TRUE;
	g_mail_box_uart_1.mailBox = UART1->D;
}
void UART2_RX_TX_IRQHandler(uart_channel_t uart_channel) {
	while (!(UART2->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_2.flag = TRUE;
	g_mail_box_uart_2.mailBox = UART2->D;
}
void UART3_RX_TX_IRQHandler(uart_channel_t uart_channel) {
	while (!(UART3->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_3.flag = TRUE;
	g_mail_box_uart_3.mailBox = UART3->D;
}
void UART4_RX_TX_IRQHandler(uart_channel_t uart_channel) {
	while (!(UART4->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_4.flag = TRUE;
	g_mail_box_uart_4.mailBox = UART4->D;
}

void UART5_RX_TX_IRQHandler(uart_channel_t uart_channel) {
	while (!(UART5->S1 & UART_S1_RDRF_MASK))
		;
	g_mail_box_uart_5.flag = TRUE;
	g_mail_box_uart_5.mailBox = UART5->D;
}

void UART_getString(uint8_t* buffer) {
	uint8_t i = 0;
	do {
		while (!(UART4->S1 & UART_S1_RDRF_MASK))
			;
		buffer[i] = UART4->D;
		i++;
	} while (UART4->D != '\r');
}

uint8_t UART4_isConnected(void) {
	return GPIO_read_pin(GPIO_A, bit_1);
}

uint8_t get_mailbox_4() {
	return g_mail_box_uart_4.mailBox;
}

uint8_t get_flag_4() {
	return g_mail_box_uart_4.flag;
}

void clear_flag_4() {
	g_mail_box_uart_4.flag = FALSE;
}

uint8_t UART_mailbox_4_decode() {
	switch (g_mail_box_uart_4.mailBox) {
	case 'U': //up //
		return ADELANTE;
		break;
	case 'D': //down
		return ATRAS;
		break;
	case 'L': //left
		return IZQUIERDA;
		break;
	case 'R': //right
		return DERECHA;
		break;
	case 'B': //square
		return DADOS;
		break;
	case 'N': //triangle
		return TRIANGULO;
		break;
	case 'G': //circle
		return CIRCULO;
		break;
	case 'A': //green
		return CENTRO;
		break;
	case 'Y': //yellow
		return ESCUDO;
		break;
	default:
		return NADA;
		break;
	}
}
