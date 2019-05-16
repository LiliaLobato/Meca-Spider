/*
 * main.h
 *
 *  Created on: 11/05/2019
 *      Author: Lilia Lobato
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include "MecaTeclado.h"
#include "GPIO.h"
#include "PushButton.h"
#include "Bits.h"
#include "RGB_external.h"
#include "UART.h"
#include "Delay.h"
#include "RGB_external.h"
#include "Meccaspider.h"
#include "Delay.h"
#include "NVIC.h"

#define GRABARMAX 10
#define RNDMAX 5

static uint8_t bluetoothValue;

//State Machine/////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum {
	AUTONOMO,
	GRABAR,
	EJECUTAR,
	GUARDIAN,
	ATAQUE,
	PASO_ADELANTE,
	PASO_ATRAS,
	VUELTA_DERECHA,
	VUELTA_IZQUIERDA,
	QUIETO,
	ALEATORIO
} arana_status;

EXTERNAL_PushButton_SW_name grabado_mov[GRABARMAX] = { NADA };
uint8_t grabado_actual = FALSE;
uint8_t ejecutar_actual = FALSE;
uint8_t aleatorio_num = FALSE;
uint8_t ejecucion_num;
uint8_t RGB_COLOR = FALSE;
uint8_t pinzas_encender = FALSE;
uint8_t pinzas_guardian = FALSE;
static arana_status arana_state = QUIETO;

typedef struct {
	arana_status time;
	uint8_t next[10];
} State_t;

const State_t FSM_Moore[11] = { { AUTONOMO, { QUIETO, PASO_ADELANTE, PASO_ATRAS,
		VUELTA_IZQUIERDA, VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN,
		ALEATORIO } },/* modo autonomo */
{ GRABAR, { QUIETO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA, VUELTA_DERECHA,
		QUIETO, QUIETO, EJECUTAR, QUIETO, QUIETO } },/* grabar */
{ EJECUTAR, { QUIETO, QUIETO, QUIETO, QUIETO, QUIETO, QUIETO, QUIETO, QUIETO,
		QUIETO, QUIETO } },/* reproducir*/
{ GUARDIAN, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, QUIETO, ALEATORIO } },/* modo guardian*/
{ ATAQUE, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, QUIETO, QUIETO, GUARDIAN, ALEATORIO } },/* modo ataque */
{ PASO_ADELANTE, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } },/* 1 paso adelante */
{ PASO_ATRAS, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } },/* 1 paso atras */
{ VUELTA_DERECHA, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } },/* 1 vuelta derecha */
{ VUELTA_IZQUIERDA, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } },/* 1 vuelta izquierda */
{ QUIETO, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } },/* no hace nada */
{ ALEATORIO, { AUTONOMO, PASO_ADELANTE, PASO_ATRAS, VUELTA_IZQUIERDA,
		VUELTA_DERECHA, GRABAR, ATAQUE, QUIETO, GUARDIAN, QUIETO } }/* movimientos aleatorios */
};

uint8_t get_ejecucion_num() {
	return ejecucion_num;
}

void set_ejecucion_num() {
	ejecucion_num = TRUE;
}

void clear_ejecucion_num() {
	ejecucion_num = FALSE;
}

void guardarSecuancia() {
	RGB_external_clear_LEDs();
	RGB_external_set_LED(RGB_CYAN, LED_1);
	RGB_external_set_LED(RGB_CYAN, LED_2);
	RGB_external_set_LED(RGB_CYAN, LED_3);
	RGB_external_set_LED(RGB_CYAN, LED_4);
	delay(900000);
	if (FALSE == UART4_isConnected()) {
		if (GRABARMAX == grabado_actual) {
			arana_state = EJECUTAR;
		} else {
			grabado_mov[grabado_actual] = get_pbn_flag();
			grabado_actual++;
		}
	} else {
		if (GRABARMAX == grabado_actual) {
			arana_state = EJECUTAR;
		} else {
			grabado_mov[grabado_actual] = bluetoothValue;
			grabado_actual++;
		}
	}
	RGB_external_clear_LEDs();
}

void limpiarSecuencia() {
	for (uint8_t i = FALSE; i < GRABARMAX; i++) {
		grabado_mov[grabado_actual] = NADA;
	}
	ejecutar_actual = FALSE;
	grabado_actual = FALSE;
}

void aleatorio() {
	if (RNDMAX > aleatorio_num) {
		if (TRUE == pinzas_encender) {
			RGB_COLOR = RGB_RED;
		} else {
			Meccaspider_Stop_atack();
			RGB_COLOR = RGB_GREEN;
		}
		if (TRUE == pinzas_guardian) {
			RGB_COLOR = RGB_BLUE;
		}

		switch (rand() % 4 + 1) {
		case ADELANTE:
			if (TRUE == pinzas_guardian) {
				Meccaspider_Stop_atack();
			}
			if (TRUE == pinzas_encender) {
				Meccaspider_atack();
			}
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_COLOR, LED_2);
			RGB_external_set_LED(RGB_COLOR, LED_3);
			Meccaspider_move(FORWARD);
			RGB_external_clear_LEDs();
			break;
		case ATRAS:
			if (TRUE == pinzas_guardian) {
				Meccaspider_Stop_atack();
			}
			if (TRUE == pinzas_encender) {
				Meccaspider_atack();
			}
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_COLOR, LED_1);
			RGB_external_set_LED(RGB_COLOR, LED_4);
			Meccaspider_move(BACKWARD);
			RGB_external_clear_LEDs();
			break;
		case DERECHA:
			if (TRUE == pinzas_guardian) {
				Meccaspider_atack();
			}
			if (TRUE == pinzas_encender) {
				Meccaspider_atack();
			}
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_COLOR, LED_3);
			RGB_external_set_LED(RGB_COLOR, LED_4);
			Meccaspider_move(RIGHT);
			RGB_external_clear_LEDs();
			break;
		case IZQUIERDA:
			if (TRUE == pinzas_guardian) {
				Meccaspider_atack();
			}
			if (TRUE == pinzas_encender) {
				Meccaspider_atack();
			}
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_COLOR, LED_1);
			RGB_external_set_LED(RGB_COLOR, LED_2);
			Meccaspider_move(LEFT);
			RGB_external_clear_LEDs();
			break;
		default:
			break;
		}
		aleatorio_num++;
	} else {
		aleatorio_num = FALSE;
		arana_state = QUIETO;
		clear_ejecucion_num();
	}

}

void ejecutarSecuencia() {
	if ((FALSE != grabado_actual) & (ejecutar_actual < grabado_actual)) {
		switch (grabado_mov[ejecutar_actual]) {
		case ADELANTE:
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_PURPLE, LED_1);
			RGB_external_set_LED(RGB_PURPLE, LED_4);
			Meccaspider_move(FORWARD);
			RGB_external_clear_LEDs();
			break;
		case ATRAS:
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_PURPLE, LED_2);
			RGB_external_set_LED(RGB_PURPLE, LED_3);
			Meccaspider_move(BACKWARD);
			RGB_external_clear_LEDs();
			break;
		case DERECHA:
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_PURPLE, LED_3);
			RGB_external_set_LED(RGB_PURPLE, LED_4);
			Meccaspider_move(RIGHT);
			RGB_external_clear_LEDs();
			break;
		case IZQUIERDA:
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_PURPLE, LED_1);
			RGB_external_set_LED(RGB_PURPLE, LED_2);
			Meccaspider_move(LEFT);
			RGB_external_clear_LEDs();
			break;
		default:
			break;
		}
		ejecutar_actual++;
	} else {
		arana_state = QUIETO;
		clear_ejecucion_num();
		limpiarSecuencia();
	}
}

void StateMachine_cambio() {
	if (FALSE == UART4_isConnected()) {
		if (TRUE == get_pbn_change_flag()) {
			//revisar buzon
			if ((GRABAR == arana_state)
					& (QUIETO != FSM_Moore[arana_state].next[get_pbn_flag()])
					& (EJECUTAR != FSM_Moore[arana_state].next[get_pbn_flag()])) {
				guardarSecuancia();
			} else if (ALEATORIO == arana_state) {
				pinzas_encender = FALSE;
				pinzas_guardian = FALSE;
			} else {
				arana_state = FSM_Moore[arana_state].next[get_pbn_flag()];
				clear_ejecucion_num();
			}
			clear_pbn_change_flag();
		}
	} else {
		//Lecutura Bluetooth
		if (get_flag_4()) {
			/**clear the reception flag*/
			clear_flag_4();
			/**Sends to the PCA the received data in the mailbox*/
			bluetoothValue = UART_mailbox_4_decode();

			//revisar buzon
			if ((GRABAR == arana_state)
					& (QUIETO != FSM_Moore[arana_state].next[bluetoothValue])
					& (EJECUTAR != FSM_Moore[arana_state].next[bluetoothValue])) {
				guardarSecuancia();
			} else if (ALEATORIO == arana_state) {
				pinzas_encender = FALSE;
				pinzas_guardian = FALSE;
			} else {
				arana_state = FSM_Moore[arana_state].next[bluetoothValue];
				clear_ejecucion_num();
			}
			/**clear the reception flag*/
			clear_flag_4();

		}
	}
}

void StateMachine_currentState() {
	switch (arana_state) {
	case AUTONOMO:
		if (FALSE == get_ejecucion_num()) {
			set_ejecucion_num();
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case GRABAR:
		if (FALSE == get_ejecucion_num()) {
			limpiarSecuencia();
			set_ejecucion_num();
		}
		break;
	case EJECUTAR:
		if (FALSE == get_ejecucion_num()) {
			set_ejecucion_num();
		}
		ejecutarSecuencia();
		break;
	case GUARDIAN:
		if (FALSE == get_ejecucion_num()) {
			set_ejecucion_num();
			pinzas_guardian = TRUE;
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case ATAQUE:
		if (FALSE == get_ejecucion_num()) {
			set_ejecucion_num();
			pinzas_encender = TRUE;
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case PASO_ADELANTE:
		if (FALSE == get_ejecucion_num()) {
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_YELLOW, LED_2);
			RGB_external_set_LED(RGB_YELLOW, LED_3);
			Meccaspider_move(FORWARD);
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case PASO_ATRAS:
		if (FALSE == get_ejecucion_num()) {
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_YELLOW, LED_1);
			RGB_external_set_LED(RGB_YELLOW, LED_4);
			Meccaspider_move(BACKWARD);
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case VUELTA_DERECHA:
		if (FALSE == get_ejecucion_num()) {
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_YELLOW, LED_3);
			RGB_external_set_LED(RGB_YELLOW, LED_4);
			Meccaspider_move(RIGHT);
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case VUELTA_IZQUIERDA:
		if (FALSE == get_ejecucion_num()) {
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_YELLOW, LED_1);
			RGB_external_set_LED(RGB_YELLOW, LED_2);
			Meccaspider_move(LEFT);
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case QUIETO:
		if (FALSE == get_ejecucion_num()) {
			Meccaspider_stop();
			RGB_external_clear_LEDs();
			RGB_external_set_LED(RGB_WHITE, LED_1);
			RGB_external_set_LED(RGB_WHITE, LED_2);
			RGB_external_set_LED(RGB_WHITE, LED_3);
			RGB_external_set_LED(RGB_WHITE, LED_4);
			set_ejecucion_num();
			pinzas_encender = FALSE;
			pinzas_guardian = FALSE;
		}
		break;
	case ALEATORIO:
		if (FALSE == get_ejecucion_num()) {
			set_ejecucion_num();
		}
		aleatorio();
		break;
	}
}

#endif /* MAIN_H_ */
