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
#include "UART.h"
#include "Delay.h"
#include "NVIC.h"

#define GRABARMAX 10

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
uint8_t ejecucion_num;
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

}

void limpiarSecuencia() {
	for (uint8_t i = FALSE; i < GRABARMAX; i++) {
		grabado_mov[grabado_actual] = NADA;
	}
	ejecutar_actual = FALSE;
	grabado_actual = FALSE;
}

void aleatorio() {
	switch (rand() % 4 + 1) {
	case ADELANTE:
		printf("adelante\n");
		break;
	case ATRAS:
		printf("atras\n");
		break;
	case DERECHA:
		printf("derecha\n");
		break;
	case IZQUIERDA:
		printf("izquierda\n");
		break;
	default:
		break;
	}

}

void ejecutarSecuencia() {
	if ((FALSE != grabado_actual) & (ejecutar_actual < grabado_actual)) {
		switch (grabado_mov[ejecutar_actual]) {
		case ADELANTE:
			printf("adelante\n");
			break;
		case ATRAS:
			printf("atras\n");
			break;
		case DERECHA:
			printf("derecha\n");
			break;
		case IZQUIERDA:
			printf("izquierda\n");
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
				printf("guardo secuencia\n");
				guardarSecuancia();
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
			//UART_put_char(UART_4, get_mailbox_4());
			bluetoothValue = UART_mailbox_4_decode();

			//revisar buzon
			if ((GRABAR == arana_state)
					& (QUIETO != FSM_Moore[arana_state].next[bluetoothValue])
					& (EJECUTAR != FSM_Moore[arana_state].next[bluetoothValue])) {
				printf("guardo secuencia\n");
				guardarSecuancia();
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
			printf("modo autonomo\n");
			set_ejecucion_num();
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case GRABAR:
		if (FALSE == get_ejecucion_num()) {
			printf("grabar\n");
			limpiarSecuencia();
			set_ejecucion_num();
		}
		break;
	case EJECUTAR:
		if (FALSE == get_ejecucion_num()) {
			printf("ejecutar\n");
			set_ejecucion_num();
		}
		ejecutarSecuencia();
		break;
	case GUARDIAN:
		if (FALSE == get_ejecucion_num()) {
			printf("modo guardian\n");
			set_ejecucion_num();
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case ATAQUE:
		if (FALSE == get_ejecucion_num()) {
			printf("modo ataque\n");
			set_ejecucion_num();
		}
		arana_state = ALEATORIO;
		clear_ejecucion_num();
		break;
	case PASO_ADELANTE:
		if (FALSE == get_ejecucion_num()) {
			printf("adelante\n");
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case PASO_ATRAS:
		if (FALSE == get_ejecucion_num()) {
			printf("atras\n");
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case VUELTA_DERECHA:
		if (FALSE == get_ejecucion_num()) {
			printf("vuelta derecha\n");
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case VUELTA_IZQUIERDA:
		if (FALSE == get_ejecucion_num()) {
			printf("vuelta izquierda\n");
			set_ejecucion_num();
		}
		arana_state = QUIETO;
		clear_ejecucion_num();
		break;
	case QUIETO:
		if (FALSE == get_ejecucion_num()) {
			printf("quieto\n");
			set_ejecucion_num();
		}
		break;
	case ALEATORIO:
		if (FALSE == get_ejecucion_num()) {
			printf("modo aleatorio\n");
			set_ejecucion_num();
		}
		aleatorio();
		break;
	}
}

#endif /* MAIN_H_ */
