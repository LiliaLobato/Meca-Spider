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
 * @file    Final_MecaAra�a.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "MecaTeclado.h"
#include "GPIO.h"
#include "PushButton.h"
#include "Bits.h"
#include "main.h"

int main(void) {

	MecaTeclado_init();

	//state machine
	set_pbn_flag(NADA);
	clear_ejecucion_num();
	static arana_status arana_state = QUIETO;

	while (1) {
		//Cambio pit para botones
		MecaTeclado_Senales();
		if (TRUE == get_pbn_change_flag()) {
			//revisar buzon
			if (GRABAR == arana_state & QUIETO != FSM_Moore[arana_state].next[get_pbn_flag()] & EJECUTAR != FSM_Moore[arana_state].next[get_pbn_flag()]) {
				printf("guardo secuencia");
			} else {
				arana_state = FSM_Moore[arana_state].next[get_pbn_flag()];
				clear_ejecucion_num();
			}
			clear_pbn_change_flag();
		}
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
				set_ejecucion_num();
			}
			break;
		case EJECUTAR:
			if (FALSE == get_ejecucion_num()) {
				printf("ejecutar\n");
				set_ejecucion_num();
			}
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
			break;
		}
	}

	return 0;
}
