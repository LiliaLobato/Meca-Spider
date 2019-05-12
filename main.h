/*
 * main.h
 *
 *  Created on: 11/05/2019
 *      Author: Lilia Lobato
 */

#ifndef MAIN_H_
#define MAIN_H_

uint8_t ejecucion_num;

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

uint8_t get_ejecucion_num(){
	return ejecucion_num;
}

void set_ejecucion_num(){
	ejecucion_num = TRUE;
}

void clear_ejecucion_num(){
	ejecucion_num = FALSE;
}


#endif /* MAIN_H_ */
