/*
 * Pit.h
 *
 *  Created on: 07/03/2019
 *      Author: Lilia Lobato
 */

#ifndef PIT_H_
#define PIT_H_

#include "stdint.h"
#include <PIT.h>

typedef enum {PIT_0,PIT_1,PIT_2,PIT_3} PIT_timer_t;

void PIT0_IRQHandler();
void PIT1_IRQHandler();
void PIT2_IRQHandler();
void PIT3_IRQHandler();

void PIT_delay(PIT_timer_t pit_timer, float system_clock, float delay);
void PIT_enable(void);

void PIT_enable_interrupt_0(PIT_timer_t pit);
void PIT_enable_interrupt_1(PIT_timer_t pit);
void PIT_enable_interrupt_2(PIT_timer_t pit);
void PIT_enable_interrupt_3(PIT_timer_t pit);

void PIT_clock_gating(void);

uint8_t PIT_get0_interrupt_flag_status(void);
uint8_t PIT_get1_interrupt_flag_status(void);
uint8_t PIT_get2_interrupt_flag_status(void);
uint8_t PIT_get3_interrupt_flag_status(void);

void PIT_clear0_interrupt_flag(void);
void PIT_clear1_interrupt_flag(void);
void PIT_clear2_interrupt_flag(void);
void PIT_clear3_interrupt_flag(void);

#endif /* PIT_H_ */
