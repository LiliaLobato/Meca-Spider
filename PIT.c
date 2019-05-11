/*
 * Pit.c
 *
 *  Created on: 07/03/2019
 *      Author: Lilia Lobato
 */

#include "PIT.h"
#include "Bits.h"
#include "MK64F12.h"
#include "GPIO.h"

static uint8_t g_pit0_intr_flag = FALSE;
static uint8_t g_pit1_intr_flag = FALSE;
static uint8_t g_pit2_intr_flag = FALSE;
static uint8_t g_pit3_intr_flag = FALSE;

static void (*pit_0_callback)(void) = 0;
static void (*pit_1_callback)(void) = 0;
static void (*pit_2_callback)(void) = 0;

void PIT_callback_init(PIT_timer_t pit_timer,void (*handler)(void))
{
	switch(pit_timer)
	{
	case PIT_0:
		pit_0_callback = handler;
		break;
	case PIT_1:
		pit_1_callback = handler;
		break;
	case PIT_2:
		pit_2_callback = handler;
		break;
	default:
		break;
	}
}

void PIT0_IRQHandler() {
	volatile uint32_t dummyRead;
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[0].TCTRL;//read control register for clear PIT flag, this is silicon bug

	if(pit_0_callback)
	{
		pit_0_callback();
	}

	g_pit0_intr_flag = TRUE;


}

void PIT1_IRQHandler() {
	GPIO_toogle_pin(GPIO_D,0);
	volatile uint32_t dummyRead;
	PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[1].TCTRL;//read control register for clear PIT flag, this is silicon bug

	if(pit_1_callback)
	{
		pit_1_callback();
	}

	g_pit1_intr_flag = TRUE;
}
void PIT2_IRQHandler() {
	volatile uint32_t dummyRead;
	PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[2].TCTRL;//read control register for clear PIT flag, this is silicon bug

	if(pit_2_callback)
	{
		pit_2_callback();
	}

	g_pit2_intr_flag = TRUE;
}
void PIT3_IRQHandler() {
	volatile uint32_t dummyRead;
	PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[3].TCTRL;//read control register for clear PIT flag, this is silicon bug
	g_pit3_intr_flag = TRUE;
}

void PIT_delay(PIT_timer_t pit_timer, float system_clock, float delay) {
	uint32_t LDVAL = 0;
	float clock_period = 0.0F;
	system_clock = system_clock / 2;
	clock_period = (1 / system_clock);
	LDVAL = (uint32_t) (delay / clock_period);
	LDVAL = LDVAL - 1;

	switch (pit_timer) {
	case PIT_0:
		PIT->CHANNEL[0].LDVAL = LDVAL;
		PIT_enable_interrupt_0(PIT_0);
		break;
	case PIT_1:
		PIT->CHANNEL[1].LDVAL = LDVAL;
		PIT_enable_interrupt_1(PIT_1);
		break;
	case PIT_2:
		PIT->CHANNEL[2].LDVAL = LDVAL;
		PIT_enable_interrupt_2(PIT_2);
		break;
	case PIT_3:
		PIT->CHANNEL[3].LDVAL = LDVAL;
		PIT_enable_interrupt_3(PIT_3);
		break;
	default:
		break;
	}

}

void PIT_enable(void) {
	PIT->MCR |= PIT_MCR_FRZ_MASK;
	PIT->MCR &= ~PIT_MCR_MDIS_MASK; /* Enable PIT*/
}

void PIT_disable(void)
{
	PIT->MCR |= PIT_MCR_MDIS_MASK; /* Disable PIT*/
}

void PIT_enable_interrupt_0(PIT_timer_t pit) {
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;//enables PIT timer interrupt
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;	//enables timer0
}
void PIT_enable_interrupt_1(PIT_timer_t pit) {
	PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK;//enables PIT timer interrupt
	PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;	//enables timer0
}
void PIT_enable_interrupt_2(PIT_timer_t pit) {
	PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TIE_MASK;//enables PIT timer interrupt
	PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TEN_MASK;	//enables timer0
}
void PIT_enable_interrupt_3(PIT_timer_t pit) {
	PIT->CHANNEL[3].TCTRL |= PIT_TCTRL_TIE_MASK;//enables PIT timer interrupt
	PIT->CHANNEL[3].TCTRL |= PIT_TCTRL_TEN_MASK;	//enables timer0
}

void PIT_clock_gating(void) {
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
}

uint8_t PIT_get0_interrupt_flag_status(void) {
	return (g_pit0_intr_flag);
}
uint8_t PIT_get1_interrupt_flag_status(void) {
	return (g_pit1_intr_flag);
}
uint8_t PIT_get2_interrupt_flag_status(void) {
	return (g_pit2_intr_flag);
}
uint8_t PIT_get3_interrupt_flag_status(void) {
	return (g_pit3_intr_flag);
}

void PIT_clear0_interrupt_flag(void) {
	g_pit0_intr_flag = FALSE;
}
void PIT_clear1_interrupt_flag(void) {
	g_pit1_intr_flag = FALSE;
}
void PIT_clear2_interrupt_flag(void) {
	g_pit2_intr_flag = FALSE;
}
void PIT_clear3_interrupt_flag(void) {
	g_pit3_intr_flag = FALSE;
}
