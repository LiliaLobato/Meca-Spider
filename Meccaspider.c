/*
 * Meccaspider.c
 *
 *  Created on: 09/05/2019
 *      Author: best buy
 */
#include "Meccaspider.h"


void Meccaspider_init(void)
{
   	/*Initial configuration for left motors*/
   	PIT_clock_gating();
   	NVIC_enable_interrupt_and_priotity(PORTC_IRQ,PRIORITY_5);//Enables the ISR for SW2.
   	NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_5);//Enables the ISR for SW3.
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ,PRIORITY_6);//Enables the ISR for PIT0.
   	NVIC_global_enable_interrupts;
	/**Configuration function for PWM*/
	FlexTimer_PWM_CH2_Init();
	FlexTimer_PWM_CH1_Init();

	GPIO_clock_gating(GPIO_C);
	GPIO_clock_gating(GPIO_B);
	gpio_pin_control_register_t output_motor_control = GPIO_MUX1;	//Configuración del GPIO
	gpio_pin_control_register_t input_motor_calibration = GPIO_MUX1 | INTR_LOGIC0;	//Configuración del GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS, &output_motor_control); //PTC17 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG, &output_motor_control); //PTC16 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS, &output_motor_control); //PTC11 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG, &output_motor_control); //PTC10 as GPIO
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_RIGHT_MOTOR_CALIBRATION, &input_motor_calibration); //PTB10 as GPIO INPUT
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_LEFT_MOTOR_CALIBRATION, &input_motor_calibration); //PTB11 as GPIO INPUT

	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_RIGHT_MOTOR_POS);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_RIGHT_MOTOR_NEG);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_LEFT_MOTOR_POS);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_LEFT_MOTOR_NEG);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_RIGHT_MOTOR_CALIBRATION);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_LEFT_MOTOR_CALIBRATION);

}

void Meccaspider_left_motor_one_step(void)
{
   	gpio_pin_control_register_t left_motor_PCR_config = PORT_PCR_MUX(0x4); //turn on configuration.
	GPIO_pin_control_register(GPIO_C, bit_3, &left_motor_PCR_config); //PTC3 ALT4 = FTM0_CH2.

   	PIT_enable();
	PIT_delay(PIT_0, CLOCK_RATE, MECCASPIDER_ONE_STEP_DELAY_L); //400ms delay for one motor period.
	PIT_enable_interrupt_0(PIT_0); //start timer

	uint8_t out_of_while = FALSE;
	while(FALSE == out_of_while)
	{
		if(TRUE == /*GPIO_read_pin(GPIO_B, MECCASPIDER_LEFT_MOTOR_CALIBRATION) /***/PIT_get0_interrupt_flag_status()) //waiting for the motors to complete its period.
		{
			PORTC->PCR[bit_3] &= ~PORT_PCR_MUX(0x4); //period completed, turn off motors.
			PIT_clear0_interrupt_flag();
			out_of_while = TRUE;
		}
	}

	PIT_disable();
}

void Meccaspider_right_motor_one_step(void)
{
   	gpio_pin_control_register_t left_motor_PCR_config = PORT_PCR_MUX(0x4); //turn on configuration.
	GPIO_pin_control_register(GPIO_C, bit_2, &left_motor_PCR_config); //PTC2 ALT4 = FTM0_CH1.

   	PIT_enable();
	PIT_delay(PIT_0, CLOCK_RATE, MECCASPIDER_ONE_STEP_DELAY_R); //400ms delay for one motor period.
	PIT_enable_interrupt_0(PIT_0); //start timer

	uint8_t out_of_while = FALSE;
	while(FALSE == out_of_while)
	{
		if(TRUE == /*GPIO_read_pin(GPIO_B, MECCASPIDER_RIGHT_MOTOR_CALIBRATION)); /**/PIT_get0_interrupt_flag_status()) //waiting for the motors to complete its period.
		{
			PORTC->PCR[bit_2] &= ~PORT_PCR_MUX(0x4); //period completed, turn off motors.
			PIT_clear0_interrupt_flag();
			out_of_while = TRUE;
		}
	}

	PIT_disable(); //stop moving
}

void Meccaspider_move(Meccaspider_direction_t direction)
{
	switch(direction)
	{
		case FORWARD:

			/** Meccaspider goes forward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			Meccaspider_right_motor_one_step();
			/** Right motor goes forward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			Meccaspider_left_motor_one_step();

		break;

		case BACKWARD:

			/** Meccaspider goes backward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			Meccaspider_right_motor_one_step();
			/** Right motor goes forward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			Meccaspider_left_motor_one_step();

		break;

		case LEFT:

			/** Meccaspider goes left*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			Meccaspider_right_motor_one_step();
			/** Right motor goes forward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			Meccaspider_left_motor_one_step();

		break;

		case RIGHT:

			/** Meccaspider goes right*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			Meccaspider_right_motor_one_step();
			/** Right motor goes forward*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			Meccaspider_left_motor_one_step();

		break;

		default:

			/**Dont move*/
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);

		break;

	}
}
