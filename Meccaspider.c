/*
 * Meccaspider.c
 *
 *  Created on: 09/05/2019
 *      Author: best buy
 */
#include "Meccaspider.h"
#include "Delay.h"

void Meccaspider_init(void)
{
   	/*Initial configuration for left motors*/
   	PIT_clock_gating();
	GPIO_clock_gating(GPIO_C);
	GPIO_clock_gating(GPIO_B);
   	NVIC_enable_interrupt_and_priotity(PORTC_IRQ,PRIORITY_5);//Enables the ISR for SW2.
   	NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_5);//Enables the ISR for SW3.
   	NVIC_enable_interrupt_and_priotity(PORTB_IRQ,PRIORITY_5);//Enables the ISR for SW3.
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ,PRIORITY_6);//Enables the ISR for PIT0.
   	NVIC_global_enable_interrupts;
	/**Configuration function for PWM*/
	FlexTimer_PWM_CH2_Init();
	FlexTimer_PWM_CH1_Init();


	gpio_pin_control_register_t output_motor_control = GPIO_MUX1;	//Configuración del GPIO
	gpio_pin_control_register_t input_motor_calibration = GPIO_MUX1 | INTR_FALLING_EDGE;	//Configuración del GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS, &output_motor_control); //PTC17 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG, &output_motor_control); //PTC16 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS, &output_motor_control); //PTC11 as GPIO
	GPIO_pin_control_register(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG, &output_motor_control); //PTC10 as GPIO
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_ATACK_MOTOR, &output_motor_control); //PTB20 as GPIO
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_RIGHT_MOTOR_CALIBRATION_SW2, &input_motor_calibration); //PTB3 as GPIO INPUT
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_LEFT_MOTOR_CALIBRATION_SW2, &input_motor_calibration); //PTB11 as GPIO INPUT
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_RIGHT_MOTOR_CALIBRATION_SW1, &input_motor_calibration); //PTB2 as GPIO INPUT
	GPIO_pin_control_register(GPIO_B, MECCASPIDER_LEFT_MOTOR_CALIBRATION_SW1, &input_motor_calibration); //PTB10 as GPIO INPUT

	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_RIGHT_MOTOR_POS);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_RIGHT_MOTOR_NEG);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_LEFT_MOTOR_POS);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, MECCASPIDER_LEFT_MOTOR_NEG);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, MECCASPIDER_ATACK_MOTOR);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_RIGHT_MOTOR_CALIBRATION_SW2); //motors period completed
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_LEFT_MOTOR_CALIBRATION_SW2);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_RIGHT_MOTOR_CALIBRATION_SW1); //motors period/2 completed
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, MECCASPIDER_LEFT_MOTOR_CALIBRATION_SW1);

	GPIO_callback_init(GPIO_B, &PushButton_external_handler_motor);

}

void Meccaspider_left_motor_one_step(void)
{
   	gpio_pin_control_register_t left_motor_PCR_config = PORT_PCR_MUX(0x4); //turn on configuration.
	GPIO_pin_control_register(GPIO_C, bit_3, &left_motor_PCR_config); //PTC3 ALT4 = FTM0_CH2.

	uint8_t out_of_while = FALSE;
	while(FALSE == out_of_while)
	{
		if (TRUE == get_pbn_flag_motor(LMC_SW2)) //waiting for the motors to complete its period.
		{
			PORTC->PCR[bit_3] &= ~PORT_PCR_MUX(0x4); //period completed, turn off motors.
			clear_pbn_flag_motor(LMC_SW2);

			out_of_while = TRUE;
		}
		if(TRUE == get_pbn_flag_motor(LMC_SW1))//half of the period?
		{
			GPIO_pin_control_register(GPIO_C, bit_2, &left_motor_PCR_config); //period/2 completed, turn on right motors
			clear_pbn_flag_motor(LMC_SW1);
		}

	}


}

void Meccaspider_right_motor_one_step(void)
{
   	gpio_pin_control_register_t left_motor_PCR_config = PORT_PCR_MUX(0x4); //turn on configuration.
	GPIO_pin_control_register(GPIO_C, bit_2, &left_motor_PCR_config); //PTC2 ALT4 = FTM0_CH1.


	uint8_t out_of_while = FALSE;
	while(FALSE == out_of_while)
	{
		if(TRUE == get_pbn_flag_motor(RMC_SW2)) //waiting for the motors to complete its period.
		{
			PORTC->PCR[bit_2] &= ~PORT_PCR_MUX(0x4); //period completed, turn off motors.
			clear_pbn_flag_motor(RMC_SW2);
			out_of_while = TRUE;
		}
		if(TRUE == get_pbn_flag_motor(RMC_SW1))//half of the period
		{
			GPIO_pin_control_register(GPIO_C, bit_3, &left_motor_PCR_config); //period/2 completed, turn on left motors.
			clear_pbn_flag_motor(RMC_SW1);
		}
	}


}

void Meccaspider_stop(void)
{
	GPIO_clear_pin(GPIO_B, MECCASPIDER_ATACK_MOTOR);
	PORTC->PCR[bit_2] &= ~PORT_PCR_MUX(0x4); //turn off motors.
	PORTC->PCR[bit_3] &= ~PORT_PCR_MUX(0x4); //turn off motors.
}

void Meccaspider_atack(void)
{
	GPIO_set_pin(GPIO_B, MECCASPIDER_ATACK_MOTOR);
}

void Meccaspider_Stop_atack(void)
{
	GPIO_clear_pin(GPIO_B, MECCASPIDER_ATACK_MOTOR);
}

void Meccaspider_move(Meccaspider_direction_t direction)
{
	uint8_t i;
	switch(direction)
	{
		case FORWARD:

			/** Meccaspider goes forward.
			 	Se modifica la polaridad de los motores*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);

			for(i = 0; i < MECCASPIDER_STEPS_NUMBER_FB; i++)
			{
				//Meccaspider takes 3 steps forward
				Meccaspider_right_motor_one_step();
				Meccaspider_left_motor_one_step();
				Meccaspider_stop();
				delay(500);
			}


		break;

		case BACKWARD:

			/** Meccaspider goes backward
			 * Se modifica la polaridad de los motores*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);

			for(i = 0; i < MECCASPIDER_STEPS_NUMBER_FB; i++)
			{
				//Meccaspider takes 3 steps backward
				Meccaspider_right_motor_one_step();
				Meccaspider_left_motor_one_step();
				Meccaspider_stop();
				delay(500);
			}

		break;

		case LEFT:

			/** Meccaspider goes left
			 * Se modifica la polaridad de los motores*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);

			for(i = 0; i < MECCASPIDER_STEPS_NUMBER_L; i++)
			{
				//Meccaspider takes 6 steps to the left
				Meccaspider_right_motor_one_step();
				Meccaspider_left_motor_one_step();
				Meccaspider_stop();
				delay(500);
			}
			Meccaspider_left_motor_one_step();
			Meccaspider_stop();

		break;

		case RIGHT:

			/** Meccaspider goes right
			 * Se modifica la polaridad de los motores*/
			GPIO_set_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_set_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);

			for(i = 0; i < MECCASPIDER_STEPS_NUMBER_R; i++)
			{
				//Meccaspider takes 6 steps to the right
				Meccaspider_right_motor_one_step();
				Meccaspider_left_motor_one_step();
				Meccaspider_stop();
				delay(500);
			}

		break;

		case ATACK:

			Meccaspider_atack();


		break;

		default:

			/**Dont move*/
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_RIGHT_MOTOR_NEG);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_POS);
			GPIO_clear_pin(GPIO_C, MECCASPIDER_LEFT_MOTOR_NEG);
			Meccaspider_stop();

		break;

	}
} //Este no es un comentario, este es un mensaje subliminal. #ConLosCandadosNo


