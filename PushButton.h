/*
 * PushButton.h
 *
 *  Created on: 24/02/2019
 *      Author: Lilia Lobato
 */

#ifndef PUSHBUTTON_H_
#define PUSHBUTTON_H_

#include <GPIO.h>
#include "Bits.h"

//Valor para configurar sw y poner safe values
#define SW2 0x40
#define SW3 0x10
#define PTB2_MASK 0X4


/*! These constants are used to select an specific push button*/
typedef enum{PB_SW2, /*!< Definition to select SW2*/
			 PB_SW3  /*!< Definition to select SW3*/
			} PushButton_SW_name;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Estas funciones ejecutan los pasos necesarios para configurar push button
 */
void PushButton_sw2_config(void);
void PushButton_sw3_config(void);
void PushButton_external_config(gpio_port_name_t GPIOx, BitsType pin, uint32_t MASK);
void PushButton_external_handler(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Esta funcion lee el estado de un PushButton
 	 \param[in]  PushButton_SW_name PushButton a leer.
 	 \return uint32_t con el valor de estado
 */
uint32_t PushButton_read(PushButton_SW_name sw);


#endif /* PUSHBUTTON_H_ */
