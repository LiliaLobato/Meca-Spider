/*
 * MecaTeclado.h
 *
 *  Created on: 08/05/2019
 *      Author: Lilia Lobato
 */

#ifndef MECATECLADO_H_
#define MECATECLADO_H_

#include <stdint.h>

#define SYSTEM_CLOCK (21000000U)
#define DELAYBARRIDO (0.006F) //164 HZ

typedef enum{K01=1,K02,K03
			} teclado_output;

void MecaTeclado_Senales();
void MecaTeclado_init();
void MecaTeclado_Senales_clean();

uint8_t MecaTeclado_get_k_teclado(void);

#endif /* MECATECLADO_H_ */
