/*
 * botones.h
 *
 *      Author: Facundo A. Lucianna
 */

#ifndef BOTONES_H_
#define BOTONES_H_

/*==================[inclusions]=============================================*/

//#include <stdint.h>
#include "sapi.h"              // <= sAPI header

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/*==================[typedef]================================================*/

// Estados BUTTON_UP, BUTTON_FALLING, BUTTON_DOWN, BUTTON_RAISING
/* Cambio:
 * UP a FALLING si detecta pulsador
 * FALLING espera 40 ms y vuelve a leer el pulsador. Si a los 40 ms sigue presionado, se pasa a DOWN, sino vuelve a UP
 * DOWN si se libera el pulsador pasa a RAISING
 * RAISING espera 40 ms y vuelve a leer el pulsador. Si a los 40 ms sigue presionado, se pasa a UP, sino vuelve a DOWN
*/

//MEF states
typedef enum {
	BUTTON_UP,
	BUTTON_FALLING,
	BUTTON_DOWN,
	BUTTON_RAISING
} FMSbutton;

/*==================[external functions declaration]=========================*/
uint8_t button_state(uint8_t buttonNumber, uint8_t debounceTime, uint8_t signalEdge, FMSbutton *state);

/*==================[end of file]============================================*/
#endif /* BOTONES_H_ */
