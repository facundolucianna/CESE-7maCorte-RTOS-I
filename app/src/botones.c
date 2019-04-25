/* Copyright 2019, Facundo Adrián Lucianna (facundolucianna@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inclusions]=============================================*/
#include "botones.h"

/*==================[external functions declaration]=========================*/
/*
 * @brief:  Analiza el estado de un boton, con sistema antirebote por software
 * mediante maquina de estado. Adaptado para funcionar con freeRTOS
 * @param:  + Numero de botón (Basado de la sAPI por lo que se puede usar TEC1,
 *          TEC2, etc.).
 *          + Tiempo de antirebote: Tiempo que en ms que el antirebote funciona,
 *          se puede cambiar el valor en caliente.
 *          + Deteccion de flanco: Si vale 0 no detecta flanco, si vale 1 si.
 *          + Variable usada para detectar el delay no bloqueante.
 *          + Maquina de estado: Maquina de estado para cada boton, para que la
 *           funcion sepa en que estado se encuentra.
 * @return: La salida es un valor igual a 1 si el boton esta apretado, 2 si hay un
 *          flanco descendente y 3 si hay un flanco ascendente.
*/
uint8_t button_state(uint8_t buttonNumber, uint8_t debounceTime, uint8_t signalEdge, FMSbutton *state)
{
    bool_t pressed = 0;                         // Variable para indicar si el boton esta apretado o no, se declara como no
                                                // cada vez que se ingresa a la función.
    TickType_t nextTime;

    switch (*state) {

        case BUTTON_UP:

            pressed = 0;

            if (!gpioRead( buttonNumber )) {        // Si se detecto un cero en el GPIO correspondiente al botón.
                *state = BUTTON_FALLING;            // Se pasa al estado BUTTON_FALLING.
            }

            break;

        case BUTTON_FALLING:                       // En el flanco de ascenso (boton cayendo)

            vTaskDelay( debounceTime /  portTICK_PERIOD_MS);        // Hacemos un delay de 100 ms

            if (!gpioRead( buttonNumber )) {    // Si sigue en cero en el GPIO correspondiente al botón, significa que el boton esta apretado.

                if (signalEdge > 0) {           // Si se eligio flanco descendente.
                    pressed = 2;                // Se detecta el flanco.
                }

                *state = BUTTON_DOWN;           // Se pasa al estado boton abajo.
            } else {
                *state = BUTTON_UP;             // Si no se detecto al boton abajo, aca no paso nada y vuelve al estado boton arriba.
            }

            break;

        case BUTTON_DOWN:

            if (gpioRead( buttonNumber )) {    // Si se detecto un uno en el GPIO correspondiente al botón (corresponde al boton suelto).
                *state = BUTTON_RAISING;       // Pasa al estado boton levantandose.
            }

            pressed = 1;                       // El boton esta apretado.

            break;

        case BUTTON_RAISING:

            pressed = 1;

            vTaskDelay( debounceTime /  portTICK_PERIOD_MS);        // Hacemos un delay de 100 ms

            if (gpioRead( buttonNumber )) {     // Si luego de pasado el delay

                if (signalEdge > 0) {           // Si se eligio flanco ascendente.
                    pressed = 3;                // Se detecta el flanco.
                }
                *state = BUTTON_UP;             // Se pasa al estado boton arriba.
            } else {
                *state = BUTTON_DOWN;
            }

            break;

        default:
            *state = BUTTON_UP;
            break;
    }

    return pressed;
}

/*==================[end of file]============================================*/
