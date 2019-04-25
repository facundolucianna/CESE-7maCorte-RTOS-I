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

/*==================[inclusiones]============================================*/

#include "freeRTOS_receptacle.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

//El sistema tendrá una tarea que controlará un botón. Esta tendrá una máquina de estado para controlar el anti-rebote de los botones.
//El botón simula si el receptáculo está abierto o no. Si el botón no está apretado (se decidió trabajar con la versión negada para evitar
//tener que físicamente colocar algún objeto con peso que deje el boton apretado), indica que el receptáculo está cerrado.
//Este estado se guardará en una variable global. Se realizarán lecturas cada 10 ms y tendrá la mínima prioridad.


// Implementacion de la tarea de supervisor
void receptacle( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    uint16_t systemStatusLocal = 0;
    uint8_t desiredTempconfigured = 0;
    uint8_t buttonState = 0;

    FMSbutton RecetacleButtonState = BUTTON_DOWN; // Maquina de estado del boton que hace del detector de la apertura del receptaculo.
    // Se inicia como boton hacia abajo por comodidad de que durantte la etapa  de PID, que el boton no este apretado

    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY) == pdTRUE ) {     // Guarda por primera vez el status

        systemStatusLocal = systemStatus;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {

        // Leer el estado del sistema
        if( xSemaphoreTake( xMutexVarSystemStatus, 0 ) == pdTRUE ) {

            systemStatusLocal = systemStatus;
            xSemaphoreGive( xMutexVarSystemStatus );

        }

        desiredTempconfigured = (uint8_t) ((DESIREDTEMPBIT & systemStatusLocal) >>  10);  //Vemos si la temperatura deseada ya fue cargada

        if (desiredTempconfigured) {  // Si la temperatura deseada esta configurada

            buttonState = button_state(RECEPTACLESENSOR, ANTIRREBOTE_MS, TRUE, &RecetacleButtonState);

            if(buttonState == 3) { // Si el receptaculo esta cerrado (flanco de ascenso)

              systemStatusLocal = systemStatusLocal | 0x0020; // Ponemos en 1 al bit del receptaculo cerrado

              if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                  systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas de qeu el receptaculo esta cerrado
                  xSemaphoreGive( xMutexVarSystemStatus );

              }

            } else {

                if(buttonState == 2) { //Flanco de descenso

                    systemStatusLocal = systemStatusLocal & 0xFFDF; // Ponemos en 0 al bit del receptaculo cerrado

                    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                        systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas de qeu el receptaculo esta abierto
                        xSemaphoreGive( xMutexVarSystemStatus );

                    }
                }
            }
        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_BUTTONS_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
