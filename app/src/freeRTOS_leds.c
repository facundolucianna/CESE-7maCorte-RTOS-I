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

#include "freeRTOS_leds.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de la tarea de supervisor
void leds( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    uint16_t systemStatusLocal = 0b000000001000000;
    uint8_t configuration = MODO_CONFIGURACION;

    uint8_t pidEnable = 0;            // Variable que indica si el PID esta encendido o no
    uint8_t temperatureState = 0;     // Variable que indica si la temperatura esta por debajo de la deseada, por encima o en el valor correcto
    uint8_t radiationState = 0;       // Variable que indica el estado de medicion de sensor de radiacion


    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    gpioWrite( LED1, OFF );
    gpioWrite( LED2, OFF );
    gpioWrite( LED3, OFF );
    gpioWrite( LEDG, OFF );
    gpioWrite( LEDR, OFF );
    gpioWrite( LEDB, OFF );  // Todos los LEDs apagados

    TickType_t tiempoBlinkingStart = xTaskGetTickCount();
    TickType_t tiempoBlinkingEnd ;

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {


    /*    Uno de los LEDs tradicionales será el LED rojo (LED1), el cual cuando el PID esté en funcionamiento, parpadeará con un periodo de 4 segundos (50% de ciclo de trabajo).
          El otro LED es el LED verde (LED 3) tendrá dos funciones:
           + La primera es titilar cada 500 ms para indicar que el sistema está esperando que se configure un valor por UART de temperatura deseada (modo configuración).
           + La segunda es prenderse de forma continua para indicarle al usuario que ya se hizo la lectura del sensor de radiación (modo fin).
          El LED RGB son 3 LEDs:
          El LED azul se encenderá cuando en el PID la temperatura está por debajo del valor esperado.
          El LED rojo se encenderá cuando en el PID la temperatura está por encima del valor esperado.
          El LED verde se encenderá cuando en el PID la temperatura esté en el valor correcto.*/


        // Leer el estado del sistema
        if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

            systemStatusLocal = systemStatus;
            xSemaphoreGive( xMutexVarSystemStatus );

        }

        configuration = (uint8_t) (0x07 & systemStatusLocal);  // Vemos en que estado está el sistema.

        switch(configuration) {

            case MODO_DETECCION :

                gpioWrite( LED1, OFF );
                gpioWrite( LED2, OFF );
                gpioWrite( LED3, OFF );
                gpioWrite( LEDG, OFF );
                gpioWrite( LEDR, OFF );
                gpioWrite( LEDB, OFF );  // Todos los LEDs apagados

                tiempoBlinkingStart = xTaskGetTickCount();

                break;

            case MODO_FUNCIONAMIENTO :

                pidEnable = (POSICION_PID & systemStatusLocal) >> 3; // Leemos el bit 3 de la variable systemStatusLocal para saber si el PID esta habilitado

                if(pidEnable) {  // Si el PID esta habilitado

                    tiempoBlinkingEnd = xTaskGetTickCount();  //Medimos los ticks que han pasado

                    if (tiempoBlinkingEnd - tiempoBlinkingStart > TIEMPO_PARPADEO_PID_MS / portTICK_RATE_MS) { // Titilamos el LED1 cada 2 segundos

                        gpioToggle(LED1);
                        tiempoBlinkingStart = xTaskGetTickCount();

                    }

                    radiationState = (uint8_t)  ((0x0300 & systemStatusLocal) >>  8);

                    if ((radiationState & 0x02) == 0x02) {

                        gpioWrite( LED3, ON );

                    }

                    temperatureState = (uint8_t) ((TEMPERATURESTATEBITS & systemStatusLocal) >>  6);  // Vemos como esta la temperatura con respecto a la deseada

                    switch (temperatureState){

                        case TEMPERATURA_ENCIMA:
                            gpioWrite( LEDG, OFF );
                            gpioWrite( LEDR, ON );
                            gpioWrite( LEDB, OFF );
                            break;

                        case TEMPERATURA_CORRECTA:

                            gpioWrite( LEDG, ON );
                            gpioWrite( LEDR, OFF );
                            gpioWrite( LEDB, OFF );
                            break;

                        default:

                            gpioWrite( LEDG, OFF );
                            gpioWrite( LEDR, OFF);
                            gpioWrite( LEDB, ON );
                            break;

                    }

                }

                break;

            case MODO_FIN :

                gpioWrite( LED1, OFF );
                gpioWrite( LED2, OFF );
                gpioWrite( LED3, ON );
                gpioWrite( LEDG, OFF );
                gpioWrite( LEDR, OFF );
                gpioWrite( LEDB, OFF );

                tiempoBlinkingStart = xTaskGetTickCount();

                break;

            default : // Modo configuracion

                tiempoBlinkingEnd = xTaskGetTickCount();  //Medimos los ticks que han pasado

                if (tiempoBlinkingEnd - tiempoBlinkingStart > TIEMPO_PARPADEO_MS / portTICK_RATE_MS) {

                  gpioToggle(LED3);
                  tiempoBlinkingStart = xTaskGetTickCount();

                }

                break;

        }


        vTaskDelayUntil( &tiempoCiclo, TIEMPO_LEDS_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
