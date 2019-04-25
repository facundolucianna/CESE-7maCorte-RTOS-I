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

#include "freeRTOS_radsensor.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

//El sistema realizará lecturas de ADC. Esta tarea estará en funcionamiento desde el momento que se inicia el sistema. Realizará lecturas del ADC cada 300 ms.
// Los valores posibles que habrá en el ADC serán GND o VCC. VCC va a representar que el sensor no está presente y GND que si lo está. Al inicio, cuando el PID no esté en funcionamiento,
//se harán lecturas del ADC. Cuando el sensor esté conectado (ADC en GND), se indicará en una variable global la presencia del sensor. Esta tarea tendrá la menor prioridad posible.
//Cuando la tarea detecte que el PID está en la temperatura correcta, está tarea realizará la lectura del sensor, la cual una vez hecha, demora 3 segundos.


// Implementacion de la tarea de supervisor
void radsensor( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    initRadSensor(); //Inicializamos el ADC para detectar el sensor

    uint16_t systemStatusLocal = 0;
    uint8_t temperatureState = 0;
    uint8_t desiredTempconfigured = 0;
    uint8_t flagTemepratureCorrect = TRUE;

    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    TickType_t timeReadingStart;
    TickType_t timeReadingEnd;

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

            if (isRadSensorConnected()) {  //Vemos si el sensor de radiacion esta conectado

                systemStatusLocal = systemStatusLocal | 0x0010; // Ponemos en 1 al bits de sensor conectado

                if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                    systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas qeu el sensor esta conectado
                    xSemaphoreGive( xMutexVarSystemStatus );

                }

            } else { // En cambio si no esta conectado

                systemStatusLocal = systemStatusLocal & 0xFFEF; // Ponemos en 0 al bits de sensor conectado

                if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                    systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas qeu el sensor no esta conectado
                    xSemaphoreGive( xMutexVarSystemStatus );

                }
            }
        }

        temperatureState = (uint8_t) ((TEMPERATURESTABLEBIT & systemStatusLocal) >>  11);  // Vemos si el PID esta en temperatura o no.

        if (temperatureState == PIDTEMPERATURACORRECTA) {

            if (flagTemepratureCorrect) {   // Si entra por primera vez a la temperatura correcta
                timeReadingStart = xTaskGetTickCount(); // El sistema empieze a medir el tiempo, simulando la medicion de radiacion

                systemStatusLocal = (0xFCFF & systemStatusLocal) | 0x0100;  // Ponemos en 1 el bit 8, es decir indicamos que se esta realizando una medicion del ADC

                if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                    systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas de qeu se esta realizando la medicion
                    xSemaphoreGive( xMutexVarSystemStatus );

                }

                flagTemepratureCorrect = FALSE;

            } else {

                timeReadingEnd = xTaskGetTickCount();

                if(timeReadingEnd - timeReadingStart > TIEMPO_LECTURA_MS / portTICK_RATE_MS) {  //Si paso el tiempo determinado a la lectura del sensor de radiacion

                    systemStatusLocal = (0xFCFF & systemStatusLocal) | 0x0300;  // Ponemos en 1 el bit 9, es decir indicamos que la medicion del ADC ya fue completada
                    // La medicion es de fantasia, no hay tal cosa

                    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                        systemStatus = systemStatusLocal;       //  Y le avisamos a todas las tareas de qeu se hizo la medicion
                        xSemaphoreGive( xMutexVarSystemStatus );

                    }
                }
            }
        } else {  // Si el PID no esta en la temperatura correcta

            flagTemepratureCorrect = TRUE;

        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_ADC_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
