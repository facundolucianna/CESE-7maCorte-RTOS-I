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

#include "freeRTOS_pid.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de la tarea de supervisor
void vueltaPID( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    // Variables de sistema version local
    uint16_t systemStatusLocal = 0;
    int16_t lastTempLocal;
    int16_t tempDesiredLocal;
    uint8_t powerHeaterLocal;

    // Variables del PID
    int16_t errorTemp = 0;
    int32_t errorInt = 0;
    uint8_t enablePID = 0;
    int32_t Kp = 9000;
    int32_t Pprop = 0;
    int32_t PIDOuput = 0;
    uint32_t PWMpre;

    // Variable que cuenta el tiemp en el rango apropiado de temperatura
    uint8_t countTimeTempCorrect = 0;

    // Iniciamos el PWM
    uint8_t pwmVal = 0;
    pwmConfig( 0, PWM_ENABLE );
    pwmConfig( PWM6, PWM_ENABLE_OUTPUT );
    pwmWrite( PWM6, pwmVal );

    // Actualizamos la variable global de la ultima temperatura
    if( xSemaphoreTake( xMutexVarLastTemp, portMAX_DELAY ) == pdTRUE ) {

        lastTempLocal = lastTemp;
        xSemaphoreGive( xMutexVarLastTemp );

    }

    // Actualizamos la variable global de la temperatura deseada
    if( xSemaphoreTake( xMutexVarTempDesired, portMAX_DELAY ) == pdTRUE ) {

        tempDesiredLocal = tempDesired;
        xSemaphoreGive( xMutexVarTempDesired );

    }

    // Obtenemos el ultimo estado del sistema
    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

        systemStatusLocal = systemStatus;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    systemStatusLocal = systemStatusLocal | 0x08; //Habilitamos el PID

    // Obtenemos el ultimo estado del sistema
    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

        systemStatus = systemStatusLocal;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {

        // Leer el estado del sistema
        if( xSemaphoreTake( xMutexVarSystemStatus, 0 ) == pdTRUE ) {
            systemStatusLocal = systemStatus;
            xSemaphoreGive( xMutexVarSystemStatus );
        }

        // Actualizamos la variable global de la temperatura deseada
        if( xSemaphoreTake( xMutexVarTempDesired, 0 ) == pdTRUE ) {
            tempDesiredLocal = tempDesired;
            xSemaphoreGive( xMutexVarTempDesired );
        }

        // Actualizamos la variable global de la ultima temperatura
        if( xSemaphoreTake( xMutexVarLastTemp, 0 ) == pdTRUE ) {
            lastTempLocal = lastTemp;
            xSemaphoreGive( xMutexVarLastTemp );
        }

        enablePID = (uint8_t) ((0x0008 & systemStatusLocal) >> 3);  // Vemos si el PID esta habilitado

        if(enablePID) {

          // El modo quedo solo proporcional
            errorTemp = lastTempLocal - tempDesiredLocal;  // Medimos el error
            Pprop  = errorTemp * Kp;                      // Calculamos la potencia
            PIDOuput = -(Pprop);
            PWMpre =  (sqrt(PIDOuput));               // Calculamos la raiz para linealizar

            if (PWMpre > 0xFF) {                        // Si esta por encima de los 8 bits, pone 0xFF en el calefactor (no puede estar mas encendido del 100%)#
                pwmVal = 0xFF;
            } else {
                pwmVal = (uint8_t) (PWMpre);
            }

            powerHeaterLocal = (pwmVal * 100) / 0xFF;       // Calculamos en porciento
            pwmWrite( PWM6, pwmVal );                           // Modificamos el PWM

            // Actualizamos los estados, por un lado le indicamos al sistema si la temperatura esta por arriba o abajo del valor (para encender los LEDs)
            systemStatusLocal = ( 0x0F3F & systemStatusLocal);

            if (errorTemp < MARGEN_ERRRO_INF) {
                systemStatusLocal = ( systemStatusLocal | 0x0040);
                countTimeTempCorrect = 0;
            }

            if (errorTemp > MARGEN_ERRRO_SUP) {
                systemStatusLocal = ( systemStatusLocal | 0x0080);
                countTimeTempCorrect = 0;
            }

            if (errorTemp > MARGEN_ERRRO_INF & errorTemp < MARGEN_ERRRO_SUP) {
                systemStatusLocal = ( systemStatusLocal | 0x00C0);
                countTimeTempCorrect = countTimeTempCorrect + 1;
            }

            if (countTimeTempCorrect == FIVE_SECONDS) {
                systemStatusLocal = ( systemStatusLocal | ESTABLE_TEMPERATURE_BIT);
                countTimeTempCorrect = 0xFF;
            }

            // Graba el nivel del calefactor
            if( xSemaphoreTake( xMutexVarPowerHeater, 0 ) == pdTRUE ) {
                powerHeater   = powerHeaterLocal;
                xSemaphoreGive( xMutexVarPowerHeater );
            }

            // Graba el estado del sistema
            if( xSemaphoreTake( xMutexVarSystemStatus, 0 ) == pdTRUE ) {
                systemStatus   = systemStatusLocal;
                xSemaphoreGive( xMutexVarSystemStatus );
            }
        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_PID_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
