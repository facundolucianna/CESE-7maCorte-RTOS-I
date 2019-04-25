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

#include "freeRTOS_temperatura.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de la tarea de supervisor
void medicionTemp( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    uint16_t systemStatusLocal = 0;
    uint8_t configuration = MODO_CONFIGURACION;
    int16_t bufferTemperatura[8];
    int32_t sumadorTemp;
    int16_t lastTempLocal;
    uint8_t punteroBuffer = 0;

    // Iniciamos el sensor de temperatura
    initTempSensor();

    //Obtenemos un primer valor
    lastTempLocal = getTempValue();

    // Llenamos el buffer
    for(punteroBuffer=0; punteroBuffer<8; punteroBuffer= punteroBuffer + 1) {
	       bufferTemperatura[punteroBuffer] = lastTempLocal;
    }

    punteroBuffer = 0;

    // Actualizamos la variable global de la ultima temperatura
    if( xSemaphoreTake( xMutexVarLastTemp, portMAX_DELAY ) == pdTRUE ) {

        lastTemp = lastTempLocal;
        xSemaphoreGive( xMutexVarLastTemp );

    }

    // Obtenemos el ultimo estado del sistema
    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

        systemStatusLocal = systemStatus;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    systemStatusLocal = systemStatusLocal | 0x0008; //Permitimos que funcione el PID ya que hay mediciones de temperatura

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

        configuration = (uint8_t) (0x07 & systemStatusLocal);  // Vemos en que estado está el sistema.

        if(configuration == MODO_FUNCIONAMIENTO) {

          bufferTemperatura[punteroBuffer] = getTempValue(); // Medimos la temperatura, y lo guardamos en el buffer

          // Aplicamos el filtro de ventana
          sumadorTemp =  (int32_t) (bufferTemperatura[0]);

          for(uint8_t count=1; count<8; count = count + 1) {
               sumadorTemp = (int32_t) (sumadorTemp + bufferTemperatura[punteroBuffer]);
          }

          sumadorTemp = (sumadorTemp / 8);

          lastTempLocal = (int16_t) sumadorTemp;

        //  lastTempLocal = getTempValue();

          // Actualizamos la variable global de la ultima temperatura
          if( xSemaphoreTake( xMutexVarLastTemp, 0 ) == pdTRUE ) {

              lastTemp = lastTempLocal;
              xSemaphoreGive( xMutexVarLastTemp );

          }


        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_TEMP_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
