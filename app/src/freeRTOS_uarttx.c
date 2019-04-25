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

#include "freeRTOS_uarttx.h"

/*==================[definiciones de datos internos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/* La UART enviará información del usuario. Al inicio antes de arrancar el PID enviará el valor de temperatura que el sistema tiene configurado. Además enviará cuando se detecte el sensor y cuando el receptáculo se cerró.

Cuando el PID esté funcionando, enviará cada 1 segundo el valor de temperatura medido. Cuando llegue al valor deseado y el sistema haga la lectura de sensor, el sistema enviará el valor de radiación medido (valor de fantasia).

La tarea  se repertirá cada 100 ms y tendrá la mínima prioridad. */


// Implementacion de la tarea de supervisor
void uarttx( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    uartInit( UART_USB, 115200); // Se inicializa la UART (de nuevo, esto limpia los registros)

    // Se inicializan las versiones locales de las variables globales del sistema
    int16_t lastTempLocal = 0;
    int16_t tempDesiredLocal = 0;
    uint16_t systemStatusLocal = 0;
    uint8_t powerHeaterLocal = 0;

    uint8_t configuration = MODO_DETECCION;

    uint8_t sensorConnected = 0;      // Variable que  indica si el sensor de radiacion esta cerrado
    uint8_t receptacleClosed = 0;     // Variable que  indica si el receptaculo esta cerrado
    uint8_t pidEnable = 0;            // Variable que indica si el PID esta encendido o no

    uint8_t radiationState = 0;       // Variable que indica el estado de medicion de sensor de radiacion

    uint8_t sensorConnectedPrev = 0;         // Ultimo estado del sensor de radiacion.
    uint8_t receptacleClosedPrev = 0;        // Ultimo estado del sensor del receptaculo.
    uint8_t pidEnablePrev = 0;               // Ultimo estado del pid.
    uint8_t radiationStatePrev = 0;          // Ultimo estado del estado de medicion de sensor de radiacion

    uint8_t mensajeFinal = 1;

    float tempCelsius = 0.0f;
    uint8_t buffer[6];

    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY) == pdTRUE ) {     // Guarda por primera vez el status

        systemStatusLocal = systemStatus;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY) == pdTRUE ) {     // Guarda por primera vez la temperatura deseada

        tempDesiredLocal = tempDesired;
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    TickType_t tiempoTemperaturaStart;
    TickType_t tiempoTemperaturaEnd;

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {

        // Leer el estado del sistema
        if( xSemaphoreTake( xMutexVarSystemStatus, 0 ) == pdTRUE ) {      // Si no pudo leer el estado por el mutex, trabaja con la version local y sigue.

            systemStatusLocal = systemStatus;
            xSemaphoreGive( xMutexVarSystemStatus );

        }

        configuration = (uint8_t) (0x07 & systemStatusLocal);  // Vemos en que estado está el sistema.

        switch(configuration) {

            case MODO_DETECCION :

                sensorConnected = (POSICION_DETECCION_SENSOR & systemStatusLocal) >> 4; // Leemos el bit 4 de la variable systemStatusLocal para saber si el sensor esta conectado (igual a 1)
                receptacleClosed = (POSICION_RECEPTACULO & systemStatusLocal) >> 5; // Leemos el bit 5 de la variable systemStatusLocal para saber si el receptaculo esta abierto (igual a 0) o cerrado (igual a 1)

                if(sensorConnected != sensorConnectedPrev) {  // Si el estado del deteccion del sensor cambio.

                    sensorConnectedPrev = sensorConnected;

                    if(sensorConnected) {
                        uartWriteString( UART_USB, "Sensor de radiacion conectado\r\n");
                    } else {
                        uartWriteString( UART_USB, "Sensor de radiacion desconectado\r\n");
                    }

                }


                if(receptacleClosed != receptacleClosedPrev) {  // Si el estado del deteccion del sensor cambio.

                    receptacleClosedPrev = receptacleClosed;

                    if(receptacleClosed) {
                        uartWriteString( UART_USB, "Receptaculo cerrado\r\n");
                    } else {
                        uartWriteString( UART_USB, "Receptaculo abierto\r\n");
                    }

                }

                tiempoTemperaturaStart = xTaskGetTickCount();

                break;

            case MODO_FUNCIONAMIENTO :

                sensorConnected = (POSICION_DETECCION_SENSOR & systemStatusLocal) >> 4; // Leemos el bit 4 de la variable systemStatusLocal para saber si el sensor esta conectado (igual a 1)
                receptacleClosed = (POSICION_RECEPTACULO & systemStatusLocal) >> 5; // Leemos el bit 5 de la variable systemStatusLocal para saber si el receptaculo esta abierto (igual a 0) o cerrado (igual a 1)
                pidEnable = (POSICION_PID & systemStatusLocal) >> 3; // Leemos el bit 3 de la variable systemStatusLocal para saber si el PID esta habilitado
                radiationState = ((0x0300 & systemStatusLocal) >>  8);


                if(pidEnable != pidEnablePrev) {  // Si el estado del PID cambio

                    pidEnablePrev = pidEnable;

                    if(pidEnable) {
                        uartWriteString( UART_USB, "PID encendido, comenzando medicion.\r\n");
                    } else {
                        uartWriteString( UART_USB, "PID apagado.\r\n");
                    }
                }

                if ((radiationState & 0x02) != 0x02)  {

                    if(pidEnable) { // Mensaje de temperatura cada 1 segundo por UART pidEnable

                        tiempoTemperaturaEnd = xTaskGetTickCount();

                        if (tiempoTemperaturaEnd - tiempoTemperaturaStart > TIEMPO_MENSAJES_TEMP_MS / portTICK_RATE_MS) {

                            if( xSemaphoreTake( xMutexVarLastTemp, 0 ) == pdTRUE ) {      // Si no pudo leer el estado por el mutex, trabaja con la version local y sigue.
                                lastTempLocal = lastTemp;
                                xSemaphoreGive( xMutexVarLastTemp );
                            }

                            if( xSemaphoreTake( xMutexVarPowerHeater, 0 ) == pdTRUE ) {      // Si no pudo leer el estado por el mutex, trabaja con la version local y sigue.
                                powerHeaterLocal = powerHeater;
                                xSemaphoreGive( xMutexVarPowerHeater );
                            }

                            tiempoTemperaturaStart = xTaskGetTickCount();

                            uartWriteString( UART_USB, "Temperatura: ");
                            tempCelsius = int16ToCelsius( lastTempLocal );      //Convertimos la temperatura a Celcius
                            int16_t tempCelInt = (int16_t) (tempCelsius * 10);  //La trabajamos un poco para poder mostrarla por UART
                            sprintf(buffer, "%d", tempCelInt);
                            uartWriteByte( UART_USB, buffer[0]);
                            uartWriteByte( UART_USB, buffer[1]);
                            uartWriteByte( UART_USB,0x2E);
                            uartWriteByte( UART_USB,buffer[2]);
                            uartWriteString( UART_USB, "\r\n");

                            printf("Potencia del calefactor: %d porciento\r\n", powerHeaterLocal);
                        }
                    }

                }

                if(sensorConnected != sensorConnectedPrev) {  // Si el estado del deteccion del sensor cambio.

                    sensorConnectedPrev = sensorConnected;

                    if(!sensorConnected) { // Si el sensor de radiacion se desconecta
                        uartWriteString( UART_USB, "Sensor de radiacion desconectado\r\n");
                        if ((radiationState & 0x02) != 0x02) { // Si ocurre antes de la medicion de radiacion
                            uartWriteString( UART_USB, "Se retiro antes de la medicion... Abortando.\r\n");
                        }
                    }

                }

                if(receptacleClosed != receptacleClosedPrev) {  // Si el estado del deteccion del sensor cambio.

                    receptacleClosedPrev = receptacleClosed;

                    if(!receptacleClosed) {
                        uartWriteString( UART_USB, "Receptaculo abierto\r\n");
                        if ((radiationState & 0x02) != 0x02) { // Si ocurre antes de la medicion de radiacion
                            uartWriteString( UART_USB, "Se retiro antes de la medicion... Abortando.\r\n");
                        }
                    }

                }

                if(radiationState != radiationStatePrev) {  // Si el estado del deteccion del sensor cambio.

                    radiationStatePrev = radiationState;

                    if ((radiationState & 0x02) == 0x02)  { // Si es que ya termino la medicion, escribe un mensaje
                        uartWriteString( UART_USB, "Medicion lista, el nivel de radiacion es de 2 rad.\r\n");
                        uartWriteString( UART_USB, "Puede retirar el sensor.\r\n");
                    } else {

                      if ((radiationState & 0x01) == 0x01)  { // Se esta realizando la medicion
                          uartWriteString( UART_USB, "Medicion de radacion en proceso.\r\n");
                      }
                    }
                }
                break;

            case MODO_FIN :

                if (mensajeFinal) {

                  uartWriteString( UART_USB, "Fin del proceso de medicion. Reiniciando.\r\n");
                  mensajeFinal = 0;

                }
                break;

            default : // Modo configuracion, aca nunca deberia estar

                __asm__("nop"); // Aca no deberia entrar nunca
                break;

        }


        vTaskDelayUntil( &tiempoCiclo, TIEMPO_UARTTX_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
