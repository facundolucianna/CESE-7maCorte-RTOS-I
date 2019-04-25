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

#include "freeRTOS_uartrx.h"

/*==================[definiciones de datos internos]=========================*/
uint8_t mensaje[3];
int16_t tempDesiredLocal;
uint8_t tempCelsium;

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de la tarea de uartrx
void uartrx( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    uint16_t systemStatusLocal = 0;

    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {
        systemStatusLocal = systemStatus;  // Leemos el estado del sistema
        xSemaphoreGive( xMutexVarSystemStatus );
    }

    uartInit( UART_USB, 115200); // Es importante ya que no solo inicia el UART, sino que limpia la RX FIFO que pudiese haberse llenado en el funcionamiento del sistema

    if (runFirstTime) {
        uartWriteString( UART_USB, "Bienvenido al sistema de control de medicion de sensor de radiacion\r\n");
        uartWriteString( UART_USB, "Ahora se va a configurar la temperatura a calentar el sensor\r\n");
        runFirstTime = 0;
    } else {
        uartWriteString( UART_USB, "Realize una nueva configuracion\r\n");
    }

    uartWriteString( UART_USB, "Elija un numero entero de temperatura entre 34 y 40 grados\r\n");
    uartWriteString( UART_USB, "O espere 10 segundos para que el sistema elija la temperatura por defecto (36 grados)...\r\n");

    uint8_t count = 0;
    uint8_t receiveFlag = 0;
    TickType_t tiempoIdleStart = xTaskGetTickCount();
    TickType_t tiempoIdleEnd;

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {

        switch(receiveFlag) {

            case ESCUCHANDO :

                if(uartReadByte( UART_USB, &mensaje[count])) {   // Si llego un mensaje

                    if(count == 0) {
                        if(mensaje[0] < 0x32 | mensaje[0] > 0x35) {  // Si el primer caracter (La decena) es distinto de 3 o 4, se guarda la temperaruta por defecto.
                            uartWriteString( UART_USB, "Temperatura incorrecta, se setea la temperatura por defecto\r\n");
                            tempDesiredLocal = TEMPERATURA_36;
                            receiveFlag = ACTUALIZANDOVARIABLES;
                        }
                    }

                    if(count == 1) {
                        if (!((mensaje[0] == 0x33 & mensaje[1] > 0x33 & mensaje[1] < 0x3A ) | (mensaje[0] == 0x34 & mensaje[1] == 0x30))) { // Si el segundo caracter (La unidad) es distinto de 0 o no está entre 4 y 9, se guarda la temperaruta por defecto.
                            uartWriteString( UART_USB, "Temperatura incorrecta, se setea la temperatura por defecto\r\n");
                            tempDesiredLocal = TEMPERATURA_36;
                            receiveFlag = ACTUALIZANDOVARIABLES;
                        }
                    }

                    if(count == 2) {
                        if(mensaje[2] != 0x0A) {  // Si el tercer caracter no es un enter, se guarda la temperaruta por defecto.
                            uartWriteString( UART_USB, "Temperatura incorrecta, se setea la temperatura por defecto\r\n");
                            tempDesiredLocal = TEMPERATURA_36;
                            systemStatusLocal = systemStatusLocal | 0x0400;
                            receiveFlag = ACTUALIZANDOVARIABLES;
                        } else {
                            uartWriteString( UART_USB, "Temperatura seteada: ");
                            uartWriteString( UART_USB, mensaje);
                            tempCelsium = (mensaje[0] - 0x30) * 10 + (mensaje[1] - 0x30);  //Transformamos ASCII a numero
                            tempDesiredLocal = CelsiusToInt16( tempCelsium);               // Lo transformamos a un numero int16_t
                            receiveFlag = ACTUALIZANDOVARIABLES;
                        }
                    }

                    count = count + 1;

                }

                tiempoIdleEnd = xTaskGetTickCount();

                if ((tiempoIdleEnd - tiempoIdleStart) > TIEMPO_10S / portTICK_RATE_MS) {
                    uartWriteString( UART_USB, "Tiempo de espera agotado, se setea la temperatura por defecto\r\n");
                    tempDesiredLocal = TEMPERATURA_36;
                    receiveFlag = ACTUALIZANDOVARIABLES;
                }

                break;

            case ACTUALIZANDOVARIABLES :

                uartWriteString( UART_USB, "Conecte el sensor de radiacion y cierre el receptaculo\r\n");

                if( xSemaphoreTake( xMutexVarTempDesired, portMAX_DELAY ) == pdTRUE ) {
                    tempDesired = tempDesiredLocal;  // Guardamos la temperatura deseada
                    xSemaphoreGive( xMutexVarTempDesired );
                }
                systemStatusLocal = systemStatusLocal | 0x0400;

                if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {
                    systemStatus = systemStatusLocal;  // Guardamos el estado del sistema
                    xSemaphoreGive( xMutexVarSystemStatus );
                }

                receiveFlag = IDLE;
                break;

            case IDLE :
                __asm__("nop");
                break;

        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_UARTRX_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
