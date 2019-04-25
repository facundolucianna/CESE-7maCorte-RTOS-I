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

#include "freeRTOS_supervisor.h"

/*==================[definiciones de datos internos]=========================*/

xTaskHandle uartRXHandle;
xTaskHandle uartTXHandle;
xTaskHandle radSensorHandle;
xTaskHandle receptacleHandle;
xTaskHandle pidHandle;
xTaskHandle temperaturaHandle;
xTaskHandle ledsHandle;

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de la tarea de supervisor
void supervisor( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

    uint16_t systemStatusLocal = 0b000000001000000;
    uint8_t configuration = MODO_CONFIGURACION;
    uint8_t cambioModo = TRUE;
    uint8_t mataPIDTemp = TRUE;

    TickType_t tiempoCiclo;
    tiempoCiclo = xTaskGetTickCount();

    // Creamos Mutexs
    xMutexVarSystemStatus = xSemaphoreCreateMutex();

    if(xMutexVarSystemStatus == NULL) {
        printf("Error de creacion de MUTEX");
        while(TRUE);
    }

    xMutexVarLastTemp = xSemaphoreCreateMutex();

    if (xMutexVarLastTemp == NULL) {
        printf("Error de creacion de MUTEX");
        while(TRUE);
    }

    xMutexVarTempDesired = xSemaphoreCreateMutex();

    if (xMutexVarTempDesired == NULL) {
        printf("Error de creacion de MUTEX");
        while(TRUE);
    }

    xMutexVarPowerHeater = xSemaphoreCreateMutex();

    if (xMutexVarPowerHeater == NULL) {
        printf("Error de creacion de MUTEX");
        while(TRUE);
    }

    if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

        systemStatus = 0b000000001000000;  // No hay medición | Temperatura debajo del valor deseado | Receptáculo abierto | Sensor no detectado | PID deshabilitado | Modo configuración
        xSemaphoreGive( xMutexVarSystemStatus );

    }

    if( xSemaphoreTake( xMutexVarLastTemp, portMAX_DELAY ) == pdTRUE ) {

        lastTemp = 0;
        xSemaphoreGive( xMutexVarLastTemp );

    }

    if( xSemaphoreTake( xMutexVarTempDesired, portMAX_DELAY ) == pdTRUE ) {

        tempDesired = 0;
        xSemaphoreGive( xMutexVarTempDesired );

    }

    if( xSemaphoreTake( xMutexVarPowerHeater, portMAX_DELAY ) == pdTRUE ) {

        powerHeater = 0;
        xSemaphoreGive( xMutexVarPowerHeater );

    }

   // ---------- REPETIR POR SIEMPRE --------------------------

    while(TRUE) {

        // Leer el estado del sistema
        if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

            systemStatusLocal = systemStatus;
            xSemaphoreGive( xMutexVarSystemStatus );

        }

        configuration = (uint8_t) (0x07 & systemStatusLocal);  // Vemos en que estado está el sistema.

        switch(configuration) {

            case MODO_DETECCION :

                // Primero verifica si es la primera vez que ingresa aca

                if (cambioModo != FALSE) {

                    cambioModo = FALSE;

                    xTaskCreate(
                        uarttx,                     // Funcion de la tarea a ejecutar
                        (const char *)"uarttx",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &uartTXHandle                               // Puntero a la tarea creada en el sistema
                    );
                    xTaskCreate(
                        radsensor,                     // Funcion de la tarea a ejecutar
                        (const char *)"radsensor",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &radSensorHandle                               // Puntero a la tarea creada en el sistema
                    );
                    xTaskCreate(
                        receptacle,                     // Funcion de la tarea a ejecutar
                        (const char *)"receptacle",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &receptacleHandle                               // Puntero a la tarea creada en el sistema
                    );

                }

                // Verificamos si el receptaculo esta cerrado y el sensor conectado
                uint8_t receptacleandRadSensorState = (uint8_t) ((0x0030 & systemStatusLocal) >>  4);

                if (receptacleandRadSensorState == 0x03) { // si estan los OK, pasamos al modo funionamiento

                  systemStatusLocal = (systemStatus & 0x0FF8) | MODO_FUNCIONAMIENTO; //Pasamos a modo funcionamiento.

                  cambioModo = TRUE;
                }

                break;

            case MODO_FUNCIONAMIENTO :

                // Primero verifica si es la primera vez que ingresa aca

                if (cambioModo != FALSE) {

                    cambioModo = FALSE;

                    xTaskCreate(
                        medicionTemp,                     // Funcion de la tarea a ejecutar
                        (const char *)"medicionTemp",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &temperaturaHandle              // Puntero a la tarea creada en el sistema
                    );

                    xTaskCreate(
                        vueltaPID,                     // Funcion de la tarea a ejecutar
                        (const char *)"vueltaPID",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY + 1,           // Prioridad de la tarea (maxima prioridad)
                        &pidHandle                               // Puntero a la tarea creada en el sistema
                    );

                }

                // Verificamos si el receptaculo esta cerrado y el sensor conectado
                receptacleandRadSensorState = (uint8_t) ((0x0030 & systemStatusLocal) >>  4);
                // Verificamos si el se termino la medicion del sensor de radiacion
                uint8_t radiationState = (uint8_t) ((0x0200 & systemStatusLocal) >>  9);

                if (radiationState) {  // Pero ya se habia hecho la medicion de radiacion, todo bien
                    if (mataPIDTemp) {
                        vTaskDelete(temperaturaHandle);    // Matamos a la tarea de sensado de temperatura
                        vTaskDelete(pidHandle);    // Matamos a la tarea del PID
                        mataPIDTemp = FALSE;
                    }
                }

                if (receptacleandRadSensorState != 0x03) { // Si alguno se cambio (se abrio el receptaculo o se saco el sensor)

                    if (mataPIDTemp) {
                        vTaskDelete(temperaturaHandle);    // Matamos a la tarea de sensado de temperatura
                        vTaskDelete(pidHandle);    // Matamos a la tarea del PID
                        mataPIDTemp = FALSE;
                    }

                    vTaskDelay( TIEMPO_1_S / portTICK_RATE_MS );  //Hacemos un delay para que salgan todos los mensaje que falten

                    vTaskDelete(radSensorHandle);    // Matamos a la tarea del sensor de radiacion
                    vTaskDelete(receptacleHandle);    // Matamos a la tarea del PID

                    cambioModo = TRUE;
                    systemStatusLocal = (systemStatusLocal & 0x0FF8) | MODO_FIN; //Pasamos a modo fin.

                }

                break;

            case MODO_FIN :

                vTaskDelay( TIEMPO_5_S / portTICK_RATE_MS );  //Hacemos un delay para que salgan todos los mensaje que falten
                vTaskDelete(uartTXHandle);    // Matamos a la tarea UART TX
                vTaskDelete(ledsHandle);    // Matamos a la tarea de los LEDs

                // Reiniciamos todas las variables
                systemStatusLocal = 0b000000001000000;
                configuration = MODO_CONFIGURACION;
                cambioModo = TRUE;
                mataPIDTemp = TRUE;

                if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {

                    systemStatus = 0b000000001000000;  // No hay medición | Temperatura debajo del valor deseado | Receptáculo abierto | Sensor no detectado | PID deshabilitado | Modo configuración
                    xSemaphoreGive( xMutexVarSystemStatus );

                }

                if( xSemaphoreTake( xMutexVarLastTemp, portMAX_DELAY ) == pdTRUE ) {

                    lastTemp = 0;
                    xSemaphoreGive( xMutexVarLastTemp );

                }

                if( xSemaphoreTake( xMutexVarTempDesired, portMAX_DELAY ) == pdTRUE ) {

                    tempDesired = 0;
                    xSemaphoreGive( xMutexVarTempDesired );

                }

                if( xSemaphoreTake( xMutexVarPowerHeater, portMAX_DELAY ) == pdTRUE ) {

                    powerHeater = 0;
                    xSemaphoreGive( xMutexVarPowerHeater );

                }
                break;

            default : // Modo configuracion

                // Primero verifica si es la primera vez que ingresa aca
                if (cambioModo != FALSE) {

                    cambioModo = FALSE;

                    xTaskCreate(
                        uartrx,                     // Funcion de la tarea a ejecutar
                        (const char *)"uartrx",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &uartRXHandle                               // Puntero a la tarea creada en el sistema
                    );

                    xTaskCreate(
                        leds,                     // Funcion de la tarea a ejecutar
                        (const char *)"leds",     // Nombre de la tarea como String amigable para el usuario
                        configMINIMAL_STACK_SIZE*2,     // Cantidad de stack de la tarea
                        0,                              // Parametros de tarea
                        tskIDLE_PRIORITY,               // Prioridad de la tarea (minima prioridad)
                        &ledsHandle                               // Puntero a la tarea creada en el sistema
                    );

                }

                // Luego verifica si debe cambiar de modo
                if ( (systemStatusLocal & VERIFICAR_TEMP_DESEADA) == VERIFICAR_TEMP_DESEADA){

                      vTaskDelete(uartRXHandle);    // Matamos a la tarea UART Rx

                      systemStatusLocal = (systemStatusLocal & 0x0FF8) | MODO_DETECCION; //Pasamos a modo deteccion.

                      cambioModo = TRUE;

                }


        }

        if (cambioModo == TRUE) {

          if( xSemaphoreTake( xMutexVarSystemStatus, portMAX_DELAY ) == pdTRUE ) {
              systemStatus = systemStatusLocal;
              xSemaphoreGive( xMutexVarSystemStatus );
          }

        }

        vTaskDelayUntil( &tiempoCiclo, TIEMPO_SUPERVISOR_MS / portTICK_RATE_MS );

    }

}

/*==================[fin del archivo]========================================*/
