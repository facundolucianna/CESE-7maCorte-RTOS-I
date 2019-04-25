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

#ifndef _FREERTOS_SUPERVISOR_H_
#define _FREERTOS_SUPERVISOR_H_

/*==================[inclusiones]============================================*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

#include "sapi.h"

#include "freeRTOS_uartrx.h"
#include "freeRTOS_leds.h"
#include "freeRTOS_uarttx.h"
#include "freeRTOS_radsensor.h"
#include "freeRTOS_receptacle.h"
#include "freeRTOS_temperatura.h"
#include "freeRTOS_pid.h"
/*==================[c++]====================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#define TIEMPO_SUPERVISOR_MS								100
#define TIEMPO_1_S	          							1000
#define TIEMPO_5_S	          							5000

#define MODO_CONFIGURACION									0
#define MODO_DETECCION											1
#define MODO_INICIO													2
#define MODO_FUNCIONAMIENTO									3
#define MODO_FIN														4

#define VERIFICAR_TEMP_DESEADA							0x0400

/*==================[tipos de datos declarados por el usuario]===============*/

/*==================[declaraciones de datos externos]========================*/

extern uint8_t runFirstTime;
extern uint16_t systemStatus;
extern int16_t lastTemp;
extern int16_t tempDesired;
extern uint8_t powerHeater;

SemaphoreHandle_t xMutexVarSystemStatus;
SemaphoreHandle_t xMutexVarLastTemp;
SemaphoreHandle_t xMutexVarTempDesired;
SemaphoreHandle_t xMutexVarPowerHeater;

/*==================[declaraciones de funciones externas]====================*/

void supervisor( void* taskParmPtr );

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _FREERTOS_SUPERVISOR_H_ */
