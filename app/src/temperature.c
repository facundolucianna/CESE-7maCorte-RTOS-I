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

#include "temperature.h"

/*==================[definiciones de datos internos]=========================*/

MPU60X0_address_t addr = MPU60X0_ADDRESS_0; // If MPU60X0 AD0 pin is connected to GND

/*==================[declaraciones de funciones internas]====================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Rutina de inicializacion del ADC.
int8_t initTempSensor( void )
{

    int8_t status = mpu60X0Init( addr ); //Inicializamos el sensor de temperatura

    return status;

}

int16_t getTempValue( void )
{

    mpu60X0ReadTemp();  //  Le pedimos al sensor que haga una lectura

    return mpu60X0GetTemperature_int();  // Y obtenemos el valor (en int)

}

int16_t CelsiusToInt16( float celsius )
{

  int16_t output = 0;
  float tempScale = 340.00f;
  float tempOffset = 41.53f;

  output = (int16_t) ((celsius - tempOffset) * tempScale);

  return output;

}

float int16ToCelsius( int16_t tempInt16 )
{
    float celcius;
    float tempScale = 340.00f;
    float tempOffset = 41.53f;

    celcius = ((tempInt16 / tempScale) + tempOffset);

    return celcius;

}


/*==================[fin del archivo]========================================*/
