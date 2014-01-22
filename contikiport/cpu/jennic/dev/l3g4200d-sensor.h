/*
 * Copyright (c) 2011
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#ifndef __L3G4200D_SENSOR_H__
#define __L3G4200D_SENSOR_H__

#include "lib/sensors.h"

extern const struct sensors_sensor l3g4200d_sensor;

#define GYRO_SENSOR "gyroscope"

#define GYRO_VALUE_X    0x0
#define GYRO_VALUE_Y    0x1
#define GYRO_VALUE_Z    0x2
#define GYRO_VALUE_TEMP 0x3

#define GYRO_L3G_RANGE           0x00 /* set the range */
#define GYRO_L3GVAL_2000DPS      0x0
#define GYRO_L3GVAL_500DPS       0x1
#define GYRO_L3GVAL_250DPS       0x2

#define GYRO_L3G_DRATE           0x01 /* set output rate and low-pass cut-off */
#define GYRO_L3GVAL_100_12_5HZ   0x0  /* XXX: add lower-power modes */
#define GYRO_L3GVAL_100_25HZ     0x1
#define GYRO_L3GVAL_200_12_5HZ   0x4
#define GYRO_L3GVAL_200_25HZ     0x5
#define GYRO_L3GVAL_200_50HZ     0x6
#define GYRO_L3GVAL_200_70HZ     0x7
#define GYRO_L3GVAL_400_20HZ     0x8
#define GYRO_L3GVAL_400_25HZ     0x9
#define GYRO_L3GVAL_400_50HZ     0xA
#define GYRO_L3GVAL_400_110HZ    0xB
#define GYRO_L3GVAL_800_30HZ     0xC
#define GYRO_L3GVAL_800_35HZ     0xD
#define GYRO_L3GVAL_800_50HZ     0xE
#define GYRO_L3GVAL_800_110HZ    0xF

#define GYRO_L3G_HIGHPASS        0x02 /* set high-pass filter cut-off frequency */
                                      /* ODR = 100Hz | 200Hz | 400Hz | 800Hz    */
#define GYRO_L3GVAL_HP0          0x00 /*        8      15      30      56       */
#define GYRO_L3GVAL_HP1          0x01 /*        4       8      15      30       */
#define GYRO_L3GVAL_HP2          0x02 /*        2       4       8      15       */
#define GYRO_L3GVAL_HP3          0x03 /*        1       2       4       8       */
#define GYRO_L3GVAL_HP4          0x04 /*        .5      1       2       4       */
#define GYRO_L3GVAL_HP5          0x05 /*        .2      .5      1       2       */
#define GYRO_L3GVAL_HP6          0x06 /*        .1      .2      .5      1       */
#define GYRO_L3GVAL_HP7          0x07 /*        .05     .1      .2      .5      */
#define GYRO_L3GVAL_HP8          0x08 /*        .02     .05     .1      .2      */
#define GYRO_L3GVAL_HP9          0x08 /*        .01     .02     .05     .1      */

#define GYRO_L3G_FILTER          0x03 /* enable/disable high- and low-pass      */
#define GYRO_L3GVAL_NONE         0x00
#define GYRO_L3GVAL_HPFILTER     0x01 /* enable only high-pass */
#define GYRO_L3GVAL_LPFILTER     0x02 /* enable only low-pass  */
#define GYRO_L3GVAL_BOTH         0x03 /* enalbe low- and high-pass (i.e. bandpass) */

#endif
