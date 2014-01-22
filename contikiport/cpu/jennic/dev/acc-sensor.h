/*
 * Copyright (c) 2009
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

#ifndef __ACC_SENSOR_H__
#define __ACC_SENSOR_H__

#include "lib/sensors.h"

extern const struct sensors_sensor acc_sensor;

#define ACC_SENSOR "acceleration"

#define ACC_VALUE_X 0x0
#define ACC_VALUE_Y 0x1
#define ACC_VALUE_Z 0x2

#define ACC_SENSOR_RATE      0x00 /* sampling rate       */
#define ACC_SENSOR_FULLRES   0x01 /* full resolution bit */
#define ACC_SENSOR_RANGE     0x02 /* range setting       */
#define ACC_SENSOR_TAPENABLE 0x03
#define ACC_SENSOR_TAPTHRESH 0x04
#define ACC_SENSOR_TAPDUR    0x05
#define ACC_SENSOR_TAPLATENT 0x06
#define ACC_SENSOR_TAPWINDOW 0x07

#define ACC_VALUE_INTSOURCE  0x08
#define ACC_VALUE_TAPSTATUS  0x09

#define ACC_LSM303_RANGE     0x00 /* resolution */
#define ACC_LSM303VAL_2G     0x00
#define ACC_LSM303VAL_4G     0x01
#define ACC_LSM303VAL_8G     0x03

#define ACC_LSM303_DRATE     0x01 /* digital output rate */
#define ACC_LSM303VAL_0_5HZ  0x02 /* low-power modes     */
#define ACC_LSM303VAL_1HZ    0x03
#define ACC_LSM303VAL_2HZ    0x04
#define ACC_LSM303VAL_5HZ    0x05
#define ACC_LSM303VAL_10HZ   0x06
#define ACC_LSM303VAL_50HZ   0x07 /* normal power-mode   */
#define ACC_LSM303VAL_100HZ  0x08
#define ACC_LSM303VAL_400HZ  0x09
#define ACC_LSM303VAL_1KHZ   0x0A

#define ACC_LSM303_HIGHPASS  0x02 /* set high-pass cut-off */
                                  /* ODR = 50Hz  | 100Hz | 400Hz | 1kHZ  */
#define ACC_LSM303VAL_HP0    0x00 /*        1    |   2   |   8   | 20    */
#define ACC_LSM303VAL_HP1    0x01 /*        .5   |   1   |   4   | 10    */
#define ACC_LSM303VAL_HP2    0x02 /*        .25  |   .5  |   2   |  5    */
#define ACC_LSM303VAL_HP3    0x03 /*        .125 |   .25 |   1   |  2.5  */

#define ACC_LSM303_FILTER    0x03 /* enable/disable high- and low-pass      */
#define ACC_LSM303VAL_NONE   0x00
#define ACC_LSM303VAL_BOTH   0x03 /* enalbe low- and high-pass (i.e. bandpass) */

#endif
