/*
 * ism330dlc_app2.h
 *
 *  Created on: 10 feb 2019
 *      Author: Emanuele
 */

#ifndef ISM330DLC_ISM330DLC_APP_H_
#define ISM330DLC_ISM330DLC_APP_H_

#include "ism330dlc.h"
#include <stdlib.h>
#include "i2c.h"

#define ISM330DLC_I2C_HANDLE                 hi2c2
//#define ISM330DLC_SPI_HANDLE                 hspi3

uint8_t ISM330DLC_Init(ism330dlc_odr_xl_t odr_xl, ism330dlc_fs_xl_t fs_xl, ism330dlc_bw0_xl_t bw0_xl, ism330dlc_input_composite_t lpf1_bw_sel);
uint8_t ISM330DLC_ReadAcceleration(float* x, float* y, float* z);
uint8_t ISM330DLC_ReadAngularRate(float* arx, float* ary, float* arz);
uint8_t ISM330DLC_ReadTemperature(float* t);
void getDev_ctx(ism330dlc_ctx_t* d);

#endif /* ISM330DLC_ISM330DLC_APP_H_ */
