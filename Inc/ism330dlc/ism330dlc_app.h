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

#define ISM330DLC_VARIANCE_MAX_DIMENSION	10000

uint8_t ISM330DLC_Init(ism330dlc_odr_xl_t odr_xl, ism330dlc_fs_xl_t fs_xl, ism330dlc_bw0_xl_t bw0_xl, ism330dlc_input_composite_t lpf1_bw_sel);
uint8_t ISM330DLC_ReadAcceleration(float* x, float* y, float* z);
uint8_t ISM330DLC_ReadAngularRate(float* arx, float* ary, float* arz);
uint8_t ISM330DLC_ReadTemperature(float* t);
void getDev_ctx(ism330dlc_ctx_t* d);
uint8_t ISM330DLC_ReadAccData();
void ISM330DLC_AccComputeStats(uint32_t* acc_samples, uint8_t acc_fullscale);
void ISM330DLC_GetMeanAcc(int16_t* buffer);
void ISM330DLC_GetVarAcc(double* buffer);
void ISM330DLC_GetStdAcc(uint16_t* buffer);
void ISM330DLC_GetMinAcc(int16_t* buffer);
void ISM330DLC_GetMaxAcc(int16_t* buffer);

void ISM330DLC_ResetAccInternals        (void);

void ISM330DLC_DataReadyCallback   (void);

#define     ACC_GET_ODR_CONFIG(odr)         (odr == 26 ? ISM330DLC_XL_ODR_26Hz : odr == 52 ? ISM330DLC_XL_ODR_52Hz : odr == 104 ? ISM330DLC_XL_ODR_104Hz : ISM330DLC_XL_ODR_208Hz)
#define     ACC_GET_FS_CONFIG(fs)           (fs == 2 ? ISM330DLC_2g : fs == 4 ? ISM330DLC_4g : fs == 8 ? ISM330DLC_8g : ISM330DLC_16g)
#define     ACC_GET_mg(fs, lsb)             (fs == 2 ? ISM330DLC_FROM_FS_2g_TO_mg(lsb) : fs == 4 ? ISM330DLC_FROM_FS_4g_TO_mg(lsb) : fs == 8 ? ISM330DLC_FROM_FS_8g_TO_mg(lsb) : ISM330DLC_FROM_FS_16g_TO_mg(lsb))

#endif /* ISM330DLC_ISM330DLC_APP_H_ */
