/*
 * voltage.h
 *
 *  Created on: 15 gen 2019
 *      Author: Emanuele
 */

#ifndef ANALOG_ANALOG_H_
#define ANALOG_ANALOG_H_

#include "main.h"
#include "adc.h"

#define MAX_SAMPLE_ANALOG 100

uint8_t get420_1(uint16_t* val);
uint8_t get420_2(uint16_t* val);
uint8_t getVIN(uint16_t* val);
uint8_t getVSTEPUP(uint16_t* val);
uint8_t getADC(void* handler, uint16_t* val, ADC_ChannelConfTypeDef* sConfig);

#endif /* ANALOG_ANALOG_H_ */
