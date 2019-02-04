/*
 * voltage.c
 *
 *  Created on: 15 gen 2019
 *      Author: Emanuele
 */

#include "voltage.h"




uint8_t get420(uint16_t* val)
{
//	uint32_t adc_value=0;
//	long sum=0;
//	for(long i=0;i<MAX_SAMPLE;i++)
//	{
//
//		if (HAL_ADC_Start(&hadc1) != HAL_OK)
//		{
//		 /* Start Conversation Error */
//		 Error_Handler();
//		}
//
//		/*##-4- Wait for the end of conversion #####################################*/
//		/*  For simplicity reasons, this example is just waiting till the end of the
//		   conversion, but application may perform other tasks while conversion
//		   operation is ongoing. */
//		if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
//		{
//		 /* End Of Conversion flag not set on time */
//		 Error_Handler();
//		}
//
//		/* Check if the continous conversion of regular channel is finished */
//		if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
//		{
//		 /*##-5- Get the converted value of regular channel  ########################*/
//		 adc_value = HAL_ADC_GetValue(&hadc1);
//		 sum+=adc_value;
//		}
//
//		if (HAL_ADC_Stop(&hadc1) != HAL_OK)
//		{
//		 /* Start Conversation Error */
//		 Error_Handler();
//		}
//
//	}
//
//	sum/=MAX_SAMPLE;
//	*val = sum;
	return 0;
}

