/*
 * voltage.c
 *
 *  Created on: 15 gen 2019
 *      Author: Emanuele
 */

#include <analog/analog.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;

uint8_t get420_1(uint16_t* val)
{
	uint8_t ret = 0;
	HAL_OPAMP_Start(&hopamp1);

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	ret = getADC(&hadc1, val, &sConfig);
	HAL_OPAMP_Stop(&hopamp1);
	return ret;
}

uint8_t get420_2(uint16_t* val)
{
	uint8_t ret = 0;

	HAL_OPAMP_Start(&hopamp2);

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	ret = getADC(&hadc2, val, &sConfig);
	HAL_OPAMP_Stop(&hopamp2);
	return ret;
}

uint8_t getVIN(uint16_t* val)
{
	uint8_t ret = 0;
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_GPIO_WritePin(EN_ADC_VIN_GPIO_Port, EN_ADC_VIN_Pin, GPIO_PIN_SET);
	HAL_Delay(300);
	ret = getADC(&hadc2, val, &sConfig);
	HAL_GPIO_WritePin(EN_ADC_VIN_GPIO_Port, EN_ADC_VIN_Pin, GPIO_PIN_RESET);
	return ret;
}

uint8_t getVSTEPUP(uint16_t* val)
{
	uint8_t ret = 0;
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	ret = getADC(&hadc1, val, &sConfig);
	return ret;
}

uint8_t getADC(void* handler, uint16_t* val, ADC_ChannelConfTypeDef* sConfig)
{
	uint32_t adc_value=0;
	long sum=0;
	if (HAL_ADCEx_Calibration_Start((ADC_HandleTypeDef*)handler, ADC_SINGLE_ENDED) !=  HAL_OK)
	{
		/* ADC Calibration Error */
		Error_Handler();
	}
	for(long i=0;i<MAX_SAMPLE_ANALOG;i++)
	{

		if (HAL_ADC_Start((ADC_HandleTypeDef*)handler) != HAL_OK)
		{
		 /* Start Conversation Error */
		 Error_Handler();
		}

		/*##-4- Wait for the end of conversion #####################################*/
		/*  For simplicity reasons, this example is just waiting till the end of the
		   conversion, but application may perform other tasks while conversion
		   operation is ongoing. */
		if (HAL_ADC_PollForConversion((ADC_HandleTypeDef*)handler, 10) != HAL_OK)
		{
		 /* End Of Conversion flag not set on time */
		 Error_Handler();
		}

		/* Check if the continous conversion of regular channel is finished */
		if ((HAL_ADC_GetState((ADC_HandleTypeDef*)handler) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
		{
		 /*##-5- Get the converted value of regular channel  ########################*/
		 adc_value = HAL_ADC_GetValue((ADC_HandleTypeDef*)handler);
		 sum+=adc_value;
		}

		if (HAL_ADC_Stop((ADC_HandleTypeDef*)handler) != HAL_OK)
		{
		 /* Start Conversation Error */
		 Error_Handler();
		}

	}

	sum/=MAX_SAMPLE_ANALOG;
	*val = sum;
	return 0;
}





