/*
 * BERKELEY.c
 *
 *  Created on: 6 Feb 2017
 *      Author: Giorgio
 */

#include "BERKELEY.h"

#include <string.h>

/**
   * @brief Reads tilt data from BERKELEY.
   * @param ptrRawData: memory address where data must be written to.
   * @retval HAL status
   */

void BERKELEY_AccBitDataRead(int16_t *ptrRawData)
{
	int idxTemp;
	uint8_t Buffer[4];

	memset(Buffer, 0x00, sizeof(Buffer));

	BERKELEY_ReadReg(BERKELEY_REG_ADDR_OUT_X_L_XL, Buffer, 4);

	for (idxTemp = 0; idxTemp < 2; idxTemp++)
	{ // data LSB @ lower address (little-endian)
		ptrRawData[idxTemp] = (int16_t) (((uint16_t) Buffer[2 * idxTemp + 1] << 8) + Buffer[2 * idxTemp]);
	}
	return;
}

/**
   * @brief Reads a specific register from the sensor via SPI.
   * @param cReadAddr: register of the address to read from.
   * @param pcBuffer: memory address where the bytes read will be written.
   * @param cNumByteToRead: number of bytes to be read.
   * @retval None
   */

void BERKELEY_ReadReg(uint8_t cReadAddr, uint8_t* pcBuffer, uint8_t cNumByteToRead)
{
	HAL_StatusTypeDef outOp = HAL_ERROR;
	uint8_t cReadByte;

	cReadAddr |= 0x80;

	HAL_GPIO_WritePin(BERK_CS_GPIO_Port, BERK_CS_Pin, GPIO_PIN_RESET);

	outOp = BERKELEYSpiTransmitReceiveBytes(&cReadAddr, &cReadByte, 1);
	cReadAddr = 0x00;
	while (cNumByteToRead >= 1 && outOp == HAL_OK)
	{
		outOp = BERKELEYSpiTransmitReceiveBytes(&cReadAddr, &cReadByte, 1);
		*pcBuffer = cReadByte;
		cNumByteToRead--;
		pcBuffer++;
	}

	HAL_GPIO_WritePin(BERK_CS_GPIO_Port, BERK_CS_Pin, GPIO_PIN_SET);

}

/**
   * @brief Writes to a specific register of the sensor via SPI.
   * @param cWriteAddr: register of the address to write to.
   * @param pcBuffer: memory address where the bytes to write will be read from.
   * @param cNumByteToWrite: number of bytes to be written.
   * @retval None
   */

void BERKELEYWriteReg(uint8_t cWriteAddr, uint8_t* pcBuffer, uint8_t cNumByteToWrite)
{
	HAL_StatusTypeDef outOp = HAL_ERROR;
	uint8_t cReadByte;

	HAL_GPIO_WritePin(BERK_CS_GPIO_Port, BERK_CS_Pin, GPIO_PIN_RESET);

	outOp = BERKELEYSpiTransmitReceiveBytes(&cWriteAddr, &cReadByte, 1);
	while (cNumByteToWrite >= 1 && outOp == HAL_OK)
	{
		outOp = BERKELEYSpiTransmitReceiveBytes(pcBuffer, &cReadByte, 1);
		cNumByteToWrite--;
		pcBuffer++;
	}

	HAL_GPIO_WritePin(BERK_CS_GPIO_Port, BERK_CS_Pin, GPIO_PIN_SET);

	return;
}

/**
   * @brief Configures the sensor.
   * @retval HAL status
   */

HAL_StatusTypeDef BERKELEY_Configure(uint16_t odr, uint16_t fs)
{
	uint8_t regCtrl1CARTValueToWrite     = 0;
	uint8_t regCtrl1CARTValueToRead      = 0;
	uint8_t regCtrl8CARTValueToWrite     = 0;
	uint8_t regCtrl8CARTValueToRead      = 0;
	uint8_t regCtrl11CARTValueToWrite    = 0;
    uint8_t regCtrl11CARTValueToRead     = 0;
    uint8_t regCtrl13CARTValueToWrite    = 0;
    uint8_t regCtrl13CARTValueToRead     = 0;
	BERKELEY_InitTypeDef BERKELEY_Config;

	BERKELEY_Config.Hp_Slope = BERKELEY_HPSLOPE_LP;
	BERKELEY_Config.Low_Pass1_Band_Width = BERKELEY_LPF1_HIGH;
	BERKELEY_Config.Low_Pass2_Band_Width = BERKELEY_LPF2_LOW;
	BERKELEY_Config.Output_Data_Rate = odr;
	BERKELEY_Config.Full_Scale = fs;
	BERKELEY_Config.Data_Ready = BERKELEY_DRDY_PULSED_EN;
	BERKELEY_Config.INT1_DRDY_XL = BERKELEY_INT1_DRDY_XL_MASK_EN;

	regCtrl1CARTValueToWrite  = 0;
	regCtrl8CARTValueToWrite  = 0;
	regCtrl11CARTValueToWrite = 0;
	regCtrl13CARTValueToWrite = 0;

	regCtrl1CARTValueToWrite = BERKELEY_Config.Full_Scale |
							   BERKELEY_Config.Low_Pass1_Band_Width |
							   BERKELEY_Config.Output_Data_Rate;


	regCtrl8CARTValueToWrite = BERKELEY_Config.Low_Pass2_Band_Width |
							   BERKELEY_Config.Hp_Slope;

	regCtrl11CARTValueToWrite = BERKELEY_Config.Data_Ready;

	regCtrl13CARTValueToWrite = BERKELEY_Config.INT1_DRDY_XL;

	BERKELEYWriteReg(BERKELEY_REG_ADDR_CTRL1_XL, &regCtrl1CARTValueToWrite, 0x01);
	BERKELEYWriteReg(BERKELEY_REG_ADDR_CTRL8_XL, &regCtrl8CARTValueToWrite, 0x01);
	BERKELEYWriteReg(BERKELEY_REG_DRDY_PULSE_CFG, &regCtrl11CARTValueToWrite, 0x01);
	BERKELEYWriteReg(BERKELEY_REG_INT1_CTRL, &regCtrl13CARTValueToWrite, 0x01);

	BERKELEY_ReadReg(BERKELEY_REG_ADDR_CTRL1_XL, &regCtrl1CARTValueToRead, 0x01);
	BERKELEY_ReadReg(BERKELEY_REG_ADDR_CTRL8_XL, &regCtrl8CARTValueToRead, 0x01);
	BERKELEY_ReadReg(BERKELEY_REG_DRDY_PULSE_CFG, &regCtrl11CARTValueToRead, 0x01);
	BERKELEY_ReadReg(BERKELEY_REG_INT1_CTRL, &regCtrl13CARTValueToRead, 0x01);

	if((regCtrl1CARTValueToRead == regCtrl1CARTValueToWrite) &&
	   (regCtrl8CARTValueToRead == regCtrl8CARTValueToWrite) &&
	   (regCtrl11CARTValueToRead == regCtrl11CARTValueToWrite) &&
	   (regCtrl13CARTValueToRead == regCtrl13CARTValueToWrite))
	return(HAL_OK);
	else
	return(HAL_ERROR);
}

/**
   * @brief Initializes the peripheral by checking the bytes of the who-am-I registers.
   * @retval HAL status
   */

HAL_StatusTypeDef BERKELEY_Initialize(void)
{
	uint8_t ReadValue = 0;

	BERKELEY_ReadReg(BERKELEY_REG_ADDR_WHO_AM_I, &ReadValue, 0x01);

	if(ReadValue == BERKELEY_REG_VAL_WHO_AM_I)
	return(HAL_OK);
	else
	return(HAL_ERROR);

}

/**
   * @brief Full duplex communication with the sensor via SPI.
   * @param txByte: pointer to the data to be sent.
   * @param rxByte: pointer to the data to be received.
   * @param numBytes: number of bytes to be read/written.
   * @retval HAL status
   */

HAL_StatusTypeDef BERKELEYSpiTransmitReceiveBytes(uint8_t *txByte, uint8_t *rxByte, uint8_t numBytes)
{
	return(HAL_SPI_TransmitReceive(&hspi2, txByte, rxByte, numBytes, 0xFFFFFFFF));
}

/**
   * @brief  Converts BERKELEY bits in mg
	 * @param  raw: acc in bits.
	 * @retval acc in mg.
   */

float BERKELEY_ConvertMG(int16_t raw, uint16_t fs)
{
    float acc_lsb2g = 0.0;

    switch(fs)
    {
        case 500:
            acc_lsb2g = LSB2G_05G;
            break;

        case 1000:
            acc_lsb2g = LSB2G_1G;
            break;

        case 2000:
            acc_lsb2g = LSB2G_2G;
            break;

        case 4000:
            acc_lsb2g = LSB2G_4G;
            break;

        default:
            return -1;
    }

    return raw * acc_lsb2g;
}

/**
   * @brief  Reads acc data from BERKELEY in LSB.
     * @param  raw_x: acc in LSB.
   * @param  raw_y: acc in LSB.
     * @retval HAL_StatusTypeDef.
   */

HAL_StatusTypeDef BERKELEY_GetRAWAcceleration(int16_t *raw_x, int16_t *raw_y)
{

    int16_t ptrRawData[2];


    BERKELEY_AccBitDataRead(ptrRawData);

    *raw_x = ptrRawData[0];
    *raw_y = ptrRawData[1];


    return HAL_OK;
}


/**
   * @brief  Reads acc data from BERKELEY in mg.
	 * @param  acc_x: acc in mg.
   * @param  acc_y: acc in mg.
	 * @retval HAL_StatusTypeDef.
   */

HAL_StatusTypeDef BERKELEY_GetAcceleration(float *acc_x, float *acc_y, uint16_t fs)
{

	int16_t ptrRawData[2];
	int16_t BERKELEY_RawX = 0;
    int16_t BERKELEY_RawY = 0;

	BERKELEY_AccBitDataRead(ptrRawData);

	BERKELEY_RawX = ptrRawData[0];
	BERKELEY_RawY = ptrRawData[1];

	*acc_x = BERKELEY_ConvertMG(BERKELEY_RawX, fs);
	*acc_y = BERKELEY_ConvertMG(BERKELEY_RawY, fs);

	return HAL_OK;
}
