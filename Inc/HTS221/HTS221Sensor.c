/**
 ******************************************************************************
 * @file    HTS221Sensor.cpp
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Implementation of a HTS221 Humidity and Temperature sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "HTS221Sensor.h"

extern uint8_t flag;
extern void vcom_Send( char *format, ... );

/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
void HTS221Sensor(void* handle, uint8_t address)
{
  _address = address;
  _handle = handle;

    /* Power down the device */
  if ( HTS221_DeActivate( _handle ) == HTS221_ERROR )
  {
    return;
  }

  /* Enable BDU */
  if ( HTS221_Set_BduMode( _handle, HTS221_ENABLE ) == HTS221_ERROR )
  {
    return;
  }
}

/**
 * @brief  Enable HTS221
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorEnable(void)
{

  if ( HTS221_Set_PowerDownMode(_handle, HTS221_SET) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  if ( HTS221_Set_Odr(_handle, HTS221_ODR_12_5HZ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  if ( HTS221_MemoryBoot(_handle) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  /* Set Temperature Resolution */
  if( HTS221_Set_AvgT(_handle, HTS221_AVGT_128) == HTS221_ERROR )
  {
	return HTS221_STATUS_ERROR;
  }

  /* Set Humidity Resolution */
  if( HTS221_Set_AvgH(_handle, HTS221_AVGH_64) == HTS221_ERROR )
  {
	return HTS221_STATUS_ERROR;
  }

  /* Power up the device */
  if ( HTS221_Activate( _handle ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

/**
 * @brief  Disable HTS221
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorDisable(void)
{
  /* Power up the device */
  if ( HTS221_DeActivate( _handle ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

/**
 * @brief  Read ID address of HTS221
 * @param  ht_id the pointer where the ID of the device is stored
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorReadID(uint8_t *ht_id)
{
  /* Read WHO AM I register */
  if ( HTS221_Get_DeviceID( _handle, ht_id ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

/**
 * @brief  Reboot memory content of HTS221
 * @param  None
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorReset(void)
{
    uint8_t tmpreg;

    /* Read CTRL_REG2 register */
    if (HTS221SensorReadReg(HTS221_CTRL_REG2, &tmpreg) != HTS221_STATUS_OK)
    {
      return HTS221_STATUS_ERROR;
    }

    /* Enable or Disable the reboot memory */
    tmpreg |= (0x01 << HTS221_BOOT_BIT);

    /* Write value to MEMS CTRL_REG2 regsister */
	if (HTS221SensorWriteReg(HTS221_CTRL_REG2, tmpreg) != HTS221_STATUS_OK)
    {
      return HTS221_STATUS_ERROR;
    }

    return HTS221_STATUS_OK;
}

/**
 * @brief  Read HTS221 output register, and calculate the humidity
 * @param  pfData the pointer to data output
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorGetHumidity(float* pfData)
{
  uint16_t uint16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Humidity( _handle, &uint16data ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  *pfData = ( float )uint16data / 10.0f;

  return HTS221_STATUS_OK;
}

/**
 * @brief  Read HTS221 output register, and calculate the temperature
 * @param  pfData the pointer to data output
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorGetTemperature(float* pfData)
{
  int16_t int16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Temperature( _handle, &int16data ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  *pfData = ( float )int16data / 10.0f;

  return HTS221_STATUS_OK;
}

/**
 * @brief  Read HTS221 output register, and calculate the humidity
 * @param  odr the pointer to the output data rate
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorGetODR(float* odr)
{
  HTS221_Odr_et odr_low_level;

  if ( HTS221_Get_Odr( _handle, &odr_low_level ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case HTS221_ODR_ONE_SHOT:
      *odr =  0.0f;
      break;
    case HTS221_ODR_1HZ     :
      *odr =  1.0f;
      break;
    case HTS221_ODR_7HZ     :
      *odr =  7.0f;
      break;
    case HTS221_ODR_12_5HZ  :
      *odr = 12.5f;
      break;
    default                 :
      *odr = -1.0f;
      return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

/**
 * @brief  Set ODR
 * @param  odr the output data rate to be set
 * @retval HTS221_STATUS_OK in case of success, an error code otherwise
 */
HTS221StatusTypeDef HTS221SensorSetODR(float odr)
{
  HTS221_Odr_et new_odr;

  new_odr = ( odr <= 1.0f ) ? HTS221_ODR_1HZ
          : ( odr <= 7.0f ) ? HTS221_ODR_7HZ
          :                   HTS221_ODR_12_5HZ;

  if ( HTS221_Set_Odr( _handle, new_odr ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}


/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval HTS221_STATUS_OK in case of success
 * @retval HTS221_STATUS_ERROR in case of failure
 */
HTS221StatusTypeDef HTS221SensorReadReg( uint8_t reg, uint8_t *data )
{

  if ( HTS221_ReadReg( _handle, reg, 1, data ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval HTS221_STATUS_OK in case of success
 * @retval HTS221_STATUS_ERROR in case of failure
 */
HTS221StatusTypeDef HTS221SensorWriteReg( uint8_t reg, uint8_t data )
{

  if ( HTS221_WriteReg( _handle, reg, 1, &data ) == HTS221_ERROR )
  {
    return HTS221_STATUS_ERROR;
  }

  return HTS221_STATUS_OK;
}

uint8_t HTS221_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  return IO_Write(handle, pBuffer, WriteAddr, nBytesToWrite);
}

uint8_t HTS221_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return IO_Read(handle, pBuffer, ReadAddr, nBytesToRead);
}

/**
 * @brief Utility function to read data.
 * @param  pBuffer: pointer to data to be read.
 * @param  RegisterAddr: specifies internal address register to be read.
 * @param  NumByteToRead: number of bytes to be read.
 * @retval 0 if ok, an error code otherwise.
 */
uint8_t IO_Read(void* handle, uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	if (handle == &hi2c2)
	{
	  status = HAL_I2C_Mem_Read((I2C_HandleTypeDef*)handle, (uint8_t)_address, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, 1000);
	}
	return status;
}

/**
 * @brief Utility function to write data.
 * @param  pBuffer: pointer to data to be written.
 * @param  RegisterAddr: specifies internal address register to be written.
 * @param  NumByteToWrite: number of bytes to write.
 * @retval 0 if ok, an error code otherwise.
 */
uint8_t IO_Write(void* handle, uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  if (handle == &hi2c2)
  {
	  status = HAL_I2C_Mem_Write((I2C_HandleTypeDef*)handle, (uint8_t)_address, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite, 1000);
  }
  return status;
}
