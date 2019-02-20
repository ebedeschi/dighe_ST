/**
 ******************************************************************************
 * @file    ism330dlc.c
 * @author  FAE
 * @version V1.0.0
 * @date    12-June-2018
 * @brief   This file provides a set of functions needed to manage the ISM330DLC
 *          sensor
 ******************************************************************************
 * @attention
 * 
 * This file is Modified by FAE TECHNOLOGY starting from origin file supplied
 * by "MCD Application Team" of (COPYRIGHT(c) STMicroelectronics)
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "stm32l4xx_hal.h"

//#include "spi.h"
//#include "gpio.h"

#include "ism330dlc.h"

/* Private structure ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
//static float acceleration_mg[3];
//static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;

/**
 *  ISM330DLC device object
 */
ism330dlc_ctx_t dev_ism330dlc;


/* Extern variables ----------------------------------------------------------*/
#ifndef ISM330DLC_SPI
extern I2C_HandleTypeDef hi2c2;
#else
extern SPI_HandleTypeDef hspi3;
#endif

/* Private functions ---------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup ISM330DLC ISM330DLC
  * @{
  */

/** @defgroup ISM330DLC_Private_Init_Functions ISM330DLC Initialize Private Functions
  * @{
  */
/**
  * @brief  Initialize ISM330DLC driver interface and load linearization coefficient
  * @param  void* handle: read / write mcu interface handler
  */
int32_t ISM330DLC_BSP_init(void *handle)
{
  if (handle != NULL)
  {
    /* */
    dev_ism330dlc.handle = handle;
    dev_ism330dlc.write_reg = ISM330DLC_Write_Reg;
    dev_ism330dlc.read_reg = ISM330DLC_Read_Reg;
    /* Check device ID */
    whoamI = 0;
    ism330dlc_device_id_get(&dev_ism330dlc, &whoamI);
    if ( whoamI != ISM330DLC_ID )
    {
      return 0;
    }
    /* Restore default configuration */
    ism330dlc_reset_set(&dev_ism330dlc, PROPERTY_ENABLE);
    do
    {
      ism330dlc_reset_get(&dev_ism330dlc, &rst);
    } while(rst);
    /* Return OK */
    return 1;    
  }
  return 0;
}

/**
  * @brief  Start ISM330DLC humidity & temperature sensor.
  */
void ISM330DLC_Start(ism330dlc_odr_xl_t data_rate_acc, ism330dlc_odr_g_t data_rate_gy)
{
  /* Enable Block Data Update */
  ism330dlc_block_data_update_set(&dev_ism330dlc, PROPERTY_ENABLE);
  /* Set Output Data Rate for Accelerometer */
  ism330dlc_xl_data_rate_set(&dev_ism330dlc, data_rate_acc);
  /* Set Output Data Rate for Gyroscope */
  ism330dlc_gy_data_rate_set(&dev_ism330dlc, data_rate_gy);
  /* Set full scale for Accelerometer (minimum g for max scale) */  
  ism330dlc_xl_full_scale_set(&dev_ism330dlc, ISM330DLC_2g);
  /* Set full scale for Gyroscope */  
  ism330dlc_gy_full_scale_set(&dev_ism330dlc, ISM330DLC_2000dps);
  
  /* Configure filtering chain(No aux interface) */  
  /* Accelerometer - analog filter */
  ism330dlc_xl_filter_analog_set(&dev_ism330dlc, ISM330DLC_XL_ANA_BW_400Hz);
  
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //ism330dlc_xl_lp1_bandwidth_set(&dev_ism330dlc, ISM330DLC_XL_LP1_ODR_DIV_4);
  
  /* Accelerometer - LPF1 + LPF2 path */   
  ism330dlc_xl_lp2_bandwidth_set(&dev_ism330dlc, ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100);
  
  /* Accelerometer - High Pass / Slope path */
  //ism330dlc_xl_reference_mode_set(&dev_ism330dlc, PROPERTY_DISABLE);
  //ism330dlc_xl_hp_bandwidth_set(&dev_ism330dlc, ISM330DLC_XL_HP_ODR_DIV_100);
  
  /* Gyroscope - filtering chain */
  ism330dlc_gy_band_pass_set(&dev_ism330dlc, ISM330DLC_HP_260mHz_LP1_STRONG); 
}

/**
  * @brief  Start ISM330DLC humidity & temperature sensor.
  */
void ISM330DLC_Stop(void)
{
  /* Device power off */
//  lps22hb_data_rate_set(&dev_ism330dlc, ISM330DLC_POWER_DOWN);
}
/**
  * @}
  */
    
/** @defgroup ISM330DLC_Private_RW_Functions ISM330DLC Read/Write Private Functions
  * @{
  */
/**
  * @brief  Write data in to ISM330DLC registers.
  * @param  void* handle: read / write mcu interface handler
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  */
int32_t ISM330DLC_Write_Reg(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  HAL_StatusTypeDef hal_response = HAL_OK;

#ifndef ISM330DLC_SPI
  if (handle == &hi2c2)
  {
    /* enable auto incremented in multiple read/write commands */
//    Reg |= 0x80;
    hal_response = HAL_I2C_Mem_Write(handle, ISM330DLC_I2C_ADD_L, Reg,
                     I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  else
	  return HAL_ERROR;
#else
  if (handle == &hspi3)
  {
    /* Enable SPI Communication */
    HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
    /* */
    hal_response |= HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    hal_response |= HAL_SPI_Transmit(handle, Bufp, len, 1000);
    /* Disable SPI Communication */
    HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
  }
  else
  {
    return 0;
  }
#endif
  return hal_response;
}

/**
  * @brief  Read data from ISM330DLC registers.
  * @param  void* handle: read / write mcu interface handler
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  */
int32_t ISM330DLC_Read_Reg(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  HAL_StatusTypeDef hal_response = HAL_OK;

#ifndef ISM330DLC_SPI
  if (handle == &hi2c2)
  {
    /* enable auto incremented in multiple read/write commands */
//    Reg |= 0x80;
    hal_response = HAL_I2C_Mem_Read(handle, ISM330DLC_I2C_ADD_L, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  else
	  return HAL_ERROR;
#else
  if (handle == &hspi3)
  {
    /* Set Bit 7 for Data Read Operation */
    Reg |= 0x80;
    /* Enable SPI Communication */
    HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
    /* */
    hal_response |= HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    hal_response |= HAL_SPI_Receive(handle, Bufp, len, 1000);
    /* Disable SPI Communication */
    HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
  }
  else
	  return HAL_ERROR;
#endif
  return hal_response;
}
/**
  * @}
  */

/** @defgroup ISM330DLC_Acc&Gyro_Private_Functions ISM330DLC Accelerometer & Gyroscope Private Functions
  * @{
  */
/**
  * @brief  Read Accelerometer & Gyroscope value from ISM330DLC
  * @param  float* acc_buf: poiter to store Accelerometer data axes X,Y,Z;
  * @param  float* gy_buf: poiter to store Gyroscope data axes X,Y,Z;
  * @retval return operation succesfull
  */
int32_t ISM330DLC_Acc_Gyro_Read(float *acc_buf, float *gy_buf)
{
  ism330dlc_reg_t reg;
  
  /* Check Accelerometer and Gyroscope data ready */
  ism330dlc_status_reg_get(&dev_ism330dlc, &reg.status_reg);
  if ((reg.status_reg.xlda) & (reg.status_reg.gda))
  {
    /* Clear previus Acc data */
    memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
    /* Read Acc field data */
    ism330dlc_acceleration_raw_get(&dev_ism330dlc, data_raw_acceleration.u8bit);
    /* Conversion Acc in [mg] */
    acc_buf[0] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[0]);
    acc_buf[1] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[1]);
    acc_buf[2] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[2]);
    
    /* Clear previus Gyro data */
    memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
    /* Read Gyro field data */
    ism330dlc_angular_rate_raw_get(&dev_ism330dlc, data_raw_angular_rate.u8bit);
    /* Conversion Gyro in [mdps] */
    gy_buf[0] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
    gy_buf[1] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
    gy_buf[2] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
    /* */
    return 1;
  }
  /* */
  return 0;
}

/**
  * @}
  */ 

/** @defgroup ISM330DLC_Temperature_Private_Functions ISM330DLC Temperature Private Functions
  * @{
  */

/**
  * @brief  Read temperature value of ISM330DLC
  * @param  DeviceAddr: I2C device address
  * @retval temperature value
  */
float ISM330DLC_T_ReadTemp(void)
{
  ism330dlc_reg_t reg;
  
  temperature_degC = -500;    /* Set Invalid Temperature */
  /* Check data ready */
  ism330dlc_status_reg_get(&dev_ism330dlc, &reg.status_reg);
  if (reg.status_reg.tda)
  {
    /* Clear previus data */
    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    /* Read Temperature register */
    ism330dlc_temperature_raw_get(&dev_ism330dlc, data_raw_temperature.u8bit);
    /* Data conversion */
    temperature_degC = ISM330DLC_FROM_LSB_TO_degC( data_raw_temperature.i16bit );
  }
  /* */
  return temperature_degC;
}

/** @defgroup ISM330DLC_Interrupt_Functions ISM330DLC Interrupt Functions
  * @{
  */

/**
  * @brief  Enable or Disable Interrupt
  * @param  val: Property to set
  */
void ISM330DLC_IT_Set(uint8_t val)
{
  if (val == PROPERTY_ENABLE)
  {
//    /* Configure IT Pin Mode as OD */
//    lps22hb_pin_mode_set(&dev_ism330dlc, ISM330DLC_OPEN_DRAIN);
//    /* Configure IT Pin Polarity Active LOW */
//    lps22hb_int_polarity_set(&dev_ism330dlc, ISM330DLC_ACTIVE_LOW);
//    /* Configure IT on Data Ready */
//    lps22hb_int_pin_mode_set(&dev_ism330dlc, ISM330DLC_DRDY_OR_FIFO_FLAGS);
//    lps22hb_drdy_on_int_set(&dev_ism330dlc, PROPERTY_ENABLE);
//    /* Enable */
//    lps22hb_int_generation_set(&dev_ism330dlc, PROPERTY_ENABLE);
  }
  else
  {
    /* Disable All Interrupt */
    //lps22hb_int_generation_set(&dev_ism330dlc, PROPERTY_DISABLE);
  }
}

/**
  * @brief  Interrupt CallBack Function
  * @param  DeviceAddr: I2C device address
  * @retval temperature value
  */
__weak void ISM330DLC_IT_CallBack(uint32_t Pin)
{
//  if (Pin == INT1_AG_Pin)
//  {
//    /* User code here */
//  }
//  else if (Pin == INT2_AG_Pin)
//  {
//    /* User code here */
//  }
}

/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
