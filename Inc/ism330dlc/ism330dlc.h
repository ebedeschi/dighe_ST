/**
 ******************************************************************************
 * @file    ism330dlc.h
 * @author  FAE
 * @version V1.0.0
 * @date    12-June-2018
 * @brief   ISM330DLC header driver file
 ******************************************************************************
 * @attention
 * 
 * This file is Modified by FAE TECHNOLOGY starting from origin file supplied
 * by "MCD Application Team" of (COPYRIGHT(c) STMicroelectronics)
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ISM330DLC__H
#define __ISM330DLC__H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "ism330dlc_reg.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @addtogroup ISM330DLC
  * @{
  */

/** @defgroup ISM330DLC_Exported_Macros ISM330DLC Exported Macros
  * @{
  */
  
/**
* @brief  Bitfield positioning.
*/
#define ISM330DLC_BIT(x) ((uint8_t)x)
/**
  * @}
  */ 

/** @defgroup ISM330DLC_Exported_Structure ISM330DLC Exported Structure
  * @{
  */
  
/**
* @brief  Structure for manage ISM330DLC device
*/
typedef struct
{
  /** ism330dlc_interface */
  ism330dlc_ctx_t dev;
  /** Interrupt callback optional pointer **/
  void *callback_it_1;
  void *callback_it_2;
}ism330dlc_device_t;
  
/**
  * @}
  */ 
  

/** @defgroup ISM330DLC_Exported_Constants ISM330DLC Exported Constants
  * @{
  */

/**
  * @brief Device Identification register.
  *        Read
  *        Default value: 0xB1
  *        7:0 This read-only register contains the device identifier that, for ISM330DLC, is set to B1h.
  */
#ifndef ISM330DLC_WHO_AM_I_REG
  #define ISM330DLC_WHO_AM_I_REG         (uint8_t)0x0F
#endif

/**
  * @brief Device Identification value.
  */
#define ISM330DLC_WHO_AM_I_VAL         (uint8_t)0x6A

/**
  * @}
  */



/** @defgroup ISM330DLC_Private_Init_Functions ISM330DLC Initialize Private Functions
  * @{
  */
int32_t ISM330DLC_BSP_init(void *handle);
void ISM330DLC_Start(ism330dlc_odr_xl_t data_rate_acc, ism330dlc_odr_g_t data_rate_gy);
void ISM330DLC_Stop(void);
/**
  * @}
  */

/** @defgroup ISM330DLC_Private_RW_Functions ISM330DLC Read/Write Private Functions
  * @{
  */
int32_t ISM330DLC_Write_Reg(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t ISM330DLC_Read_Reg(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
/**
  * @}
  */
    
/** @defgroup ISM330DLC_Acc&Gyro_Private_Functions ISM330DLC Accelerometer & Gyroscope Private Functions
  * @{
  */
int32_t ISM330DLC_Acc_Gyro_Read(float *acc_buf, float *gy_buf);
/**
  * @}
  */

/** @defgroup ISM330DLC_Temperature_Private_Functions ISM330DLC Temperature Private Functions
  * @{
  */
float ISM330DLC_T_ReadTemp(void);
/**
  * @}
  */

/** @defgroup ISM330DLC_Interrupt_Functions ISM330DLC Interrupt Functions
  * @{
  */
void ISM330DLC_IT_Set(uint8_t val);
__weak void ISM330DLC_IT_CallBack(uint32_t Pin);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __ISM330DLC__H */

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
