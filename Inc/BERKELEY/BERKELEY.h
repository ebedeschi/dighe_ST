/*
 * BERKELEY.h
 *
 *  Created on: 6 Feb 2017
 *      Author: Giorgio
 */

#ifndef BERKELEY_BERKELEY_H_
#define BERKELEY_BERKELEY_H_

/*---- Includes ----*/

#include "main.h"
#include <stdint.h>
#include "spi.h"
/*---- Defines ----*/

/*-- Registers Addresses --*/

#define BERKELEY_REG_ADDR_CTRL1_XL			0x10
#define BERKELEY_REG_ADDR_CTRL6_XL			0x15
#define BERKELEY_REG_ADDR_CTRL8_XL			0x17
#define BERKELEY_REG_ADDR_OUT_X_L_XL        0x28
#define BERKELEY_REG_ADDR_OUT_X_H_XL        0x29
#define BERKELEY_REG_ADDR_OUT_Y_L_XL        0x2A
#define BERKELEY_REG_ADDR_OUT_Y_H_XL        0x2B
#define BERKELEY_REG_ADDR_WHO_AM_I          0x0F
#define BERKELEY_REG_DRDY_PULSE_CFG         0x0B
#define BERKELEY_REG_INT1_CTRL              0x0D

/*-- Registers Values --*/

#define BERKELEY_REG_VAL_WHO_AM_I           0x6B //new berkeley

/*-- Registers Parameters Masks and Values --*/

#define BERKELEY_LPF1_MASK                  ((uint8_t)0x02)

#define BERKELEY_LPF1_LOW 	                ((uint8_t)0x00)
#define BERKELEY_LPF1_HIGH    			    ((uint8_t)0x02)

#define BERKELEY_LPF2_MASK                  ((uint8_t)0x80)

#define BERKELEY_LPF2_LOW 	                ((uint8_t)0x00)
#define BERKELEY_LPF2_HIGH    			    ((uint8_t)0x80)

#define BERKELEY_HPSLOPE_MASK               ((uint8_t)0x04)

#define BERKELEY_HPSLOPE_LP	                ((uint8_t)0x00)
#define BERKELEY_HPSLOPE_HP                 ((uint8_t)0x04)

#define BERKELEY_ODR_MASK                   ((uint8_t)0xF0)

#define BERKELEY_ODR_104HZ	                ((uint8_t)0x40)
#define BERKELEY_ODR_208HZ                  ((uint8_t)0x50)

#define BERKELEY_FS_MASK                    ((uint8_t)0x0C)

#define BERKELEY_FS_05G              		((uint8_t)0x00)
#define BERKELEY_FS_1G                		((uint8_t)0x08)
#define BERKELEY_FS_2G                		((uint8_t)0x0C)
#define BERKELEY_FS_4G                		((uint8_t)0x04)

#define BERKELEY_DRDY_MASK                  ((uint8_t)0x80)

#define BERKELEY_DRDY_PULSED_EN	            ((uint8_t)0x80)

#define BERKELEY_INT1_DRDY_XL_MASK          ((uint8_t)0x01)

#define BERKELEY_INT1_DRDY_XL_MASK_EN	    ((uint8_t)0x01)

/*-- Sensitivities --*/

#define LSB2G_4G 						    0.000122F
#define LSB2G_2G 						    0.000061F
#define LSB2G_1G 						    0.0000305F
#define LSB2G_05G 						    0.00001525F


#define BERKELEY_FROM_FS_05G_TO_MG(lsb)     (lsb*LSB2G_05G*1000)
#define BERKELEY_FROM_FS_1G_TO_MG(lsb)      (lsb*LSB2G_1G*1000)
#define BERKELEY_FROM_FS_2G_TO_MG(lsb)      (lsb*LSB2G_2G*1000)
#define BERKELEY_FROM_FS_4G_TO_MG(lsb)      (lsb*LSB2G_4G*1000)

/*---- Variables ----*/

/*---- Structures ----*/

typedef struct
{
  uint8_t Low_Pass1_Band_Width;
  uint8_t Low_Pass2_Band_Width;
  uint8_t Hp_Slope;
  uint8_t Output_Data_Rate;
  uint8_t Full_Scale;
  uint8_t Data_Ready;
  uint8_t INT1_DRDY_XL;
} BERKELEY_InitTypeDef;

/*---- Prototypes ----*/

HAL_StatusTypeDef    BERKELEY_Configure(uint16_t odr, uint16_t fs);
HAL_StatusTypeDef    BERKELEY_Initialize(void);
HAL_StatusTypeDef    BERKELEYSpiTransmitReceiveBytes(uint8_t *txByte, uint8_t *rxByte, uint8_t numBytes);
HAL_StatusTypeDef    BERKELEY_GetRAWAcceleration(int16_t *raw_x, int16_t *raw_y);
HAL_StatusTypeDef    BERKELEY_GetAcceleration(float *acc_x, float *acc_y, uint16_t fs);
void                 BERKELEY_AccBitDataRead(int16_t *ptrRawData);
void                 BERKELEY_ReadReg(uint8_t cReadAddr, uint8_t* pcBuffer, uint8_t cNumByteToRead);
float                BERKELEY_ConvertMG(int16_t raw, uint16_t fs);

#endif /* BERKELEY_BERKELEY_H_ */
