/** **************************************************************************
 ** @file     berkeley_app.h
 ** @author   ST Microelectronics
 ** @brief    berkeley application header
 **
 ** $Revision:: 1                    $
 ** $Date:: 2018-01-22 16:40:33 +0100 #$
 ** *************************************************************************/

#include "BERKELEY.h"
#include <stdlib.h>

uint8_t     BERKELEY_ReadTiltData                (int16_t *tilt_buffer, uint8_t stats_flag);
void        BERKELEY_TiltComputeStats            (uint32_t tilt_samples, uint8_t tilt_fullscale);
void        BERKELEY_GetMeanTilt                 (float *buffer);
void        BERKELEY_GetStdTilt                  (float *buffer);
void        BERKELEY_GetMinTilt                  (float *buffer);
void        BERKELEY_GetMaxTilt                  (float *buffer);

void        BERKELEY_ResetTiltInternals          (void);

void        BERKELEY_DataReadyCallback         (void);

void        BERKELEY_Init(uint16_t odr, uint16_t fs);


#define     TILT_GET_ODR_CONFIG(odr)         (odr == 104 ? BERKELEY_ODR_104HZ : BERKELEY_ODR_208HZ)
#define     TILT_GET_FS_CONFIG(fs)           (fs == 500 ? BERKELEY_FS_05G : fs == 1000 ? BERKELEY_FS_1G : fs == 2000 ? BERKELEY_FS_2G : BERKELEY_FS_4G)
#define     TILT_GET_mg(fs, lsb)             (fs == 500 ? BERKELEY_FROM_FS_05G_TO_MG(lsb) : fs == 1000 ? BERKELEY_FROM_FS_1G_TO_MG(lsb) : fs == 2 ? BERKELEY_FROM_FS_2G_TO_MG(lsb) : BERKELEY_FROM_FS_4G_TO_MG(lsb))

/**********************************************************************/
