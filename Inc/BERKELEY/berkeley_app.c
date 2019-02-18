/** **************************************************************************
 ** @file     berkeley_app.c
 ** @author   ST Microelectronics
 ** @brief    berkeley application
 **
 ** $Revision:: 1                    $
 ** $Date:: 2018-01-22 16:40:33 +0100 #$
 ** *************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "berkeley_app.h"
//#include "hal.h"
#include "main.h"

/** **************************************************************************
 ** @brief Private variables for berkeley tilt data
 ** *************************************************************************/
static int64_t tilt_accumulator[2] = {0};
static uint64_t tilt_square_accumulator[2] = {0};

static int16_t tilt_min_lsb[2] = {0};
static int16_t tilt_max_lsb[2] = {0};

static double tilt_mean[2] = {0.0};
static double tilt_var[2] = {0.0};
static double tilt_std[2] = {0.0};
static double tilt_min[2] = {0.0};
static double tilt_max[2] = {0.0};

/** **************************************************************************
 ** @brief berkeley data available trigger
 ** *************************************************************************/
volatile uint8_t tim_berkeley_read_tilt_data = 0;

/** **************************************************************************
 ** @brief Read tiltnetometer data and tiltumulate for averaging
 ** @param none
 ** @return 1
 ** *************************************************************************/
uint8_t BERKELEY_ReadTiltData(int16_t *tilt_buffer, uint8_t stats_flag)
{
    uint8_t j;

    BERKELEY_GetRAWAcceleration(&tilt_buffer[0], &tilt_buffer[1]);

    if(stats_flag)
    {
        for(j = 0; j < 2; j++)
        {
            tilt_accumulator[j] += tilt_buffer[j];
            tilt_square_accumulator[j] += (uint64_t)(tilt_buffer[j]* tilt_buffer[j]);

            if( (tilt_buffer[j] < tilt_min_lsb[j]) || (tilt_min_lsb[j] == 0))
            {
                tilt_min_lsb[j] = tilt_buffer[j];
            }
            else if( (tilt_buffer[j] > tilt_max_lsb[j])  || (tilt_max_lsb[j] == 0))
            {
                tilt_max_lsb[j] = tilt_buffer[j];
            }
        }
    }

    return 1;
}

/** **************************************************************************
 ** @brief Compute mean, variance and std for tilt data
 ** @param tilt_samples number of samples
 ** @param tilt_fullscale full scale in gauss
 ** @return none
 ** *************************************************************************/
void BERKELEY_TiltComputeStats(uint32_t tilt_samples, uint8_t tilt_fullscale)
{
    uint8_t i= 0;
    //float tilt_sensitivity = MAG_GET_SENSITIVITY(tilt_fullscale);

    for(i = 0; i < 2; i++ )
    {
        tilt_mean[i] = ((double)tilt_accumulator[i] / tilt_samples);

        tilt_var[i] = ( ((double)tilt_square_accumulator[i] / tilt_samples) - (tilt_mean[i] * tilt_mean[i]) );

        tilt_mean[i] = TILT_GET_mg(tilt_fullscale, tilt_mean[i]);

        tilt_var[i] = TILT_GET_mg(tilt_fullscale,TILT_GET_mg(tilt_fullscale, tilt_var[i]));

        tilt_std[i] = sqrt(tilt_var[i]);

        tilt_min[i] = TILT_GET_mg(tilt_fullscale, tilt_min_lsb[i]);

        tilt_max[i] = TILT_GET_mg(tilt_fullscale, tilt_max_lsb[i]);
    }
}

/** **************************************************************************
 ** @brief Get mean tilt data
 ** @param buffer
 ** @return none
 ** *************************************************************************/
void BERKELEY_GetMeanTilt(float *buffer)
{
    buffer[0] = (float)tilt_mean[0];
    buffer[1] = (float)tilt_mean[1];
}

/** **************************************************************************
 ** @brief Get std tilt data
 ** @param buffer
 ** @return none
 ** *************************************************************************/
void BERKELEY_GetStdTilt(float *buffer)
{
    buffer[0] = (float)tilt_std[0];
    buffer[1] = (float)tilt_std[1];
}

/** **************************************************************************
 ** @brief Get std mag data
 ** @param buffer
 ** @return none
 ** *************************************************************************/
void BERKELEY_GetMinTilt(float *buffer)
{
    buffer[0] = (float)tilt_min[0];
    buffer[1] = (float)tilt_min[1];
}

/** **************************************************************************
 ** @brief Get min mag data
 ** @param buffer
 ** @return none
 ** *************************************************************************/
void BERKELEY_GetMaxTilt(float *buffer)
{
    buffer[0] = (float)tilt_max[0];
    buffer[1] = (float)tilt_max[1];
}

/** **************************************************************************
 ** @brief Reset internal accumulators
 ** @param none
 ** @return none
 ** *************************************************************************/
void BERKELEY_ResetTiltInternals(void)
{
    uint8_t i;

    for(i = 0; i < 2; i++)
    {
        tilt_accumulator[i] = 0;
        tilt_square_accumulator[i] = 0;
    }
}

/** **************************************************************************
 ** @brief BERKELEY_ drdy callback
 ** @param none
 ** @return none
 ** *************************************************************************/
void BERKELEY_DataReadyCallback(void)
{
    tim_berkeley_read_tilt_data = 1;
}


void BERKELEY_Init(uint16_t odr, uint16_t fs)
{
    /*
    *  Initialize mems driver interface
    */
    BERKELEY_Configure(TILT_GET_ODR_CONFIG(odr), TILT_GET_FS_CONFIG(fs));
    BERKELEY_Initialize();

}

/**********************************************************************/
