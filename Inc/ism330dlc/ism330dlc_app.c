/*
 * ism330dlc_app2.c
 *
 *  Created on: 10 feb 2019
 *      Author: Emanuele
 */

#include <ism330dlc/ism330dlc_app.h>

static uint8_t whoamI, rst;
ism330dlc_ctx_t dev_ctx;
ism330dlc_ctrl1_xl_t dev_ctrl1_xl;
ism330dlc_ctrl2_g_t dev_ctrl2_g;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
float acceleration_mg[3];
float angular_rate_mdps[3];
float temperature_degC;

void getDev_ctx(ism330dlc_ctx_t* d)
{
	d = &dev_ctx;
}

uint8_t ISM330DLC_Init(ism330dlc_odr_xl_t odr_xl, ism330dlc_fs_xl_t fs_xl, ism330dlc_bw0_xl_t bw0_xl, ism330dlc_input_composite_t lpf1_bw_sel)
{
	dev_ctx.write_reg = ISM330DLC_Write_Reg;
	dev_ctx.read_reg = ISM330DLC_Read_Reg;
	dev_ctx.handle = &ISM330DLC_I2C_HANDLE;

	/*
	*  Check device ID
	*/
	whoamI = 0;
	ism330dlc_device_id_get(&dev_ctx, &whoamI);
//    if ( whoamI != ISM330DLC_ID )
//        return HAL_ERROR;

	dev_ctrl1_xl.odr_xl = odr_xl; // ISM330DLC_XL_ODR_12Hz5
	dev_ctrl1_xl.fs_xl = fs_xl; // ISM330DLC_2g
	dev_ctrl1_xl.bw0_xl = bw0_xl; // ISM330DLC_XL_ANA_BW_400Hz
	dev_ctrl1_xl.lpf1_bw_sel = lpf1_bw_sel;// ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100

	dev_ctrl2_g.odr_g = ISM330DLC_GY_ODR_12Hz5;
	dev_ctrl2_g.fs_g = ISM330DLC_2000dps;

	/*
	*  Restore default configuration
	*/
	ism330dlc_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		ism330dlc_reset_get(&dev_ctx, &rst);
	} while (rst);
	/*
	*  Enable Block Data Update
	*/
	ism330dlc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/*
	* Set Output Data Rate
	*/
	ism330dlc_xl_data_rate_set(&dev_ctx, dev_ctrl1_xl.odr_xl);
	ism330dlc_gy_data_rate_set(&dev_ctx, dev_ctrl2_g.odr_g);
	/*
	* Set full scale
	*/
	ism330dlc_xl_full_scale_set(&dev_ctx, dev_ctrl1_xl.fs_xl);
	ism330dlc_gy_full_scale_set(&dev_ctx, dev_ctrl2_g.fs_g);

	/*
	* Configure filtering chain(No aux interface)
	*/
	/* Accelerometer - analog filter */
	ism330dlc_xl_filter_analog_set(&dev_ctx, dev_ctrl1_xl.bw0_xl);

	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	//ism330dlc_xl_lp1_bandwidth_set(&dev_ctx, ISM330DLC_XL_LP1_ODR_DIV_4);

	/* Accelerometer - LPF1 + LPF2 path */
	ism330dlc_xl_lp2_bandwidth_set(&dev_ctx, dev_ctrl1_xl.lpf1_bw_sel);

	/* Accelerometer - High Pass / Slope path */
	//ism330dlc_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
	//ism330dlc_xl_hp_bandwidth_set(&dev_ctx, ISM330DLC_XL_HP_ODR_DIV_100);

	/* Gyroscope - filtering chain */
	ism330dlc_gy_band_pass_set(&dev_ctx, ISM330DLC_HP_260mHz_LP1_STRONG);

	return HAL_OK;

}

uint8_t ISM330DLC_ReadAcceleration(float* x, float* y, float* z)
{
	uint8_t err = 0;
	ism330dlc_status_reg_t reg;
	err = ism330dlc_status_reg_get(&dev_ctx, &reg);
	if(err != HAL_OK)
		return err;

	if (reg.xlda)
	{
		memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
		err = ism330dlc_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		if(err != HAL_OK)
			return err;
		switch(dev_ctrl1_xl.fs_xl)
		{
		case ISM330DLC_2g:
			  acceleration_mg[0] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[0]);
			  acceleration_mg[1] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[1]);
			  acceleration_mg[2] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[2]);
			break;
		case ISM330DLC_4g:
			  acceleration_mg[0] = ISM330DLC_FROM_FS_4g_TO_mg( data_raw_acceleration.i16bit[0]);
			  acceleration_mg[1] = ISM330DLC_FROM_FS_4g_TO_mg( data_raw_acceleration.i16bit[1]);
			  acceleration_mg[2] = ISM330DLC_FROM_FS_4g_TO_mg( data_raw_acceleration.i16bit[2]);
			break;
		case ISM330DLC_8g:
			  acceleration_mg[0] = ISM330DLC_FROM_FS_8g_TO_mg( data_raw_acceleration.i16bit[0]);
			  acceleration_mg[1] = ISM330DLC_FROM_FS_8g_TO_mg( data_raw_acceleration.i16bit[1]);
			  acceleration_mg[2] = ISM330DLC_FROM_FS_8g_TO_mg( data_raw_acceleration.i16bit[2]);
			break;
		case ISM330DLC_16g:
			  acceleration_mg[0] = ISM330DLC_FROM_FS_16g_TO_mg( data_raw_acceleration.i16bit[0]);
			  acceleration_mg[1] = ISM330DLC_FROM_FS_16g_TO_mg( data_raw_acceleration.i16bit[1]);
			  acceleration_mg[2] = ISM330DLC_FROM_FS_16g_TO_mg( data_raw_acceleration.i16bit[2]);
			break;
		default:
			break;
		}
	}
	*x = acceleration_mg[0];
	*y = acceleration_mg[1];
	*z = acceleration_mg[2];
	return HAL_OK;
}

uint8_t ISM330DLC_ReadAngularRate(float* arx, float* ary, float* arz)
{
	uint8_t err = 0;
	ism330dlc_status_reg_t reg;
	err = ism330dlc_status_reg_get(&dev_ctx, &reg);
	if(err != HAL_OK)
		return err;

	if (reg.gda)
	{
		memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
		ism330dlc_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
		if(err != HAL_OK)
			return err;
		switch(dev_ctrl2_g.fs_g)
		{
			case ISM330DLC_125dps:
				  angular_rate_mdps[0] = ISM330DLC_FROM_FS_125dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
				  angular_rate_mdps[1] = ISM330DLC_FROM_FS_125dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
				  angular_rate_mdps[2] = ISM330DLC_FROM_FS_125dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
				break;
			case ISM330DLC_250dps:
				  angular_rate_mdps[0] = ISM330DLC_FROM_FS_250dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
				  angular_rate_mdps[1] = ISM330DLC_FROM_FS_250dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
				  angular_rate_mdps[2] = ISM330DLC_FROM_FS_250dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
				break;
			case ISM330DLC_500dps:
				  angular_rate_mdps[0] = ISM330DLC_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
				  angular_rate_mdps[1] = ISM330DLC_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
				  angular_rate_mdps[2] = ISM330DLC_FROM_FS_500dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
				break;
			case ISM330DLC_1000dps:
				  angular_rate_mdps[0] = ISM330DLC_FROM_FS_1000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
				  angular_rate_mdps[1] = ISM330DLC_FROM_FS_1000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
				  angular_rate_mdps[2] = ISM330DLC_FROM_FS_1000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
				break;
			case ISM330DLC_2000dps:
				  angular_rate_mdps[0] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
				  angular_rate_mdps[1] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
				  angular_rate_mdps[2] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
				break;
			default:
				break;
		}
	}
	*arx = angular_rate_mdps[0];
	*ary = angular_rate_mdps[1];
	*arz = angular_rate_mdps[2];
	return HAL_OK;
}

uint8_t ISM330DLC_ReadTemperature(float* t)
{
	uint8_t err = 0;
	ism330dlc_status_reg_t reg;
	err = ism330dlc_status_reg_get(&dev_ctx, &reg);
	if(err != HAL_OK)
		return err;

	if (reg.tda)
	{
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		err = ism330dlc_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		if(err != HAL_OK)
			return err;
		temperature_degC = ISM330DLC_FROM_LSB_TO_degC( data_raw_temperature.i16bit );
	}
	*t = temperature_degC;
	return HAL_OK;
}
