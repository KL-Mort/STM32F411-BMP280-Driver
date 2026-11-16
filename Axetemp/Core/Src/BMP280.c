#include "BMP280.h"
#include <stddef.h>

/*Чит регістра ID 0xD0*/
int32_t BMP280_ReadID(BMP280 *dev, uint8_t *chip_id)
{
	if(!dev || !dev -> bus.read || !chip_id)
	{
		return BMP280_ERR_ID;
	}
	return dev->bus.read(dev->bus.context, BMP280_REG_CHIP_ID, chip_id, 1 );
}


/*чит регістра статусу 0xF3*/
int32_t BMP280_GetStatus(BMP280 *dev, bool *is_measuring, bool *is_updating)
{
	if (!dev || !dev->bus.read){
		return BMP280_ERR;
	}
	uint8_t status=0;

	if (dev->bus.read(dev->bus.context, BMP280_REG_STATUS, &status, 1 )){
		return BMP280_ERR;
	}
	if (is_measuring){
		*is_measuring = (status & BMP280_STATUS_MEASURING) ? true : false;
	}
	if (is_updating){
			*is_updating = (status & BMP280_STATUS_IM_UPDATE) ? true : false;
		}

	return BMP280_OK;
}
/*запис інформації*/
int32_t BMP280_SetConfig(BMP280 *dev, const BMP280_Config *config){
	if (!dev || !dev->bus.write || !config){
		return BMP280_ERR;
	}
	// reg 0xF5 [7:5]standby   [4:2]filter    [0]spi3w_en
	uint8_t	reg_config = (uint8_t)((config->standby <<5)| (config->filter<<2));
	if (dev ->bus.write (dev->bus.context, BMP280_REG_CONFIG, &reg_config, 1) != 0){
		return BMP280_ERR;
	}
	uint8_t	reg_ctrl = (uint8_t)((config->osrs_t <<5)| (config->osrs_p<<2)|config->mode);
		if (dev ->bus.write (dev->bus.context, BMP280_REG_CTRL_MEAS, &reg_ctrl, 1) != 0){
			return BMP280_ERR;
		}
		return BMP280_OK;
}

int32_t BMP280_SetMode(BMP280 *dev, BMP280_Mode mode){
	if (!dev || !dev->bus.read || !dev->bus.write){
		return BMP280_ERR;
	}

	uint8_t reg_ctrl = 0;
	if (dev->bus.read(dev->bus.context, BMP280_REG_CTRL_MEAS, &reg_ctrl, 1) != 0){
		return BMP280_ERR;
	}

	reg_ctrl = (reg_ctrl & 0xFC) | ((uint8_t)mode & 0X03);
	if (dev->bus.write(dev->bus.context, BMP280_REG_CTRL_MEAS, &reg_ctrl, 1) !=0){
		return BMP280_ERR;
	}
	return BMP280_OK;
}


/*звірення коефіієнтів і id*/

int32_t BMP280_Init(BMP280 *dev){
	if (!dev || !dev->bus.read || !dev-> bus.write){
		return BMP280_ERR;
		}
//ID
	uint8_t chip_id =0;
	if( BMP280_ReadID(dev, &chip_id) !=0 || chip_id != BMP280_CHIP_ID_VAL){
		return BMP280_ERR_ID;
	}
/* калібрування 0x88*/
	uint8_t calib[28];
	if (dev->bus.read(dev->bus.context, BMP280_REG_CALIB_00, calib, 24) !=0 ){
		return BMP280_ERR;
	}
/*коефіцієнти */
	dev->dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
	dev->dig_T2 = (uint16_t)(calib[3] << 8 | calib[2]);
	dev->dig_T3 = (uint16_t)(calib[5] << 8 | calib[4]);

	dev->dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
	dev->dig_P2 = (uint16_t)(calib[9] << 8 | calib[8]);
	dev->dig_P3 = (uint16_t)(calib[11] << 8 | calib[10]);
	dev->dig_P4 = (uint16_t)(calib[13] << 8 | calib[12]);
	dev->dig_P5 = (uint16_t)(calib[15] << 8 | calib[14]);
	dev->dig_P6 = (uint16_t)(calib[17] << 8 | calib[16]);
	dev->dig_P7 = (uint16_t)(calib[19] << 8 | calib[18]);
	dev->dig_P8 = (uint16_t)(calib[21] << 8 | calib[20]);
	dev->dig_P9 = (uint16_t)(calib[23] << 8 | calib[22]);

	dev->t_fine=0;
		return BMP280_OK;
}

/*читання 6 байт 0xF7--0xFC*/

int32_t BMP280_ReadRaw(BMP280 *dev, int32_t *raw_temp, int32_t *raw_press){

	if(!dev || !dev->bus.read || !raw_temp|| !raw_press)
	{
		return BMP280_ERR;
	}

uint8_t raw[6];
	if (dev->bus.read(dev->bus.context, BMP280_REG_PRESS_MSB, raw, 6) !=0){
		return BMP280_ERR;
	}

	//значення АЦП
	*raw_press = (int32_t)((((uint32_t)raw[0]) << 12) | (((uint32_t)raw[1]) << 4) | (((uint32_t)raw[2]) >> 4));
	*raw_temp = (int32_t)((((uint32_t)raw[3]) << 12) | (((uint32_t)raw[4]) << 4) | (((uint32_t)raw[5]) >> 4));

	if (*raw_temp == 0 && *raw_press == 0){
		return BMP280_ERR;
	}
	return BMP280_OK;
}

/* формули компенсації*/

int32_t BMP280_Compen(BMP280 *dev, int32_t raw_temp, int32_t raw_press, float *temperature, float *pressure){
	if(!dev || !temperature || !pressure)
	{
		return BMP280_ERR;
	}
	// температурна компенсація
	int32_t var1 = ((((raw_temp>>3) -
				((int32_t)dev->dig_T1<<1))) *
				(int32_t)dev->dig_T2)>>11;
	int32_t var2 = (((((raw_temp >> 4) -
            	(int32_t)dev->dig_T1) *
				((raw_temp >> 4) -
				(int32_t)dev->dig_T1)) >> 12) *
				(int32_t)dev->dig_T3) >> 14;
	dev->t_fine = var1 + var2;
	*temperature = (float)((dev->t_fine * 5 + 128)>>8) / 100.0f;

	//компенсація тиску
	int64_t pvar1, pvar2, p;
	pvar1 = (int64_t)dev->t_fine - 128000;
	pvar2 = pvar1 * pvar1 * (int64_t)dev->dig_P6;
	pvar2 += (pvar1*(int64_t)dev->dig_P5)<<17;
	pvar2 += ((int64_t)dev->dig_P4) << 35;
	pvar1 = ((pvar1 * pvar1 * (int64_t)dev->dig_P3)>>8) + ((pvar1*(int64_t)dev->dig_P2)<<12);
	pvar1 = (((((int64_t)1) << 47) + pvar1)) * ((int64_t)dev->dig_P1) >> 33;

	if(pvar1 == 0){
		*pressure= 0.0f;
		return BMP280_ERR;
		}

	p = 1048576 - raw_press;
	p = (((p<<31)-pvar2)*3125)/pvar1;
	pvar1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
 	pvar2 = (((int64_t)dev->dig_P8) >> 8) + (((int64_t)dev->dig_P7) << 4);
	p = ((p+pvar1 +pvar2)>>8) + (((int64_t)dev->dig_P7) << 4);

	*pressure = (float)p / 256.0f / 100.0f;
	return BMP280_OK;

}

/*use function*/
int32_t BMP280_ReadData(BMP280 *dev, float *temperature, float *pressure)
{
	int32_t raw_temp = 0;
	int32_t raw_press = 0;

	if(BMP280_ReadRaw(dev, &raw_temp, &raw_press) != BMP280_OK)
	{
		return BMP280_ERR;
	}

	return BMP280_Compen(dev, raw_temp, raw_press, temperature, pressure);

}

int32_t BMP280_ReadDataForce(BMP280 *dev, float *tempurature, float *pressure){
	if (!dev|| !tempurature || !pressure){
		return BMP280_ERR;
	}
	if (BMP280_SetMode(dev, BMP280_MODE_FORCE)!= BMP280_OK){
		return BMP280_ERR;
	}
	bool is_meas = true;
	uint16_t time = 200;
	while (is_meas && time >0){
		if (BMP280_GetStatus (dev, &is_meas, NULL) != BMP280_OK){
			return BMP280_ERR;
		}
		time--;
	}
	if (time == 0){
		return BMP280_OK;
	}
	return BMP280_ReadData(dev, tempurature, pressure);
}






