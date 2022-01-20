#include "bmx280.h"

#ifdef SET_BME280

//-----------------------------------------------------------------------------

static bmx280_calib_t calib;
result_t sens = {0.0};
regs_t regs;
//const uint32_t min_wait_ms = 150;
//const uint32_t max_wait_ms = 1000;
uint8_t bmx_data[24] = {0};

//-----------------------------------------------------------------------------
HAL_StatusTypeDef i2c_master_read_sensor(uint8_t reg, uint8_t *data_rd, uint16_t size)
{
HAL_StatusTypeDef rt = HAL_ERROR;

	if ((size) && (data_rd)) {
		rt  = HAL_I2C_Master_Transmit(portI2C, BMP280_ADDR << 1, &reg, 1, min_wait_ms);

		i2c_rxReady = 0;
		rt |= HAL_I2C_Master_Receive_DMA(portI2C, BMP280_ADDR << 1, data_rd, size);
		uint32_t sch = max_wait_ms;
		while (!i2c_rxReady && --sch) { HAL_Delay(1); }
		if (!sch) rt = HAL_TIMEOUT;

		//rt |= HAL_I2C_Master_Receive(portI2C, BMP280_ADDR << 1, data_rd, size, max_wait_ms);
		//i2c_rxReady = 1;
	}

    return rt;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef i2c_master_reset_sensor(uint8_t *chip_id)
{
HAL_StatusTypeDef rt = HAL_ERROR;
uint8_t dat[] = {BMP280_REG_RESET, BMP280_RESET_VALUE};

	if (chip_id) {
		rt = HAL_I2C_Master_Transmit(portI2C, BMP280_ADDR << 1, dat, 2, min_wait_ms);
		if (rt == HAL_OK) {
			dat[0] = BMP280_REG_ID;
			rt  = HAL_I2C_Master_Transmit(portI2C, BMP280_ADDR << 1, dat, 1, min_wait_ms);
			rt |= HAL_I2C_Master_Receive(portI2C, BMP280_ADDR << 1, chip_id, 1, min_wait_ms);
		}
	}

    return rt;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef i2c_master_test_sensor(regs_t *rg)
{
HAL_StatusTypeDef rt = HAL_OK;
uint8_t dat[] = {BMP280_REG_CTRL,
    		         BMP280_OSRS_T | BMP280_OSRS_P | BMP280_FORCED1_MODE,
					 BMP280_REG_CONFIG,
					 BMP280_CONF_T_SB | BMP280_CONF_FILTER | BMP280_CONF_SPI3W,
					 BME280_REG_CTRL_HUM, //for BME280_SENSOR only
					 BME280_OSRS_H};      //for BME280_SENSOR only
uint16_t len = sizeof(dat);

    if (rg->id != BME280_SENSOR) len -= 2;

    i2c_txReady = 0;
    rt = HAL_I2C_Master_Transmit_DMA(portI2C, BMP280_ADDR << 1, dat, len);
    uint32_t sch = max_wait_ms;
    while (!i2c_txReady && --sch) HAL_Delay(1);
    if (!sch) rt = HAL_TIMEOUT;
    //rt = HAL_I2C_Master_Transmit(portBMP, BMP280_ADDR << 1, dat, len, max_wait_ms);
    //i2c_txReady = 1;

	if (rt == HAL_OK) {
		HAL_Delay(50);
		dat[0] = BMP280_REG_STATUS;
		rt = HAL_I2C_Master_Transmit(portI2C, BMP280_ADDR << 1, dat, 1, min_wait_ms);
		rt |= HAL_I2C_Master_Receive(portI2C, BMP280_ADDR << 1, dat, 3, max_wait_ms);
		if (rt == HAL_OK) {
			rg->stat = dat[0];
			rg->mode = dat[1];
			rg->conf = dat[2];
		}
	}

    return rt;
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef bmx280_readCalibrationData(uint8_t chip_id)
{
HAL_StatusTypeDef err = HAL_ERROR;
//uint8_t data[24] = {0};

    if (i2c_master_read_sensor(BMP280_REG_CALIB, bmx_data, 24) == HAL_OK) {
        memset(&calib, 0, sizeof(bmx280_calib_t));
        calib.dig_T1 = (bmx_data[1] << 8) | bmx_data[0];
        calib.dig_T2 = (bmx_data[3] << 8) | bmx_data[2];
        calib.dig_T3 = (bmx_data[5] << 8) | bmx_data[4];
        calib.dig_P1 = (bmx_data[7] << 8) | bmx_data[6];
        calib.dig_P2 = (bmx_data[9] << 8) | bmx_data[8];
        calib.dig_P3 = (bmx_data[11] << 8) | bmx_data[10];
        calib.dig_P4 = (bmx_data[13] << 8) | bmx_data[12];
        calib.dig_P5 = (bmx_data[15] << 8) | bmx_data[14];
        calib.dig_P6 = (bmx_data[17] << 8) | bmx_data[16];
        calib.dig_P7 = (bmx_data[19] << 8) | bmx_data[18];
        calib.dig_P8 = (bmx_data[21] << 8) | bmx_data[20];
        calib.dig_P9 = (bmx_data[23] << 8) | bmx_data[22];

        if (chip_id == BME280_SENSOR) {//humidity
            // Read section 0xA1
            if (i2c_master_read_sensor(0xA1, bmx_data, 1) != HAL_OK) goto outm;
            calib.dig_H1 = bmx_data[0];
            // Read section 0xE1
            if (i2c_master_read_sensor(0xE1, bmx_data, 7) != HAL_OK) goto outm;
            calib.dig_H2 = (bmx_data[1] << 8) | bmx_data[0];
            calib.dig_H3 = bmx_data[2];
            calib.dig_H4 = (bmx_data[3] << 4) | (0x0f & bmx_data[4]);
            calib.dig_H5 = (bmx_data[5] << 4) | ((bmx_data[4] >> 4) & 0x0F);
            calib.dig_H6 = bmx_data[6];
        }

        err = HAL_OK;
    }

outm:

    return err;
}
//-----------------------------------------------------------------------------
void bmx280_CalcAll(result_t *ssen, int32_t chip_id, int32_t tp, int32_t pp, int32_t hh)
{
double var1, var2, p, var_H;
double t1, p1, h1 = -1.0;

	//Temp // Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
    var1 = (((double) tp) / 16384.0 - ((double)calib.dig_T1)/1024.0) * ((double)calib.dig_T2);
    var2 = ((((double) tp) / 131072.0 - ((double)calib.dig_T1)/8192.0) * (((double)tp)/131072.0 - ((double) calib.dig_T1)/8192.0)) * ((double)calib.dig_T3);
    // t_fine carries fine temperature as global value
    int32_t t_fine = (int32_t)(var1 + var2);
    t1 = (var1 + var2) / 5120.0;

    //Press // Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) calib.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) calib.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) calib.dig_P4) * 65536.0);
    var1 = (((double) calib.dig_P3) * var1 * var1 / 524288.0 + ((double) calib.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) calib.dig_P1);
    if (var1 == 0.0) {
        p = 0;
    } else {
        p = 1048576.0 - (double)pp;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double) calib.dig_P9) * p * p / 2147483648.0;
        var2 = p * ((double) calib.dig_P8) / 32768.0;
        p = p + (var1 + var2 + ((double) calib.dig_P7)) / 16.0;
    }
    p1 = (p/100) * 0.75006375541921;//convert hPa to mmHg

    if (chip_id == BME280_SENSOR) {// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
        var_H = (((double)t_fine) - 76800.0);
        var_H = (hh - (((double)calib.dig_H4) * 64.0 + ((double)calib.dig_H5) / 16384.0 * var_H)) *
                (((double)calib.dig_H2) / 65536.0 * (1.0 + ((double)calib.dig_H6) / 67108864.0 * var_H *
                (1.0 + ((double)calib.dig_H3) / 67108864.0 * var_H)));
        var_H = var_H * (1.0 - ((double)calib.dig_H1) * var_H / 524288.0);
        if (var_H > 100.0) {
        	var_H = 100.0;
        } else {
        	if (var_H < 0.0) var_H = 0.0;
        }
        h1 = var_H;
    }

    ssen->temp = (float)t1;
    ssen->pres = (float)p1;
    ssen->humi = (float)h1;

}

//-----------------------------------------------------------------------------

#endif

