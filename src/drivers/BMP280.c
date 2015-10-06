/**
 * @file   BMP280.c
 * @brief  Airpressure sensor(BOSH BMP280) driver for TZ10xx
 *
 * @author Cerevo Inc.
 */

/*
Copyright 2015 Cerevo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

/** Includes **/
#include <stdint.h>
#include <stdbool.h>
/* MCU support. */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "Driver_I2C.h"

#include "BMP280.h"

#include "utils.h"

static ARM_DRIVER_I2C *tz10xx_drv_i2c;

static int32_t t_fine = -1;

static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

/* reset */
bool BMP280_drv_reset(void)
{
    int len;
    uint8_t dat[2];
    /* reset */
    dat[0] = BMP280_REG_RESET;
    dat[1] = BMP280_BIT_RESET;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 2, false);
    if (len < 2) {
        return false;
    }
    return true;
}

/* id */
uint8_t BMP280_drv_id_get(void)
{
    int len;
    uint8_t dat[1];
    /* Select id register. */
    dat[0] = BMP280_REG_ID;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    /* Read register. */
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* status */
uint8_t BMP280_drv_status_get(void)
{
    int len;
    uint8_t dat[1];
    /* Select status register. */
    dat[0] = BMP280_REG_STATUS;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    /* Read register. */
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* ctrl_meas */
bool BMP280_drv_ctrl_meas_set(uint8_t val)
{
    int len;
    uint8_t dat[2];

    dat[0] = BMP280_REG_CTRL_MEAS;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 2, false);
    if (len < 2) {
        return false;
    }
    
    return true;
}

uint8_t BMP280_drv_ctrl_meas_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BMP280_REG_CTRL_MEAS;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* config */
bool BMP280_drv_config_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BMP280_REG_CONFIG;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 2, false);
    if (len < 2) {
        return false;
    }
    
    return true;
}

uint8_t BMP280_drv_config_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BMP280_REG_CONFIG;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* press */
uint32_t BMP280_drv_press_get(void)
{
    int len;
    uint8_t dat[3];
    int32_t adc_p;
    int64_t var1, var2, p;
    
    if (t_fine == -1) {
        /* Never maesured temperature */
        return 0xffffffff;
    }
    
    dat[0] = BMP280_REG_PRESS_MSB;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0xffffffff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 3, false);
    if (len < 3) {
        return 0xffffffff;
    }
    
    /* calc */
    
    adc_p = ((int32_t)dat[0] << 12) | ((int32_t)dat[1] << 4) | ((int32_t)dat[2] >> 4);
    /*
    var1 = ((int64_t)t_fine >> 1) - 128000;
    var2 = var1 + var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + ((int64_t)dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) {
        return 0;
    }
    
    p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
    return (uint32_t)p;
    */
    
	var1 = ((int64_t)t_fine) - 128000; 
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_p;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

	return (uint32_t)p;
    
}

/* temp */
int32_t BMP280_drv_temp_get(void)
{
    int len;
    uint8_t dat[3];
    int32_t adc_t;
    int32_t var1, var2;
    
    dat[0] = BMP280_REG_TEMP_MSB;
    len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, dat, 1, true);
    if (len < 1) {
        return 0x80000000;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, dat, 3, false);
    if (len < 3) {
        return 0x80000000;
    }
    
    /* calc */
    adc_t = ((int32_t)dat[0] << 12) | ((int32_t)dat[1] << 4) | ((int32_t)dat[2] >> 4);
    var1 = ((((adc_t >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
    var2 = (((((adc_t >> 4) - (int32_t)dig_T1) * ((adc_t >> 4) - (int32_t)dig_T1)) >> 12) * dig_T3) >> 14;

    t_fine = var1 + var2;
    
    return (t_fine * 5 + 128) >> 8;
}

/* */
static bool bmp280_drv_calib_prams_get(void)
{
    int len;
    uint8_t addr, dat[24];
    
    for (int i = 0; i < sizeof(dat); i++) {
        addr = BMP280_REG_CALIB00 + i;
        len = tz10xx_drv_i2c->SendData(BMP280_I2C_ADDR, &addr, 1, true);
        if (len < 1) {
            return false;
        }
        len = tz10xx_drv_i2c->ReceiveData(BMP280_I2C_ADDR, &dat[i], 1, false);
        if (len < 1) {
            return false;
        }
    }
    
    /* Temperature */
    dig_T1 = (dat[1] << 8) | dat[0];
    dig_T2 = (dat[3] << 8) | dat[2];
    dig_T3 = (dat[5] << 8) | dat[4];
    
    /* Airpressur */
    dig_P1 = (dat[ 7] << 8) | dat[ 6];
    dig_P2 = (dat[ 9] << 8) | dat[ 8];
    dig_P3 = (dat[11] << 8) | dat[10];
    dig_P4 = (dat[13] << 8) | dat[12];
    dig_P5 = (dat[15] << 8) | dat[14];
    dig_P6 = (dat[17] << 8) | dat[16];
    dig_P7 = (dat[19] << 8) | dat[18];
    dig_P8 = (dat[21] << 8) | dat[20];
    dig_P9 = (dat[23] << 8) | dat[22];
    
    return true;
}

/*
 * Initialize.
 */
bool BMP280_drv_init(ARM_DRIVER_I2C *i2c_drv)
{
    uint8_t id = 0x00;
    
    tz10xx_drv_i2c = i2c_drv;
    
    if (tz10xx_drv_i2c->Initialize(NULL) != ARM_I2C_OK) {
        return false;
    }
    
    if (tz10xx_drv_i2c->PowerControl(ARM_POWER_FULL) != ARM_I2C_OK) {
        return false;
    }
    
    if (tz10xx_drv_i2c->BusSpeed(ARM_I2C_BUS_SPEED_STANDARD) != ARM_I2C_OK) {
        return false;
    }
    
    /* reset */
    BMP280_drv_reset();
    for (int i = 0; i < 100; i++) {
        id = BMP280_drv_id_get();
        if (id == 0x58) {
            break;
        }
        Usleep(1000);
    }
    if (id != 0x58) {   /* BMP280*/
        /* timeout */
        return false;
    }

    Usleep(1000);

    /* read calibration datas. */
    if (bmp280_drv_calib_prams_get() == false) {
        return false;
    }
    
    /* startup */
    BMP280_drv_ctrl_meas_set(0x2f); /* Orversampling temp x1 press x4, NormalMode */    
    BMP280_drv_config_set(0x80);    /* tstandby=500ms, filter off */
    Usleep(500);    /* Wait one `tstandby' cycle. */
        
    /* Get initial Temperature. */
    return (BMP280_drv_temp_get() != 0x80000000);
}
