/**
 * @file   BQ24250.c
 * @brief  Battery charger(Ti BQ24250) driver for TZ10xx.
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

#include "BQ24250.h"

#include "utils.h"

static ARM_DRIVER_I2C *tz10xx_drv_i2c;


/* reset */
static bool bq24250_drv_reset(void)
{
    uint8_t val;
    
    val = BQ24250_drv_reg02_get();
    val |= 0x80;    /* set reset flag. */
    
    return BQ24250_drv_reg02_set(val);
}

/* Register #1 */
bool BQ24250_drv_reg01_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_01;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg01_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_01;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #2 */
bool BQ24250_drv_reg02_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_02;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg02_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_02;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #3 */
bool BQ24250_drv_reg03_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_03;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg03_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_03;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #4 */
bool BQ24250_drv_reg04_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_04;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg04_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_04;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #5 */
bool BQ24250_drv_reg05_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_05;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg05_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_05;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #6 */
bool BQ24250_drv_reg06_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_06;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg06_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_06;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/* Register #7 */
bool BQ24250_drv_reg07_set(uint8_t val)
{
    int len;
    uint8_t dat[2];
    
    dat[0] = BQ24250_REG_07;
    dat[1] = val;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 2, true);
    if (len < 2) {
        return false;
    }
    return true;
}

uint8_t BQ24250_drv_reg07_get(void)
{
    int len;
    uint8_t dat[1];
    
    dat[0] = BQ24250_REG_07;
    len = tz10xx_drv_i2c->SendData(BQ24250_I2C_ID, dat, 1, false);
    if (len < 1) {
        return 0xff;
    }
    
    len = tz10xx_drv_i2c->ReceiveData(BQ24250_I2C_ID, dat, 1, true);
    if (len < 1) {
        return 0xff;
    }
    
    return dat[0];
}

/*
 * Initialize.
 */
bool BQ24250_drv_init(ARM_DRIVER_I2C *i2c_drv, bool ts_enable)
{
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
    
    if (bq24250_drv_reset() == false) {
        return false;
    }
   
    if (ts_enable == false) {
        Usleep(200000);
        /* TS Disable */
        if (BQ24250_drv_reg06_set(BQ24250_DEF_06) == false) {
            return false;
        }
    }
    
    return true;
}
