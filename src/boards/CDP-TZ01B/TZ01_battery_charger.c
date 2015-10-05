/**
 * @file   TZ01_battery_charger.c
 * @brief  LiPo battery charger library for Cerevo CDP-TZ1B
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
#include "Driver_I2C.h"

#include "BQ24250.h"

extern ARM_DRIVER_I2C Driver_I2C2;

static uint8_t regs[7];

/*
 * Initialize battery charger library.
 */
bool TZ01_battery_charger_init(bool thermistor_enable)
{
    for (int i = 0; i < sizeof(regs); i++) {
        regs[i] = 0x00;
    }
    
    if (BQ24250_drv_init(&Driver_I2C2, thermistor_enable) == false) {
        return false;
    }
    
    return true;
}

/*
 * Set/Restore default configuration.
 */
bool TZ01_battery_charger_set_configs(void)
{
    if (BQ24250_drv_reg01_set(BQ24250_DEF_01) == false) {
        return false;
    }
    if (BQ24250_drv_reg02_set(BQ24250_DEF_02) == false) {
        return false;
    }
    if (BQ24250_drv_reg03_set(BQ24250_DEF_03) == false) {
        return false;
    }
    if (BQ24250_drv_reg04_set(BQ24250_DEF_04) == false) {
        return false;
    }
    if (BQ24250_drv_reg05_set(BQ24250_DEF_05) == false) {
        return false;
    }
    if (BQ24250_drv_reg06_set(BQ24250_DEF_06) == false) {
        return false;
    }
    if (BQ24250_drv_reg07_set(BQ24250_DEF_07) == false) {
        return false;
    }
    
    return true;
}

/*
 * Get configurations.
 */
uint8_t* TZ01_battery_charger_get_configs(void)
{
    regs[0] = BQ24250_drv_reg01_get();
    regs[1] = BQ24250_drv_reg02_get();
    regs[2] = BQ24250_drv_reg03_get();
    regs[3] = BQ24250_drv_reg04_get();
    regs[4] = BQ24250_drv_reg05_get();
    regs[5] = BQ24250_drv_reg06_get();
    regs[6] = BQ24250_drv_reg07_get();
    
    return regs;
}
