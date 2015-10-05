/**
 * @file   TZ01_airpressure.c
 * @brief  Airpressure sensor library for Cerevo CDP-TZ01B.
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

#include "BMP280.h"
#include "TZ01_airpressure.h"

extern ARM_DRIVER_I2C Driver_I2C1;

/*
 * Initialize airpressure sensor library.
 */
bool TZ01_airpressure_init(void)
{
    if (BMP280_drv_init(&Driver_I2C1) == false) {
        return false;
    }
        
    return true;
}

/*
 * Read temperature value.(UNIT: digree Celsius)
 */
float TZ01_airpressure_temp_read(void)
{
    uint32_t t;
    t = BMP280_drv_temp_get();
        
    return (float)t / 100.0f;
}

/*
 * Read airpressure value.(UNIT: Pa)
 */
float TZ01_airpressure_press_read(void)
{
    uint32_t p;
    p = BMP280_drv_press_get();
    
    return (float)p / 256.0f;
}
