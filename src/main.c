/**
 * @file   main.c
 * @brief  Application main.
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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* MCU support. */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "SPI_TZ10xx.h"
#include "Driver_I2C.h"
/* Board support. */
#include "TZ01_system.h"
#include "TZ01_console.h"
#include "MPU-9250.h"
#include "BMP280.h"
#include "utils.h"

#include "ble.h"
#include "pwm_out.h"

uint8_t buf[10];
uint8_t msg[80];
uint64_t uniq;

extern TZ10XX_DRIVER_PMU  Driver_PMU;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern TZ10XX_DRIVER_SPI  Driver_SPI3;  //9軸モーションセンサー
extern ARM_DRIVER_I2C     Driver_I2C1;  //気圧センサー
extern ARM_DRIVER_I2C     Driver_I2C2;  //充電IC

static bool init(void)
{
    uint8_t id = 0;
    uint32_t val;
    
    Usleep(500000);
    if (BLE_init_dev() == 1) {
        return false;
    }
    TZ01_system_init();
    TZ01_console_init();
    
    /* HyouRowGan ID */
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_6, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_7, 0);
    Driver_GPIO.Configure(6, GPIO_DIRECTION_INPUT_PULL_UP, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(7, GPIO_DIRECTION_INPUT_PULL_UP, GPIO_EVENT_DISABLE, NULL);
    
    Driver_GPIO.ReadPin(6, &val);
    id =  (val == 0) ? 0x01 : 0x00;
    Driver_GPIO.ReadPin(7, &val);
    id |= (val == 0) ? 0x02 : 0x00;
    
    //LED
    Driver_GPIO.Configure(11, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL); 
    
    /* Peripheral */
    //GPIO
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_16, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_17, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_18, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_19, 0);
    Driver_GPIO.Configure(16, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(17, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(18, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(19, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(20, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(21, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(22, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(23, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    
    //PWM
    pwm_out_init();
    
    //9軸モーションセンサー
    if (MPU9250_drv_init(&Driver_SPI3)) {
        MPU9250_drv_start_maesure(MPU9250_BIT_ACCEL_FS_SEL_16G, MPU9250_BIT_GYRO_FS_SEL_2000DPS, MPU9250_BIT_DLPF_CFG_20HZ, MPU9250_BIT_A_DLPFCFG_20HZ);
    } else {
        return false;
    }
    //気圧センサー
    if (BMP280_drv_init(&Driver_I2C1) == false) {
        return false;
    }
    
    //BLELib init
    BLE_init(id);
    
    return true;
}

int main(void)
{
    /* Initialize */
    if (init() != true) {
        TZ01_console_puts("init(): failed.\r\n");
        Driver_GPIO.WritePin(TZ01_SYSTEM_PWSW_PORT_HLD, 0);
        Driver_GPIO.WritePin(TZ01_SYSTEM_PWSW_PORT_LED, 0);
        goto term;
    }
    
    int div = Driver_PMU.GetPrescaler(PMU_CD_PPIER0);
    sprintf(msg, "SystemCoreClock=%ld PMU_CD_PPIER0 divider=%d\r\n", SystemCoreClock, div);
    TZ01_console_puts(msg);

    for (;;) {
        if (TZ01_system_run() == RUNEVT_POWOFF) {
            /* Power off operation detected */
            break;
        }
        
        if (BLE_main() < 0) {
            TZ01_console_puts("BLE_run() failed.\r\n");
        }
    }
    BLE_stop();
term:
    TZ01_console_puts("Program terminated.\r\n");
    return 0;
}
