/**
 * @file   TZ01_motion_tracker.c
 * @brief  Motion tracker library for Cerevo CDP-TZ01B.
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
#include <math.h>
/* MCU support. */
#include "TZ10xx.h"
#include "SPI_TZ10xx.h"

#include "MPU-9250.h"
#include "TZ01_motion_tracker.h"

extern TZ10XX_DRIVER_SPI Driver_SPI3;

/*
 * Initialize Motion tracker library.
 */
bool TZ01_motion_tracker_init(void)
{
    if (MPU9250_drv_init(&Driver_SPI3) == false) {
        return false;
    }
    if (MPU9250_drv_start_maesure(MPU9250_BIT_ACCEL_FS_SEL_16G, MPU9250_BIT_GYRO_FS_SEL_2000DPS, MPU9250_BIT_DLPF_CFG_20HZ, MPU9250_BIT_A_DLPFCFG_20HZ) == false) {
        return false;
    }
    return true;
}

/*
 * Read Gyro
 *  rx, ry, rz: raw value.
 *  x, y, z   : computed value.(UNIT: dig/sec)
 */
bool TZ01_motion_tracker_gyro_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z)
{
    MPU9250_gyro_val gyro;
    gyro.raw_x = 0;
    gyro.raw_y = 0;
    gyro.raw_z = 0;

    if (MPU9250_drv_read_gyro(&gyro)) {
        if (rx != NULL) {
            *rx = gyro.raw_x;
        }
        if (ry != NULL) {
            *ry = gyro.raw_y;
        }
        if (rz != NULL) {
            *rz = gyro.raw_z;
        }

        if (x != NULL) {
            *x = gyro.x;
        }
        if (y != NULL) {
            *y = gyro.y;
        }
        if (z != NULL) {
            *z = gyro.z;
        }

        return true;
    }
    return false;
}

/*
 * Read Accel
 *  rx, ry, rz: raw value.
 *  x, y, z   : comuted value.(UNIT: 1G)
 */
bool TZ01_motion_tracker_accel_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z)
{
    MPU9250_accel_val acc;
    acc.raw_x = 0;
    acc.raw_y = 0;
    acc.raw_z = 0;

    if (MPU9250_drv_read_accel(&acc)) {
        if (rx != NULL) {
            *rx = acc.raw_x;
        }
        if (ry != NULL) {
            *ry = acc.raw_y;
        }
        if (rz != NULL) {
            *rz = acc.raw_z;
        }

        if (x != NULL) {
            *x = acc.x;
        }
        if (y != NULL) {
            *y = acc.y;
        }
        if (z != NULL) {
            *z = acc.z;
        }

        return true;
    }
    return false;
}

/*
 * Read Chip Temperature
 *  rt: Raw value.
 *   t: Computed value(digC)
 */
bool TZ01_motion_tracker_temperature_read(uint16_t *rt, float *t)
{
    float ft;
    MPU9250_temperature_val temp;
    temp.raw = 0;

    if (MPU9250_drv_read_temperature(&temp)) {
        if (rt != NULL) {
            *rt = temp.raw;
        }

        if (t != NULL) {
            ft = (float)(int16_t)temp.raw;
            *t = ((ft - 21) / 333.87) + 21;
        }
        return true;
    }
    return false;
}

/*
 * Read Magnetometer
 *  rx, ry, rz: Raw value.
 *  x, y, z   : Computed value(uT)
 */
bool TZ01_motion_tracker_magnetometer_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z)
{
    MPU9250_magnetometer_val mag;
    mag.raw_x = 0;
    mag.raw_y = 0;
    mag.raw_z = 0;

    if (MPU9250_drv_read_magnetometer(&mag)) {
        if (rx != NULL) {
            *rx = mag.raw_x;
        }
        if (ry != NULL) {
            *ry = mag.raw_y;
        }
        if (rz != NULL) {
            *rz = mag.raw_z;
        }

        if (x != NULL) {
            *x = mag.x;
        }
        if (y != NULL) {
            *y = mag.y;
        }
        if (z != NULL) {
            *z = mag.z;
        }

        return true;
    }
    return false;
}

/*
 * Computed AXIS angle from Accel
 */
bool TZ01_motion_tracker_acc_axis_angle(float *pitch_rad, float *roll_rad)
{
    MPU9250_accel_val acc;
    if (MPU9250_drv_read_accel(&acc)) {
        TZ01_motion_tracker_compute_axis_angle(acc.x, acc.y, acc.z, pitch_rad, roll_rad);
        return true;
    }
    return false;
}

/*
 * Compute axis angle from Accel
 */
void TZ01_motion_tracker_compute_axis_angle(float acc_x, float acc_y, float acc_z, float *pitch_rad, float *roll_rad)
{
    float x_rad, y_rad, z_rad;

    x_rad = atan(acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2)));
    y_rad = atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2)));
    z_rad = atan(sqrt(pow(acc_x, 2) + pow(acc_y, 2)) / acc_z);

    if (z_rad >= 0.0) {
        /* first quadrant & fourth quadrant */
        /* pitch */
        if (pitch_rad != NULL) {
            if (x_rad >= 0.0) {
                *pitch_rad = x_rad;             /* first quadrant */
            } else {
                *pitch_rad = (2 * PI) + x_rad;  /* fourth quadrant */
            }
        }
        /* roll */
        if (roll_rad != NULL) {
            if (y_rad >= 0.0) {
                *roll_rad = y_rad;              /* first quadrant */
            } else {
                *roll_rad = (2 * PI) + y_rad;   /* fourth quadrant */
            }
        }
    } else {
        /* second quadrant & third quadrant */
        /* pitch */
        if (pitch_rad != NULL) {
            *pitch_rad = PI - x_rad;
        }
        /* roll */
        if (roll_rad != NULL) {
            *roll_rad = PI - y_rad;
        }
    }
}
