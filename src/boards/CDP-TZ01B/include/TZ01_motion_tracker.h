/**
 * @file   TZ01_motion_tracker.h
 * @brief  Motion tracker library for Cerevo CDP-TZ01B.
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

#ifndef _TZ01_MOTION_TRACKER_H_
#define _TZ01_MOTION_TRACKER_H_

#include <stdint.h>
#include <stdbool.h>

#define PI (3.1415926535)

/**
 * @brief   Initialize the motion tracker library.
 * @return  Initiaize result.
 * @retval  true    Success.
 * @retval  false   Failed.
 */
bool TZ01_motion_tracker_init(void);

/**
 * @brief Read Gyro value.
 * @param[out]  rx  Raw value of X-axis/Roll.
 * @param[out]  ry  Raw value of Y-axis/Pitch.
 * @param[out]  rz  Raw value of Z-axis/Yaw.
 * @param[out]  x   Computed value of X-axis/Pitch.(UINT is `degree/s')
 * @param[out]  y   Computed value of Y-axis/Roll. (UINT is `degree/s')
 * @param[out]  z   Computed value of Z-axis/Yaw.  (UINT is `degree/s')
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.(Ex. Driver state missmatch.)
 */
bool TZ01_motion_tracker_gyro_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z);

/**
 * @brief Read Accel value.
 * @param[out]  rx  Raw value of X-axis.
 * @param[out]  ry  Raw value of Y-axis.
 * @param[out]  rz  Raw value of Z-axis.
 * @param[out]  x   Computed value of X-axis.(UINT is `G')
 * @param[out]  y   Computed value of Y-axis.(UINT is `G')
 * @param[out]  z   Computed value of Z-axis.(UINT is `G')
 * @return          Read result.
 * @retval          true    Success.
 * @retval          false   Failed.(Ex. Driver state missmatch.)
 */
bool TZ01_motion_tracker_accel_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z);

/**
 * @brief Read chip temperature value.
 * @param[out]  rt  Raw value.
 * @param[out]  t   Computed value.(Unit is `degree Celsius')
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.(Ex. Driver state missmatch.)
 */
bool TZ01_motion_tracker_temperature_read(uint16_t *rt, float *t);

/**
 * @brief Read Magnetometer value.
 * @param[out]  rx  Raw value of X-axis.
 * @param[out]  ry  Raw value of Y-axis.
 * @param[out]  rz  Raw value of Z-axis.
 * @param[out]  x   Computed value of X-axis.(UINT is `uH')
 * @param[out]  y   Computed value of Y-axis.(UINT is `uH')
 * @param[out]  z   Computed value of Z-axis.(UINT is `uH')
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.(Ex. Driver state missmatch.)
 */
bool TZ01_motion_tracker_magnetometer_read(uint16_t *rx, uint16_t *ry, uint16_t *rz, float *x, float *y, float *z);

/**
 * @brief Computed AXIS angle from Accel.
 * @param[out]  pitch_rad (UNIT is `radian')
 * @param[out]  roll_rad  (UNIT is `radian')
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.(Ex. Driver state missmatch.)
 */
bool TZ01_motion_tracker_acc_axis_angle(float *pitch_rad, float *roll_rad);

/**
 * @brief Comute AXIS angle from maesured accel values.
 * @param[in]   acc_x       X-axis accel value(Unit is `G')
 * @param[in]   acc_y       Y-axis accel value(Unit is `G')
 * @param[in]   acc_z       Z-axis accel value(Unit is `G')
 * @param[out]  pitch_rad   Computed pitch (UNIT is `radian')
 * @param[out]  roll_rad    Computed roll  (UNIT is `radian')
 */
void TZ01_motion_tracker_compute_axis_angle(float acc_x, float acc_y, float acc_z, float *pitch_rad, float *roll_rad);

#endif
