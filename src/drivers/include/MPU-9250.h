/**
 * @file   MPU-9250.h
 * @brief  9-axis sensor(InvenSense MPU-9250) driver for TZ10xx.
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

#ifndef _MPU_9250_H_
#define _MPU_9250_H_

#include <stdint.h>
#include <stdbool.h>
#include "SPI_TZ10xx.h"

/**
 * @struct  MPU9250_gyro_val
 * @brief   MPU9250 gyro value.
 */
typedef struct {
    float x;        /*!< Computed X-axis value (Unit is digree/s) */
    float y;        /*!< Computed Y-axis value (Unit is digree/s) */
    float z;        /*!< Computed Z-axis value (Unit is digeee/s) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_gyro_val;

/**
 * @struct  MPU9250_accel_val
 * @brief   MPU9250 accel value.
 */
typedef struct {
    float x;        /*!< Computed X-axis value (G) */
    float y;        /*!< Computed Y-axis value (G) */
    float z;        /*!< Computed Z-axis value (G) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_accel_val;

/**
 * @struct  MPU9250_magnetometer_val
 * @brief   AK8963 magnetometer value.
 */
typedef struct {
    float x;        /*!< Computed X-axis value (uH) */
    float y;        /*!< Computed Y-axis value (uH) */
    float z;        /*!< Computed Z-axis value (uH) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_magnetometer_val;

/**
 * @struct  MPU9250_temperature_val
 * @brief   MPU9250 chip temperature value.
 */
typedef struct {
    uint16_t raw;   /*!< Raw chip temperature */
} MPU9250_temperature_val;

/*
 * MPU-9250
 */
/** @name MPU-9250 Registers
    Register address offsets.
*/
/* @{ */
#define MPU9250_REG_SELF_TEST_X_GYRO    (0)     /*!< SELF_TEST_X_GYRO register */
#define MPU9250_REG_SELF_TEST_Y_GYRO    (1)     /*!< SELF_TEST_Y_GYRO register */
#define MPU9250_REG_SELF_TEST_Z_GYRO    (2)     /*!< SELF_TEST_Z_GYRO register */
/* --- */
#define MPU9250_REG_SELF_TEST_X_ACCEL   (13)    /*!< SELF_TEST_X_ACCEL register */
#define MPU9250_REG_SELF_TEST_Y_ACCEL   (14)    /*!< SELF_TEST_Y_ACCEL register */
#define MPU9250_REG_SELF_TEST_Z_ACCEL   (15)    /*!< SELF_TEST_Z_ACCEL register */
/* --- */
#define MPU9250_REG_XG_OFFSET_HL        (19)    /*!< XG_OFFSET_HL register */
#define MPU9250_REG_XG_OFFSET_HH        (20)    /*!< XG_OFFSET_HH register */
#define MPU9250_REG_YG_OFFSET_HL        (21)    /*!< YG_OFFSET_HL register */
#define MPU9250_REG_YG_OFFSET_HH        (22)    /*!< YG_OFFSET_HH register */
#define MPU9250_REG_ZG_OFFSET_HL        (23)    /*!< ZG_OFFSET_HL register */
#define MPU9250_REG_ZG_OFFSET_HH        (23)    /*!< ZG_OFFSET_HH register */
#define MPU9250_REG_SMPLRT_DIV          (25)    /*!< SMPLRT_DIV register */
#define MPU9250_REG_CONFIG              (26)    /*!< CONFIG register */
#define MPU9250_REG_GYRO_CONFIG         (27)    /*!< GYRO_CONFIG register */
#define MPU9250_REG_ACCEL_CONFIG        (28)    /*!< ACCEL_CONFIG register */
#define MPU9250_REG_ACCEL_CONFIG2       (29)    /*!< ACCEL_CONFIG2 regster */
#define MPU9250_REG_LP_ACCEL_ODR        (30)    /*!< LP_ACCEL_ODR register */
#define MPU9250_REG_WOM_THR             (31)    /*!< WOM_THR register */
/* --- */
#define MPU9250_REG_FIFO_EN             (35)    /*!< FIFO_EN */
#define MPU9250_REG_I2C_MST_CTRL        (36)    /*!< I2C_MST_CTRL register */
#define MPU9250_REG_I2C_SLV0_ADDR       (37)    /*!< I2C_SLV0_ADDR register */
#define MPU9250_REG_I2C_SLV0_REG        (38)    /*!< I2C_SLV0_REG register */
#define MPU9250_REG_I2C_SLV0_CTRL       (39)    /*!< I2C_SLV0_CTRL register */
#define MPU9250_REG_I2C_SLV1_ADDR       (40)    /*!< I2C_SLV1_ADDR register */
#define MPU9250_REG_I2C_SLV1_REG        (41)    /*!< I2C_SLV1_REG register */
#define MPU9250_REG_I2C_SLV1_CTRL       (42)    /*!< I2C_SLV1_CTRL register */
#define MPU9250_REG_I2C_SLV2_ADDR       (43)    /*!< I2C_SLV2_ADDR register */
#define MPU9250_REG_I2C_SLV2_REG        (44)    /*!< I2C_SLV2_REG register */
#define MPU9250_REG_I2C_SLV2_CTRL       (45)    /*!< I2C_SLV2_CTRL register */
#define MPU9250_REG_I2C_SLV3_ADDR       (46)    /*!< I2C_SLV3_ADDR register */
#define MPU9250_REG_I2C_SLV3_REG        (47)    /*!< I2C_SLV3_REG register */
#define MPU9250_REG_I2C_SLV3_CTRL       (48)    /*!< I2C_SLV3_CTRL regster */
#define MPU9250_REG_I2C_SLV4_ADDR       (49)    /*!< I2C_SLV4_ADDR register */
#define MPU9250_REG_I2C_SLV4_REG        (50)    /*!< I2C_SLV4_REG register */
#define MPU9250_REG_I2C_SLV4_DO         (51)    /*!< I2C_SLV4_DO regster */
#define MPU9250_REG_I2C_SLV4_CTRL       (52)    /*!< I2C_SLV4_CTRL register */
#define MPU9250_REG_I2C_SLV4_DI         (53)    /*!< I2C_SLV4_DI register */
#define MPU9250_REG_I2C_MST_STATUS      (54)    /*!< I2C_MST_STATUS register */
#define MPU9250_REG_INT_PIN_CFG         (55)    /*!< INT_PIN_CFG register */
#define MPU9250_REG_INT_ENABLE          (56)    /*!< INT_ENABLE register */
/* --- */
#define MPU9250_REG_INT_STATUS          (58)    /*!< INT_STATUS register */
#define MPU9250_REG_ACCEL_XOUT_HL       (59)    /*!< ACCEL_XOUT_HL register */
#define MPU9250_REG_ACCEL_XOUT_HH       (60)    /*!< ACCEL_XOUT_HH register */
#define MPU9250_REG_ACCEL_YOUT_HL       (61)    /*!< ACCEL_YOUT_HL register */
#define MPU9250_REG_ACCEL_YOUT_HH       (62)    /*!< ACCEL_YOUT_HH register */
#define MPU9250_REG_ACCEL_ZOUT_HL       (63)    /*!< ACCEL_ZOUT_HL register */
#define MPU9250_REG_ACCEL_ZOUT_HH       (64)    /*!< ACCEL_ZOUT_HH register */
#define MPU9250_REG_TEMP_HL             (65)    /*!< TEMP_HL register */
#define MPU9250_REG_TEMP_HH             (66)    /*!< TEMP_HH register */
#define MPU9250_REG_GYRO_XOUT_HL        (67)    /*!< GYRO_XOUT_HL register */
#define MPU9250_REG_GYRO_XOUT_HH        (68)    /*!< GYRO_XOUT_HH register */
#define MPU9250_REG_GYRO_YOUT_HL        (69)    /*!< GYRO_YOUT_HL register */
#define MPU9250_REG_GYRO_YOUT_HH        (70)    /*!< GYRO_YOUT_HH register */
#define MPU9250_REG_GYRO_ZOUT_HL        (71)    /*!< GYRO_ZOUT_HL register */
#define MPU9250_REG_GYRO_ZOUT_HH        (72)    /*!< GYRO_ZOUT_HH register */
#define MPU9250_REG_EXT_SENS_DATA_00    (73)    /*!< EXT_SENS_DATA_00 registers (24Byte) */
/* --- */
#define MPU9250_REG_I2C_SLV0_DO         (99)    /*!< I2C_SLV0_DO register */
#define MPU9250_REG_I2C_SLV1_DO         (100)   /*!< I2C_SLV1_DO register */
#define MPU9250_REG_I2C_SLV2_DO         (101)   /*!< I2C_SLV2_DO register */
#define MPU9250_REG_I2C_SLV3_DO         (102)   /*!< I2C_SLV3_DO register */
#define MPU9250_REG_I2C_MST_DELAY_CTRL  (103)   /*!< I2C_MST_DELAY_CTRL register */
#define MPU9250_REG_SIGNAL_PATH_RESET   (104)   /*!< SIGNAL_PATH_RESET register */
#define MPU9250_REG_MOT_DETECT_CTRL     (105)   /*!< MOT_DETECT_CTRL register */
#define MPU9250_REG_USER_CTRL           (106)   /*!< USER_CTRL register */
#define MPU9250_REG_PWR_MGMT_1          (107)   /*!< PWR_MGMT_1 register */
#define MPU9250_REG_PWR_MGMT_2          (108)   /*!< PWR_MGMT_2 register */
/* --- */
#define MPU9250_REG_FIFO_COUNT_HL       (114)   /*!< FIFO_COUNT_HL register */
#define MPU9250_REG_FIFO_COUNT_HH       (115)   /*!< FIFO_COUNT_HH register */
#define MPU9250_REG_FIFO_R_W            (116)   /*!< FIFO_R_W register */
#define MPU9250_REG_WHO_AM_I            (117)   /*!< WHO_AM_I register */
/* --- */
#define MPU9250_REG_XA_OFFSET_HL        (119)   /*!< XA_OFFSET_HL register */
#define MPU9250_REG_XA_OFFSET_HH        (120)   /*!< XA_OFFSET_HH register */
/* --- */
#define MPU9250_REG_YA_OFFSET_HL        (122)   /*!< YA_OFFSET_HL register */
#define MPU9250_REG_YA_OFFSET_HH        (123)   /*!< YA_OFFSET_HH register */
/* --- */
#define MPU9250_REG_ZA_OFFSET_HL        (125)   /*!< ZA_OFFSET_HL register */
#define MPU9250_REG_ZA_OFFSET_HH        (126)   /*!< ZA_OFFSET_HH register */
/* @} */

/**
 * AK8963 (Included MPU-9250 package.)
 */
 /** @name AK8963 I2C Address */
 /* @{ */
#define AK8963_I2C_ADDR                 (0x0c)  /*!< AK8963 I2C address. */
/* @} */

/** @name AK8963 Register Address */
/* @{ */
#define AK8963_REG_WIA                  (0x00)  /*!< WIA register */
#define AK8963_REG_INFO                 (0x01)  /*!< INFO register */
#define AK8963_REG_ST1                  (0x02)  /*!< ST1 register */
#define AK8963_REG_HX_LH                (0x03)  /*!< HX_LH register */
#define AK8963_REG_HX_HH                (0x04)  /*!< HX_HH register */
#define AK8963_REG_HY_LH                (0x05)  /*!< HY_LH register */
#define AK8963_REG_HY_HH                (0x06)  /*!< HY_HH register */
#define AK8963_REG_HZ_LH                (0x07)  /*!< HZ_LH register */
#define AK8963_REG_HZ_HH                (0x08)  /*!< HZ_HH register */
#define AK8963_REG_ST2                  (0x09)  /*!< ST2 register */
#define AK8963_REG_CNTL1                (0x0A)  /*!< CNTL1 register */
#define AK8963_REG_CNTL2                (0x0B)  /*!< CNTL2 register */
#define AK8963_REG_ASTC                 (0x0C)  /*!< ASTC register */
/* --- */
#define AK8963_REG_ASAX                 (0x10)  /*!< ASAX register */
#define AK8963_REG_ASAY                 (0x11)  /*!< ASAY register */
#define AK8963_REG_ASAZ                 (0x12)  /*!< ASAZ register */
/* @} */

/**
 * @enum MPU9250_BIT_ACCEL_FS_SEL
 * MPU9250 Accel full scale select.
 */
typedef enum {
    MPU9250_BIT_ACCEL_FS_SEL_2G = 0x00,     /*!< +-2G */
    MPU9250_BIT_ACCEL_FS_SEL_4G = 0x08,     /*!< +-4G */
    MPU9250_BIT_ACCEL_FS_SEL_8G = 0x10,     /*!< +-8G */
    MPU9250_BIT_ACCEL_FS_SEL_16G = 0x18,    /*!< +-16G */
    MPU9250_BIT_ACCEL_FS_SEL_MASK = 0x18,   /*!< Mask */
}   MPU9250_BIT_ACCEL_FS_SEL;

/**
 * @enum MPU9250_BIT_GYRO_FS_SEL
 * MPU9250 Gyro full scale select.
 */
typedef enum {
    MPU9250_BIT_GYRO_FS_SEL_250DPS = 0x00,  /*!< +250dps */
    MPU9250_BIT_GYRO_FS_SEL_500DPS = 0x08,  /*!< +500dps */
    MPU9250_BIT_GYRO_FS_SEL_1000DPS = 0x10, /*!< +1000dps */
    MPU9250_BIT_GYRO_FS_SEL_2000DPS = 0x18, /*!< +2000dps */
    MPU9250_BIT_GYRO_FS_SEL_MASK = 0x18,    /*!< Mask */
}   MPU9250_BIT_GYRO_FS_SEL;

/**
 * @enum MPU9250_BIT_DLPF_CFG
 * MPU9250 Gyro DLPF config.
 */
typedef enum {
    MPU9250_BIT_DLPF_CFG_250HZ = 0x00,      /*!< Band width 250Hz */
    MPU9250_BIT_DLPF_CFG_184HZ = 0x01,      /*!< Band width 184Hz */
    MPU9250_BIT_DLPF_CFG_92HZ = 0x02,       /*!< Band width 92Hz */
    MPU9250_BIT_DLPF_CFG_41HZ = 0x03,       /*!< Band width 41Hz */
    MPU9250_BIT_DLPF_CFG_20HZ = 0x04,       /*!< Band width 20Hz */
    MPU9250_BIT_DLPF_CFG_10HZ = 0x05,       /*!< Band width 10Hz */
    MPU9250_BIT_DLPF_CFG_5HZ = 0x06,        /*!< Band width 5Hz */
    MPU9250_BIT_DLPF_CFG_3600HZ = 0x07,     /*!< Band width 3600Hz */
    MPU9250_BIT_DLPF_CFG_MASK = 0x07,       /*!< Mask */
}   MPU9250_BIT_DLPF_CFG;

/**
 * @enum MPU9250_BIT_A_DLPFCFG
 * MPU9250 Accel DLPF config.
 */
typedef enum {
    MPU9250_BIT_A_DLPFCFG_460HZ = 0x00,     /*!< Band width 460Hz */
    MPU9250_BIT_A_DLPFCFG_184HZ = 0x01,     /*!< Band width 184Hz */
    MPU9250_BIT_A_DLPFCFG_92HZ = 0x02,      /*!< Band width 92Hz */
    MPU9250_BIT_A_DLPFCFG_41HZ = 0x03,      /*!< Band width 41Hz */
    MPU9250_BIT_A_DLPFCFG_20HZ = 0x04,      /*!< Band width 20Hz */
    MPU9250_BIT_A_DLPFCFG_10HZ = 0x05,      /*!< Band width 10Hz */
    MPU9250_BIT_A_DLPFCFG_5HZ = 0x06,       /*!< Band width 5Hz */
    MPU9250_BIT_A_DLPFCFG_460HZ_2 = 0x07,   /*!< Band width 460Hz */
    MPU9250_BIT_A_DLPFCFG_MASK = 0x07,      /*!< Mask */
}   MPU9250_BIT_A_DLPFCFG;

/**
 * @brief      Initialize MPU9250 driver.
 * @param[in]  spi_drv TZ10xx driver information.
 * @return     Initialize result
 * @retval     true    Success.
 * @retval     false   Failed.
 */
bool MPU9250_drv_init(TZ10XX_DRIVER_SPI *spi_drv);

/**
 * @brief       Start maesure.
 * @param[in]   gyro_fs     Gyro full scale select.
 * @param[in]   accel_fs    Accel full scale select.
 * @param[in]   dlpf_cfg    Gyro DLPF config.
 * @param[in]   a_dlpfcfg   Accel DLPF config.
 * @return      Result
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool MPU9250_drv_start_maesure(MPU9250_BIT_GYRO_FS_SEL gyro_fs, MPU9250_BIT_ACCEL_FS_SEL accel_fs, MPU9250_BIT_DLPF_CFG dlpf_cfg, MPU9250_BIT_A_DLPFCFG a_dlpfcfg);

/**
 * @brief   Stop maesure.
 * @return  Result
 * @retval  true    Success.
 * @retval  false   Failed.
 */
bool MPU9250_drv_stop_maesure(void);

/**
 * @brief       Read maesured gyro value.
 * @param[out]  gyro_val Read values.
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool MPU9250_drv_read_gyro(MPU9250_gyro_val *gyro_val);

/**
 * @brief       Read maesured accel value.
 * @param[out]  accel_val Read values.
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool MPU9250_drv_read_accel(MPU9250_accel_val *accel_val);

/**
 * @brief       Read maesured temperature value.
 * @param[out]  temperature_val Read values.
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool MPU9250_drv_read_temperature(MPU9250_temperature_val *temperature_val);

/**
 * @brief       Read magnetometer value.
 * @param[out]  magnetometer_val Read values.
 * @return      Read result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool MPU9250_drv_read_magnetometer(MPU9250_magnetometer_val *magnetometer_val);

#endif
