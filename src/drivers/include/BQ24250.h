/**
 * @file   BQ24250.h
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

#ifndef _BQ24250_H_
#define _BQ24250_H_

#include <stdint.h>
#include <stdbool.h>
#include "Driver_I2C.h"

#define BQ24250_I2C_ID  (0x6a)

/** @name BQ24250 register addresses */
/* @{ */
#define BQ24250_REG_01  (0x00)  /*!< Register 1 */
#define BQ24250_REG_02  (0x01)  /*!< Register 2 */
#define BQ24250_REG_03  (0x02)  /*!< Register 3 */
#define BQ24250_REG_04  (0x03)  /*!< Register 4 */
#define BQ24250_REG_05  (0x04)  /*!< Register 5 */
#define BQ24250_REG_06  (0x05)  /*!< Register 6 */
#define BQ24250_REG_07  (0x06)  /*!< Register 7 */
/* @} */

/**
 * @name BQ24250 register initial value.
 * @note Do not easily change the initial values.
 */
/* @{ */
#define BQ24250_DEF_01  (0x00)  /*!< Register 1 initial value */
#define BQ24250_DEF_02  (0x0c)  /*!< Register 2 initial value */
#define BQ24250_DEF_03  (0x8f)  /*!< Register 3 initial value */
#define BQ24250_DEF_04  (0xf8)  /*!< Register 4 initial value */
#define BQ24250_DEF_05  (0x02)  /*!< Register 5 initial value */
#define BQ24250_DEF_06  (0xa0)  /*!< Register 6 initial value */
#define BQ24250_DEF_07  (0xe0)  /*!< Register 7 initial value */
/* @} */


bool BQ24250_drv_reg01_set(uint8_t val);
uint8_t BQ24250_drv_reg01_get(void);
bool BQ24250_drv_reg02_set(uint8_t val);
uint8_t BQ24250_drv_reg02_get(void);
bool BQ24250_drv_reg03_set(uint8_t val);
uint8_t BQ24250_drv_reg03_get(void);
bool BQ24250_drv_reg04_set(uint8_t val);
uint8_t BQ24250_drv_reg04_get(void);
bool BQ24250_drv_reg05_set(uint8_t val);
uint8_t BQ24250_drv_reg05_get(void);
bool BQ24250_drv_reg06_set(uint8_t val);
uint8_t BQ24250_drv_reg06_get(void);
bool BQ24250_drv_reg07_set(uint8_t val);
uint8_t BQ24250_drv_reg07_get(void);

/**
 * @brief Initialize BQ24250
 * @param[in]   i2c_drv     TZ10xx I2C driver information.
 * @param[in]   ts_enable   Thermister enable. ture: enable, false: disable
 * @return      Initialize result
 * @retval      true    Success
 * @retval      false   Failed
 */
bool BQ24250_drv_init(ARM_DRIVER_I2C *i2c_drv, bool ts_enable);

#endif
