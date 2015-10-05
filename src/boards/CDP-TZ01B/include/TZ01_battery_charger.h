/**
 * @file    TZ01_battery_charger.h
 * @brief   LiPo battery charger library for Cerevo CDP-TZ01B
 * @author  Cerevo Inc.
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

#ifndef _TZ01_BATTERY_CHARGER_H_
#define _TZ01_BATTERY_CHARGER_H_

/**
 * @brief       Battery charger library initialize
 * @param[in]   thermistor_enable
 *              false   Thermistor disable.
 *              true    Thermistor enable.
 * @return      Initiaize result.
 * @retval      true    Success.
 * @retval      false   Failed.
*/
bool TZ01_battery_charger_init(bool thermistor_enable);

/**
 * @brief       Set default configurations
 * @return      Setting result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool TZ01_battery_charger_set_configs(void);

/**
 * @brief       Get configurations.
 * @return      Ponter to register buffer
 * @retval      [0] Reg1 value.
 * @retval      [1] Reg2 value.
 * @retval      [2] Reg3 value.
 * @retval      [3] Reg4 value.
 * @retval      [4] Reg5 value.
 * @retval      [5] Reg6 value.
 * @retval      [6] Reg7 value.
 */
uint8_t* TZ01_battery_charger_get_configs(void);

#endif
