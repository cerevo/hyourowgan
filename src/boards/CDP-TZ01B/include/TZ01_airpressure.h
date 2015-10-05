/**
 * @file    TZ01_airpressure.h
 * @brief   Airpressure sensor library for Cerevo CDP-TZ01B.
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

#ifndef _TZ01_AIRPRESSURE_H_
#define _TZ01_AIRPRESSURE_H_

/**
 * @brief   Initialize the airpressure sensor library.
 * @return  Initiaize result.
 * @retval  true     Success.
 * @retval  false    Failed.
 * @note
 */
bool TZ01_airpressure_init(void);

/**
 * @brief   Read temperature value.
 * @return  The read temperature value.(Uinit is `digree Celsius')
 */
float TZ01_airpressure_temp_read(void);

/**
 * @brief   Read airpressure value.
 * @return  The read airpressur value.(UNIT is `Pa')
 */
float TZ01_airpressure_press_read(void);

#endif
