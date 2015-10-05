/**
 * @file   TZ01_system.h
 * @brief  System library for Cerevo CDP-TZ01B.
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

#ifndef _TZ01_SYSTEM_H_
#define _TZ01_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "tz01_system_conf.h"

/**
 * @enum  TZ01_system_RUN_EVT
 * @brief The type of event that occurrd in TZ01_system_run().
 */
typedef enum {
    RUNEVT_NONE,        /*!< None */
    RUNEVT_LO_VOLT,     /*!< Lo voltage detected. */
    RUNEVT_POWOFF,      /*!< Power off operation detected. */
}   TZ01_system_RUNEVT;

/**
 * @brief Initialize system library.
 *
 * @return Initialize result.
 * @retval true : Success.
 * @retval false: Failed.
 */
bool TZ01_system_init(void);

/**
 * @brief Running system library.
 *
 * @return Running result.
 * @retval Event that occurrd.
 */
TZ01_system_RUNEVT TZ01_system_run(void);




/**
 * @struct TZ01_SYSTEM_TICK_INFO
 * @brief Tick timer information.
 */
typedef struct {
    bool        is_active;  /*!< Active flag (false: Deactive, true: Active) */
    uint32_t    timeout;    /*!< Timeout value. */
} TZ01_SYSTEM_TICK_INFO;

/**
 * @struct TZ01_SYSTEM_TICK
 * @brief Tick timer
 */
typedef struct {
    TZ01_SYSTEM_TICK_INFO  timers[_TICK_NO_COUNT];   /*!< Tick timer list */
} TZ01_SYSTEM_TICK;

/**
 * @brief Clear tick timer
 *
 * @return Clear result
 * @retval true : Success.
 * @retval false: Failed.
 */
bool TZ01_system_tick_clear(void);

/**
 * @brief Start tick timer
 *
 * @param[in] tim_no     Tick timer number.
 * @param[in] ms_timeout Timeout(UNIT: ms)
 * @return Result
 * @retval true : Success.
 * @retval false: Failed.
 */
bool TZ01_system_tick_start(TZ01_SYSTEM_TICK_NO tim_no, uint32_t ms_timeout);

/**
 * @brief Stop tick timer
 *
 * @param[in] tim_no Tick timer number.
 * @return Result
 * @retval true : Success.
 * @retval false: Failed.
 */
bool TZ01_system_tick_stop(TZ01_SYSTEM_TICK_NO tim_no);

/**
 * @brief Get tick timer activity.
 *
 * @param[in] tim_no Tick timer number.
 * @return Tick timer activity.
 * @retval true : Active
 * @retval false: Deactive
 */
bool TZ01_system_tick_is_active(TZ01_SYSTEM_TICK_NO tim_no);

/**
 * @brief Check tick timer timeout.
 *
 * @param tim_no Tick timer number.
 * @return Timeout
 * @retval true : Timeout.
 * @retval false: Not timeout.
 */
bool TZ01_system_tick_check_timeout(TZ01_SYSTEM_TICK_NO tim_no);


#endif /* _TZ01_SYSTEM_H_ */
