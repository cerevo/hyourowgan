/**
 * @file   tz01_system_conf.h
 * @brief  Cerevo CDP-TZ01B system configuration.
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

#ifndef _TZ01_SYSTEM_CONF_H_
#define _TZ01_SYSTEM_CONF_H_

/** @name TICK TIMER */
/* @{ */
#define TZ01_SYSTEM_TICK_TMR_NO      (0) /*!< TMR device no. */
/* @} */

/** @name TICK TIMER NUMBERS */
/* @{ */
/** @attention SYSTICK_NO_* and _TICK_NO_COUNT, do not edit. */
/** @enum TZ01_SYSTEM_TICK_TMR_NO */
typedef enum {
    /* SYSTEM DEFINITION TIMERS */
    SYSTICK_NO_LED_BLINK,       /*!< SYSTEM: Hartbeat LED interval. @note DO NOT EDIT. */
    SYSTICK_NO_PWSW_CHECK,      /*!< SYSTEM: Check PowerSW interval. @note DO NOT EDIT. */
    /* USER DEFINITION TIMERS */
    USRTICK_NO_GPIO_INTERVAL,   /*!< USER: Check GPIO interval. */
    USRTICK_NO_BLE_MAIN,        /*!< USER: BLE event interval. */
    /* TERMINATER */
    _TICK_NO_COUNT              /*!< TERMINATER: Tick timer count. @note DO NOT EDIT. */
} TZ01_SYSTEM_TICK_NO;
/* @} */

/** @name POWER MANAGE */
/* @{ */
#define TZ01_POW_MGR  1                     /*!< Enable Power management(Power Sw/Power Hold/Lo voltage detect) */
#define TZ01_HARTBEAT 1                     /*!< Enable Hartbeat */
#define TZ01_SYSTEM_PWSW_PORT_LED    (10)   /*!< OUT: Power LED */
#if TZ01_POW_MGR
#define TZ01_SYSTEM_PWSW_PORT_HLD    (3)    /*!< OUT: Power Hold   */
#define TZ01_SYSTEM_PWSW_PORT_SW     (1)    /*!< IN:  Power Switch */
#define TZ01_SYSTEM_PWSW_PORT_UVD    (4)    /*!< IN:  UVdetect */

#define TZ01_SYSTEM_PW_HLD_DELAY    (0)   /*!< Power Hold delay(us) */
#define TZ01_SYSTEM_PWSW_CHK_INTVAL (400)       /*!< PowerSwitch: Check interval (ms) */
#define TZ01_SYSTEM_PW_OFF_CNT      (5)         /*!< PowerSwitch: Power off count (1 - 16) */
#endif
/* @} */

/** @name CONSOLE */
/* @{ */
#define TZ01_CONSOLE_ENABLE  (1)    /*!< Console enable 0:diasble 1:enable */
#if TZ01_CONSOLE_ENABLE
    #define TZ01_CONSOLE_UART_CH (1)        /*!< UART channel */
    #define TZ01_CONSOLE_BAUD    (9600)     /*!< UART Boudrate */
#endif
/* @} */

#endif //_TZ01_SYSTEM_CONF_H_
