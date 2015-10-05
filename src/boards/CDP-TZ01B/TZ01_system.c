/**
 * @file   TZ01_system.c
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

/** Includes **/
#include <stdint.h>
#include <stdbool.h>
/* MCU support. */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "TMR_TZ10xx.h"
/* Board support. */
#include "TZ01_system.h"
#include "utils.h"

#if TZ01_CONSOLE_ENABLE
    #include "TZ01_console.h"
    #define tz01_system_log(x)    TZ01_console_puts(x)
#else
    #define tz01_system_log(x)
#endif

/** Variables **/
static TZ01_SYSTEM_TICK tz01_system_tick;

static uint16_t sw_history;
static uint16_t uv_history;

/* TZ10xx drivers. */
extern TZ10XX_DRIVER_PMU  Driver_PMU;
static const TZ10XX_DRIVER_PMU  *tz10xx_drv_pmu = &Driver_PMU;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
static const TZ10XX_DRIVER_GPIO *tz10xx_drv_gpio = &Driver_GPIO;
#if TZ01_SYSTEM_TICK_TMR_NO == 0
    extern TZ10XX_DRIVER_TMR  Driver_TMR0;
    static const TZ10XX_DRIVER_TMR  *tz10xx_drv_tmr = &Driver_TMR0;
#elif TZ01_SYSTEM_TICK_TMR_NO == 1
    extern TZ10XX_DRIVER_TMR_Driver_TMR1;
    static const TZ10XX_DRIVER_TMR  *tz10xx_drv_tmr = &Driver_TMR1;
#else
    #error Invalid TMR device no.
#endif

/** Power switch control functions. **/
#if TZ01_POW_MGR
static bool tz01_system_pwsw_powon(void)
{
    /* Clear sw history. */
    sw_history = 0xffff;
    uv_history = 0xffff;

    /* `PowerHold' assert. */
    if (tz10xx_drv_gpio->WritePin(TZ01_SYSTEM_PWSW_PORT_HLD, 1) == GPIO_ERROR) {
        return false;
    }

    /* PowerLED(`LED1') assert. */
    if (tz10xx_drv_gpio->WritePin(TZ01_SYSTEM_PWSW_PORT_LED, 1) == GPIO_ERROR) {
        return false;
    }
    return true;
}

static bool tz01_system_pwsw_powoff(void)
{
    tz01_system_log("POWER OFF\r\n");
    /* Clear sw history. */
    sw_history = 0xffff;
    uv_history = 0xffff;

    /* `PowerHold' negate. */
    if (tz10xx_drv_gpio->WritePin(TZ01_SYSTEM_PWSW_PORT_HLD, 0) == GPIO_ERROR) {
        return false;
    }

    /* PowerLED(`LED1') nagete. */
    if (tz10xx_drv_gpio->WritePin(TZ01_SYSTEM_PWSW_PORT_LED, 0) == GPIO_ERROR) {
        return false;
    }
    return true;
}

static int tz01_system_pwsw_check(void)
{
    uint32_t val;

    if (tz10xx_drv_gpio->ReadPin(TZ01_SYSTEM_PWSW_PORT_SW, &val) == GPIO_ERROR) {
        return -1;
    }

    return (val & 0x01);
}

static int tz01_system_pwsw_uvdetect_check(void)
{
    uint32_t val;

    if (tz10xx_drv_gpio->ReadPin(TZ01_SYSTEM_PWSW_PORT_UVD, &val) == GPIO_ERROR) {
        return -1;
    }

    return (val & 0x01);
}
#endif

/** Tick timer manage functions. **/
bool TZ01_system_tick_clear(void)
{
    for (int i = 0; i < _TICK_NO_COUNT; i++) {
        tz01_system_tick.timers[i].is_active = false;
        tz01_system_tick.timers[i].timeout = 0;
    }
    return true;
}

/**
 * Start tick timer
 *
 */
bool TZ01_system_tick_start(TZ01_SYSTEM_TICK_NO tim_no, uint32_t ms_timeout)
{
    uint32_t tick_curr;
    uint32_t prescale, tmr_clock;
    
    prescale = tz10xx_drv_pmu->GetPrescaler(PMU_CD_PPIER0);
    tmr_clock = 48000 / prescale;   //kHz

    if (tim_no >= _TICK_NO_COUNT) {
        return false;
    }

    tick_curr = tz10xx_drv_tmr->GetValue();
    tz01_system_tick.timers[tim_no].timeout = tick_curr - (uint32_t)((ms_timeout * tmr_clock) - 1);
    tz01_system_tick.timers[tim_no].is_active = true;

    return true;
}

bool TZ01_system_tick_stop(TZ01_SYSTEM_TICK_NO tim_no)
{
    if (tim_no >= _TICK_NO_COUNT) {
        return false;
    }

    tz01_system_tick.timers[tim_no].is_active = false;
    tz01_system_tick.timers[tim_no].timeout = 0;

    return true;
}

bool TZ01_system_tick_is_active(TZ01_SYSTEM_TICK_NO tim_no)
{
    if (tim_no >= _TICK_NO_COUNT) {
        return false;
    }

    return tz01_system_tick.timers[tim_no].is_active;
}

bool TZ01_system_tick_check_timeout(TZ01_SYSTEM_TICK_NO tim_no)
{
    uint32_t tick_curr;
    int32_t diff;

    if (tim_no >= _TICK_NO_COUNT) {
        return false;
    }

    if (tz01_system_tick.timers[tim_no].is_active == false) {
        return false;
    }

    tick_curr = tz10xx_drv_tmr->GetValue();
    diff = (int32_t)((uint32_t)tick_curr - (uint32_t)tz01_system_tick.timers[tim_no].timeout);
    if (diff < 0) {
        TZ01_system_tick_stop(tim_no);
        return true;
    }

    return false;
}

/** System **/
bool TZ01_system_init(void)
{
#if TZ01_POW_MGR
    Usleep(TZ01_SYSTEM_PW_HLD_DELAY);
#endif
    /* PMU */
    tz10xx_drv_pmu->Initialize(NULL);
    tz10xx_drv_pmu->SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_PLL);
    tz10xx_drv_pmu->SetPrescaler(PMU_CD_MPIER, 1);
    tz10xx_drv_pmu->SetPrescaler(PMU_CD_PPIER0, 2);
    tz10xx_drv_pmu->SetPrescaler(PMU_CD_PPIER1, 4);
    tz10xx_drv_pmu->SetPrescaler(PMU_CD_PPIER2, 4);
#if TZ01_POW_MGR
    tz10xx_drv_pmu->StandbyInputBuffer(PMU_IO_FUNC_GPIO_1, 0);   /* Power Switch */
    tz10xx_drv_pmu->StandbyInputBuffer(PMU_IO_FUNC_GPIO_4, 0);   /* UVdetect */
#endif
    /* GPIO */
    tz10xx_drv_gpio->Initialize();
    tz10xx_drv_gpio->PowerControl(ARM_POWER_FULL);
    /** DO **/
    tz10xx_drv_gpio->Configure(TZ01_SYSTEM_PWSW_PORT_LED, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);  /* Power LED  */
#if TZ01_POW_MGR
    tz10xx_drv_gpio->Configure(TZ01_SYSTEM_PWSW_PORT_HLD, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);  /* Power Hold */
#endif
    /** DI **/
#if TZ01_POW_MGR
    tz10xx_drv_gpio->Configure(TZ01_SYSTEM_PWSW_PORT_SW,  GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);  /* Power Switch */
    tz10xx_drv_gpio->Configure(TZ01_SYSTEM_PWSW_PORT_UVD, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);  /* UVdetect */
#endif
    /* TMR */
    if (tz10xx_drv_tmr->Initialize(NULL, 0) == TMR_OK) {
        tz10xx_drv_tmr->Configure(32, TMR_COUNT_MODE_FREE_RUN, 1);
    }
    if (tz10xx_drv_tmr->IsRunning() == false) {
        tz10xx_drv_tmr->PowerControl(ARM_POWER_FULL);
        if (tz10xx_drv_tmr->Start(0xfffffffe) == TMR_ERROR) {
            return false;
        }
    }
#if TZ01_POW_MGR
    /* Power On */
    if (tz01_system_pwsw_powon() == false) {
        return false;
    }
#endif
    /* SW check timer enable. */
    TZ01_system_tick_start(SYSTICK_NO_PWSW_CHECK, 0);
    TZ01_system_tick_start(SYSTICK_NO_LED_BLINK, 500);

    return true;
}

TZ01_system_RUNEVT TZ01_system_run(void)
{
    static uint8_t led_v = 0;
#if TZ01_POW_MGR
    static const uint16_t PWSW_MASK = (1 << TZ01_SYSTEM_PW_OFF_CNT) - 1;
    
    if (TZ01_system_tick_check_timeout(SYSTICK_NO_PWSW_CHECK)) {
        TZ01_system_tick_start(SYSTICK_NO_PWSW_CHECK, TZ01_SYSTEM_PWSW_CHK_INTVAL);
        /* Power switch */
        sw_history <<= 1;
        sw_history &= PWSW_MASK;
        sw_history |= tz01_system_pwsw_check();

        if (sw_history == 0x00) {
            /* PowerSwitch Hold. (TZ01_SYSTEM_PWSW_CHK_INTVAL * TZ01_SYSTEM_PW_OFF_CNT)[ms] */
            if (tz01_system_pwsw_powoff()) {
                return RUNEVT_POWOFF;
            }
        }
        /* UVdetect */
        uv_history <<= 1;
        uv_history &= 0x0007;
        uv_history |= tz01_system_pwsw_uvdetect_check();
        if (uv_history == 0x00) {
            if (tz01_system_pwsw_powoff()) {
                return RUNEVT_LO_VOLT;
            }
        }
    }
#endif
    /* Blink hart beat LED. */
#if TZ01_HARTBEAT
    if (TZ01_system_tick_check_timeout(SYSTICK_NO_LED_BLINK)) {
        TZ01_system_tick_start(SYSTICK_NO_LED_BLINK, 500);

        led_v = (led_v == 0) ? 1 : 0;
        tz10xx_drv_gpio->WritePin(TZ01_SYSTEM_PWSW_PORT_LED, led_v);
    }
#endif
    return RUNEVT_NONE;
}
