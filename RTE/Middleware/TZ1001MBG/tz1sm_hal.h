/**
 * @file tz1sm_hal.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0 Smart
 * @version V1.0.0
 * @date $LastChangedDate$
 * @note
 */

/*
 * COPYRIGHT (C) 2014
 * TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS COMPANY
 * ALL RIGHTS RESERVED.
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS
 * IS". TOSHIBA CORPORATION MAKES NO OTHER WARRANTY OF ANY KIND,
 * WHETHER EXPRESS, IMPLIED OR, STATUTORY AND DISCLAIMS ANY AND ALL
 * IMPLIED WARRANTIES OF MERCHANTABILITY, SATISFACTORY QUALITY, NON
 * INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA
 * CORPORATION RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE
 * SOURCE CODE IN LATER REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR
 * CHANGES IN THE DOCUMENTATION OR THE PRODUCTS OR TECHNOLOGIES
 * DESCRIBED THEREIN AT ANY TIME.
 * 
 * TOSHIBA CORPORATION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE
 * SOURCE CODE OR ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO,
 * LOST REVENUES, DATA OR PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL
 * OR CONSEQUENTIAL NATURE, PUNITIVE DAMAGES, LOSS OF PROPERTY OR LOSS
 * OF PROFITS ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT, OR
 * BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR PROBABILITY
 * OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 */


#ifndef _TZ1SM_HAL_H_
#define _TZ1SM_HAL_H_

#include "tz1sm_config.h"

/*
 * The code of a timer and exclusion control is a very simple
 * sample. For this reason, in case this sample code is applied, it
 * recommends changing into better mounting suitable for the
 * environment, or using suitable RTOS.
 */

/*
 * Toolchain
 *
 */
#if   defined(__CC_ARM)
#define TWIC_TOOLCHAIN_ARMCC
#undef  TWIC_TOOLCHAIN_EWARM
#undef  TWIC_TOOLCHAIN_GCC
#elif defined(__ICCARM__)
#undef  TWIC_TOOLCHAIN_ARMCC
#define TWIC_TOOLCHAIN_EWARM
#undef  TWIC_TOOLCHAIN_GCC
#elif defined(__GNUC__)
#undef  TWIC_TOOLCHAIN_ARMCC
#undef  TWIC_TOOLCHAIN_EWARM
#define TWIC_TOOLCHAIN_GCC
#endif

#if defined(TWIC_TOOLCHAIN_ARMCC) || defined(TWIC_TOOLCHAIN_EWARM)

#define TWIC_TYPES__TYPES
#define TWIC_STDIO__STDIO
#define TWIC_POSIX__POSIX
#define TWIC_PMU__CMSIS_PMU
#define TWIC_PMU__CMSIS_RTC
#define TWIC_PMU__CMSIS_MISC
#define TWIC_PMU__CMSIS_MISC
#define TWIC_GPIO__CMSIS_GPIO
#define TWIC_UART__CMSIS_UART
#define TWIC_EVENT_CALLBACK
#define TWIC_LECE_ISR_LOWPOWER_WAKEUP
#define TWIC_MCU_PERIPHERAL_POWER_PROFILE
#define TZ1EM_POWER_PROFILE
#undef  TWIC_LECE_ISR_LOWPOWER_STATUS_EVENT
#undef  TWIC_RTOS_TOOL_X1

#elif defined(TWIC_TOOLCHAIN_GCC)

#define TWIC_TYPES__TYPES
#undef  TWIC_STDIO__STDIO
#define TWIC_POSIX__POSIX
#define TWIC_PMU__CMSIS_PMU
#define TWIC_PMU__CMSIS_MISC
#define TWIC_GPIO__CMSIS_GPIO
#define TWIC_UART__CMSIS_UART
#define TWIC_EVENT_CALLBACK
#define TWIC_LECE_ISR_LOWPOWER_WAKEUP
#define TWIC_MCU_PERIPHERAL_POWER_PROFILE
#define TZ1EM_POWER_PROFILE
#undef  TWIC_LECE_ISR_LOWPOWER_STATUS_EVENT
#undef  TWIC_RTOS_TOOL_X1

#else /* something else */

#undef  TWIC_TYPES__TYPES
#undef  TWIC_STDIO__STDIO
#define TWIC_POSIX__POSIX
#undef  TWIC_GPIO__CMSIS_GPIO
#undef  TWIC_UART__CMSIS_UART
#undef  TWIC_PMU__CMSIS_PMU
#undef  TWIC_PMU__CMSIS_RTC
#undef  TWIC_PMU__CMSIS_MISC
#define TWIC_EVENT_CALLBACK
#define TWIC_LECE_ISR_LOWPOWER_WAKEUP
#define TWIC_MCU_PERIPHERAL_POWER_PROFILE
#define TZ1EM_POWER_PROFILE
#undef  TWIC_LECE_ISR_LOWPOWER_STATUS_EVENT
#undef  TWIC_RTOS_TOOL_X1

#endif

/*
 * DATA TYPES
 *
 */

#if defined(TWIC_TYPES__TYPES)

#include <stdint.h>
#include <stdbool.h>
#define TWIC_BITS_PER_LONG (32)

#else

#include <stdbool.h>
#include <sys/types.h>
#include <stdint.h>
#define TWIC_BITS_PER_LONG (32)

#endif /* TWIC_TYPES__TYPES */


/*
 * POSIX
 *
 */

#if defined(TWIC_POSIX__POSIX)

#include <string.h>

#else

void *memset(void *s, int c, unsigned long length);
void *memcpy(void * __restrict__ dest,
             const void * __restrict__ src, unsigned long n);

#endif /* TWIC_POSIX__POSIX */


/*
 * STDIO
 *
 */

#if defined(TWIC_STDIO__STDIO) && defined(TWIC_DEBUG_LOG_TRACE)

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#endif /* TWIC_STDIO__STDIO && TWIC_DEBUG_LOG_TRACE */


/*
 * DEBUG
 *
 */

void tz1smHalDebugUartInit(void);
void tz1smHalDebugUartUnInit(void);
  
#if defined(TWIC_STDIO__STDIO) && defined(TWIC_DEBUG_LOG_TRACE)

#define _twicLog(fmt, ...)  printf("%s: "fmt"%s", __func__, __VA_ARGS__)
#define _twicTrace()        printf("%s:%d \r\n", __func__, __LINE__)
#define _twicPrintf(...)    printf(__VA_ARGS__)

#else

#define _twicLog(fmt, ...) /* VOID */
#define _twicTrace()       /* VOID */
#define _twicPrintf(...)   /* VOID */

#endif /* TWIC_STDIO__STDIO && TWIC_DEBUG_LOG_TRACE */

#if defined(TWIC_DEBUG_LOG_TRACE)

#define twicLog(...)    _twicLog(__VA_ARGS__, "")
#define twicTrace()     _twicTrace()
#define twicPrintf(...) _twicPrintf(__VA_ARGS__)

#else

#define twicLog(...)    /* VOID */
#define twicTrace()     /* VOID */
#define twicPrintf(...) /* VOID */

#endif /* TWIC_DEBUG_LOG_TRACE */


/*
 * RTOS
 *
 */

#if defined(TWIC_RTOS__CMSIS_RTOS)
#include "tz1sm_hal_cmsisrtos.h"
#elif defined(TWIC_RTOS__FREERTOS)
#include "tz1sm_hal_freertos.h"
#elif defined(TWIC_RTOS__NONOS) || defined(TWIC_RTOS__FREERTOS_TICKLESS)
#include "tz1sm_hal_nonos.h"
#else
#error Please define runtime environment.
#endif /* TWIC_RTOS__CMSIS_RTOS */


/*
 * GPIO
 *
 */

#if defined(TWIC_GPIO__CMSIS_GPIO)
#include "GPIO_TZ10xx.h"
#endif /* TWIC_GPIO__CMSIS_GPIO */


/*
 * PMU
 *
 */
#if defined(TWIC_PMU__CMSIS_PMU)
#include "PMU_TZ10xx.h"
#endif /* TWIC_PMU__CMSIS_PMU */

#define TZ1SM_HAL_HI_SLEEP0_TO_ACTIVE_US (32)
#define TZ1SM_HAL_HI_SLEEP1_TO_ACTIVE_US (31)
#define TZ1SM_HAL_HI_SLEEP2_TO_ACTIVE_US (31)
#define TZ1SM_HAL_HI_WAIT_TO_ACTIVE_US (1708)
#define TZ1SM_HAL_HI_WAIT_RETENTION_TO_ACTIVE_US (1714)
#define TZ1SM_HAL_HI_RETENTION_TO_ACTIVE_US (1736)
#define TZ1SM_HAL_HI_RTC_TO_ACTIVE_US (60000)
#define TZ1SM_HAL_HI_STOP_TO_ACTIVE_US (60000)

#define TZ1SM_HAL_UM_SLEEP0_TO_ACTIVE_US (37)
#define TZ1SM_HAL_UM_SLEEP1_TO_ACTIVE_US (35)
#define TZ1SM_HAL_UM_SLEEP2_TO_ACTIVE_US (35)
#define TZ1SM_HAL_UM_WAIT_TO_ACTIVE_US (1694)
#define TZ1SM_HAL_UM_WAIT_RETENTION_TO_ACTIVE_US (1700)
#define TZ1SM_HAL_UM_RETENTION_TO_ACTIVE_US (1722)
#define TZ1SM_HAL_UM_RTC_TO_ACTIVE_US (60000)
#define TZ1SM_HAL_UM_STOP_TO_ACTIVE_US (60000)

#define TZ1SM_HAL_LM_SLEEP0_TO_ACTIVE_US (67)
#define TZ1SM_HAL_LM_SLEEP1_TO_ACTIVE_US (64)
#define TZ1SM_HAL_LM_SLEEP2_TO_ACTIVE_US (66)
#define TZ1SM_HAL_LM_WAIT_TO_ACTIVE_US (1392)
#define TZ1SM_HAL_LM_WAIT_RETENTION_TO_ACTIVE_US (1400)
#define TZ1SM_HAL_LM_RETENTION_TO_ACTIVE_US (1422)
#define TZ1SM_HAL_LM_RTC_TO_ACTIVE_US (60000)
#define TZ1SM_HAL_LM_STOP_TO_ACTIVE_US (60000)

#define TZ1SM_HAL_LO_SLEEP0_TO_ACTIVE_US (148)
#define TZ1SM_HAL_LO_SLEEP1_TO_ACTIVE_US (144)
#define TZ1SM_HAL_LO_SLEEP2_TO_ACTIVE_US (150)
#define TZ1SM_HAL_LO_WAIT_TO_ACTIVE_US (202)
#define TZ1SM_HAL_LO_WAIT_RETENTION_TO_ACTIVE_US (206)
#define TZ1SM_HAL_LO_RETENTION_TO_ACTIVE_US (229)
#define TZ1SM_HAL_LO_RTC_TO_ACTIVE_US (60000)
#define TZ1SM_HAL_LO_STOP_TO_ACTIVE_US (60000)

#define TZ1SM_HAL_VF_LAG_HI_TO_UM_US (750)
#define TZ1SM_HAL_VF_LAG_HI_TO_LM_US (440)
#define TZ1SM_HAL_VF_LAG_HI_TO_LO_US (320)

#define TZ1SM_HAL_VF_LAG_UM_TO_HI_US (750)
#define TZ1SM_HAL_VF_LAG_UM_TO_LM_US (440)
#define TZ1SM_HAL_VF_LAG_UM_TO_LO_US (320)

#define TZ1SM_HAL_VF_LAG_LM_TO_HI_US (710)
#define TZ1SM_HAL_VF_LAG_LM_TO_UM_US (710)
#define TZ1SM_HAL_VF_LAG_LM_TO_LO_US (280)

#define TZ1SM_HAL_VF_LAG_LO_TO_HI_US (1820)
#define TZ1SM_HAL_VF_LAG_LO_TO_UM_US (1820)
#define TZ1SM_HAL_VF_LAG_LO_TO_LM_US (1470)

typedef enum tz1smHalVf {
  TZ1SM_HAL_VF_48M12,
  TZ1SM_HAL_VF_36M11,
  TZ1SM_HAL_VF_12M10,
  TZ1SM_HAL_VF_04M09,
  TZ1SM_HAL_VF_NONE
} tz1smHalVf_t;

typedef enum tz1smHalOm { /* Operation Mode */
  TZ1SM_HAL_OM_ACTIVE,
  TZ1SM_HAL_OM_SLEEP0,
  TZ1SM_HAL_OM_SLEEP1,
  TZ1SM_HAL_OM_SLEEP2,
  TZ1SM_HAL_OM_WAIT,
  TZ1SM_HAL_OM_WAIT_RETENTION,
  TZ1SM_HAL_OM_RETENTION,
  TZ1SM_HAL_OM_RTC,
  TZ1SM_HAL_OM_STOP,
  TZ1SM_HAL_OM_NONE = 0xff
} tz1smHalOm_t;

#define TZ1SM_HAL_PCD_NONE        (0)
#define TZ1SM_HAL_PCD_SRAM1       (1u <<  0)
#define TZ1SM_HAL_PCD_FLASH       (1u <<  1)
#define TZ1SM_HAL_PCD_UART0       (1u <<  2)
#define TZ1SM_HAL_PCD_UART1       (1u <<  3)
#define TZ1SM_HAL_PCD_UART2       (1u <<  4)
#define TZ1SM_HAL_PCD_PP1         (1u <<  5)
#define TZ1SM_HAL_PCD_ADC12       (1u <<  6)
#define TZ1SM_HAL_PCD_ADC24       (1u <<  7)
#define TZ1SM_HAL_PCD_DMAC        (1u <<  8)
#define TZ1SM_HAL_PCD_USB         (1u <<  9)
#define TZ1SM_HAL_PCD_ENC         (1u << 10)
#define TZ1SM_HAL_PCD_SPI0        (1u << 11)
#define TZ1SM_HAL_PCD_SPI1        (1u << 12)
#define TZ1SM_HAL_PCD_SPI2        (1u << 13)
#define TZ1SM_HAL_PCD_SPI3        (1u << 14)
#define TZ1SM_HAL_PCD_I2C0        (1u << 15)
#define TZ1SM_HAL_PCD_I2C1        (1u << 16)
#define TZ1SM_HAL_PCD_I2C2        (1u << 17)
#define TZ1SM_HAL_PCD_WDT         (1u << 18)

typedef unsigned int tz1smHalPcd_t;

/* Callback on wakeup */
typedef void(*tz1smHalWakeupCb_t)(tz1smHalPcd_t, tz1smHalOm_t, tz1smHalVf_t, uint32_t);

#if defined(TWIC_PMU__CMSIS_PMU)
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

/* Connect to BLE GPIO2. (IN)
 * Low:  Active mode,
 * High: Low Power mode */
#define TZ1SM_HAL_GPIO_BLE_STATUS (PMU_IO_FUNC_GPIO_29)

/* Connect to BLE GPIO1. (IN)
 * Low:  Host Wake up,
 * High: Permit to Low Power mode(MCU) */
#define TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP (PMU_IO_FUNC_GPIO_30)
/* Check for TZ1SM_HAL_WAKE_FACTOR_GPIO_30 */

typedef enum tz1smHalWe { /* weit event */
  TZ1SM_HAL_ACT_EVENT_DISABLE = PMU_WAKEUP_EVENT_DISABLE,
  TZ1SM_HAL_ACT_EVENT_EDGE_POS = PMU_WAKEUP_EVENT_EDGE_POS,
  TZ1SM_HAL_ACT_EVENT_EDGE_NEG = PMU_WAKEUP_EVENT_EDGE_NEG,
  TZ1SM_HAL_ACT_EVENT_EDGE_BOTH = PMU_WAKEUP_EVENT_EDGE_BOTH,
} tz1smHalWe_t;

#define TZ1SM_HAL_ACT_FACTOR_GPIO_0        (PMU_WAKEUP_FACTOR_GPIO_0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_1        (PMU_WAKEUP_FACTOR_GPIO_1)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_2        (PMU_WAKEUP_FACTOR_GPIO_2)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_3        (PMU_WAKEUP_FACTOR_GPIO_3)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_4        (PMU_WAKEUP_FACTOR_GPIO_4)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_5        (PMU_WAKEUP_FACTOR_GPIO_5)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_6        (PMU_WAKEUP_FACTOR_GPIO_6)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_7        (PMU_WAKEUP_FACTOR_GPIO_7)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_24       (PMU_WAKEUP_FACTOR_GPIO_24)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_25       (PMU_WAKEUP_FACTOR_GPIO_25)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_26       (PMU_WAKEUP_FACTOR_GPIO_26)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_27       (PMU_WAKEUP_FACTOR_GPIO_27)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_30       (PMU_WAKEUP_FACTOR_GPIO_30)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_BROWNOUT (PMU_WAKEUP_FACTOR_BROWNOUT)
#define TZ1SM_HAL_ACT_FACTOR_RTC           (PMU_WAKEUP_FACTOR_RTC)
#define TZ1SM_HAL_ACT_FACTOR_NONE          (0)

/*
 * @brief
 * Start BLE PMU resource
 */
tz1smHalStatus_t tz1smHalLePmuStart(void);

/*
 * @brief
 * Stop BLE PMU resource
 */
tz1smHalStatus_t tz1smHalLePmuStop(const bool stop_osc32K);

/*
 * @brief
 * Get 32kHz Low Power Clock Status.
 */
tz1smHalStatus_t tz1smHalLePmuClockSourceState32K(void);

uint8_t tz1smHalPmuButtonInit(void);
uint8_t tz1smHalPmuButtonFinalize(void);
uint8_t tz1smHalPmuLedInit(void);
uint8_t tz1smHalPmuLedFinalize(void);
tz1smHalStatus_t tz1smHalInitializePcdSystem(void);
tz1smHalStatus_t tz1smStartClock(const tz1smHalPcd_t pcd);
tz1smHalStatus_t tz1smStopClock(const tz1smHalPcd_t pcd);
tz1smHalStatus_t tz1smHalSetVf(const tz1smHalVf_t vf);
tz1smHalVf_t tz1smHalGetCurrentVf(void);
tz1smHalStatus_t tz1smHalPmuLowPowerPcd(
  const tz1smHalPcd_t, uint32_t * const, tz1smHalOm_t, tz1smHalVf_t,
  tz1smHalWakeupCb_t);
tz1smHalStatus_t tz1smHalPmuHighPowerPcd(
  const tz1smHalPcd_t, uint32_t * const, tz1smHalOm_t, tz1smHalVf_t,
  tz1smHalWakeupCb_t);
tz1smHalStatus_t tz1smHalConfigureWakeup(
  const uint32_t factor, const tz1smHalWe_t event);
tz1smHalStatus_t tz1smHalEnableWakeup(
  const uint32_t factor, const bool enable);
tz1smHalStatus_t tz1smHalOperationMode(tz1smHalOm_t mode);

#else

typedef enum tz1smHalWe { /* weit event */
  TZ1SM_HAL_ACT_EVENT_DISABLE,
  TZ1SM_HAL_ACT_EVENT_EDGE_POS,
  TZ1SM_HAL_ACT_EVENT_EDGE_NEG,
  TZ1SM_HAL_ACT_EVENT_EDGE_BOTH,
} tz1smHalWe_t;

#define TZ1SM_HAL_ACT_FACTOR_GPIO_0        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_1        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_2        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_3        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_4        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_5        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_6        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_7        (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_24       (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_25       (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_26       (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_27       (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_30       (0)
#define TZ1SM_HAL_ACT_FACTOR_GPIO_BROWNOUT (0)
#define TZ1SM_HAL_ACT_FACTOR_RTC           (0)
#define TZ1SM_HAL_ACT_FACTOR_NONE          (0)

tz1smHalStatus_t tz1smHalLePmuStart(void);
tz1smHalStatus_t tz1smHalLePmuStop(const bool stop_osc32K);
tz1smHalStatus_t tz1smHalLePmuClockSourceState32K(void);
tz1smHalStatus_t tz1smHalInitializePcdSystem(void);
tz1smHalStatus_t tz1smStartClock(const tz1smHalPcd_t pcd);
tz1smHalStatus_t tz1smStopClock(const tz1smHalPcd_t pcd);
tz1smHalStatus_t tz1smHalSetVf(const tz1smHalVf_t vf);
tz1smHalVf_t tz1smHalGetCurrentVf(void);
tz1smHalStatus_t tz1smHalPmuLowPowerPcd(
  const tz1smHalPcd_t, uint32_t * const, tz1smHalOm_t, tz1smHalVf_t,
  tz1smHalWakeupCb_t);
tz1smHalStatus_t tz1smHalPmuHighPowerPcd(
  const tz1smHalPcd_t, uint32_t * const, tz1smHalOm_t, tz1smHalVf_t,
  tz1smHalWakeupCb_t);
tz1smHalStatus_t tz1smHalConfigureWakeup(
  const uint32_t factor, const tz1smHalWe_t event);
tz1smHalStatus_t tz1smHalEnableWakeup(
  const uint32_t factor, const bool enable);
tz1smHalStatus_t tz1smHalOperationMode(tz1smHalOm_t mode);

#warning V0.0.1 is no supporting the other PMU Drivers other than CMSIS_PMU.
#endif

/*
 * EVENT CALLBACK
 *
 */

#if defined(TWIC_EVENT_CALLBACK)
typedef struct {
  void (*intr_ble_uart_rx)(void);
  void (*intr_ble_host_wakeup)(void);
  void (*intr_ble_status_changed)(void);  
} tz1smHalCb_t;

/*
 * @brief
 * regiser event call back functions.
 */
void tz1smHalRegisterCb(const tz1smHalCb_t * const cb);

#else

#warning V0.0.1 is no supporting event call back

#endif /* TWIC_EVENT_CALLBACK */

/*
 * GPIO
 *
 */

#if defined(TWIC_GPIO__CMSIS_GPIO)

/* In performing hardware control, it surely uses CMSIS GPIO API in
 * the following functions. Coding of the following functions is
 * carried out using CMSIS API. */

/* Connect to BLE Reset Pin. (OUT)
 * Low:  Reset,
 * High: No Reset */
#define TZ1SM_HAL_GPIO_BLE_RESETX 28

/* Connect to BLE GPIO0. (OUT)
 * Low:  Permit to Low Power mode,
 * High: Active mode */
#define TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP 31

/*
 * @brief
 * Initialize GPIO 
 */
void tz1smHalGpioInit(void);

/*
 * @brief
 * Finalize GPIO 
 */
void tz1smHalGpioUnInit(void);

/*
 * @brief
 * Read GPIO Value
 * @param[in]   pnr GPIO Pin No.
 * @param[out]  status read status(true:high, false:low)
 * @return      execution status (TZ1SM_HAL_STATUS_OK:ok,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 */

tz1smHalStatus_t tz1smHalGpioRead(const uint32_t pnr, bool * const status);

/*
 * @brief
 * Write GPIO Value
 * @param[in]   pnr GPIO Pin No.
 * @param[in]  on_off write value(true:high, false:low)
 * @return      execution status (TZ1SM_HAL_STATUS_OK:ok,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 */
tz1smHalStatus_t tz1smHalGpioWrite(const uint32_t pnr, bool const level);

/*
 * @brief
 * Reset BLE
 * @param[in]  on_off reset value(true:reset, false:no reset)
 * @return      execution status (TZ1SM_HAL_STATUS_OK:ok,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 */
tz1smHalStatus_t tz1smHalGpioBleReset(const bool reset);

/*
 * @brief
 * Suppress HPD
 * @param[in]  enable (true:Suppress HPD, false:no interference)
 */
void tz1smHalSuppressHpd(const bool enable);

/*
 * @brief
 * Read BLE CPU Status
 * @return false:ble cpu active, true:ble cpu low power mode
 */

bool tz1smHalGpioBleLowpowerStatus(void);

/*
 * @brief
 * Read GPIO BLE Host Wakeup Pin(true:UART is Active, false:No data)
 */
bool tz1smHalGpioBleHostWakeupStatus(void);

/*
 * @brief
 * Read GPIO BLE Request Wake Up Pin Value.
 */

bool tz1smHalGpioBleHostLowpowerStatus(void);

uint8_t tz1smHalGpioButtonInit(void (handler)(uint32_t pin));
uint8_t tz1smHalGpioButtonFinalize(void);
uint8_t tz1smHalGpioLedInit(void);
uint8_t tz1smHalGpioLedFinalize(void);

#else

#define TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP 30

void tz1smHalGpioInit(void);
void tz1smHalGpioUnInit(void);
bool tz1smHalGpioBleHostLowpowerStatus(void);
bool tz1smHalGpioBleLowpowerStatus(void);
bool tz1smHalGpioBleHostWakeupStatus(void);
tz1smHalStatus_t tz1smHalGpioBleReset(const bool reset);

#warning V0.0.1 is no supporting the other GPIO Drivers other than CMSIS_GPIO.

#endif

/*
 * @brief
 * Set BLE SLEEP Mode
 * @param[in] on_off mode value(true:ble active mode, false:permit ble
 *            low power mode)
 *
 */
void tz1smHalGpioBleWakeUp(const bool activation);

/*
 * UART
 *
 */

#if defined(TWIC_UART__CMSIS_UART)

/* In performing hardware control, it surely uses CMSIS UART API. In
 * using GPIO, it lets tz1smHalGpio surely mounted in this file pass,
 * and calls CMSIS GPIO API. Coding of the following functions is
 * carried out using CMSIS API. */

/*
 * @brief
 * Initialize UART
 */
void tz1smHalUartInit(uint32_t br, bool fc);

/*
 * @brief
 * Uninitialize UART
 */
void tz1smHalUartUnInit(void);

/*
 * @brief
 * Enable or Disable CTS/RTS
 * @param[in]   on_off(true:enable CTS/RTS, false:disable CTS/RTS).
 */
tz1smHalStatus_t tz1smHalUartControl(uint32_t br, bool fc);

/*
 * @brief
 * Flush UART Tx Buffer
 */
void tz1smHalUartTxBufFlush(void);

/*
 * @brief
 *  * Flush UART Rx Buffer
 */
void tz1smHalUartRxBufFlush(void);

/*
 * @brief
 * Asynchronous Send Data.
 * @param[in]   data send data
 * @param[in]   length send data length[byte]
 * @return      execution status (TZ1SM_HAL_STATUS_OK:normal,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 *                                TZ1SM_HAL_STATUS_ERROR_CTS:CTS Time is long)
 *                                TZ1SM_HAL_STATUS_ERROR_RESOURCE:No copy to buffer)
 */
tz1smHalStatus_t tz1smHalUartPostData(const uint8_t * const data,
                                    const uint16_t length);

/*
 * @brief
 * Synchronous Send Data.
 * @param[in]   data send data
 * @param[in]   length send data length[byte]
 * @return      execution status (TZ1SM_HAL_STATUS_OK:normal,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 *                                TZ1SM_HAL_STATUS_ERROR_CTS:CTS Time is long)
 */
tz1smHalStatus_t tz1smHalUartSendData(const uint8_t * const data,
                                    const uint16_t length);

/*
 * @brief
 * Receive Data.
 * @param[out]   data receive data
 * @param[out]   rest receive length[byte]
 * @return      execution status (TZ1SM_HAL_STATUS_OK:not stored data,
 *                                TZ1SM_HAL_STATUS_EVENT_MESSAGE:stored data,
 *                                TZ1SM_HAL_STATUS_ERROR_DRIVER:Driver Error,
 */
tz1smHalStatus_t tz1smHalUartPeekData(uint8_t * const data,
                                    uint16_t * const receive_length);

/*
 * @brief
 * Receive Data to software receive buffer.
 */
void tz1smHalUartGetData(void);

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
/*
 * @brief
 * Handles the power profile of BLE UART.
 */
void tz1smHalUartLowPower(bool enable);
bool tz1smHalUartDataAvailable(void);
#endif

#else

/* V0.0.1 is no supporting the other UART Drivers other than
 * CMSIS_UART. Here are just the stab of UART drivers.
 */

void tz1smHalUartInit(uint32_t br, bool fc);
void tz1smHalUartUnInit(void);
void tz1smHalUartTxBufFlush(void);
void tz1smHalUartRxBufFlush(void);
tz1smHalStatus_t tz1smHalUartPostData(const uint8_t * const data,
                                    const uint16_t length);
tz1smHalStatus_t tz1smHalUartSendData(const uint8_t * const data,
                                    const uint16_t length);
tz1smHalStatus_t tz1smHalUartPeekData(uint8_t * const data,
                                    uint16_t * const receive_length);
void tz1smHalUartGetData(void);

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
void tz1smHalUartLowPower(bool enable);
bool tz1smHalUartDataAvailable(void);
#endif

tz1smHalStatus_t tz1smHalUartControl(uint32_t br, bool fc);

#endif

#endif /* _TZ1SM_HAL_H_ */
