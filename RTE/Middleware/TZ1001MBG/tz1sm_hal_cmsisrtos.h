/**
 * @file tz1sm_hal_cmsisrtos.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0 Smart
 * @version V0.0.3
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


#ifndef _TZ1SM_HAL_CMSISRTOS_H_
#define _TZ1SM_HAL_CMSISRTOS_H_

#include "cmsis_os.h"

#define tz1smHalTimer_t osTimerDef_t
#define TZ1SM_HAL_TIMER osTimer
#define TZ1SM_HAL_TIMER_INIT osTimerDef
#define tz1smHalTimerId osTimerId
#define tz1smHalTimerCreate osTimerCreate
#define tz1smHalTimerDelete osTimerDelete
#define tz1smHalTimerStart osTimerStart
#define tz1smHalTimerStop osTimerStop
#define tz1smHalMutexId_t osMutexId_t
#define tz1smHalMutexDef_t osMutexDef_t
#define TZ1SM_HAL_MUTEX_INIT osMutexDef
#define TZ1SM_HAL_MUTEX osMutex
#define tz1smHalMutexCreate osMutexCreate
#define tz1smHalMutexWait osMutexWait
#define tz1smHalMutexRelease osMutexRelease
#define tz1smHalMutexDelete osMutexDelete

typedef enum {
  TZ1SM_HAL_STATUS_OK = osOK,
  TZ1SM_HAL_STATUS_EVENT_SIGNAL = osEventSignal,
  TZ1SM_HAL_STATUS_EVENT_MESSAGE = osEventMessage,
  TZ1SM_HAL_STATUS_EVENT_MAIL = osEventMail,
  TZ1SM_HAL_STATUS_EVENT_TIMEOUT = osEventTimeout,
  TZ1SM_HAL_STATUS_ERROR_PARAMETER = osErrorParameter,
  TZ1SM_HAL_STATUS_ERROR_RESOURCE = osErrorResource,
  TZ1SM_HAL_STATUS_ERROR_TIMEOUT_RESOURCE = osErrorTimeoutResource,
  TZ1SM_HAL_STATUS_ERROR_ISR = osErrorISR,
  TZ1SM_HAL_STATUS_ERROR_ISR_RECURSIVE = osErrorISRRecursive,
  TZ1SM_HAL_STATUS_ERROR_PRIORITY = osErrorPriority,
  TZ1SM_HAL_STATUS_ERROR_NOMEMORY = osErrorNoMemory,
  TZ1SM_HAL_STATUS_ERROR_VALUE = osErrorValue,
  TZ1SM_HAL_STATUS_ERROR_OS = osErrorOS,
  TZ1SM_HAL_STATUS_ERROR_CTS,
  TZ1SM_HAL_STATUS_ERROR_DRIVER,
  TZ1SM_HAL_STATUS_SOURCE_STOPPED,
  TZ1SM_HAL_STATUS_SOURCE_PREPARING,
  TZ1SM_HAL_STATUS_SOURCE_RUNNING,
  TZ1SM_HAL_STATUS_IGNORE,
  TZ1SM_HAL_STATUS_RESERVED = os_status_reserved,
} tz1smHalStatus_t;

typedef enum {
  TZ1SM_HAL_TIMER_ONCE = osTimerOnce,
  TZ1SM_HAL_TIMER_PERIODIC = osTimerPeriodic,
} tz1smHalTimerBehavior_t;

void tz1smHalOsYeild(void);

/*
 * @brief
 * Enters at a critical session or comes out.
 */
#define TZ1SM_HAL_INTR_STATUS_DEF volatile uint32_t tz1sm_hal_intr_status = 0
#define TZ1SM_HAL_INTR_STATUS tz1sm_hal_intr_status
#define TZ1SM_HAL_IRQ_DISABLE_SAVE() {              \
    tz1sm_hal_intr_status = __get_PRIMASK();        \
    if (!tz1sm_hal_intr_status) __disable_irq(); }
#define TZ1SM_HAL_IRQ_ENABLE_RESTORE() {          \
    if (!tz1sm_hal_intr_status) __enable_irq(); }
#define TZ1SM_HAL_IRQ_GET_IPSR() __get_IPSR()

#endif /* _TZ1SM_HAL_CMSISRTOS_H_ */