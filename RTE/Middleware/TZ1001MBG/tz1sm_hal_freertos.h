/**
 * @file tz1sm_hal_freertos.h
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


#ifndef _TZ1SM_HAL_FREERTOS_H_
#define _TZ1SM_HAL_FREERTOS_H_

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#define TZ1SM_HAL_MSEC_PER_SEC (1000L)

typedef uint32_t tz1smHalTimerId;

#define osOK                   (0)
#define osEventSignal          (0x08)
#define osEventMessage         (0x10)
#define osEventMail            (0x20)
#define osEventTimeout         (0x40)
#define osErrorParameter       (0x80)
#define osErrorResource        (0x81)
#define osErrorTimeoutResource (0xC1)
#define osErrorISR             (0x82)
#define osErrorISRRecursive    (0x83)
#define osErrorPriority        (0x84)
#define osErrorNoMemory        (0x85)
#define osErrorValue           (0x86)
#define osErrorOS              (0xFF)
#define os_status_reserved     (0x7FFFFFFF) /* prevent from enum
                                             * down-size compiler
                                             * optimization. */
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

#define osTimerOnce     (0x01)
#define osTimerPeriodic (0x02)

typedef uint32_t tz1smHalTimerBehavior_t;
#define TZ1SM_HAL_TIMER_ONCE     osTimerOnce
#define TZ1SM_HAL_TIMER_PERIODIC osTimerPeriodic
#define TZ1SM_HAL_TIMER_RUN (1 << 7)

typedef struct {
  tz1utListHead_t sibling;
  uint32_t period;
  uint32_t timestamp;
  tz1smHalTimerBehavior_t type;
  void const *argument;
  void (*function)(void const *);
  TimerHandle_t xTimer;  /* FreeRTOS Timer Handler */
  int32_t lTimerID;  /* FreeRTOS Timer ID */
} tz1smHalTimer_t;

#define TZ1SM_HAL_TIMER(name) &tz1smHalTimer_##name
#define TZ1SM_HAL_TIMER_INIT(name, function)     \
  static tz1smHalTimer_t tz1smHalTimer_##name = { \
    NULL,NULL,0,0,0,NULL,function,{0},0}

tz1smHalTimerId tz1smHalTimerCreate(
  const tz1smHalTimer_t *timer, tz1smHalTimerBehavior_t type, void *argument);
tz1smHalStatus_t tz1smHalTimerDelete(tz1smHalTimerId timer_id);
tz1smHalStatus_t tz1smHalTimerStart(
  tz1smHalTimerId timer_id, uint32_t millisec);
tz1smHalStatus_t tz1smHalTimerStop(tz1smHalTimerId	timer_id);
void tz1smHalTimerFakeAlarmTrigger(uint32_t hz);

typedef SemaphoreHandle_t tz1smHalMutexId_t;
typedef uint16_t tz1smHalMutexDef_t;
#define TZ1SM_HAL_MUTEX(name) &tz1smHalMutex_##name
#define TZ1SM_HAL_MUTEX_INIT(name) \
  static tz1smHalMutexDef_t tz1smHalMutex_##name

tz1smHalMutexId_t tz1smHalMutexCreate(tz1smHalMutexDef_t * const mutex_def);
tz1smHalStatus_t tz1smHalMutexWait(const tz1smHalMutexId_t mutex_id,
                                 const uint32_t millisec);
tz1smHalStatus_t tz1smHalMutexRelease(tz1smHalMutexId_t mutex_id);
tz1smHalStatus_t tz1smHalMutexDelete(tz1smHalMutexId_t mutex_id);

void tz1smHalOsYeild(void);

/*
 * @brief
 * Enters at a critical session or comes out.
 */
#define TZ1SM_HAL_INTR_STATUS_DEF volatile uint32_t tz1sm_hal_intr_status = 0
#define TZ1SM_HAL_INTR_STATUS tz1sm_hal_intr_status
#define TZ1SM_HAL_IRQ_DISABLE_SAVE() {             \
    tz1sm_hal_intr_status = __get_PRIMASK();       \
    if (!tz1sm_hal_intr_status) __disable_irq(); }
#define TZ1SM_HAL_IRQ_ENABLE_RESTORE() {          \
    if (!tz1sm_hal_intr_status) __enable_irq(); }
#define TZ1SM_HAL_IRQ_GET_IPSR() __get_IPSR()

#endif /* _TZ1SM_HAL_FREERTOS_H_ */
