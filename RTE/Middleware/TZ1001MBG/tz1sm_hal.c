/**
 * @file tz1sm_hal.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @date $LastChangedDate$
 * @note
 */

/*
 * COPYRIGHT (C) 2014
 * TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS COMPANY
 * ALL RIGHTS RESERVED
 */
#include <stdint.h>
#include <stdio.h>

#include "RTE_Device.h"
#include "tz1sm_config.h"
#include "tz1ut_list.h"
#include "tz1sm_hal.h"

#undef TZ1EM_ENABLE_LOG

#if defined(TWIC_RTOS__CMSIS_RTOS)
#elif defined(TWIC_RTOS__FREERTOS)
#include "FreeRTOS.h"
#include "timers.h"
#endif

#if defined(TWIC_GPIO__CMSIS_GPIO)
#include "TZ10xx.h"
#include "GPIO_TZ10xx.h"
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
#endif

#if defined(TWIC_UART__CMSIS_UART)
#include "TZ10xx.h"
#include "Driver_UART.h"
extern ARM_DRIVER_UART Driver_UART2;
#endif

#if defined(TWIC_PMU__CMSIS_PMU)
extern TZ10XX_DRIVER_PMU Driver_PMU;
#endif

#if defined(TWIC_PMU__CMSIS_RTC)
#include "TZ10xx.h"
#include "RTC_TZ10xx.h"
extern TZ10XX_DRIVER_RTC Driver_RTC;
#endif

#if defined(TWIC_PMU__CMSIS_MISC)
#include "Driver_NOR.h"
#include "NOR_TZ10xx.h"
extern TZ10XX_DRIVER_NOR Driver_NOR0;
#if RTE_UART0
#include "Driver_UART.h"
extern ARM_DRIVER_UART Driver_UART0;
#endif
#if RTE_UART1
#include "Driver_UART.h"
extern ARM_DRIVER_UART Driver_UART1;
#endif
#if RTE_ADCC12
#include "ADCC_TZ10xx_common.h"
#include "ADCC12_TZ10xx.h"
extern TZ10XX_DRIVER_ADCC12 Driver_ADCC12;
#endif
#if RTE_ADCC24
#include "ADCC_TZ10xx_common.h"
#include "ADCC24_TZ10xx.h"
extern TZ10XX_DRIVER_ADCC24 Driver_ADCC24;
#endif
#if RTE_SDMAC
#include "SDMAC_TZ10xx.h"
extern TZ10XX_DRIVER_SDMAC Driver_SDMAC;
#endif
#if RTE_USB2FS
#include "USBD_TZ10xx.h"
extern TZ10XX_DRIVER_USBD Driver_USBD0;
#endif
#if RTE_AESA
#include "AESA_TZ10xx.h"
extern TZ10XX_DRIVER_AESA Driver_AESA;
#endif
#if RTE_SPI0
#include "SPI_TZ10xx.h"
extern TZ10XX_DRIVER_SPI Driver_SPI0;
#endif
#if RTE_SPI1
#include "SPI_TZ10xx.h"
extern TZ10XX_DRIVER_SPI Driver_SPI1;
#endif
#if RTE_SPI2
#include "SPI_TZ10xx.h"
extern TZ10XX_DRIVER_SPI Driver_SPI2;
#endif
#if RTE_SPI3
#include "SPI_TZ10xx.h"
extern TZ10XX_DRIVER_SPI Driver_SPI3;
#endif
#if RTE_I2C0
#include "Driver_I2C.h"
extern ARM_DRIVER_I2C Driver_I2C0;
#endif
#if RTE_I2C1
#include "Driver_I2C.h"
extern ARM_DRIVER_I2C Driver_I2C1;
#endif
#if RTE_I2C2
#include "Driver_I2C.h"
extern ARM_DRIVER_I2C Driver_I2C2;
#endif
#if RTE_WDT
#include "WDT_TZ10xx.h"
extern TZ10XX_DRIVER_WDT Driver_WDT;
#endif
#endif  /* #if defined(TWIC_PMU__CMSIS_MISC) */

#if defined(TWIC_UART__CMSIS_UART)
#if defined(TWIC_DEBUG_LOG_TRACE)
#define TWIC_UART_DBG_TX_BUFFER_SIZE 1024
#if (TWIC_DEBUG_UART_NUMBER == 0)
extern ARM_DRIVER_UART Driver_UART0;
#define TWIC_DEBUG_UART_CH (&Driver_UART0)
#elif (TWIC_DEBUG_UART_NUMBER == 1)
extern ARM_DRIVER_UART Driver_UART1;
#define TWIC_DEBUG_UART_CH (&Driver_UART1)
#endif
#endif
#endif

/*
 * The code of a timer and exclusion control is a very simple
 * sample. For this reason, in case this sample code is applied, it
 * recommends changing into better mounting suitable for the
 * environment, or using suitable RTOS.
 */

#if defined(TZ1EM_ENABLE_LOG)
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "Driver_UART.h"
/* UART CMSIS interface */
extern ARM_DRIVER_UART Driver_UART0;

static const char *power_mode_string[] = {
  "ACTIVE",
  "SLEEP0",
  "SLEEP1",
  "SLEEP2",
  "WAIT",
  "WAIT_RET",
  "RETENTION",
  "RTC",
  "STOP",
  "SLEEPDEEP0",
  "SLEEPDEEP1",
  "SLEEPDEEP2"
};

static const char *voltage_mode_string[] = {
  "ModeA",
  "ModeB",
  "ModeC",
  "ModeD",
};

void print_log(const char *str)
{
  int len;
  int r;

  len = strlen(str);

  while (len > 0) {
    r = Driver_UART0.WriteData((const uint8_t *)str, len);
    if (r < 0) {
      return; /* error */
    } else {
      str += r;
      len -= r;
    }
  }
  /* wait for finish uart output */
  while (!Driver_UART0.TxDone()) {
    /* DO NOTHING */
  }
}
#endif


/*
 * DEBUG USE_UART
 *
 */

#if defined(TWIC_DEBUG_LOG_TRACE) && defined(TWIC_UART__CMSIS_UART)

#if !defined(TWIC_TOOLCHAIN_EWARM)
struct __FILE { int handle; /* Add whatever is needed */ };
FILE __stdout;
FILE __stdin;
#endif

/* callback handler */
static void uart_dbg_handler(ARM_UART_EVENT e);
static void uart_dbg_handler_tx(void);

/* tx buffer */
static uint8_t uart_dbg_tx_buffer[TWIC_UART_DBG_TX_BUFFER_SIZE];
static uint16_t uart_dbg_tx_rp = 0; /* count up by HW FIFO */
static uint16_t uart_dbg_tx_wp = 0; /* count up by application */

void tz1smHalDebugUartInit(void)
{
  ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;

  drv->Initialize(uart_dbg_handler, (1u << ARM_UART_EVENT_RX_TIMEOUT)
                  | (1u << ARM_UART_EVENT_TX_THRESHOLD)
                  | (1u << ARM_UART_EVENT_RX_THRESHOLD));
  drv->Configure(115200, 8, ARM_UART_PARITY_NONE, ARM_UART_STOP_BITS_1,
                 ARM_UART_FLOW_CONTROL_NONE);
  drv->SetTxThreshold(8);
  drv->SetRxThreshold(8);
  drv->PowerControl(ARM_POWER_FULL);

  return;
}

void tz1smHalDebugUartUnInit(void)
{
  ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;

  drv->Uninitialize();

  return;
}

#if !defined(TWIC_TOOLCHAIN_EWARM)
int fputc(int ch, FILE *f)
{
  ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;
  uint16_t rest_size;
  uint32_t ipsr;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  ipsr = __get_IPSR();
  if (0 == ipsr)
    TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  /* check buffer size */
  if (uart_dbg_tx_wp != uart_dbg_tx_rp) {
    if (uart_dbg_tx_wp >= uart_dbg_tx_rp) {
      rest_size = (TWIC_UART_DBG_TX_BUFFER_SIZE - uart_dbg_tx_wp) +
        uart_dbg_tx_rp - 1;
    }
    else {
      rest_size = uart_dbg_tx_rp - uart_dbg_tx_wp - 1;
    }
    
    if (0 == rest_size) {
      /* buffer over flow */
      uint8_t msg[] = {"\n\n\rOver Flow\n\n\r"};

      drv->FlushTxBuffer();
      uart_dbg_tx_rp = uart_dbg_tx_wp;
      drv->WriteData(msg, sizeof(msg));
      /* wait for finish uart output */
      while (!drv->TxDone()); /* DO NOTHING */
    }
  }
  
  uart_dbg_tx_buffer[uart_dbg_tx_wp++] = (uint8_t)ch;
  
  if (TWIC_UART_DBG_TX_BUFFER_SIZE == uart_dbg_tx_wp) {
    uart_dbg_tx_wp = 0;
  }
	
  uart_dbg_handler_tx();
  
  if (0 == ipsr)
    TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return ch;
}
#else
static size_t check_dbg_buffer_size(void)
{
	size_t rest_size;
	/* check buffer size */
	if( uart_dbg_tx_wp != uart_dbg_tx_rp ){
		if( uart_dbg_tx_wp >= uart_dbg_tx_rp ){
			rest_size = (TWIC_UART_DBG_TX_BUFFER_SIZE - uart_dbg_tx_wp);
		}else{
			rest_size = uart_dbg_tx_rp - uart_dbg_tx_wp - 1;
		}
	}else{
		rest_size = TWIC_UART_DBG_TX_BUFFER_SIZE;
	}
	return rest_size;
}

size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
	ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;
	uint32_t ipsr;
	size_t rest_size, copy_size;
	size_t buf_index = 0;
	TZ1SM_HAL_INTR_STATUS_DEF;
	
	ipsr = __get_IPSR();
	if (0 == ipsr)
    TZ1SM_HAL_IRQ_DISABLE_SAVE();

	while (bufSize > buf_index) {
		rest_size = check_dbg_buffer_size();
		if (0 == rest_size) {
			/* buffer over flow */
			uint8_t msg[] = {"\n\n\rOver Flow\n\n\r"};
			
			drv->FlushTxBuffer();
			uart_dbg_tx_rp = uart_dbg_tx_wp;
			drv->WriteData(msg, sizeof(msg));
			/* wait for finish uart output */
			while (!drv->TxDone()); /* DO NOTHING */
		}
		if (rest_size > bufSize - buf_index) {
			copy_size = bufSize - buf_index;
		} else {
			copy_size = rest_size;
		}
		memcpy(&uart_dbg_tx_buffer[uart_dbg_tx_wp], &buf[buf_index], copy_size);
		buf_index += copy_size;
		uart_dbg_tx_wp += copy_size;
		if (TWIC_UART_DBG_TX_BUFFER_SIZE == uart_dbg_tx_wp) {
			uart_dbg_tx_wp = 0;
		}
		uart_dbg_handler_tx();
	}
  
	if (0 == ipsr)
	  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
	
	return bufSize;
}
#endif

static void uart_dbg_handler_tx(void)
{
  ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;
  int32_t count;
  uint16_t total = 0;
  
  if (uart_dbg_tx_rp != uart_dbg_tx_wp) {
    /* send from tx buffer at first.*/
    if (uart_dbg_tx_wp < uart_dbg_tx_rp) {
      count = drv->WriteData(&uart_dbg_tx_buffer[uart_dbg_tx_rp],
                             (TWIC_UART_DBG_TX_BUFFER_SIZE - uart_dbg_tx_rp));
      /* wait for finish uart output */
      while (!drv->TxDone());
      if (0 > count) {
        goto END;
      }
      else {
        uart_dbg_tx_rp += count;
        total += count;
      }
      if (uart_dbg_tx_rp == TWIC_UART_DBG_TX_BUFFER_SIZE) {
        uart_dbg_tx_rp = 0;
      }
    }
    
    if (uart_dbg_tx_wp > uart_dbg_tx_rp) {
      count = drv->WriteData(&uart_dbg_tx_buffer[uart_dbg_tx_rp],
                             (uart_dbg_tx_wp - uart_dbg_tx_rp));
      /* wait for finish uart output */
      while (!drv->TxDone());
      if (0 > count) {
        goto END;
      }
      else{
        uart_dbg_tx_rp += count;
        total += count;
      }
    }
  }
  
END:
  return;
}

static void uart_dbg_handler_rx(void)
{
	ARM_DRIVER_UART *drv = TWIC_DEBUG_UART_CH;
	int32_t count;
	uint8_t buf[1];

	while(true){
		count = drv->ReadData(buf, 1);
		if( count <= 0 ){
			break;
		}
	}

}

static void uart_dbg_handler(ARM_UART_EVENT e)
{
  switch (e) {
  case ARM_UART_EVENT_TX_THRESHOLD:
    uart_dbg_handler_tx();
    break;
  case ARM_UART_EVENT_RX_TIMEOUT:
  case ARM_UART_EVENT_RX_THRESHOLD:
    uart_dbg_handler_rx();
    break;
  default:
    break;
  }

  return;
}

#else

void tz1smHalDebugUartInit(void) {;}
void tz1smHalDebugUartUnInit(void) {;}
  
#endif

/*
 * POSIX
 *
 */

#if defined(TWIC_POSIX__POSIX)

#else

void *memset(void *s, int c, unsigned long length)
{
  char *ptr = (char *)b;

  while (length--)
    *ptr++ = c;

  return b;
}

void *memcpy(
  void * __restrict__ dest, const void * __restrict__ src, unsigned long n)
{
  register char *_dest = dest;
  register const char *_src = src;
  
  if (0 == n)
    return dest;
  else {
    register const char *bottom = _src + n;
    do
      *_dest++ = *_src++;
    while (_src != bottom);
  }

  return dest;
}

#endif


/*
 * RTOS
 *
 */

#if defined(TWIC_RTOS__CMSIS_RTOS)

/*
 * Task.
 *
 */

void tz1smHalOsYeild(void) { return; } /* Fix me. */

#elif defined(TWIC_RTOS__FREERTOS)

TZ1UT_LIST_DEF(tz1sm_hal_timer_chain_root);

void vTimerCallback(TimerHandle_t pxTimer)
{
  tz1smHalTimer_t *timer;
  int32_t lTimerID;
  tz1utListHead_t *list = TZ1UT_LIST(tz1sm_hal_timer_chain_root);
    
  lTimerID = ( int32_t ) pvTimerGetTimerID( pxTimer );
  
  tz1utListEach(tz1smHalTimer_t, timer, list, sibling) {
    if (timer->lTimerID == lTimerID) {
      timer->function(timer->argument);
      break;
    }
  }

  return;
}

tz1smHalTimerId tz1smHalTimerCreate(const tz1smHalTimer_t *timer,
                                    tz1smHalTimerBehavior_t type,
                                    void *argument)
{
  tz1smHalTimer_t *_timer = (tz1smHalTimer_t *)timer;
  tz1utListHead_t *node = &_timer->sibling;
  static int32_t count = 0;
  UBaseType_t uxAutoReload;
  
  tz1utListInit(node);
  _timer->period = 0;
  _timer->timestamp = 0;
  _timer->type = type;
  _timer->argument = argument;

  _timer->lTimerID = count;

  if (TZ1SM_HAL_TIMER_ONCE == type)
    uxAutoReload = pdFALSE;
  else
    uxAutoReload = pdTRUE;
    
  _timer->xTimer = xTimerCreate(
    "Timer", 100, uxAutoReload, (void *)count++, vTimerCallback);
  
  if (_timer->xTimer != NULL) {
    tz1utListAdd(node, TZ1UT_LIST(tz1sm_hal_timer_chain_root));
    return (tz1smHalTimerId)node;
  }

  return 0;
}

tz1smHalStatus_t tz1smHalTimerDelete(tz1smHalTimerId timer_id)
{
  tz1utListHead_t *node;
  tz1smHalTimer_t *timer;
  BaseType_t pdRet;
  
  node = (tz1utListHead_t *)timer_id;
  timer = tz1utListContainerOf(node, tz1smHalTimer_t, sibling);
  pdRet = xTimerDelete(timer->xTimer, (1000 / portTICK_RATE_MS));
  
  if (pdRet == pdTRUE) {
    tz1utListDel(node);
    tz1utListInit(node);
    return TZ1SM_HAL_STATUS_OK;
  }

  return TZ1SM_HAL_STATUS_ERROR_OS;
}

tz1smHalStatus_t
tz1smHalTimerStart(tz1smHalTimerId timer_id, uint32_t millisec)
{
  tz1utListHead_t *node;
  tz1smHalTimer_t *timer;
  BaseType_t pdRet = pdTRUE;
  
  node = (tz1utListHead_t *)timer_id;
  timer = tz1utListContainerOf(node, tz1smHalTimer_t, sibling);
  timer->period = millisec;

  if ( (millisec / portTICK_RATE_MS) != 0 ) {
    pdRet = xTimerChangePeriod(timer->xTimer, (millisec / portTICK_RATE_MS),
                               (1000 / portTICK_RATE_MS));
    pdRet |= xTimerStart(timer->xTimer, (1000 / portTICK_RATE_MS));
  }
  
  if (pdRet == pdTRUE) return TZ1SM_HAL_STATUS_OK;

  return TZ1SM_HAL_STATUS_ERROR_OS;
}

tz1smHalStatus_t tz1smHalTimerStop(tz1smHalTimerId timer_id)
{
  tz1utListHead_t *node;
  tz1smHalTimer_t *timer;
  BaseType_t pdRet;
  
  node = (tz1utListHead_t *)timer_id;
  timer = tz1utListContainerOf(node, tz1smHalTimer_t, sibling);
  pdRet = xTimerStop(timer->xTimer, (1000 / portTICK_RATE_MS));
  
  if (pdRet == pdTRUE) return TZ1SM_HAL_STATUS_OK;

  return TZ1SM_HAL_STATUS_ERROR_OS;
}

void tz1smHalTimerFakeAlarmTrigger(uint32_t hz)
{
  return;
}


/*
 * MUTEX
 *
 */

/* Create and Initialize a Mutex object. */
tz1smHalMutexId_t tz1smHalMutexCreate(tz1smHalMutexDef_t * const mutex_def)
{
  QueueHandle_t xHandle;

  *mutex_def = 0;
  xHandle = xSemaphoreCreateBinary();
  xSemaphoreGive( xHandle );

  return xHandle;
}

/* Wait until a Mutex becomes available. */
tz1smHalStatus_t tz1smHalMutexWait(const tz1smHalMutexId_t mutex_id,
                                 const uint32_t millisec)
{
  BaseType_t pdRet;
  
  pdRet = xSemaphoreTake(mutex_id, (millisec / portTICK_RATE_MS));
  if ( pdRet == pdTRUE )
    return TZ1SM_HAL_STATUS_OK;

  return TZ1SM_HAL_STATUS_ERROR_RESOURCE;
}

/* Release a Mutex that was obtained by osMutexWait. */
tz1smHalStatus_t tz1smHalMutexRelease(tz1smHalMutexId_t mutex_id)
{
  BaseType_t pdRet;

  pdRet = xSemaphoreGive( mutex_id );
  if ( pdRet == pdTRUE )
    return TZ1SM_HAL_STATUS_OK;

  return TZ1SM_HAL_STATUS_ERROR_RESOURCE;
}

/* Delete a Mutex that was created by osMutexCreate. */
tz1smHalStatus_t tz1smHalMutexDelete(tz1smHalMutexId_t mutex_id)
{
  vSemaphoreDelete(mutex_id);

  return TZ1SM_HAL_STATUS_OK;
}


/*
 * Task.
 *
 */

void tz1smHalOsYeild(void)
{
  taskYIELD();
}

#else

TZ1UT_LIST_DEF(tz1sm_hal_timer_chain_root);
static uint32_t tz1sm_hal_jiffies;
extern uint32_t SystemCoreClock;

tz1smHalTimerId tz1smHalTimerCreate(const tz1smHalTimer_t *timer,
                                  tz1smHalTimerBehavior_t type,
                                  void *argument)
{
  tz1smHalTimer_t *_timer = (tz1smHalTimer_t *)timer;
  tz1utListHead_t *node = &_timer->sibling;

  tz1utListInit(node);
  _timer->period = 0;
  _timer->timestamp = 0;
  _timer->type = type;
  _timer->argument = argument;
  tz1utListAdd(node, TZ1UT_LIST(tz1sm_hal_timer_chain_root));
  
  return (tz1smHalTimerId)node;
}

tz1smHalStatus_t tz1smHalTimerDelete(tz1smHalTimerId timer_id)
{
  tz1utListHead_t *node = (tz1utListHead_t *)timer_id;
  
  tz1utListDel(node);
  
  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t
tz1smHalTimerStart(tz1smHalTimerId timer_id, uint32_t millisec)
{
  tz1utListHead_t *node;
  tz1smHalTimer_t *timer;
  
  node = (tz1utListHead_t *)timer_id;
  timer = tz1utListContainerOf(node, tz1smHalTimer_t, sibling);
  timer->type |= TZ1SM_HAL_TIMER_RUN;
  timer->period = millisec;
  timer->timestamp = tz1sm_hal_jiffies;
  
  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t tz1smHalTimerStop(tz1smHalTimerId timer_id)
{
  tz1utListHead_t *node;
  tz1smHalTimer_t *timer;
  
  node = (tz1utListHead_t *)timer_id;
  timer = tz1utListContainerOf(node, tz1smHalTimer_t, sibling);
  timer->type &= ~TZ1SM_HAL_TIMER_RUN;
  
  return TZ1SM_HAL_STATUS_OK;
}

void tz1smHalTimerFakeAlarmTrigger(uint32_t hz)
{
  tz1smHalTimer_t *timer;
  uint32_t j_to_millisec, t_to_millisec;

  if (TZ1SM_HAL_MSEC_PER_SEC >= hz)
    j_to_millisec = (TZ1SM_HAL_MSEC_PER_SEC / hz) * tz1sm_hal_jiffies;
  else
    j_to_millisec = (tz1sm_hal_jiffies + (hz / TZ1SM_HAL_MSEC_PER_SEC) - 1) /
        (hz / TZ1SM_HAL_MSEC_PER_SEC);

  tz1utListEach(tz1smHalTimer_t,
                timer, TZ1UT_LIST(tz1sm_hal_timer_chain_root), sibling) {
    if (!(timer->type & TZ1SM_HAL_TIMER_RUN))
      continue;

    if (TZ1SM_HAL_MSEC_PER_SEC >= hz)
      t_to_millisec = (TZ1SM_HAL_MSEC_PER_SEC / hz) * timer->timestamp;
    else
      t_to_millisec = (timer->timestamp + (hz / TZ1SM_HAL_MSEC_PER_SEC) - 1) /
          (hz / TZ1SM_HAL_MSEC_PER_SEC);

    if (j_to_millisec < t_to_millisec) {
      if (t_to_millisec - j_to_millisec < timer->period)
        continue;
    } else {
      if (j_to_millisec - t_to_millisec < timer->period)
        continue;
    }
    timer->function(timer->argument);
    timer->timestamp = tz1sm_hal_jiffies;
    if ((timer->type & TZ1SM_HAL_TIMER_ONCE))
      timer->type &= ~TZ1SM_HAL_TIMER_RUN;
  }
  
  tz1sm_hal_jiffies++;

  return;
}


/*
 * MUTEX
 *
 */

/* Create and Initialize a Mutex object. */
tz1smHalMutexId_t tz1smHalMutexCreate(tz1smHalMutexDef_t * const mutex_def)
{
  *mutex_def = 0;
  return (tz1smHalMutexId_t)mutex_def;
}

/* Wait until a Mutex becomes available. */
tz1smHalStatus_t tz1smHalMutexWait(const tz1smHalMutexId_t mutex_id,
                                 const uint32_t millisec)
{
  return (0 == *(tz1smHalMutexDef_t *)mutex_id) ?
    TZ1SM_HAL_STATUS_OK : TZ1SM_HAL_STATUS_ERROR_RESOURCE;
}

/* Release a Mutex that was obtained by osMutexWait. */
tz1smHalStatus_t tz1smHalMutexRelease(tz1smHalMutexId_t mutex_id)
{
  *(tz1smHalMutexDef_t *)mutex_id = 0;
  return TZ1SM_HAL_STATUS_OK;
}

/* Delete a Mutex that was created by osMutexCreate. */
tz1smHalStatus_t tz1smHalMutexDelete(tz1smHalMutexId_t mutex_id)
{
  *(tz1smHalMutexDef_t *)mutex_id = 0;
  return TZ1SM_HAL_STATUS_OK;
}


/*
 * Task.
 *
 */

void tz1smHalOsYeild(void) { return; }

#endif


/*
 * PMU
 *
 */

#if defined(TWIC_PMU__CMSIS_PMU)

tz1smHalStatus_t tz1smHalLePmuStart(void)
{
  PMU_STATUS status = PMU_OK;
  
  /* PMU_CD_MPIER  :CPU,SRAM and SPI
   * PMU_CD_PPIER0 :GPIO,SPIM2/3
   * PMU_CD_PPIER1 :SPIM0/1,I2C0/1, UART0/1
   * PMU_CD_PPIER2 :UART2 */

#if defined(TWIC_DEBUG_LOG_TRACE) || defined(TWIC_DEBUG_LOG_UART)
#if (TWIC_DEBUG_UART_NUMBER == 0)
  tz1smStartClock(TZ1SM_HAL_PCD_UART0|TZ1SM_HAL_PCD_PP1);
#elif (TWIC_DEBUG_UART_NUMBER == 1)
  tz1smStartClock(TZ1SM_HAL_PCD_UART1|TZ1SM_HAL_PCD_PP1);
#endif  
#endif

  /* UARTCLK(PMU_CSM_UART2) must be less than 5/3 * PCLK(PMU_CD_PPIER2) */
#if defined(TZ1XXX_BOARD_BLUEBIRD)
  if (PMU_VOLTAGE_MODE_C != (Driver_PMU.GetVoltageMode()))
    return TZ1SM_HAL_STATUS_ERROR_RESOURCE;
  Driver_PMU.SetPrescaler(PMU_CD_UART2, 3);
  Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_OSC12M);
#else
  tz1smStartClock(TZ1SM_HAL_PCD_UART2|TZ1SM_HAL_PCD_PP1);
#endif
#if 0
  //pmu_status = Driver_PMU.SelectClockSource(
  //PMU_CSM_UART2, PMU_CLOCK_SOURCE_SIOSC4M);
  //if (pmu_status != PMU_OK)
  //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
  //pmu_status = Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 1);
  //if (pmu_status != PMU_OK)
  //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
  //pmu_status = Driver_PMU.SetPrescaler(PMU_CD_UART2, 1);
  //if (pmu_status != PMU_OK)
  //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
  
  /* The IO setup of the PINs, connected to the internal BLE Controller. */
#if defined(TZ1SM_HAL_GPIO_BLE_STATUS)
  status = Driver_PMU.StandbyInputBuffer(TZ1SM_HAL_GPIO_BLE_STATUS, 0);
  if (status != PMU_OK) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
#if defined(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP)
  status = Driver_PMU.StandbyInputBuffer(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP, 0);
  if (status != PMU_OK) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
  /* 32kHz Clock source choice. */
#if defined(TWIC_LECE_LOWPOWER)
#if defined(TWIC_LECE_CLK32K_SLPXOIN_OSC32K_FROM_CG_CLK32K_OUT)
  /* tz1smHalInitializePcdSystem dose it. */
  //Driver_PMU.SelectClockSource(PMU_CSM_RTC, PMU_CLOCK_SOURCE_OSC32K);
  //status = Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC32K);
  //if (status != PMU_OK)
  //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
#if defined(TWIC_LECE_CLK32K_SLPXOIN_CLOCK)
  status = Driver_PMU.Enable32kOut(1);
  if (status != PMU_OK) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
#endif
  
  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t tz1smHalLePmuStatus(void)
{
  PMU_CLOCK_SOURCE_STATE clock_state;
  tz1smHalStatus_t ret;
  
  clock_state = Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_OSC12M);
  switch(clock_state) {
  case PMU_CLOCK_SOURCE_STATE_STOPPED:
    ret = TZ1SM_HAL_STATUS_SOURCE_STOPPED;
    break;
  case PMU_CLOCK_SOURCE_STATE_PREPARING:
    ret = TZ1SM_HAL_STATUS_SOURCE_PREPARING;
    break;
  case PMU_CLOCK_SOURCE_STATE_RUNNING:
    ret = TZ1SM_HAL_STATUS_SOURCE_RUNNING;
    break;
  default:
    ret = TZ1SM_HAL_STATUS_ERROR_DRIVER;
    break;
  }

  return ret;
}

tz1smHalStatus_t tz1smHalLePmuStop(const bool stop_osc32K)
{
  PMU_STATUS status;
  
  /* INPUT PINs */
#if defined(TZ1SM_HAL_GPIO_BLE_STATUS)
  status = Driver_PMU.StandbyInputBuffer(TZ1SM_HAL_GPIO_BLE_STATUS, 1);
  if (PMU_OK != status) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif  
#if defined(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP)
  status = Driver_PMU.StandbyInputBuffer(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP, 1);
  if (PMU_OK != status) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
  /* tz1smStopClock should do it. */
  //if (true == stop_osc12M) {
  //PMU_CLOCK_SOURCE_STATE clock_state;
  //clock_state = Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_OSC12M);
  //if (PMU_CLOCK_SOURCE_STATE_STOPPED != clock_state) {
  //status = Driver_PMU.StopClockSource(PMU_CLOCK_SOURCE_OSC12M);
  //if (status != PMU_OK)
  //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
  //}
  //}

#if defined(TWIC_LECE_LOWPOWER)
  /* 32kHz Clock source choice. */
  if (true == stop_osc32K) {
#if defined(TWIC_LECE_CLK32K_SLPXOIN_OSC32K_FROM_CG_CLK32K_OUT)
    /* tz1smStopClock should do it. */
    //status = Driver_PMU.StopClockSource(PMU_CLOCK_SOURCE_OSC32K);
    //if (status != PMU_OK)
    //return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
#if defined(TWIC_LECE_CLK32K_SLPXOIN_CLOCK)
    status = Driver_PMU.Enable32kOut(0);
    if (PMU_OK != status) return TZ1SM_HAL_STATUS_ERROR_DRIVER;
#endif
  }
#else
  (void)stop_osc32K;
#endif
  
  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t tz1smHalLePmuClockSourceState32K(void)
{
#if defined(TWIC_LECE_CLK32K_SLPXOIN_CLOCK)
  tz1smHalStatus_t ret;
  PMU_CLOCK_SOURCE_STATE clock_state;
  
  clock_state = Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_OSC32K);
  switch(clock_state) {
  case PMU_CLOCK_SOURCE_STATE_STOPPED:
    ret = TZ1SM_HAL_STATUS_SOURCE_STOPPED;
    break;
  case PMU_CLOCK_SOURCE_STATE_PREPARING:
    ret = TZ1SM_HAL_STATUS_SOURCE_PREPARING;
    break;
  case PMU_CLOCK_SOURCE_STATE_RUNNING:
    ret = TZ1SM_HAL_STATUS_SOURCE_RUNNING;
    break;
  default:
    ret = TZ1SM_HAL_STATUS_ERROR_DRIVER;
    break;
  }

  return ret;
#else
  /*
   * The low power clock must be somehow confirmed.
   * It depends on the board design.
   */
  return TZ1SM_HAL_STATUS_IGNORE;
#endif  
}

uint8_t tz1smHalPmuButtonInit(void)
{
#if defined(TWIC_BUTTON_GPIO_NO)
  PMU_STATUS pmu_status;
  
  if ((TWIC_BUTTON_GPIO_NO > 0 && TWIC_BUTTON_GPIO_NO < 16) ||
      (TWIC_BUTTON_GPIO_NO > 23 && TWIC_BUTTON_GPIO_NO < 32)) {
    pmu_status = Driver_PMU.StandbyInputBuffer(
      (PMU_IO_FUNC)TWIC_BUTTON_GPIO_NO, 0);
    if (pmu_status != PMU_OK)
      return 1;
  }
#endif

  return 0;
}

uint8_t tz1smHalPmuButtonFinalize(void)
{
#if defined(TWIC_BUTTON_GPIO_NO)
  PMU_STATUS pmu_status;
  
  if ((TWIC_BUTTON_GPIO_NO > 0 && TWIC_BUTTON_GPIO_NO < 16) ||
      (TWIC_BUTTON_GPIO_NO > 23 && TWIC_BUTTON_GPIO_NO < 32)) {
    pmu_status = Driver_PMU.StandbyInputBuffer(
      (PMU_IO_FUNC)TWIC_BUTTON_GPIO_NO, 1);
    if (pmu_status != PMU_OK)
      return 1;
  }
#endif

  return 0;  
}

uint8_t tz1smHalPmuLedInit(void) { return 0; }

uint8_t tz1smHalPmuLedFinalize(void) { return 0; }

/**
 * @brief Initialize PCD system
 * @return \ref TZ1SM_HAL_STATUS
 */
tz1smHalStatus_t tz1smHalInitializePcdSystem(void)
{
  Driver_PMU.Initialize(NULL);

  Driver_PMU.SetPowerDomainState(PMU_PD_FLASH, PMU_PD_MODE_OFF);
  Driver_PMU.SetPowerDomainStateLowPowerMode(
    PMU_PD_FLASH, PMU_PD_MODE_OFF, PMU_PD_MODE_OFF, PMU_PD_MODE_OFF);

  Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_PLL);

  Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 12); /* GPIO */
  Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 0);
  Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 0);

  Driver_PMU.SelectClockSource(PMU_CSM_CPUST, PMU_CLOCK_SOURCE_SIOSC4M);
  //Driver_PMU.SetPrescaler(PMU_CD_CPUST, 8); /* SYS TICK */
  Driver_PMU.SetCpuLowFrequencyOnSleep(true);

  Driver_PMU.SetPrescaler(PMU_CD_UART0, 0);
  Driver_PMU.SetPrescaler(PMU_CD_UART1, 0);
  Driver_PMU.SetPrescaler(PMU_CD_UART2, 0);
  Driver_PMU.SetPrescaler(PMU_CD_SPIC, 0);
  Driver_PMU.SetPrescaler(PMU_CD_USBI, 0);

  /* Setup GPIO */
  Driver_GPIO.Initialize();
  Driver_GPIO.PowerControl(ARM_POWER_FULL);

  /* Setup RTC */
  Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC32K);
  Driver_PMU.SelectClockSource(PMU_CSM_RTC, PMU_CLOCK_SOURCE_OSC32K);
#if defined(TWIC_PMU__CMSIS_RTC)
  Driver_RTC.Initialize();
#endif

  /* release retention state of output buffer */
  Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PM, 0);
  Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PP1, 0);

  return TZ1SM_HAL_STATUS_OK;
}

/**
 * @brief Setup Frequency
 * @parama[in] pcd Power Cotrol Domain
 * @return \ref TZ1SM_HAL_STATUS
 */
tz1smHalStatus_t tz1smStartClock(const tz1smHalPcd_t pcd)
{
	if (TZ1SM_HAL_PCD_PP1 & pcd) {
		Driver_PMU.SetPrescaler(PMU_CD_PPIER1,
						Driver_PMU.GetPrescaler(PMU_CD_PPIER0));
	}

	if (TZ1SM_HAL_PCD_UART0 & pcd) {
		if (PMU_VOLTAGE_MODE_D == Driver_PMU.GetVoltageMode()) {
			Driver_PMU.SelectClockSource(PMU_CSM_UART0, PMU_CLOCK_SOURCE_SIOSC4M);
			Driver_PMU.SetPrescaler(PMU_CD_UART0, 1);
		} else {
			Driver_PMU.SetPrescaler(PMU_CD_UART0, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART0, PMU_CLOCK_SOURCE_OSC12M);
		}
	}

	if (TZ1SM_HAL_PCD_UART1 & pcd)  {
		if (PMU_VOLTAGE_MODE_D == Driver_PMU.GetVoltageMode()) {
			Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_SIOSC4M);
		    Driver_PMU.SetPrescaler(PMU_CD_UART1, 1);
		} else {
		    Driver_PMU.SetPrescaler(PMU_CD_UART1, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_OSC12M);
		}
	}

	if (TZ1SM_HAL_PCD_UART2 & pcd) {
		if (PMU_VOLTAGE_MODE_D == Driver_PMU.GetVoltageMode()) {
			Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_SIOSC4M);
			Driver_PMU.SetPrescaler(PMU_CD_UART2, 1);
		} else {
			Driver_PMU.SetPrescaler(PMU_CD_UART2, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_OSC12M);
		}
		Driver_PMU.SetPrescaler(PMU_CD_PPIER2,
								Driver_PMU.GetPrescaler(PMU_CD_PPIER0));
	}

	if (TZ1SM_HAL_PCD_ADC12 & pcd) {
		Driver_PMU.SelectClockSource(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_SIOSC4M);
		Driver_PMU.SetPrescaler(PMU_CD_ADCC12, 1);
	}

	if (TZ1SM_HAL_PCD_ADC24 & pcd) {
		Driver_PMU.SelectClockSource(PMU_CSM_ADCC24, PMU_CLOCK_SOURCE_SIOSC4M);
		Driver_PMU.SetPrescaler(PMU_CD_ADCC24, 1);
	}

	if (TZ1SM_HAL_PCD_USB & pcd) {
		Driver_PMU.SelectClockSource(PMU_CSM_USB,  PMU_CLOCK_SOURCE_PLL);
		Driver_PMU.SetPrescaler(PMU_CD_USBI, 1);
		Driver_PMU.SetPrescaler(PMU_CD_USBB, 2);
	}

	if (TZ1SM_HAL_PCD_FLASH & pcd) {
		Driver_PMU.SetPowerDomainState(PMU_PD_FLASH, PMU_PD_MODE_ON);
		Driver_PMU.SetPrescaler(PMU_CD_SPIC, 1);
	}

	if (TZ1SM_HAL_PCD_DMAC & pcd) {
		Driver_PMU.SetPowerDomainState(PMU_PD_DMAC, PMU_PD_MODE_ON);
	}

	if (TZ1SM_HAL_PCD_ENC & pcd) {
		Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
	}
#if RTE_SPI0
	if (TZ1SM_HAL_PCD_SPI0 & pcd) {
		Driver_SPI0.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_SPI1
	if (TZ1SM_HAL_PCD_SPI1 & pcd) {
		Driver_SPI1.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_SPI2
	if (TZ1SM_HAL_PCD_SPI2 & pcd) {
		Driver_SPI2.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_SPI3
	if (TZ1SM_HAL_PCD_SPI3 & pcd) {
		Driver_SPI3.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_I2C0
	if (TZ1SM_HAL_PCD_I2C0 & pcd) {
		Driver_I2C0.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_I2C1
	if (TZ1SM_HAL_PCD_I2C1 & pcd) {
		Driver_I2C1.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_I2C2
	if (TZ1SM_HAL_PCD_I2C2 & pcd) {
		Driver_I2C2.PowerControl(ARM_POWER_FULL);
	}
#endif
#if RTE_WDT
	if (TZ1SM_HAL_PCD_WDT & pcd) {
		Driver_WDT.PowerControl(ARM_POWER_FULL);
	}
#endif
	
	return TZ1SM_HAL_STATUS_OK;
}

/**
 * @brief Stop Frequency
 * @parama[in] pcd Power Cotrol Domain
 * @return \ref TZ1SM_HAL_STATUS
 */
tz1smHalStatus_t tz1smStopClock(const tz1smHalPcd_t pcd)
{
	uint32_t uc = 0;

	return tz1smHalPmuLowPowerPcd(
		pcd, &uc, TZ1SM_HAL_OM_STOP, TZ1SM_HAL_VF_04M09, NULL);
}

tz1smHalVf_t tz1smHalGetCurrentVf(void)
{
	return (tz1smHalVf_t)Driver_PMU.GetVoltageMode();
}

tz1smHalStatus_t tz1smHalSetVf(const tz1smHalVf_t vf)
{
	PMU_VOLTAGE_MODE new_mode = (PMU_VOLTAGE_MODE)vf;
	PMU_VOLTAGE_MODE old_mode = (PMU_VOLTAGE_MODE)tz1smHalGetCurrentVf();

	if (new_mode != old_mode) {
#if defined(TZ1EM_ENABLE_LOG)
		print_log("[[LOG_TZ1EM] Change VoltageMode : ");
		print_log(voltage_mode_string[vf]);
		print_log("]\r\n");
#endif
	
		Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_SIOSC4M);
		Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 1);
		if (Driver_PMU.GetPrescaler(PMU_CD_PPIER1) != 0) {
			Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 1);
		}
		if (Driver_PMU.GetPrescaler(PMU_CD_PPIER2) != 0) {
			Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 1);
		}
		
		if (PMU_VOLTAGE_MODE_A == old_mode || PMU_VOLTAGE_MODE_B == old_mode) {
			Driver_PMU.StopClockSource(PMU_CLOCK_SOURCE_PLL);
		}
		
		if (PMU_VOLTAGE_MODE_D != old_mode && PMU_VOLTAGE_MODE_D == new_mode) {
			if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART0)) {
				Driver_PMU.SelectClockSource(PMU_CSM_UART0, PMU_CLOCK_SOURCE_SIOSC4M);
				Driver_PMU.SetPrescaler(PMU_CD_UART0, 1);
			}
			if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART1)) {
				Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_SIOSC4M);
				Driver_PMU.SetPrescaler(PMU_CD_UART1, 1);
			}
			if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART2)) {
				Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_SIOSC4M);
				Driver_PMU.SetPrescaler(PMU_CD_UART2, 1);
			}
			Driver_PMU.StopClockSource(PMU_CLOCK_SOURCE_OSC12M);
		}
		
		Driver_PMU.SetVoltageMode(new_mode);

		if (PMU_VOLTAGE_MODE_A == new_mode) {
			Driver_PMU.SetPLLFrequency(48000000); /* 48MHz */
		} else if (PMU_VOLTAGE_MODE_B == new_mode) {
			Driver_PMU.SetPLLFrequency(36000000); /* 36MHz */
		}

		if (PMU_VOLTAGE_MODE_D != new_mode) {
			if (PMU_VOLTAGE_MODE_D == old_mode) {
				Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC12M);
				while (Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_OSC12M) !=
					 PMU_CLOCK_SOURCE_STATE_RUNNING) {
					/* DO NOTHING */
				}
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART0)) {
					Driver_PMU.SetPrescaler(PMU_CD_UART0, 3);
					Driver_PMU.SelectClockSource(PMU_CSM_UART0, PMU_CLOCK_SOURCE_OSC12M);
				}
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART1)) {
					Driver_PMU.SetPrescaler(PMU_CD_UART1, 3);
					Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_OSC12M);
				}
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_UART2)) {
					Driver_PMU.SetPrescaler(PMU_CD_UART2, 3);
					Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_OSC12M);
				}
			}
			if (PMU_VOLTAGE_MODE_A == new_mode || PMU_VOLTAGE_MODE_B == new_mode) {
				Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_PLL);
				while (PMU_CLOCK_SOURCE_STATE_RUNNING !=
							 Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_PLL)) {
				}
				if (PMU_VOLTAGE_MODE_A == new_mode) {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 12);
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 12);
					}
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 12);
					}
				} else {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 9);
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 9);
					}
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 9);
					}
				}
                
				Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_PLL);
			} else {
				/* PMU_VOLTAGE_MODE_C */
				Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 3);
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 3);
				}
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 3);
				}
				Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_OSC12M);
			}
		}
	}
	return TZ1SM_HAL_STATUS_OK;
}

/**
 * @brief Set PCD Low Power
 * @parama[in] pcd Power Cotrol Domain
 * @parama[in] uc PCD use counter
 * @return \ref TZ1SM_HAL_STATUS
 *
 * 
 */
tz1smHalStatus_t tz1smHalPmuLowPowerPcd(
    const tz1smHalPcd_t pcd, uint32_t * const uc, tz1smHalOm_t om,
    tz1smHalVf_t vf, tz1smHalWakeupCb_t cb)
{
    uint32_t previous_wakeup_factor;
    
    if (TZ1SM_HAL_OM_SLEEP1 > om);
    else if ((TZ1SM_HAL_PCD_SRAM1 & pcd) && !(TZ1SM_HAL_PCD_SRAM1 & *uc)) {
        Driver_PMU.SetPowerDomainState(PMU_PD_SRAM1, PMU_PD_MODE_OFF);
        *uc |= TZ1SM_HAL_PCD_SRAM1;
    }
    if (TZ1SM_HAL_OM_SLEEP0 > om);
    else if ((TZ1SM_HAL_PCD_FLASH & pcd) && !(TZ1SM_HAL_PCD_FLASH & *uc)) {
        Driver_NOR0.PowerControl(ARM_POWER_OFF);
		Driver_PMU.SetPrescaler(PMU_CD_SPIC, 0);
        Driver_PMU.SetPowerDomainState(PMU_PD_FLASH, PMU_PD_MODE_OFF);
        Driver_PMU.SetPowerDomainStateLowPowerMode(PMU_PD_FLASH, PMU_PD_MODE_OFF, PMU_PD_MODE_OFF, PMU_PD_MODE_OFF);
        *uc |= TZ1SM_HAL_PCD_FLASH;
    }
#if RTE_UART0 && defined(SOFTWARE_GATING)
    if (TZ1SM_HAL_OM_SLEEP2 > om);
    else if ((TZ1SM_HAL_PCD_UART0 & pcd) && !(TZ1SM_HAL_PCD_UART0 & *uc)) {
        Driver_UART0.PowerControl(ARM_POWER_LOW);
        *uc |= TZ1SM_HAL_PCD_UART0;
    }
#endif
#if RTE_UART1 && defined(SOFTWARE_GATING)
    if (TZ1SM_HAL_OM_SLEEP2 > om);
    else if ((TZ1SM_HAL_PCD_UART1 & pcd) && !(TZ1SM_HAL_PCD_UART1 & *uc)) {
        Driver_UART1.PowerControl(ARM_POWER_LOW);
        *uc |= TZ1SM_HAL_PCD_UART1;
    }
#endif
#if RTE_UART2
    if (TZ1SM_HAL_OM_SLEEP2 > om);
    else if ((TZ1SM_HAL_PCD_UART2 & pcd) && !(TZ1SM_HAL_PCD_UART2 & *uc)) {
        Driver_UART2.PowerControl(ARM_POWER_LOW);
        *uc |= TZ1SM_HAL_PCD_UART2;
    }
#endif
    
#if RTE_ADCC12 && defined(SOFTWARE_GATING)
    if (TZ1SM_HAL_OM_SLEEP2 > om);
    else if ((TZ1SM_HAL_PCD_ADC12 & pcd) && !(TZ1SM_HAL_PCD_ADC12 & *uc)) {
        Driver_ADCC12.PowerControl(ARM_POWER_OFF);
        Driver_PMU.SetPowerDomainState(PMU_PD_ADCC12, PMU_PD_MODE_OFF);
        *uc |= TZ1SM_HAL_PCD_ADC12;
    }
#endif
#if RTE_ADCC24 && defined(SOFTWARE_GATING)
    if (TZ1SM_HAL_OM_SLEEP2 > om);
    else if ((TZ1SM_HAL_PCD_ADC24 & pcd) && !(TZ1SM_HAL_PCD_ADC24 & *uc)) {
        Driver_ADCC24.PowerControl(ARM_POWER_OFF);
    Driver_PMU.SetPowerDomainState(PMU_PD_ADCC24, PMU_PD_MODE_OFF);
    *uc |= TZ1SM_HAL_PCD_ADC24;
  }
#endif
#if RTE_SDMAC && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP1 > om);
  else if ((TZ1SM_HAL_PCD_DMAC & pcd) && !(TZ1SM_HAL_PCD_DMAC & *uc)) {
    Driver_SDMAC.PowerControl(ARM_POWER_OFF);
    Driver_PMU.SetPowerDomainState(PMU_PD_DMAC, PMU_PD_MODE_OFF);
    *uc |= TZ1SM_HAL_PCD_DMAC;
  }
#endif
#if RTE_USB2FS && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP0 > om && TZ1SM_HAL_VF_48M12 == vf);
  else if ((TZ1SM_HAL_PCD_USB & pcd) && !(TZ1SM_HAL_PCD_USB & *uc)) {
    Driver_USBD0.PowerControl(ARM_POWER_OFF);
    Driver_PMU.SetPrescaler(PMU_CD_USBI, 0);
    Driver_PMU.SetPrescaler(PMU_CD_USBB, 0);
    *uc |= TZ1SM_HAL_PCD_USB;
  }
#endif
#if RTE_AESA && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP0 > om);
  if ((TZ1SM_HAL_PCD_ENC & pcd) && !(TZ1SM_HAL_PCD_ENC & *uc)) {
    Driver_AESA.PowerControl(ARM_POWER_OFF);
    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_OFF);
    *uc |= TZ1SM_HAL_PCD_ENC;
  }
#endif
#if RTE_SPI0 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_SPI0 & pcd) && !(TZ1SM_HAL_PCD_ENC & *uc)) {
    Driver_SPI0.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_ENC;
  }
#endif
#if RTE_SPI1 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_SPI1 & pcd) && !(TZ1SM_HAL_PCD_SPI1 & *uc)) {
    Driver_SPI1.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_SPI1;
  }
#endif
#if RTE_SPI2 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_SPI2 & pcd) && !(TZ1SM_HAL_PCD_SPI2 & *uc)) {
    Driver_SPI2.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_SPI2;
  }
#endif
#if RTE_SPI3 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_SPI3 & pcd) && !(TZ1SM_HAL_PCD_SPI3 & *uc)) {
    Driver_SPI3.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_SPI3;
  }
#endif
#if RTE_I2C0 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_I2C0 & pcd) && !(TZ1SM_HAL_PCD_I2C0 & *uc)) {
    Driver_I2C0.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_I2C0;
  }
#endif
#if RTE_I2C1 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_I2C1 & pcd) && !(TZ1SM_HAL_PCD_I2C1 & *uc)) {
    Driver_I2C1.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_I2C1;
  }
#endif
#if RTE_I2C2 && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  else if ((TZ1SM_HAL_PCD_I2C2 & pcd) && !(TZ1SM_HAL_PCD_I2C2 & *uc)) {
    Driver_I2C2.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_I2C2;
  }
#endif
#if RTE_WDT && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_OM_SLEEP2 > om);
  if ((TZ1SM_HAL_PCD_WDT & pcd) && !(TZ1SM_HAL_PCD_WDT & *uc)) {
    Driver_WDT.PowerControl(ARM_POWER_LOW);
    *uc |= TZ1SM_HAL_PCD_WDT;
  }
#endif
/* PMU_CD_MPIER  :CPU,SRAM and SPI
 * PMU_CD_PPIER0 :GPIO,SPIM2/3
 * PMU_CD_PPIER1 :SPIM0/1,I2C0/1, UART0/1 */
  if (!((TZ1SM_HAL_PCD_SPI0 & *uc) && (TZ1SM_HAL_PCD_SPI1 & *uc) &&
        (TZ1SM_HAL_PCD_I2C0 & *uc) && (TZ1SM_HAL_PCD_I2C1 & *uc) &&
        (TZ1SM_HAL_PCD_UART0 & *uc) && (TZ1SM_HAL_PCD_UART1 & *uc)));
  else if ((TZ1SM_HAL_PCD_PP1 & pcd) && !(TZ1SM_HAL_PCD_PP1 & *uc)) {
    Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 0);
    *uc |= TZ1SM_HAL_PCD_PP1;
  }

  if (cb != NULL) {
		previous_wakeup_factor = Driver_PMU.GetWakeupFactor();
    cb(pcd, om, vf, previous_wakeup_factor);
  }

  return TZ1SM_HAL_STATUS_OK;
}

/**
 * @brief Set PCD High Power
 * @parama[in] pcd Power Cotrol Domain
 * @parama[in] uc PCD use counter
 * @return \ref TZ1SM_HAL_STATUS
 */
tz1smHalStatus_t tz1smHalPmuHighPowerPcd(
  const tz1smHalPcd_t pcd, uint32_t * const uc, tz1smHalOm_t om,
  tz1smHalVf_t vf, tz1smHalWakeupCb_t cb)
{
	uint32_t wakeup_factor;
	
  if ((TZ1SM_HAL_PCD_PP1 & pcd) && (TZ1SM_HAL_PCD_PP1 & *uc)) {
    Driver_PMU.SetPrescaler(
      PMU_CD_PPIER1, Driver_PMU.GetPrescaler(PMU_CD_PPIER0));
    *uc &= ~TZ1SM_HAL_PCD_PP1;
  }
  if ((TZ1SM_HAL_PCD_SRAM1 & pcd) && (TZ1SM_HAL_PCD_SRAM1 & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_SRAM1, PMU_PD_MODE_ON);
    *uc &= ~TZ1SM_HAL_PCD_SRAM1;
  }
  if ((TZ1SM_HAL_PCD_FLASH & pcd) && (TZ1SM_HAL_PCD_FLASH & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_FLASH, PMU_PD_MODE_ON);
    Driver_PMU.SetPrescaler(PMU_CD_SPIC, 1);
    *uc &= ~TZ1SM_HAL_PCD_FLASH;
  }
#if RTE_UART0 && defined(SOFTWARE_GATING)
	if ((TZ1SM_HAL_PCD_UART0 & pcd) && (TZ1SM_HAL_PCD_UART0 & *uc)) {
		Driver_UART0.PowerControl(ARM_POWER_FULL);
		*uc &= ~TZ1SM_HAL_PCD_UART0;
	}
#endif
#if RTE_UART1 && defined(SOFTWARE_GATING)
	if ((TZ1SM_HAL_PCD_UART1 & pcd) && (TZ1SM_HAL_PCD_UART1 & *uc)) {
		Driver_UART1.PowerControl(ARM_POWER_FULL);
		*uc &= ~TZ1SM_HAL_PCD_UART1;
	}
#endif
#if RTE_UART2
	if ((TZ1SM_HAL_PCD_UART2 & pcd) && (TZ1SM_HAL_PCD_UART2 & *uc)) {
		Driver_UART2.PowerControl(ARM_POWER_FULL);
		*uc &= ~TZ1SM_HAL_PCD_UART2;
	}
#endif
#if RTE_ADCC12 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_ADC12 & pcd) && (TZ1SM_HAL_PCD_ADC12 & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_ADCC12, PMU_PD_MODE_ON);
    *uc &= ~TZ1SM_HAL_PCD_ADC12;
  }
#endif
#if RTE_ADCC24 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_ADC24 & pcd) && (TZ1SM_HAL_PCD_ADC24 & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_ADCC24, PMU_PD_MODE_ON);
    *uc &= ~TZ1SM_HAL_PCD_ADC24;
  }
#endif
#if RTE_SDMAC && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_DMAC & pcd) && (TZ1SM_HAL_PCD_DMAC & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_DMAC, PMU_PD_MODE_ON);
    *uc &= ~TZ1SM_HAL_PCD_DMAC;
  }
#endif
#if RTE_USB2FS && defined(SOFTWARE_GATING)
  if (TZ1SM_HAL_VF_48M12 != vf);
  else if ((TZ1SM_HAL_PCD_USB & pcd) && (TZ1SM_HAL_PCD_USB & *uc)) {
    Driver_PMU.SetPrescaler(PMU_CD_USBI, 1);
    Driver_PMU.SetPrescaler(PMU_CD_USBB, 2);
    *uc &= ~TZ1SM_HAL_PCD_USB;
  }
#endif
#if RTE_AESA && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_ENC & pcd) && (TZ1SM_HAL_PCD_ENC & *uc)) {
    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
    *uc &= ~TZ1SM_HAL_PCD_ENC;
  }
#endif
#if RTE_SPI0 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_SPI0 & pcd) && (TZ1SM_HAL_PCD_SPI0 & *uc)) {
    Driver_SPI0.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_SPI0;
  }
#endif
#if RTE_SPI1 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_SPI1 & pcd) && (TZ1SM_HAL_PCD_SPI1 & *uc)) {
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_SPI1;
  }
#endif
#if RTE_SPI2 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_SPI2 & pcd) && (TZ1SM_HAL_PCD_SPI2 & *uc)) {
    Driver_SPI2.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_SPI2;
  }
#endif
#if RTE_SPI3 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_SPI3 & pcd) && (TZ1SM_HAL_PCD_SPI3 & *uc)) {
    Driver_SPI3.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_SPI3;
  }
#endif
#if RTE_I2C0 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_I2C0 & pcd) && (TZ1SM_HAL_PCD_I2C0 & *uc)) {
    Driver_I2C0.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_I2C0;
  }
#endif
#if RTE_I2C1 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_I2C1 & pcd) && (TZ1SM_HAL_PCD_I2C1 & *uc)) {
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_I2C1;
  }
#endif
#if RTE_I2C2 && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_I2C2 & pcd) && (TZ1SM_HAL_PCD_I2C2 & *uc)) {
    Driver_I2C2.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_I2C2;
  }
#endif
#if RTE_WDT && defined(SOFTWARE_GATING)
  if ((TZ1SM_HAL_PCD_WDT & pcd) && (TZ1SM_HAL_PCD_WDT & *uc)) {
    Driver_WDT.PowerControl(ARM_POWER_FULL);
    *uc &= ~TZ1SM_HAL_PCD_WDT;
  }
#endif

  if (cb != NULL) {
		wakeup_factor = Driver_PMU.GetWakeupFactor();
    cb(pcd, om, vf, wakeup_factor);
  }

  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t
tz1smHalConfigureWakeup(const uint32_t factor, const tz1smHalWe_t event)
{
  PMU_STATUS pmu_status;
  
  if (TZ1SM_HAL_ACT_FACTOR_NONE == factor)
    return TZ1SM_HAL_STATUS_OK;
  
  pmu_status = Driver_PMU.ConfigureWakeup(factor, (PMU_WAKEUP_EVENT)event);
  if (PMU_OK != pmu_status)
    return TZ1SM_HAL_STATUS_ERROR_DRIVER;

  return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t tz1smHalEnableWakeup(const uint32_t factor, const bool enable)
{
	PMU_STATUS pmu_status;
	
	if (TZ1SM_HAL_ACT_FACTOR_NONE == factor)
		return TZ1SM_HAL_STATUS_OK;

	pmulv->IRQ_STATUS = factor;

	pmu_status = Driver_PMU.EnableWakeup(factor, enable);
	if (PMU_OK != pmu_status)
		return TZ1SM_HAL_STATUS_ERROR_DRIVER;

	return TZ1SM_HAL_STATUS_OK;
}

tz1smHalStatus_t tz1smHalOperationMode(tz1smHalOm_t mode)
{
	PMU_STATUS pmu_status;
	PMU_VOLTAGE_MODE voltage_mode;
	PMU_POWER_MODE power_mode;

	if (TZ1SM_HAL_OM_ACTIVE == mode || TZ1SM_HAL_OM_NONE == mode)
		return TZ1SM_HAL_STATUS_OK;

	if ((0x80 & mode) == 0x80) {
		mode &= 0x7f;
		if (TZ1SM_HAL_OM_SLEEP2 == mode) {
			power_mode = PMU_POWER_MODE_SLEEPDEEP2;
		} else if (TZ1SM_HAL_OM_SLEEP1 == mode) {
			power_mode = PMU_POWER_MODE_SLEEPDEEP1;
		} else if (TZ1SM_HAL_OM_SLEEP0 == mode) {
			power_mode = PMU_POWER_MODE_SLEEPDEEP0;
		} else {
			power_mode = (PMU_POWER_MODE)mode;
		}
	} else {
		power_mode = (PMU_POWER_MODE)mode;
	}
#if defined(TZ1EM_ENABLE_LOG)
	print_log("[[LOG_TZ1EM] Change PowerMode : ");
	print_log(power_mode_string[power_mode]);
	print_log("]\r\n");
#endif

	pmu_status = Driver_PMU.SetPowerMode(power_mode);

	/* release retention state of output buffer */
	/* RetainOutputBuffer */
	Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PM, 0);
	Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PP1, 0);

	if (PMU_ERROR_IGNORED == pmu_status) {
		return TZ1SM_HAL_STATUS_IGNORE;
	} else if (PMU_OK != pmu_status) {
		return TZ1SM_HAL_STATUS_ERROR_DRIVER;
	}

	voltage_mode = Driver_PMU.GetVoltageMode();
	if (TZ1SM_HAL_OM_SLEEP2 < mode) {
		if (voltage_mode != PMU_VOLTAGE_MODE_D) {
			Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC12M);
			while (Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_OSC12M)
				!= PMU_CLOCK_SOURCE_STATE_RUNNING) {
				/* DO NOTHING */
			}

			if (PMU_VOLTAGE_MODE_A == voltage_mode) {
				Driver_PMU.SetPLLFrequency(48000000); /* 48MHz */
			} else if (PMU_VOLTAGE_MODE_B == voltage_mode) {
				Driver_PMU.SetPLLFrequency(36000000); /* 36MHz */
			}

			Driver_PMU.SetPrescaler(PMU_CD_UART0, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART0, PMU_CLOCK_SOURCE_OSC12M);
			
			Driver_PMU.SetPrescaler(PMU_CD_UART1, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_OSC12M);
			
			Driver_PMU.SetPrescaler(PMU_CD_UART2, 3);
			Driver_PMU.SelectClockSource(PMU_CSM_UART2, PMU_CLOCK_SOURCE_OSC12M);

			if (PMU_VOLTAGE_MODE_A == voltage_mode ||
				PMU_VOLTAGE_MODE_B == voltage_mode) {
				/* PMU_VOLTAGE_MODE_A or PMU_VOLTAGE_MODE_B */
				Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_PLL);
				while (Driver_PMU.GetClockSourceState(PMU_CLOCK_SOURCE_PLL)
					!= PMU_CLOCK_SOURCE_STATE_RUNNING) {
					/* DO NOTHING */
				}
				if (PMU_VOLTAGE_MODE_A == voltage_mode) {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 12);
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 12);
					}
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 12);
					}
				} else {
					Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 9);
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 9);
					}
					if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
						Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 9);
					}
				}
              Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_PLL);
			} else {
				/* PMU_VOLTAGE_MODE_C */
				Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 3);
				if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER1)) {
                    Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 3);
                }
                if (0 != Driver_PMU.GetPrescaler(PMU_CD_PPIER2)) {
                    Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 3);
                }
				Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_OSC12M);
			}
		}
	}
	return TZ1SM_HAL_STATUS_OK;
}

#else

#warning V0.0.1 is no supporting the other PMU Drivers other than CMSIS_PMU.

tz1smHalStatus_t tz1smHalLePmuStart(void) { return TZ1SM_HAL_STATUS_OK; }
void tz1smHalLePmuUnStop(void) { return; }
tz1smHalStatus_t tz1smHalLePmuStatus(void)
{ return TZ1SM_HAL_STATUS_SOURCE_RUNNING; }

uint8_t tz1smHalPmuButtonFinalize(void) { return 0; }
uint8_t tz1smHalPmuLedInit(void) { return 0; }

uint8_t tz1smHalPmuLedFinalize(void) { return 0; }

tz1smHalStatus_t tz1smHalLePmuClockSourceState32K(void) { return 0; }

tz1smHalStatus_t tz1smHalLePmuStop(const bool stop_osc32K) { return 0; }

tz1smHalStatus_t tz1smHalInitializePmuSystem(void)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalSetVf(const tz1smHalVf_t vf)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalVf_t tz1smHalGetCurrentVf(void) { return TZ1SM_HAL_VF_NONE; };
tz1smHalStatus_t tz1smHalPmuLowPowePcd(const tz1smHalPcd_t pcd)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalPmuHighPowePcd(const tz1smHalPcd_t pcd)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t
tz1smHalConfigureWakeup(const uint32_t factor, const tz1smHalWe_t event)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalEnableWakeup(const uint32_t factor, const bool enable)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t  tz1smHalOperationMode(tz1smHalOm_t mode)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smStartClock(const tz1smHalPcd_t pcd)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smStopClock(const tz1smHalPcd_t pcd)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalPmuHighPowerPcd(
  const tz1smHalPcd_t pcd, uint32_t * const uc, tz1smHalOm_t om,
  tz1smHalVf_t vf, tz1smHalWakeupCb_t cb)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalPmuLowPowerPcd(
  const tz1smHalPcd_t pcd, uint32_t * const uc, tz1smHalOm_t om,
  tz1smHalVf_t vf, tz1smHalWakeupCb_t cb)
{ return TZ1SM_HAL_STATUS_OK; }
tz1smHalStatus_t tz1smHalInitializePcdSystem(void)
{ return TZ1SM_HAL_STATUS_OK; }

#endif


/*
 * EVENT CALLBACK
 *
 */

#if defined(TWIC_EVENT_CALLBACK)

static tz1smHalCb_t hal_cb = {NULL};

/*
 * @brief
 * regiser event call back functions.
 */
void tz1smHalRegisterCb(const tz1smHalCb_t * const cb)
{
  hal_cb.intr_ble_uart_rx = cb->intr_ble_uart_rx;
  hal_cb.intr_ble_host_wakeup = cb->intr_ble_host_wakeup;
  hal_cb.intr_ble_status_changed = cb->intr_ble_status_changed;

  return;
}

#else

void tz1smHalRegisterCb(const tz1smHalCb_t * const cb)
{
  return;
}

#endif

/*
 * GPIO
 *
 */

#if defined(TWIC_GPIO__CMSIS_GPIO)

#if defined(TWIC_DEBUG_LOG_TRACE)
/* debug */
#define TWIC_GPIO_INFO(...)
#define TWIC_GPIO_ERROR(...) printf(__VA_ARGS__);
#else
#define TWIC_GPIO_INFO(...)
#define TWIC_GPIO_ERROR(...)
#endif

/* In performing hardware control, it surely uses CMSIS GPIO API in
 * the following functions. Coding of the following functions is
 * carried out using CMSIS API. */

tz1sm_gpio_cts_configuration_t tz1sm_gpio_cts_setup;
static GPIO_STATUS tz1smHalGpioConfigure(uint32_t pin, GPIO_DIRECTION dir,
                                         GPIO_EVENT event,
                                         GPIO_SignalEvent_t cb_event)
{
  if (TZ1SM_HAL_GPIO_BLE_CTS == pin) {
    tz1sm_gpio_cts_setup.inheritance = true;
    tz1sm_gpio_cts_setup.dir = dir;
    tz1sm_gpio_cts_setup.event = event;
    tz1sm_gpio_cts_setup.cb_event = cb_event;
  }
  return Driver_GPIO.Configure(pin, dir, event, cb_event);
}

/*
 * @brief
 * 1.Initialize GPIO
 * 2.Power Control FULL.
 * 3.Configure GPIO
 *   BLE_GPIO0	MCU_GPIO31	Request Wake Up. (Dir:Output 2mA)
 *   BLE_GPIO1	MCU_GPIO30	Host Wake Up. (Dir:Input Pull-Up)
 *   BLE_GPIO2	MCU_GPIO29	BLE Status(Dir:Input Pull-Up)
 *   BLE_RESETX	MCU_GPIO28	BLE Reset. (Dir:Output 2mA)
 *   BLE_DCDCEN TWIC_LECE_DCDCEN_GPIO_NUMBER BLE DCDC Enable.
 */
void tz1smHalGpioInit(void)
{
  GPIO_STATUS sts = Driver_GPIO.PowerControl(ARM_POWER_FULL);

  if (GPIO_OK != sts)
    TWIC_GPIO_ERROR("PowerControl(ARM_POWER_FULL)=%d\r\n", sts);
  /* Reset Pin Configuration */
  sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_RESETX,
                              GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE,
                              NULL);
  if (GPIO_OK != sts) {
    TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_RESETX) = %d\r\n", sts);
  }

  tz1smHalGpioBleReset(true);

#if defined(TWIC_LECE_DCDCEN)
  sts = tz1smHalGpioConfigure(TWIC_LECE_DCDCEN_GPIO_NUMBER,
                              GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE,
                              NULL);
  if (GPIO_OK != sts) {
    TWIC_GPIO_ERROR("Conf(TWIC_LECE_DCDCEN_GPIO_NUMBER)=%d\r\n", sts);
  }
  tz1smHalGpioWrite(TWIC_LECE_DCDCEN_GPIO_NUMBER, true);
#endif
  
  return;
}

void tz1smHalGpioUnInit(void)
{
  tz1smHalGpioBleReset(true);

#if defined(TWIC_LECE_DCDCEN)
  tz1smHalGpioWrite(TWIC_LECE_DCDCEN_GPIO_NUMBER, false);
#endif

  return;
}

/*
 * @brief
 * 1. Read GPIO Pin Value.
 */
tz1smHalStatus_t tz1smHalGpioRead(const uint32_t pnr, bool * const status)
{
  GPIO_STATUS sts;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;
  uint32_t val;

  sts = Driver_GPIO.ReadPin(pnr, &val);
  if (GPIO_OK != sts) {
    TWIC_GPIO_ERROR("ReadPin()=%d\r\n", sts);
    ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
    goto END;
  }
  *status = (1 == val) ? true : false;

END:
  return ret_sts;
}

/*
 * @brief
 * 1. Write GPIO Pin.
 */
tz1smHalStatus_t tz1smHalGpioWrite(const uint32_t pnr, bool const level)
{
  GPIO_STATUS sts;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;

  sts = Driver_GPIO.WritePin(pnr, (true == level) ? 1 : 0);
  if (GPIO_OK != sts) {
    TWIC_GPIO_ERROR("WritePin()=%d\r\n", sts);
    ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
  }

  return ret_sts;
}

void tz1smHalIsrBleHostWakeup(uint32_t pin)
{
#if defined(TWIC_RTOS_TOOL_X1)
  portENTER_CRITICAL();
  vTracePrintF(xTraceOpenLabel("HOST_WAKE_UP"), "%d",pin);
  portEXIT_CRITICAL();
#endif
  hal_cb.intr_ble_host_wakeup();
}

void tz1smHalIsrBleStatus(uint32_t pin)
{
#if defined(TWIC_RTOS_TOOL_X1)
  portENTER_CRITICAL();
  vTracePrintF(xTraceOpenLabel("STATUS"), "%d",pin);
  portEXIT_CRITICAL();
#endif
  hal_cb.intr_ble_status_changed();
}

/*
 * @brief
 * 1. Write GPIO BLE Reset Pin(high:reset, low:no reset)
 */
tz1smHalStatus_t tz1smHalGpioBleReset(const bool reset)
{
  GPIO_STATUS sts;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;

  if (false == reset) {
    /* no reset, configuration GPIO for Power On */
#if defined(TZ1SM_HAL_GPIO_BLE_STATUS)
#if defined(TWIC_LECE_ISR_LOWPOWER_STATUS_EVENT)
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_STATUS,
                                GPIO_DIRECTION_INPUT_PULL_UP,
                                GPIO_EVENT_EDGE_NEG,
                                tz1smHalIsrBleStatus);
#else
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_STATUS,
                                GPIO_DIRECTION_INPUT_PULL_UP,
                                GPIO_EVENT_DISABLE,
                                NULL);
#endif
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_STATUS) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
#endif
#if defined(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP)
#if defined(TWIC_LECE_ISR_LOWPOWER_WAKEUP)
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP,
                                GPIO_DIRECTION_INPUT_PULL_UP,
                                GPIO_EVENT_EDGE_BOTH,
                                tz1smHalIsrBleHostWakeup);
#else
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP,
                                GPIO_DIRECTION_INPUT_PULL_UP,
                                GPIO_EVENT_DISABLE,
                                NULL);
#endif
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
#endif
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP,
                                GPIO_DIRECTION_OUTPUT_2MA,
                                GPIO_EVENT_DISABLE, NULL);
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
    tz1smHalSuppressHpd(true);
  }
  
  ret_sts = tz1smHalGpioWrite(TZ1SM_HAL_GPIO_BLE_RESETX,
                              (true == reset) ? false : true);
  if (ret_sts != TZ1SM_HAL_STATUS_OK) {
    ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
    goto END;
  }

  if (true == reset) {
    /* reset, configuration GPIO for Power Off */
#if defined(TZ1SM_HAL_GPIO_BLE_STATUS)
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_STATUS,
                                GPIO_DIRECTION_INPUT_HI_Z,
                                GPIO_EVENT_DISABLE, NULL);
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_STATUS) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
#endif    
#if defined(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP)
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP,
                                GPIO_DIRECTION_INPUT_HI_Z,
                                GPIO_EVENT_DISABLE, NULL); 
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
#endif    
    sts = tz1smHalGpioConfigure(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP,
                                GPIO_DIRECTION_INPUT_HI_Z,
                                GPIO_EVENT_DISABLE, NULL);
    if (GPIO_OK != sts) {
      TWIC_GPIO_ERROR("Conf(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP) = %d\r\n", sts);
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    }
    tz1smHalSuppressHpd(false);
  }

END:
  
  return ret_sts;
}


/*
 * @brief
 * Write GPIO BLE Request Wakeup Pin(true:ble active mode,
 * false:permit ble low power mode)
 */
void tz1smHalGpioBleWakeUp(const bool activation)
{
  tz1smHalGpioWrite(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP, activation);
}

/*
 * @brief
 * Read GPIO BLE Status Pin Value.
 */
bool tz1smHalGpioBleLowpowerStatus(void)
{
  bool val;

  tz1smHalGpioRead(TZ1SM_HAL_GPIO_BLE_STATUS, &val);

  return val;
}

/*
 * @brief
 * Read GPIO BLE Host Wakeup Pin(true:UART is Active, false:No data)
 */
bool tz1smHalGpioBleHostWakeupStatus(void)
{
  bool val;

  tz1smHalGpioRead(TZ1SM_HAL_GPIO_BLE_HOST_WAKE_UP, &val);

  return val;
}

/*
 * @brief
 * Read GPIO BLE Request Wake Up Pin Value.
 */
bool tz1smHalGpioBleHostLowpowerStatus(void)
{
  bool val;

  tz1smHalGpioRead(TZ1SM_HAL_GPIO_BLE_REQUEST_WAKE_UP, &val);
  
  return (true == val) ? false : true;
}

uint8_t tz1smHalGpioButtonInit(void (handler)(uint32_t pin))
{
  GPIO_STATUS ret;
  
  ret = tz1smHalGpioConfigure(TWIC_BUTTON_GPIO_NO, GPIO_DIRECTION_INPUT_HI_Z,
                              GPIO_EVENT_EDGE_NEG, handler);

  if (ret != GPIO_OK)
    return 1;

  return 0;
}

uint8_t tz1smHalGpioButtonFinalize(void)
{
  GPIO_STATUS ret;
  
  ret = tz1smHalGpioConfigure(TWIC_BUTTON_GPIO_NO, GPIO_DIRECTION_INPUT_HI_Z,
                              GPIO_EVENT_DISABLE, NULL);
  if (ret != GPIO_OK) return 1;

  return 0;
}

uint8_t tz1smHalGpioLedInit(void)
{
  GPIO_STATUS sts;

  /* LED Pin Configuration */
#if defined(TWIC_LED_GPIO_LED1)
  sts = tz1smHalGpioConfigure(TWIC_LED_GPIO_LED1, GPIO_DIRECTION_OUTPUT_2MA,
                              GPIO_EVENT_DISABLE, NULL);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED1 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED2)
  sts = tz1smHalGpioConfigure(TWIC_LED_GPIO_LED2, GPIO_DIRECTION_OUTPUT_2MA,
                              GPIO_EVENT_DISABLE, NULL);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED2 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED3)
  sts = tz1smHalGpioConfigure(TWIC_LED_GPIO_LED3, GPIO_DIRECTION_OUTPUT_2MA,
                              GPIO_EVENT_DISABLE, NULL);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED3 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED4)
  sts = tz1smHalGpioConfigure(TWIC_LED_GPIO_LED4, GPIO_DIRECTION_OUTPUT_2MA,
                              GPIO_EVENT_DISABLE, NULL);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED4 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED5)
  sts = tz1smHalGpioConfigure(TWIC_LED_GPIO_LED5, GPIO_DIRECTION_OUTPUT_2MA,
                              GPIO_EVENT_DISABLE, NULL);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED5 | 0x80;
#endif

  return 0;
}

uint8_t tz1smHalGpioLedFinalize(void)
{
  GPIO_STATUS sts = GPIO_OK;
  
#if defined(TWIC_LED_GPIO_LED1)
  sts = Driver_GPIO.WritePin(TWIC_LED_GPIO_LED1, 0);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED1 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED2)
  sts = Driver_GPIO.WritePin(TWIC_LED_GPIO_LED2, 0);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED2 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED3)
  sts = Driver_GPIO.WritePin(TWIC_LED_GPIO_LED3, 0);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED3 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED4)
  sts = Driver_GPIO.WritePin(TWIC_LED_GPIO_LED4, 0);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED4 | 0x80;
#endif
#if defined(TWIC_LED_GPIO_LED5)
  sts = Driver_GPIO.WritePin(TWIC_LED_GPIO_LED5, 0);
  if (GPIO_OK != sts) return TWIC_LED_GPIO_LED5 | 0x80;
#endif

  return 0 & sts;
}

#else

#warning V0.0.1 is no supporting the other GPIO Drivers other than CMSIS_GPIO.

void tz1smHalGpioInit(void) { return; }

void tz1smHalGpioUnInit(void) { return; }

tz1smHalStatus_t tz1smHalGpioBleReset(const bool reset)
{
  return TZ1SM_HAL_STATUS_RESERVED;
}

void tz1smHalGpioBleWakeUp(const bool activation)
{
  return;
}

uint8_t tz1smHalGpioButtonInit(void *(handler)(uint32_t pin)) { return 0; }

uint8_t tz1smHalGpioButtonFinalize(void) { return 0; }

uint8_t tz1smHalGpioLedInit(void) { return 0; }

uint8_t tz1smHalGpioLedFinalize(void) { return 0; }
  
bool tz1smHalGpioBleHostLowpowerStatus(void) { return false; }

bool tz1smHalGpioBleLowpowerStatus(void) { return false; }

#endif


/*
 * UART
 *
 */

#if defined(TWIC_UART__CMSIS_UART)

#define TWIC_UART_BLE_DRIVER_CH Driver_UART2
#define TWIC_UART_BLE_HW_FIFO_SIZE 16
#if defined(TWIC_BLE_HWIP_V41)
#define TWIC_UART_BLE_TX_BUFFER_SIZE (196*2)
#else
#define TWIC_UART_BLE_TX_BUFFER_SIZE (136*2)
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
#define TWIC_UART_BLE_RX_BUFFER_SIZE (1064*3)
#else
#if defined(TWIC_BLE_HWIP_V41)
#define TWIC_UART_BLE_RX_BUFFER_SIZE (196*TWIC_CONFIG_RX_BUFFER_SIZE)
#else
#define TWIC_UART_BLE_RX_BUFFER_SIZE (136*TWIC_CONFIG_RX_BUFFER_SIZE)
#endif
#endif
#define TWIC_UART_BLE_TX_CTS_CHECK_COUNT (0x1000) /* T.B.D */

#if defined(TWIC_DEBUG_LOG_TRACE)
/* debug */
#define twicUartLog(...)   printf(__VA_ARGS__);
#define TWIC_UART_INFO(...)
#define TWIC_UART_ERROR(...) printf(__VA_ARGS__);
#else
#define twicUartLog(...)
#define TWIC_UART_INFO(...)
#define TWIC_UART_ERROR(...)
#endif

/* callback handler */
static void uart_ble_handler(ARM_UART_EVENT e);

/* rx buffer */
static uint8_t twic_ble_ua_rxb[TWIC_UART_BLE_RX_BUFFER_SIZE];
static uint16_t twic_ble_ua_rxrp; /* count up by application */
static uint16_t twic_ble_ua_rxwp; /* count up by HW FIFO */

/* tx buffer */
static uint8_t twic_ble_ua_txb[TWIC_UART_BLE_TX_BUFFER_SIZE];
static uint16_t twic_ble_ua_txrp; /* count up by HW FIFO */
static uint16_t twic_ble_ua_txwp; /* count up by application */

/* UART Setting */
#define TWIC_BLE_UA_BITS (8)
#define TWIC_BLE_UA_PARITY (ARM_UART_PARITY_NONE)
#define TWIC_BLE_UA_STOP (ARM_UART_STOP_BITS_1)
static ARM_UART_FLOW_CONTROL twic_ble_ua_fc;

/* In performing hardware control, it surely uses CMSIS UART API. In
 * using GPIO, it lets tz1smHalGpio surely mounted in this file pass,
 * and calls CMSIS GPIO API. Coding of the following functions is
 * carried out using CMSIS API. */

#if defined(TWIC_RTOS_TOOL_X1)
traceLabel xRxTraceUserEvent;
traceLabel xTxTraceUserEvent;
#endif

/*
 * @brief
 * 1. Initialize UART Driver
 * 2. Configure UART Driver
 * 3. Set Tx Threshold.
 * 4. Set Rx Threshold.
 * 5. Power Mode is FULL.
 */
void tz1smHalUartInit(uint32_t br, bool fc)
{
  ARM_UART_STATUS sts;
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  if (true == fc) twic_ble_ua_fc = ARM_UART_FLOW_CONTROL_RTS_CTS;
  else twic_ble_ua_fc = ARM_UART_FLOW_CONTROL_NONE;
  
  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  sts = drv->Initialize(uart_ble_handler,
                        (1u << ARM_UART_EVENT_RX_OVERRUN)       |
                        (1u << ARM_UART_EVENT_RX_BREAK)         |
                        (1u << ARM_UART_EVENT_RX_PARITY_ERROR)  |
                        (1u << ARM_UART_EVENT_RX_FRAMING_ERROR) |
                        (1u << ARM_UART_EVENT_RX_TIMEOUT)       |
                        (1u << ARM_UART_EVENT_TX_THRESHOLD)     |
                        (1u << ARM_UART_EVENT_RX_THRESHOLD));
  
  if (ARM_UART_OK != sts) {
    TWIC_UART_ERROR("Initialize()=%d\r\n", sts);
  }
#if 0  
  /* TZ1x SIOSC4M (NO TCO (Temperature Compensated Oscillator)) WORKAROUND >>.
     SIOSC4M provides a compact oscillator having PM independence but less
     satisfactory frequency temperature characteristics.
     ATTENTION: This settings must be removed when the UA CLOCK is not
     provided from the SIOSC4M.
  */
  if (115200 == br) br = 115600;
  /* << TZ1x SIOSC4M (NO TCO (Temperature Compensated Oscillator)) WORKAROUND */
#endif  
  sts = drv->Configure(br, TWIC_BLE_UA_BITS, TWIC_BLE_UA_PARITY,
                       TWIC_BLE_UA_STOP, twic_ble_ua_fc);
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("TZBT 1 Configure()=%d\r\n", sts);
  
  sts = drv->SetTxThreshold(14);
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("SetTxThreshold()=%d\r\n", sts);
  
  sts = drv->SetRxThreshold(2);
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("SetTxThreshold()=%d\r\n", sts);
  
  sts = drv->PowerControl(ARM_POWER_FULL);
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("PowerControl()=%d\r\n", sts);
  
  /* initialize internel variable */
  twic_ble_ua_rxrp = 0;
  twic_ble_ua_rxwp = 0;
  
  twic_ble_ua_txrp = 0;
  twic_ble_ua_txwp = 0;
  
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

#if defined(TWIC_RTOS_TOOL_X1)
  xRxTraceUserEvent = xTraceOpenLabel( "rx" );
  xTxTraceUserEvent = xTraceOpenLabel( "tx" );
#endif
  
  return;
}


/*
 * @brief
 * 1. Enable or Disable CTS/RTS(true:enable CTS/RTS, false:disable CTS/RTS).
 */

tz1smHalStatus_t tz1smHalUartControl(uint32_t br, bool fc)
{
  tz1smHalStatus_t status = TZ1SM_HAL_STATUS_OK;
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  ARM_UART_STATUS sts;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (true == fc) twic_ble_ua_fc = ARM_UART_FLOW_CONTROL_RTS_CTS;
  else twic_ble_ua_fc = ARM_UART_FLOW_CONTROL_NONE;
  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  drv->PowerControl(ARM_POWER_LOW);
#if 0
  /* TZ1x SIOSC4M (NO TCO (Temperature Compensated Oscillator)) WORKAROUND >>.
     SIOSC4M provides a compact oscillator having PM independence but less
     satisfactory frequency temperature characteristics.

     ATTENTION: This settings must be removed when the UA CLOCK is not
     provided from the SIOSC4M.
  */
  if (115200 == br) br = 115600;
  /* << TZ1x SIOSC4M (NO TCO (Temperature Compensated Oscillator)) WORKAROUND */
#endif  
  sts = drv->Configure(br, TWIC_BLE_UA_BITS, TWIC_BLE_UA_PARITY,
                       TWIC_BLE_UA_STOP, twic_ble_ua_fc);
  drv->PowerControl(ARM_POWER_FULL);
  if (ARM_UART_OK != sts) {
    TWIC_UART_ERROR("TZBT 2 Configure()=%d\r\n", sts);
    status = TZ1SM_HAL_STATUS_ERROR_DRIVER;
  }
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return status;
}


/*
 * @brief
 * 1. Uninitialize UART Driver.
 */
void tz1smHalUartUnInit(void)
{
  ARM_UART_STATUS sts;
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  drv->PowerControl(ARM_POWER_LOW);
  sts = drv->Uninitialize();
  if (ARM_UART_OK != sts) {
    TWIC_UART_ERROR("Uninitialize()=%d\r\n", sts);
  }
  
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return;
}

/*
 * @brief
 * 1. flush UART Driver's Transmit FIFO and software transmit buffer.
 * Note: RTS is not changed
 */
void tz1smHalUartTxBufFlush(void)
{
  ARM_UART_STATUS sts;
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  /* clear tx buffer */
  sts = drv->FlushTxBuffer();
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("FlushTxBuffer()=%d\r\n", sts);
  twic_ble_ua_txrp = 0;
  twic_ble_ua_txwp = 0;
  
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  
  return;
}

/*
 * @brief
 * 1. flush UART Driver's Receive FIFO and software receive buffer.
 */
void tz1smHalUartRxBufFlush(void)
{
  ARM_UART_STATUS sts;
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  TZ1SM_HAL_INTR_STATUS_DEF;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  /* clear rx buffer */
  sts = drv->FlushRxBuffer();
  if (ARM_UART_OK != sts) TWIC_UART_ERROR("FlushRxBuffer()=%d\r\n", sts);
  twic_ble_ua_rxrp = 0;
  twic_ble_ua_rxwp = 0;
  
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return;
}

/*
 * @brief
 * 1. send data from software transmit buffer.
 * 2. check whether could store data to software transmit buffer.
 * 3. send data from data. if UART Driver's FIFO is rest.
 * 4. copy data to software transmit buffer.
 * 5. if ok, return TZ1SM_HAL_STATUS_OK.
 *    if UART Driver occured error, return TZ1SM_HAL_STATUS_ERROR_DRIVER.
 *    if CTS interval is too long time, return TZ1SM_HAL_STATUS_ERROR_CTS.
 *    if Not copy to software transmit buffer, return
 *    TZ1SM_HAL_STATUS_ERROR_RESOURCE.
 */

tz1smHalStatus_t
tz1smHalUartPostData(const uint8_t * const data, const uint16_t length)
{
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  int32_t count;
  uint16_t total = 0;
  uint16_t tmp,rest_size;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
#if defined(TWIC_RTOS_TOOL_X1)
  if (length > 4)
    vTracePrintF(xTraceOpenLabel(__FILE__),
                 "%d:%x %x", __LINE__, data[3], data[4]);
#endif
  
#if defined(TWIC_DEBUG_LOG_UART)
  {
    int __i;
    twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
    twicUartLog("TX= ");
    for (__i = 0 ; __i < length ; __i++) {
      twicUartLog("%02x ",data[__i]);
    }
    twicUartLog("\r\n");
  }
#endif

  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  if (twic_ble_ua_txrp != twic_ble_ua_txwp) {
    /* send from tx buffer at first.*/
    if (twic_ble_ua_txwp < twic_ble_ua_txrp) {
      count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                             (TWIC_UART_BLE_TX_BUFFER_SIZE - twic_ble_ua_txrp));
      if (0 > count) {
        ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
        goto END;
      }
      else {
        twic_ble_ua_txrp += count;
        total += count;
      }
      if (TWIC_UART_BLE_TX_BUFFER_SIZE == twic_ble_ua_txrp)
        twic_ble_ua_txrp = 0;
    }
    
    if (twic_ble_ua_txwp > twic_ble_ua_txrp &&
        TWIC_UART_BLE_HW_FIFO_SIZE > total) {
      count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                             (twic_ble_ua_txwp - twic_ble_ua_txrp));
      if (0 > count) {
        ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
        goto END;
      } else {
        twic_ble_ua_txrp += count;
        total += count;
      }
    }
  }

  /* check buffer size */
  if (twic_ble_ua_txwp >= twic_ble_ua_txrp) {
    rest_size = (TWIC_UART_BLE_TX_BUFFER_SIZE - twic_ble_ua_txwp) +
      twic_ble_ua_txrp - 1;
  } else rest_size = twic_ble_ua_txrp - twic_ble_ua_txwp - 1;
  
  if (rest_size < length) {
    ret_sts = TZ1SM_HAL_STATUS_ERROR_RESOURCE;
    goto END;
  }
  
  /* send data buffer at second */
  tmp = 0;
  if (twic_ble_ua_txrp == twic_ble_ua_txwp &&
      TWIC_UART_BLE_HW_FIFO_SIZE > total) {
    count = drv->WriteData(&data[0], length);
    if (0 > count) {
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    } else {
      tmp += count;
      total += count;
    }
  }
  
  if (tmp != length) {
    /* copy to tx buffer */
    if (twic_ble_ua_txwp >= twic_ble_ua_txrp) {
      while (tmp < length && TWIC_UART_BLE_TX_BUFFER_SIZE > twic_ble_ua_txwp) {
        twic_ble_ua_txb[twic_ble_ua_txwp++] = data[tmp++];
      }
      if (TWIC_UART_BLE_TX_BUFFER_SIZE == twic_ble_ua_txwp)
        twic_ble_ua_txwp = 0;
    }
    
    while (tmp < length) twic_ble_ua_txb[twic_ble_ua_txwp++] = data[tmp++];
  }
  
END:
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  
  return ret_sts;
}

/*
 * @brief
 * 1. send data from software transmit buffer.
 * 2. send data from data.
 * 3. wait for transmit data.
 * 4. if UART Driver occured error, return TZ1SM_HAL_STATUS_ERROR_DRIVER.
 *    if CTS interval is too long time, return TZ1SM_HAL_STATUS_ERROR_CTS.
 */

tz1smHalStatus_t
tz1smHalUartSendData(const uint8_t * const data, const uint16_t length)
{
  
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  int32_t count;
  uint16_t tmp;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;
  ARM_UART_MODEM_STATUS modem_status;
  uint32_t cts_count = 0;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
#if defined(TWIC_DEBUG_LOG_UART)
  {
    int __i;
    twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
    twicUartLog("TX= ");
    for (__i = 0 ; __i < length ; __i++) {
      twicUartLog("%02x ",data[__i]);
    }
    twicUartLog("\r\n");
  }
#endif
  
  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  if (twic_ble_ua_txrp != twic_ble_ua_txwp) {
    /* send from tx buffer at first.*/
    if (twic_ble_ua_txwp < twic_ble_ua_txrp) {
      while (TWIC_UART_BLE_TX_BUFFER_SIZE > twic_ble_ua_txrp) {
        count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                               TWIC_UART_BLE_TX_BUFFER_SIZE - twic_ble_ua_txrp);
        if (0 == count) {
          if (ARM_UART_FLOW_CONTROL_RTS_CTS == twic_ble_ua_fc) {
            /* check CTS value */
            modem_status = drv->GetModemStatus();
            if (1 == modem_status.cts) {
              if (TWIC_UART_BLE_TX_CTS_CHECK_COUNT < ++cts_count) {
                /* CTS too long time */
                ret_sts = TZ1SM_HAL_STATUS_ERROR_CTS;
                goto END;
              }
            } else cts_count = 0;
          }
        } else if (0 > count) {
          ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
          goto END;
        } else {
          twic_ble_ua_txrp += count;
          cts_count = 0;
        }
      }
      twic_ble_ua_txrp = 0;
    }
    
    while (twic_ble_ua_txrp < twic_ble_ua_txwp) {
      count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                             (twic_ble_ua_txwp - twic_ble_ua_txrp));
      if (0 == count) {
        if (ARM_UART_FLOW_CONTROL_RTS_CTS == twic_ble_ua_fc) {
          /* check CTS value */
          modem_status = drv->GetModemStatus();
          if (1 == modem_status.cts) {
            if (TWIC_UART_BLE_TX_CTS_CHECK_COUNT < ++cts_count) {
              /* CTS too long time */
              ret_sts = TZ1SM_HAL_STATUS_ERROR_CTS;
              goto END;
            }
          } else cts_count = 0;
        }
      } else if (0 > count) {
        ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
        goto END;
      } else {
        twic_ble_ua_txrp += count;
        cts_count = 0;
      }
    }
  }
  
  /* send data buffer */
  tmp = 0;
  while (tmp < length) {
    count = drv->WriteData(&data[tmp], (length - tmp));
    if (0 == count) {
      if (ARM_UART_FLOW_CONTROL_RTS_CTS == twic_ble_ua_fc) {
        /* check CTS value */
        modem_status = drv->GetModemStatus();
        if (1 == modem_status.cts) {
          if (TWIC_UART_BLE_TX_CTS_CHECK_COUNT < ++cts_count) {
            /* CTS too long time */
            ret_sts = TZ1SM_HAL_STATUS_ERROR_CTS;
            goto END;
          }
        } else cts_count = 0;
      }
    } else if (0 > count) {
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    } else {
      tmp += count;
      cts_count = 0;
    }
  }
  
  /* check finish to send */
  while (!(drv->TxDone())) {
    if (ARM_UART_FLOW_CONTROL_RTS_CTS == twic_ble_ua_fc) {
      /* check CTS value */
      modem_status = drv->GetModemStatus();
      if (1 == modem_status.cts) {
        if (TWIC_UART_BLE_TX_CTS_CHECK_COUNT < ++cts_count) {
          /* CTS too long time */
          ret_sts = TZ1SM_HAL_STATUS_ERROR_CTS;
          goto END;
        }
      } else cts_count = 0;
    }
  }
  
END:
  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  
  return ret_sts;
}

/*
 * @brief
 * 1. read data by UART Driver, then store software receive buffer.
 * 2. data is first byte on software receive buffer.
 *    receive_length is rest size of received on software receive buffer.
 * 3. if stored data, return TZ1SM_HAL_STATUS_EVENT_MESSAGE.
 *    if not stored data, return TZ1SM_HAL_STATUS_OK.
 * 
 * Note: This function is called by receive interrupt or task context.
 */
tz1smHalStatus_t
tz1smHalUartPeekData(uint8_t * const data, uint16_t * const receive_length)
{
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  uint32_t len;
  uint32_t ipsr;
  int32_t count;
  uint16_t total = 0;
  tz1smHalStatus_t ret_sts = TZ1SM_HAL_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  *receive_length = 0;
  ipsr = __get_IPSR();
  if (0 == ipsr) TZ1SM_HAL_IRQ_DISABLE_SAVE();

  /* receive from HW RX FIFO at first.*/
  if (twic_ble_ua_rxrp <= twic_ble_ua_rxwp) {
    len = TWIC_UART_BLE_RX_BUFFER_SIZE - twic_ble_ua_rxwp;
    if (0 == twic_ble_ua_rxrp) len -= 1;
    count = drv->ReadData(&twic_ble_ua_rxb[twic_ble_ua_rxwp], len);
    if (0 == count);
    else if (0 > count) {
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    } else {
#if defined(TWIC_DEBUG_LOG_UART)
      {
        int __i;
        twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
        twicUartLog("rx_wp=%d,rx_rp=%d:",twic_ble_ua_rxwp,twic_ble_ua_rxrp);
        for (__i = 0 ; __i < count ; __i++) {
          twicUartLog("%02x ",twic_ble_ua_rxb[twic_ble_ua_rxwp + __i]);
        }
        twicUartLog("\r\n");
      }
#endif
      twic_ble_ua_rxwp += count;
      total += count;
    }
    if (TWIC_UART_BLE_RX_BUFFER_SIZE == twic_ble_ua_rxwp) twic_ble_ua_rxwp = 0;
  }
  
  if (twic_ble_ua_rxrp > twic_ble_ua_rxwp &&
      TWIC_UART_BLE_HW_FIFO_SIZE > total) {
    len = twic_ble_ua_rxrp - twic_ble_ua_rxwp - 1;
    count = drv->ReadData(&twic_ble_ua_rxb[twic_ble_ua_rxwp], len);
    if (0 > count) {
      ret_sts = TZ1SM_HAL_STATUS_ERROR_DRIVER;
      goto END;
    } else {
#if defined(TWIC_DEBUG_LOG_UART)
      {
        int __i;
        twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
        twicUartLog("rx_wp=%d,rx_rp=%d:",twic_ble_ua_rxwp,twic_ble_ua_rxrp);
        for (__i = 0 ; __i < count ; __i++) {
          twicUartLog("%02x ",twic_ble_ua_rxb[twic_ble_ua_rxwp + __i]);
        }
        twicUartLog("\r\n");
      }
#endif
      twic_ble_ua_rxwp += count;
      total += count;
    }
  }
  
  if (twic_ble_ua_rxwp != twic_ble_ua_rxrp) {
    *data = twic_ble_ua_rxb[twic_ble_ua_rxrp++];
    if (TWIC_UART_BLE_RX_BUFFER_SIZE == twic_ble_ua_rxrp) twic_ble_ua_rxrp = 0;
    if (twic_ble_ua_rxwp >= twic_ble_ua_rxrp)
      len = twic_ble_ua_rxwp - twic_ble_ua_rxrp;
    else
      len = (TWIC_UART_BLE_RX_BUFFER_SIZE - twic_ble_ua_rxrp) +
        twic_ble_ua_rxwp;
    *receive_length = len;
    ret_sts = TZ1SM_HAL_STATUS_EVENT_MESSAGE;
#if defined(TWIC_DEBUG_LOG_UART)
    {
      twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
      twicUartLog("rx_wp=%d,rx_rp=%d:",twic_ble_ua_rxwp,twic_ble_ua_rxrp);
      twicUartLog("d=%02x\r\n",*data);
    }
#endif
  }
  
END:
  if (0 == ipsr) TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  
  return ret_sts;
}

/*
 * @brief
 * 1. read data by UART Driver, then store software receive buffer.
 * 2. if software receive buffer is too small, flush UART Driver RX,
 *    Not flush software receive buffer.
 * 
 * Note: tz1smHalUartPeekData() function is same behavior this function.
 *       This function is called by receive interrupt or task context.
 */

void tz1smHalUartGetData(void)
{
  /* ARM_UART_STATUS sts; */
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  uint32_t ipsr;
  int32_t count;
  uint16_t total = 0;
  uint16_t receive_size;
  TZ1SM_HAL_INTR_STATUS_DEF;
  
  ipsr = __get_IPSR();
  if (0 == ipsr) TZ1SM_HAL_IRQ_DISABLE_SAVE();
  
  /* receive from HW RX FIFO at first.*/
  while (twic_ble_ua_rxrp <= twic_ble_ua_rxwp) {
    if (0 == twic_ble_ua_rxrp) {
      count = drv->ReadData(&twic_ble_ua_rxb[twic_ble_ua_rxwp],
                            TWIC_UART_BLE_RX_BUFFER_SIZE - twic_ble_ua_rxwp -1);
    } else {
      count = drv->ReadData(&twic_ble_ua_rxb[twic_ble_ua_rxwp],
                            TWIC_UART_BLE_RX_BUFFER_SIZE - twic_ble_ua_rxwp);
    }
    if (0 == count) break;
    else if (0 > count) goto END;
    else {
#if defined(TWIC_DEBUG_LOG_UART)
      {
        int __i;
        twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
        twicUartLog("rx_wp=%d,rx_rp=%d:",twic_ble_ua_rxwp,twic_ble_ua_rxrp);
        for (__i = 0 ; __i < count ; __i++) {
          twicUartLog("%02x ",twic_ble_ua_rxb[twic_ble_ua_rxwp + __i]);
        }
        twicUartLog("\r\n");
      }
#endif
      twic_ble_ua_rxwp += count;
      total += count;
    }
    if (TWIC_UART_BLE_RX_BUFFER_SIZE == twic_ble_ua_rxwp) twic_ble_ua_rxwp = 0;
  }
  
  while (twic_ble_ua_rxrp > (twic_ble_ua_rxwp + 1)) {
    count = drv->ReadData(&twic_ble_ua_rxb[twic_ble_ua_rxwp],
                          (twic_ble_ua_rxrp - twic_ble_ua_rxwp - 1));
    if (0 == count) break;
    else if (0 > count) goto END;
    else {
#if defined(TWIC_DEBUG_LOG_UART)
      {
        int __i;
        twicUartLog("%s:%d:",__FUNCTION__,__LINE__);
        twicUartLog("rx_wp=%d,rx_rp=%d:",twic_ble_ua_rxwp,twic_ble_ua_rxrp);
        for (__i = 0 ; __i < count ; __i++) {
          twicUartLog("%02x ",twic_ble_ua_rxb[twic_ble_ua_rxwp + __i]);
        }
        twicUartLog("\r\n");
      }
#endif
      twic_ble_ua_rxwp += count;
      total += count;
    }
  }
  
  /* check sw fifo size */
  if (0 == total) {
    if (twic_ble_ua_rxwp >= twic_ble_ua_rxrp)
      receive_size = twic_ble_ua_rxwp - twic_ble_ua_rxrp;
    else
      receive_size = (TWIC_UART_BLE_RX_BUFFER_SIZE - twic_ble_ua_rxrp) +
        twic_ble_ua_rxwp;
    if ((TWIC_UART_BLE_RX_BUFFER_SIZE -1) == receive_size) {
      /* sw fifo is full. clear rx buffer.
         sts = drv->FlushRxBuffer();
         if (ARM_UART_OK != sts)
         TWIC_UART_ERROR("FlushRxBuffer()=%d\r\n", sts);
      */
    }
  }
  
END:
  if (0 == ipsr) TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return;
}

/*
 *@brief callback function for UART(TX)
 */
static void uart_ble_handler_tx(void)
{
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  int32_t count;
  uint16_t total = 0;
  
  if (twic_ble_ua_txrp != twic_ble_ua_txwp) {
    /* send from tx buffer at first.*/
    if (twic_ble_ua_txwp < twic_ble_ua_txrp) {
      count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                             TWIC_UART_BLE_TX_BUFFER_SIZE - twic_ble_ua_txrp);
      if (0 > count) goto END;
      else { twic_ble_ua_txrp += count; total += count; }
      if (twic_ble_ua_txrp == TWIC_UART_BLE_TX_BUFFER_SIZE)
        twic_ble_ua_txrp = 0;
    }
    
    if (twic_ble_ua_txwp > twic_ble_ua_txrp &&
        TWIC_UART_BLE_HW_FIFO_SIZE > total) {
      count = drv->WriteData(&twic_ble_ua_txb[twic_ble_ua_txrp],
                             twic_ble_ua_txwp - twic_ble_ua_txrp);
      if (0 > count) goto END;
      else { twic_ble_ua_txrp += count; total += count; }
    }
  }
  
END:
  return;
}

/*
 *@brief callback function for UART(RX)
 */
static void uart_ble_handler_rx(void)
{
  tz1smHalUartGetData();
#if defined(TWIC_EVENT_CALLBACK)
  if (twic_ble_ua_rxrp != twic_ble_ua_rxwp && hal_cb.intr_ble_uart_rx != NULL) {
    hal_cb.intr_ble_uart_rx();
#if defined(TWIC_DEBUG_LOG_UART)
    twicUartLog("intr_ble_uart_rx called\r\n");
#endif
  }
#endif
}

/*
 *@brief callback function for BLE UART
 */

static void uart_ble_handler(ARM_UART_EVENT e)
{
#if defined(TWIC_RTOS_TOOL_X1)
  portENTER_CRITICAL();
  vTraceStoreISRBegin(1);
  portEXIT_CRITICAL();
#endif
  
  switch (e) {
  case ARM_UART_EVENT_TX_THRESHOLD:
    TWIC_UART_INFO("[TX_THRESHOLD]");
#if defined(TWIC_RTOS_TOOL_X1)
    vTraceUserEvent( xTxTraceUserEvent );
#endif
    uart_ble_handler_tx();
    break;
    
  case ARM_UART_EVENT_RX_THRESHOLD:
    TWIC_UART_INFO("[RX_THRESHOLD]");
#if defined(TWIC_RTOS_TOOL_X1)
    vTraceUserEvent( xRxTraceUserEvent );
#endif
    uart_ble_handler_rx();
    break;
    
  case ARM_UART_EVENT_RX_TIMEOUT:
#if defined(TWIC_DEBUG_LOG_UART)
    {
      twicUartLog("RX_TIMEOUT\r\n");
    }
#endif
    TWIC_UART_INFO("[RX_TIMEOUT]");
#if defined(TWIC_RTOS_TOOL_X1)
    vTraceUserEvent( xRxTraceUserEvent );
#endif
    uart_ble_handler_rx();
    break;
    
  case ARM_UART_EVENT_RX_OVERRUN:
    TWIC_UART_INFO("[RX_OVERRUN]");
    break;
  case ARM_UART_EVENT_RX_BREAK:
    TWIC_UART_INFO("[RX_BREAK]");
    break;
  case ARM_UART_EVENT_RX_PARITY_ERROR:
    TWIC_UART_INFO("[RX_PARITY_ERROR]");
    break;
  case ARM_UART_EVENT_RX_FRAMING_ERROR:
    TWIC_UART_INFO("[RX_FRAMING_ERROR]");
    break;
  default:
    break;
  }

#if defined(TWIC_RTOS_TOOL_X1)
  portENTER_CRITICAL();
  vTraceStoreISREnd();
  portEXIT_CRITICAL();
#endif
  
  return;
}

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
void tz1smHalUartLowPower(bool enable)
{
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;

  if (false == enable) {
    /* UART_TZ10xx rev 20659 accommodated this procedure.
      drv->Configure();
      if (ARM_UART_FLOW_CONTROL_NONE)
      drv->SetModemControl(ARM_UART_RTS_SET);
    */
    drv->PowerControl(ARM_POWER_FULL);
  } else {
    /* UART_TZ10xx rev 20659 accommodated this procedure.
       drv->Configure();
       drv->SetModemControl(ARM_UART_RTS_CLEAR);
    */
    Driver_UART2.PowerControl(ARM_POWER_LOW);
  }
  
  return;
}

bool tz1smHalUartDataAvailable(void)
{
  ARM_DRIVER_UART *drv = &TWIC_UART_BLE_DRIVER_CH;
  
  if (0 < drv->DataAvailable()) return true;
  if (twic_ble_ua_rxrp != twic_ble_ua_rxwp) return true;

  return false;
}
#endif

void tz1smHalSuppressHpd(const bool enable)
{
#if defined(TWIC_LECE_BOOT_TZ1041_PRIMARY_SAMPLE) || defined(TWIC_LECE_SUPPRESS_HPD)
  static tz1sm_gpio_cts_configuration_t cts_setup;

#if (RTE_UA2_CTS_N_ID != 1) && (RTE_GPIO_11_ID != 2)
#define IOMUX_UA2_CTS_N_FMODE (0)/* HW FLOW is OFF. Pin is except CTS. */
#elif (RTE_UA2_CTS_N_ID == 1) && (RTE_GPIO_11_ID != 2)
#define IOMUX_UA2_CTS_N_FMODE (1)/* HW FLOW is ON. Pin is except CTS. */
#elif (RTE_UA2_CTS_N_ID != 1) && (RTE_GPIO_11_ID == 2)
#define IOMUX_UA2_CTS_N_FMODE (3)/* HW FLOW is OFF. Pin is CTS. */
#else
#error Conflicted IO pin in IOMUX_UA2_CTS_N_FMODE
#endif
#if (RTE_GPIO_0_ID != 1)
#define IOMUX_GPIO_0_FMODE 0
#else
#define IOMUX_GPIO_0_FMODE 1
#endif
#if (RTE_GPIO_1_ID != 1)
#define IOMUX_GPIO_1_FMODE 0
#else
#define IOMUX_GPIO_1_FMODE 1
#endif
#if (RTE_GPIO_2_ID != 1)
#define IOMUX_GPIO_2_FMODE 0
#else
#define IOMUX_GPIO_2_FMODE 1
#endif
#if (RTE_GPIO_3_ID != 1)
#define IOMUX_GPIO_3_FMODE 0
#else
#define IOMUX_GPIO_3_FMODE 1
#endif
#if (RTE_GPIO_4_ID != 1)
#define IOMUX_GPIO_4_FMODE 0
#else
#define IOMUX_GPIO_4_FMODE 1
#endif
#if (RTE_GPIO_5_ID != 1)
#define IOMUX_GPIO_5_FMODE 0
#else
#define IOMUX_GPIO_5_FMODE 1
#endif
#if (RTE_GPIO_6_ID != 1)
#define IOMUX_GPIO_6_FMODE 0
#else
#define IOMUX_GPIO_6_FMODE 1
#endif
#if (RTE_GPIO_7_ID != 1)
#define IOMUX_GPIO_7_FMODE 0
#else
#define IOMUX_GPIO_7_FMODE 1
#endif
#if (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID != 1)
#define IOMUX_GPIO_8_FMODE 0
#elif (RTE_GPIO_8_ID == 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID != 1)
#define IOMUX_GPIO_8_FMODE 1
#elif (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID == 1) && (RTE_TRACEDATA0_ID != 1)
#define IOMUX_GPIO_8_FMODE 2
#elif (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID == 1)
#define IOMUX_GPIO_8_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_8_FMODE
#endif
#if (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID != 1)
#define IOMUX_GPIO_9_FMODE 0
#elif (RTE_GPIO_9_ID == 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID != 1)
#define IOMUX_GPIO_9_FMODE 1
#elif (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID == 1) && (RTE_TRACEDATA1_ID != 1)
#define IOMUX_GPIO_9_FMODE 2
#elif (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID == 1)
#define IOMUX_GPIO_9_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_9_FMODE
#endif
#if (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID != 1)
#define IOMUX_GPIO_10_FMODE 0
#elif (RTE_GPIO_10_ID == 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID != 1)
#define IOMUX_GPIO_10_FMODE 1
#elif (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID == 1) && (RTE_TRACEDATA2_ID != 1)
#define IOMUX_GPIO_10_FMODE 2
#elif (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID == 1)
#define IOMUX_GPIO_10_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_10_FMODE
#endif
#if (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID != 1)
#define IOMUX_GPIO_11_FMODE 0
#elif (RTE_GPIO_11_ID == 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID != 1)
#define IOMUX_GPIO_11_FMODE 1
#elif (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID == 1) && (RTE_TRACEDATA3_ID != 1)
#define IOMUX_GPIO_11_FMODE 2
#elif (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID == 1)
#define IOMUX_GPIO_11_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_11_FMODE
#endif
#if (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID != 1)
#define IOMUX_GPIO_12_FMODE 0
#elif (RTE_GPIO_12_ID == 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID != 1)
#define IOMUX_GPIO_12_FMODE 1
#elif (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID == 2) && (RTE_CAPTURE0_ID != 1)
#define IOMUX_GPIO_12_FMODE 2
#elif (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID == 1)
#define IOMUX_GPIO_12_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_12_FMODE
#endif
#if (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID != 1)
#define IOMUX_GPIO_13_FMODE 0
#elif (RTE_GPIO_13_ID == 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID != 1)
#define IOMUX_GPIO_13_FMODE 1
#elif (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID == 2) && (RTE_CAPTURE1_ID != 1)
#define IOMUX_GPIO_13_FMODE 2
#elif (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID == 1)
#define IOMUX_GPIO_13_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_13_FMODE
#endif
#if (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID != 1)
#define IOMUX_GPIO_14_FMODE 0
#elif (RTE_GPIO_14_ID == 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID != 1)
#define IOMUX_GPIO_14_FMODE 1
#elif (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID == 2) && (RTE_UA0_RTS_N_ID != 1)
#define IOMUX_GPIO_14_FMODE 2
#elif (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID == 1)
#define IOMUX_GPIO_14_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_14_FMODE
#endif
#if (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID != 1)
#define IOMUX_GPIO_15_FMODE 0
#elif (RTE_GPIO_15_ID == 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID != 1)
#define IOMUX_GPIO_15_FMODE 1
#elif (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID == 2) && (RTE_UA0_CTS_N_ID != 1)
#define IOMUX_GPIO_15_FMODE 2
#elif (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID == 1)
#define IOMUX_GPIO_15_FMODE 3
#else
#error Conflicted IO pin in IOMUX_GPIO_15_FMODE
#endif
#if 1 == IOMUX_GPIO_11_FMODE
  static bool level = false;
#endif
  
  if (true == enable) {
#if 11 != TZ1SM_HAL_GPIO_BLE_CTS
#warning "Please check for the H4 CTS GPIO number."
#endif
#if 1 == IOMUX_GPIO_11_FMODE
    tz1smHalGpioRead(TZ1SM_HAL_GPIO_BLE_CTS, &level);
#endif
    /* Setup of "!RTE_UART2_HW_FLOW" && "OUTPUT is UART_CTS". */
    gconf->FMODE_CFG3_b.UA2_CTS_N_FMODE = 3;
#if 1 == IOMUX_GPIO_11_FMODE
    /* OFF GPIO_11 not to change the current level. */
    gconf->FMODE_CFG0 = (IOMUX_GPIO_0_FMODE) | (IOMUX_GPIO_1_FMODE << 2)
      | (IOMUX_GPIO_2_FMODE   << 4) | (IOMUX_GPIO_3_FMODE  <<  6)
      | (IOMUX_GPIO_4_FMODE   << 8) | (IOMUX_GPIO_5_FMODE  << 10)
      | (IOMUX_GPIO_6_FMODE  << 12) | (IOMUX_GPIO_7_FMODE  << 14)
      | (IOMUX_GPIO_8_FMODE  << 16) | (IOMUX_GPIO_9_FMODE  << 18)
      | (IOMUX_GPIO_10_FMODE << 20) | (0 << 22)
      | (IOMUX_GPIO_12_FMODE << 24) | (IOMUX_GPIO_13_FMODE << 26)
      | (IOMUX_GPIO_14_FMODE << 28) | ((uint32_t)IOMUX_GPIO_15_FMODE << 30);
#endif
    cts_setup = tz1sm_gpio_cts_setup;
    Driver_GPIO.Configure(TZ1SM_HAL_GPIO_BLE_CTS, GPIO_DIRECTION_OUTPUT_2MA,
                          GPIO_EVENT_DISABLE, NULL);
    tz1smHalGpioWrite(TZ1SM_HAL_GPIO_BLE_CTS, false);
  } else { /* false == enable */
#if 1 == IOMUX_GPIO_11_FMODE
    gconf->FMODE_CFG0 = (IOMUX_GPIO_0_FMODE) | (IOMUX_GPIO_1_FMODE << 2)
      | (IOMUX_GPIO_2_FMODE   << 4) | (IOMUX_GPIO_3_FMODE  <<  6)
      | (IOMUX_GPIO_4_FMODE   << 8) | (IOMUX_GPIO_5_FMODE  << 10)
      | (IOMUX_GPIO_6_FMODE  << 12) | (IOMUX_GPIO_7_FMODE  << 14)
      | (IOMUX_GPIO_8_FMODE  << 16) | (IOMUX_GPIO_9_FMODE  << 18)
      | (IOMUX_GPIO_10_FMODE << 20) | (IOMUX_GPIO_11_FMODE << 22)
      | (IOMUX_GPIO_12_FMODE << 24) | (IOMUX_GPIO_13_FMODE << 26)
      | (IOMUX_GPIO_14_FMODE << 28) | ((uint32_t)IOMUX_GPIO_15_FMODE << 30);
    if (true == cts_setup.inheritance) {
      Driver_GPIO.Configure(TZ1SM_HAL_GPIO_BLE_CTS, cts_setup.dir,
                            cts_setup.event, cts_setup.cb_event);
      tz1smHalGpioWrite(TZ1SM_HAL_GPIO_BLE_CTS, level);
    }
#else
    Driver_GPIO.Configure(TZ1SM_HAL_GPIO_BLE_CTS, GPIO_DIRECTION_INPUT_HI_Z,
                          GPIO_EVENT_DISABLE, NULL);
#endif
    gconf->FMODE_CFG3_b.UA2_CTS_N_FMODE = IOMUX_UA2_CTS_N_FMODE;
  }
#endif
}

#else

/* V0.0.1 is no supporting the other UART Drivers other than
 * CMSIS_UART. Here are just the stab of UART drivers.
 */

tz1smHalStatus_t tz1smHalUartControl(uint32_t br, bool fc)
{
  return TZ1SM_HAL_STATUS_OK;
}
void tz1smHalUartInit(void) {;}
void tz1smHalUartUnInit(void) {;}
void tz1smHalUartTxBufFlush(void) {;}
void tz1smHalUartRxBufFlush(void) {;}
tz1smHalStatus_t tz1smHalUartPostData(const uint8_t * const data,
                                    const uint16_t length)
{
  return TZ1SM_HAL_STATUS_OK;
}
tz1smHalStatus_t tz1smHalUartSendData(const uint8_t * const data,
                                    const uint16_t length)
{
  return TZ1SM_HAL_STATUS_OK;
}
tz1smHalStatus_t tz1smHalUartPeekData(uint8_t * const data,
                                    uint16_t * const receive_length)
{
  return TZ1SM_HAL_STATUS_OK;
}
void tz1smHalUartGetData(void) {;}

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
void tz1smHalUartLowPower(bool enable) { return; }
#endif

void tz1smHalSuppressHpd(const bool enable) {;}

#endif
