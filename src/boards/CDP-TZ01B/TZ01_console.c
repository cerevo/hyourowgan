/**
 * @file   TZ01_console.c
 * @brief  Serial console library for Cerevo CDP-TZ01B.
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
#include "Driver_UART.h"
/* Board support. */
#include "TZ01_system.h"
#include "TZ01_console.h"

#if TZ01_CONSOLE_ENABLE

#define PUTS_LEN_MAX    (120)

extern TZ10XX_DRIVER_PMU  Driver_PMU;
#if TZ01_CONSOLE_UART_CH == 0
    /* UART0 */
    extern ARM_DRIVER_UART Driver_UART0;
    ARM_DRIVER_UART *tz10xx_drv_uart = &Driver_UART0;
    #define PMU_CSM_UART    PMU_CSM_UART0
    #define PMU_CD_UART     PMU_CD_UART0
    /*-------*/
#elif TZ01_CONSOLE_UART_CH == 1
    /* UART1 */
    extern ARM_DRIVER_UART Driver_UART1;
    ARM_DRIVER_UART *tz10xx_drv_uart = &Driver_UART1;
    #define PMU_CSM_UART    PMU_CSM_UART1
    #define PMU_CD_UART     PMU_CD_UART1
    /*------*/
#else
    #error Invalid UART ch.
#endif

bool TZ01_console_init(void)
{
    /* PMU */
    Driver_PMU.SelectClockSource(PMU_CSM_UART, PMU_CLOCK_SOURCE_OSC12M);
    Driver_PMU.SetPrescaler(PMU_CD_UART, 1);
    /* UART */
    tz10xx_drv_uart->Initialize(0, 0);
    tz10xx_drv_uart->Configure(TZ01_CONSOLE_BAUD, 8, ARM_UART_PARITY_NONE, ARM_UART_STOP_BITS_1, ARM_UART_FLOW_CONTROL_NONE);
    tz10xx_drv_uart->PowerControl(ARM_POWER_FULL);

    return true;
}

bool TZ01_console_getc(uint8_t *c)
{
    uint8_t val;
    
    if (c == NULL) {
        return false;
    }
    
    if (tz10xx_drv_uart->DataAvailable() == false) {
         return false;
    }
    
    if (tz10xx_drv_uart->ReadData(&val, 1) != 1) {
        return false;
    }

    *c = val;
    return true;
}

bool TZ01_console_putc(uint8_t c)
{
    if (tz10xx_drv_uart->WriteData(&c, 1) != 1) {
        return false;
    }
    return true;
}

int TZ01_console_gets(char *buf, int max_len)
{
    int i;
    int ret;
    uint8_t val;
    
    if (buf == NULL) {
        return -1;
    }
    
    if (max_len < 1) {
        return -1;
    }
    
    for (i = 0; i < max_len; i++) {
        if (tz10xx_drv_uart->DataAvailable() > 0) {
            ret = tz10xx_drv_uart->ReadData(&val, 1);
            if (ret == -1) {
                return false;
            }
            buf[i] = (char)val;
            
            if (val == '\0') {
                break;
            }
        } else {
            break;
        }
    }
    
    return i;
}

int TZ01_console_puts(char *str)
{
    int len, total_len;
    int ret;
    
    if (str == NULL) {
        return -1;
    }
    
    len = 0;
    while (str[len++] != '\0') {
        if (len > 128) {
            return -1;
        }
    }
    
    total_len = len;
    while (len > 0) {
        ret = tz10xx_drv_uart->WriteData((uint8_t *)str, len);
        if (ret == -1) {
            return -1;
        } else {
            str += ret;
            len -= ret;
        }        
    }
    
    return total_len;
}

int TZ01_console_read(uint8_t *buf, int len)
{
    if (buf == NULL) {
        return -1;
    }
    
    if (len < 1) {
        return -1;
    }
    
    return tz10xx_drv_uart->ReadData(buf, len);
}

int TZ01_console_write(uint8_t *buf, int len)
{
    if (buf == NULL) {
        return -1;
    }
    
    if (len < 1) {
        return -1;
    }
    
    return tz10xx_drv_uart->WriteData(buf, len);
}
#endif  /* TZ01_CONSOLE_ENABLE */
