/**
 * @file   TZ01_console.h
 * @brief  Serial console library for Cerevo CDP-TZ01B.
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

#ifndef _TZ01_CONSOLE_H_
#define _TZ01_CONSOLE_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief       Initialize serial console
 * @return      Initialize result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool TZ01_console_init(void);

/**
 * @brief       Get character from serial console.
 * @param[out]  c Got character.
 * @return      Result
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool TZ01_console_getc(uint8_t *c);

/**
 * @brief       Put character to serial console.
 + @param[in]   c   character code.
 * @return      Result.
 * @retval      true    Success.
 * @retval      false   Failed.
 */
bool TZ01_console_putc(uint8_t c);

/**
 * @brief       Get string from serial console.
 * @param[out]  buf     string buffer.
 * @param[in]   max_len Max string length.
 * @return      Number of characters actually read.
 */
int TZ01_console_gets(char *buf, int max_len);

/**
 * @brief       Put String to serial console.
 * @param[in]   buf String buffer.
 * @note        `buf' should have been null terminated.
 * @return      Number of byte written.
 */
int TZ01_console_puts(char *buf);

/**
 * @brief       Read byte data from serial console.
 * @param[out]  buf Buffer pointer.
 * @param[in]   len Read length.
 * @return      Number of bytes actually read.
 */
int TZ01_console_read(uint8_t *buf, int len);

/**
 * @brief       Write byte data to serial console.
 * @param[in]   buf Buffer pointer.
 * @param[in]   len Write data length.
 * @return      Number of byte written.
 */
int TZ01_console_write(uint8_t *buf, int len);

#endif  //_TZ01_CONSOLE_H_
