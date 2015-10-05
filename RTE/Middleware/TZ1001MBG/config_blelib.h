/**
 * @file config_blelib.h
 * @brief configuration for BLELib
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

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#include "twic_interface.h"

#ifndef _CONFIG_BLELIB_H_
#define _CONFIG_BLELIB_H_

#ifdef __cplusplus
extern "C" {
#endif

// <h> BLELib configuration
//   <q> enable debug print
#define BLELIB_DEBUG_PRINT (0)

//   <o> maximum attribute number <1-255>
#define BLELIB_ENTRY_SIZE_MAX 	(40)
#if (BLELIB_ENTRY_SIZE_MAX < 1) || (BLELIB_ENTRY_SIZE_MAX > 255)
#error BLELIB_ENTRY_SIZE_MAX must be in range from 1 to 255
#endif
	
//   <o> inner buffer number <1-255>
#define BLELIB_INNER_BUF_NUM (8)
#if (BLELIB_INNER_BUF_NUM < 1) || (BLELIB_INNER_BUF_NUM > 255)
#error BLELIB_INNER_BUF_NUM must be in range from 1 to 255
#endif

//	<o>	multiple read maximum number <1-255>
#define BLELIB_MULTIPLE_READ_MAX_NUM	(10)
#if (BLELIB_MULTIPLE_READ_MAX_NUM < 1) || (BLELIB_MULTIPLE_READ_MAX_NUM > 255)
#error BLELIB_MULTIPLE_READ_MAX_NUM must be in range from 1 to 255
#endif

//   <o> punctuation of UART2(us) <3-8192>
#define BLELIB_UART2_PUNCTUATION (12)
#if (BLELIB_UART2_PUNCTUATION < 3) || (BLELIB_UART2_PUNCTUATION > 8192)
#error BLELIB_UART2_PUNCTUATION must be in range from 3 to 8192
#endif
// </h>

// <h> Advertising parameter
//   <o> minimum interval (n * 0.625ms) <0x20-0x4000>
#define BLELIB_MIN_ADVERTISING_INTERVAL (0x0320)
#if (BLELIB_MIN_ADVERTISING_INTERVAL < 0x20) || (BLELIB_MIN_ADVERTISING_INTERVAL > 0x4000)
#error BLELIB_MIN_ADVERTISING_INTERVAL must be in range from 0x20 to 0x4000
#endif
//   <o> maximum interval (n * 0.625ms) <0x20-0x4000>
#define BLELIB_MAX_ADVERTISING_INTERVAL (0x0320)
#if (BLELIB_MAX_ADVERTISING_INTERVAL < 0x20) || (BLELIB_MAX_ADVERTISING_INTERVAL > 0x4000)
#error BLELIB_MAX_ADVERTISING_INTERVAL must be in range from 0x20 to 0x4000
#endif
//   <o> Advertising type
//     <0=> Connectable advertising
//     <2=> Scannable advertising
//     <3=> Non connectable advertising
#define BLELIB_ADVERTISING_TYPE_NUM (0)
#if BLELIB_ADVERTISING_TYPE_NUM == 0
#define BLELIB_ADVERTISING_TYPE TWIC_ADV_TYPE_IND
#elif BLELIB_ADVERTISING_TYPE_NUM == 2
#define BLELIB_ADVERTISING_TYPE TWIC_ADV_TYPE_SCAN_IND
#elif BLELIB_ADVERTISING_TYPE_NUM == 3
#define BLELIB_ADVERTISING_TYPE TWIC_ADV_TYPE_NONCONN_IND
#else 
#error Not supported Advertising type
#endif
//   <h> Advertising channel
//     <q> Channel 37
#define BLELIB_ADVERTISING_CHANNEL_37 (1)
//     <q> Channel 38
#define BLELIB_ADVERTISING_CHANNEL_38 (1)
//     <q> Channel 39
#define BLELIB_ADVERTISING_CHANNEL_39 (1)
//   </h>
#if BLELIB_ADVERTISING_CHANNEL_37 == 0 && BLELIB_ADVERTISING_CHANNEL_38 == 0 && BLELIB_ADVERTISING_CHANNEL_39 == 0
#error At least one channel must be specified
#endif
// </h>
	
	
// <h> Connection parameter for low power on
//   <o> minimum connection interval (n * 1.25ms) <6-3200>
#define BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_ON (40)
#if (BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_ON < 6) || (BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_ON > 3200)
#error BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_ON must be in range from 6 to 3200
#endif
//   <o> maximum connection interval (n * 1.25ms) <6-3200>
#define BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_ON (100)
#if (BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_ON < 6) || (BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_ON > 3200)
#error BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_ON must be in range from 6 to 3200
#endif
//   <o> slave latency <0-499>
#define BLELIB_SLAVE_LATENCY_LOWPOWER_ON (4)
#if (BLELIB_SLAVE_LATENCY_LOWPOWER_ON < 0) || (BLELIB_SLAVE_LATENCY_LOWPOWER_ON > 499)
#error BLELIB_SLAVE_LATENCY_LOWPOWER_ON must be in range from 0 to 499
#endif
//   <o> supervision timeout (n * 10ms) <10-3200>
#define BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_ON (400)
#if (BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_ON < 10) || (BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_ON > 3200)
#error BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_ON must be in range from 10 to 3200
#endif
//   <o> minimum CE length (n * 0.625ms) <0x0-0xffff>
#define BLELIB_MIN_CE_LENGTH_LOWPOWER_ON (0x20)
#if (BLELIB_MIN_CE_LENGTH_LOWPOWER_ON < 0) || (BLELIB_MIN_CE_LENGTH_LOWPOWER_ON > 0xffff)
#error BLELIB_MIN_CE_LENGTH_LOWPOWER_ON must be in range from 0x0000 to 0xffff
#endif
//   <o> maximum CE length (n * 0.625ms) <0x0-0xffff>
#define BLELIB_MAX_CE_LENGTH_LOWPOWER_ON (0x30)
#if (BLELIB_MAX_CE_LENGTH_LOWPOWER_ON < 0) || (BLELIB_MAX_CE_LENGTH_LOWPOWER_ON > 0xffff)
#error BLELIB_MAX_CE_LENGTH_LOWPOWER_ON must be in range from 0x0000 to 0xffff
#endif
// </h>

// <h> Connection parameter for low power off
//   <o> minimum connection interval (n * 1.25ms) <6-3200>
#define BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_OFF (6)
#if (BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_OFF < 6) || (BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_OFF > 3200)
#error BLELIB_MIN_CONNECTION_INTERVAL_LOWPOWER_OFF must be in range from 6 to 3200
#endif
//   <o> maximum connection interval (n * 1.25ms) <6-3200>
#define BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_OFF (9)
#if (BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_OFF < 6) || (BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_OFF > 3200)
#error BLELIB_MAX_CONNECTION_INTERVAL_LOWPOWER_OFF must be in range from 6 to 3200
#endif
//   <o> slave latency <0-499>
#define BLELIB_SLAVE_LATENCY_LOWPOWER_OFF (4)
#if (BLELIB_SLAVE_LATENCY_LOWPOWER_OFF < 0) || (BLELIB_SLAVE_LATENCY_LOWPOWER_OFF > 499)
#error BLELIB_SLAVE_LATENCY_LOWPOWER_OFF must be in range from 0 to 499
#endif
//   <o> supervision timeout (n * 10ms) <10-3200>
#define BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_OFF (400)
#if (BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_OFF < 10) || (BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_OFF > 3200)
#error BLELIB_SUPERVISION_TIMEOUT_LOWPOWER_OFF must be in range from 10 to 3200
#endif
//   <o> minimum CE length (n * 0.625ms) <0x0-0xffff>
#define BLELIB_MIN_CE_LENGTH_LOWPOWER_OFF (0x20)
#if (BLELIB_MIN_CE_LENGTH_LOWPOWER_OFF < 0) || (BLELIB_MIN_CE_LENGTH_LOWPOWER_OFF > 0xffff)
#error BLELIB_MIN_CE_LENGTH_LOWPOWER_OFF must be in range from 0x0000 to 0xffff
#endif
//   <o> maximum CE length (n * 0.625ms) <0x0-0xffff>
#define BLELIB_MAX_CE_LENGTH_LOWPOWER_OFF (0x30)
#if (BLELIB_MAX_CE_LENGTH_LOWPOWER_OFF < 0) || (BLELIB_MAX_CE_LENGTH_LOWPOWER_OFF > 0xffff)
#error BLELIB_MAX_CE_LENGTH_LOWPOWER_OFF must be in range from 0x0000 to 0xffff
#endif
// </h>

// <h> Scan parameter
//   <o> scan interval (n * 0.625ms) <0x4-0x4000>
#define BLELIB_SCAN_INTERVAL	(0x2000)
#if (BLELIB_SCAN_INTERVAL < 0x4) || (BLELIB_SCAN_INTERVAL > 0x4000)
#error BLELIB_SCAN_INTERVAL must be in range from 0x4 to 0x4000
#endif
//   <o> scan window (n * 0.625ms) <0x4-0x4000>
#define BLELIB_SCAN_WINDOW (0x2000)
#if (BLELIB_SCAN_WINDOW < 0x4) || (BLELIB_SCAN_WINDOW > 0x4000)
#error BLELIB_SCAN_WINDOW must be in range from 0x4 to 0x4000
#endif
// </h>

// <h> SMP parameter
//   <o> IO capability
//     <0=> Display Only
//     <1=> Display Yes No
//     <2=> Keyboard Only
//     <3=> No Input No Output
//     <4=> Keyboard Display
#define BLELIB_IO_CAPABILITY (3)
#if (BLELIB_IO_CAPABILITY < 0) || (BLELIB_IO_CAPABILITY > 4)
#error BLELIB_IO_CAPABILITY must be in range from 0 to 4
#endif
//   <q> OOB authentication data is available
#define BLELIB_OOB_DATA_FLAG (0)
//   <q> Bonding request
#define BLELIB_AUTH_REQ_BONDING (1)
//   <q> MITM protection request
#define BLELIB_AUTH_REQ_MITM (0)
//   <o> Maximum encode key size <7-16>
#define BLELIB_MAX_ENCODE_KEY_SIZE (7)
#if (BLELIB_MAX_ENCODE_KEY_SIZE < 7) || (BLELIB_MAX_ENCODE_KEY_SIZE > 16)
#error BLELIB_MAX_ENCODE_KEY_SIZE must be in range from 7 to 16
#endif
//   <q> Initiator distributes enckey
#define BLELIB_INITIATOR_DISTRIBUTES_ENCKEY (1)
//   <q> Initiator distributes idkey
#define BLELIB_INITIATOR_DISTRIBUTES_IDKEY (1)
//   <q> Initiator distributes sign
#define BLELIB_INITIATOR_DISTRIBUTES_SIGN (1)
//   <q> Responder distributes enckey
#define BLELIB_RESPONDER_DISTRIBUTES_ENCKEY (1)
//   <q> Responder distributes idkey
#define BLELIB_RESPONDER_DISTRIBUTES_IDKEY (1)
//   <q> Responder distributes sign
#define BLELIB_RESPONDER_DISTRIBUTES_SIGN (1)
// </h>

// <<< end of configuration section >>>

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_BLELIB_H_ */
