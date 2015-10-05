/**
 * @file twic_util_lemng.h
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0 Smart
 * @version V1.0.0.FS (Free Sample - The information in this code is
 * subject to change without notice and should not be construed as a
 * commitment by TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS
 * COMPANY.
 * @note TZ1EM provides the automatic low energy consumption. The low
 * energy consumption of interal BLE Processor and UART2 is managed by
 * the part of TWIC BLE CE (BLE Controller Extension). Please refer to
 * the twicIfLeCe API group. TZ1EM combines the HOST Core low energy
 * consumption with the BLE CE and the other peripheral modules.
 */

/*
 * COPYRIGHT (C) 2015
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

#ifndef __TWIC_UTIL_LEMNG_H__
#define __TWIC_UTIL_LEMNG_H__

#include "twic_interface.h"

extern void twicUtPrintVal(const char *name, const uint8_t len,
                           const uint8_t * value);
extern void twicUtPrintBdAddr(const uint8_t * const addr);
extern void twicUtDisplayTwicErrorList(void);

extern void twicUtDoEvent(void);
extern bool twicUtPeekInApiWithValue(twicConnIface_t *cif, uint8_t fidx,
                                     uint8_t *aret, uint8_t (*const data)[23]);
extern bool twicUtPeekInApi(twicConnIface_t *cif, uint8_t fidx, uint8_t *aret);
extern bool twicUtDbPeekInApi(twicConnIface_t *cif, uint8_t fidx, uint8_t eidx,
                              uint8_t *aret);
extern bool twicUtCheckAndDoEvent(twicStatus_t status);
extern twicStatus_t twicUtLeDiscoverable(twicConnIface_t * const cif,
                                         uint16_t min_interval,
                                         uint16_t max_interval,
                                         uint8_t advertising_type,
                                         uint8_t own_address_type,
                                         uint8_t direct_address_type,
                                         uint64_t direct_address,
                                         uint8_t advertising_channel_map,
                                         uint8_t advertising_filter_policy);
extern twicStatus_t twicUtLeConnectionUpdate(twicConnIface_t * const cif,
                                             uint16_t conn_int_min,
                                             uint16_t conn_int_max,
                                             uint16_t slave_latency,
                                             uint16_t supervison_timeout);
extern twicStatus_t
twicUtGattServerWriteCharacteristics(twicConnIface_t * const cif,
                                     const uint8_t cha, const uint16_t length,
                                     const uint8_t * const value);
extern twicStatus_t
twicUtGattClientExgMtu(twicConnIface_t * const cif, const uint16_t rx_mtu_size);

extern twicStatus_t twicUtGattServerExgMtuResponse(twicConnIface_t * const cif,
                                                   const uint16_t rx_mtu_size);
extern twicStatus_t
twicUtGattServerCharValMultiReadOutResponse(twicConnIface_t * const cif,
                                            uint8_t eidx, uint8_t code);
extern twicStatus_t
twicUtGattServerCharValReadOutResponse(twicConnIface_t * const cif,
                                       uint8_t eidx, uint8_t code);
extern twicStatus_t
twicUtGattServerCharDespReadOutResponse(twicConnIface_t * const cif,
                                        uint8_t eidx);
extern twicStatus_t
twicUtGattServerCharValWriteInResponse(twicConnIface_t * const cif,
                                       uint8_t eidx, uint8_t code);
extern twicStatus_t
twicUtGattServerCharDespWriteInResponse(twicConnIface_t * const cif,
                                        uint8_t eidx);
extern uint8_t twicUtGattNotification(twicConnIface_t * const cif,
                                      uint8_t eidx, uint8_t *data, uint8_t len);
extern uint8_t twicUtGattIndication(twicConnIface_t * const cif,
                                    uint8_t eidx, uint8_t *data, uint8_t len);
extern twicStatus_t twicUtLeStopAdvertising(twicConnIface_t * const cif);
extern twicStatus_t twicUtLeDisconnect(twicConnIface_t * const cif,
                                       const twicBdaddr_t * const bd);
extern twicStatus_t twicUtLeCeLowPowerMode(twicConnIface_t * const cif,
                                           bool conn_scan, bool deep);
extern twicStatus_t twicUtLeSetScanEnable(twicConnIface_t * const cif,
                                          uint16_t interval, uint16_t window,
                                          bool active, bool random,
                                          bool whitelist, bool duplicates);
extern twicStatus_t twicUtLeSetScanDisable(twicConnIface_t * const cif);
extern twicStatus_t twicUtLeCreateConnection(twicConnIface_t * const cif,
                                             uint16_t interval, uint16_t window,
                                             bool whitelist, bool peer,
                                             twicBdaddr_t * addr, uint16_t min,
                                             uint16_t max, uint16_t latency,
                                             uint16_t timeout, bool own);
extern twicStatus_t twicUtLeCreateConnectionCancel(twicConnIface_t * const cif);
extern twicStatus_t twicUtLeCeLowPower(twicConnIface_t * const cif,
                                       bool control, bool conn, bool shutdown);

#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||            \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) || \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                 \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||          \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID) ||       \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS) ||              \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||             \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||         \
  defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE) ||            \
  defined(TWIC_API_GATTCLIENTRELIABLEWRITE) ||                       \
  defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE) ||                \
  defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE) ||          \
  defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE) ||      \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR) ||        \
  defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES) ||              \
  defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR) ||       \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||         \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR) ||    \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE) ||        \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
extern twicIfLeClientCb_t twicUtCcb;
extern void twicUtGattClientCleanup(void);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
extern twicStatus_t
twicUtGattDiscoverPrimaryServiceByServiceUuid(twicConnIface_t * const cif,
                                              uint16_t start_handle,
                                              uint16_t end_handle,
                                              uint64_t uuid_lsb,
                                              uint64_t uuid_msb,
                                              uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
twicStatus_t
twicUtGattClientDiscoverAllCharacteristics(twicConnIface_t * const cif,
                                           uint16_t start_handle,
                                           uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
twicStatus_t twicUtGattClientDiscoverAllDescriptors(twicConnIface_t * const cif,
                                                    uint16_t start_handle,
                                                    uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
twicStatus_t
twicUtGattClientWriteCharacteristicDescriptor(twicConnIface_t * const cif,
                                              uint16_t handle, uint8_t length,
                                              uint8_t * const value);
#endif

#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
twicStatus_t
twicUtGattClientWriteCharacteristicValue(twicConnIface_t * const cif,
                                         uint16_t handle, uint8_t length,
                                         uint8_t * const value);
#endif

#endif /* __TWIC_UTIL_LEMNG_H__ */
