/**
 * @file twic_util_lemng.c
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

#include "twic_interface.h"
#include "twic_util_macro.h"
#include "twic_util_service.h"
#include "twic_util_advertise.h"
#include "twic_util_lemng.h"

void twicUtPrintVal(const char *name, const uint8_t len, const uint8_t * value)
{
  uint8_t i;
  twicPrintf("%s : 0x ", name);
  for (i = 0; i < len; i++) twicPrintf("%02x ", value[i]);
  twicPrintf("\r\n");
}

void twicUtPrintBdAddr(const uint8_t * const addr)
{
  twicPrintf("bdaddr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

void twicUtDisplayTwicErrorList(void)
{
  twicPrintf("TWIC_STATUS_OK = 0x%x.\r\n", TWIC_STATUS_OK);
  twicPrintf("TWIC_STATUS_EVENT_SIGNAL = 0x%x.\r\n",
             TWIC_STATUS_EVENT_SIGNAL);
  twicPrintf("TWIC_STATUS_EVENT_MESSAGE = 0x%x.\r\n",
             TWIC_STATUS_EVENT_MESSAGE);
  twicPrintf("TWIC_STATUS_EVENT_MAIL = 0x%x.\r\n",
             TWIC_STATUS_EVENT_MAIL);
  twicPrintf("TWIC_STATUS_EVENT_TIMEOUT = 0x%x.\r\n",
             TWIC_STATUS_EVENT_TIMEOUT);
  twicPrintf("TWIC_STATUS_ERROR_PARAMETER = 0x%x.\r\n",
             TWIC_STATUS_ERROR_PARAMETER);
  twicPrintf("TWIC_STATUS_ERROR_RESOURCE = 0x%x.\r\n",
             TWIC_STATUS_ERROR_RESOURCE);
  twicPrintf("TWIC_STATUS_ERROR_TIMEOUT_RESOURCE = 0x%x.\r\n",
             TWIC_STATUS_ERROR_TIMEOUT_RESOURCE);
  twicPrintf("TWIC_STATUS_ERROR_ISR = 0x%x.\r\n",
             TWIC_STATUS_ERROR_ISR);
  twicPrintf("TWIC_STATUS_ERROR_ISR_RECURSIVE = 0x%x.\r\n",
             TWIC_STATUS_ERROR_ISR_RECURSIVE);
  twicPrintf("TWIC_STATUS_ERROR_PRIORITY = 0x%x.\r\n",
             TWIC_STATUS_ERROR_PRIORITY);
  twicPrintf("TWIC_STATUS_ERROR_NOMEMORY = 0x%x.\r\n",
             TWIC_STATUS_ERROR_NOMEMORY);
  twicPrintf("TWIC_STATUS_ERROR_VALUE = 0x%x.\r\n",
             TWIC_STATUS_ERROR_VALUE);
  twicPrintf("TWIC_STATUS_ERROR_OS = 0x%x.\r\n",
             TWIC_STATUS_ERROR_OS);
  twicPrintf("TWIC_STATUS_SOURCE_STOPPED = 0x%x.\r\n",
             TWIC_STATUS_SOURCE_STOPPED);
  twicPrintf("TWIC_STATUS_SOURCE_PREPARING = 0x%x.\r\n",
             TWIC_STATUS_SOURCE_PREPARING);
  twicPrintf("TWIC_STATUS_SOURCE_RUNNING = 0x%x.\r\n",
             TWIC_STATUS_SOURCE_RUNNING);
  twicPrintf("TWIC_STATUS_ERROR_IO = 0x%x.\r\n",
             TWIC_STATUS_ERROR_IO);
  twicPrintf("TWIC_STATUS_ERROR_DATA = 0x%x.\r\n",
             TWIC_STATUS_ERROR_DATA);
  twicPrintf("TWIC_STATUS_REQUEST_HANDLE = 0x%x.\r\n",
             TWIC_STATUS_REQUEST_HANDLE);
  twicPrintf("TWIC_STATUS_UNDER_PROCESSING = 0x%x.\r\n",
             TWIC_STATUS_UNDER_PROCESSING);
  twicPrintf("TWIC_STATUS_COMPLETION = 0x%x.\r\n",
             TWIC_STATUS_COMPLETION);
  twicPrintf("TWIC_STATUS_OPERATION_NOT_PERMITTED = 0x%x.\r\n",
             TWIC_STATUS_OPERATION_NOT_PERMITTED);
  twicPrintf("TWIC_STATUS_DISCONNECTED = 0x%x.\r\n",
             TWIC_STATUS_DISCONNECTED);
  twicPrintf("TWIC_STATUS_COMMAND_ELIMINATION = 0x%x.\r\n",
             TWIC_STATUS_COMMAND_ELIMINATION);
  twicPrintf("TWIC_STATUS_WAITING_FOR_ACTIVATION = 0x%x.\r\n",
             TWIC_STATUS_WAITING_FOR_ACTIVATION);
  twicPrintf("TWIC_STATUS_ERROR_HAL = 0x%x.\r\n",
             TWIC_STATUS_ERROR_HAL);
  twicPrintf("TWIC_STATUS_ERROR_TZ1EM = 0x%x.\r\n",
             TWIC_STATUS_ERROR_TZ1EM);
  twicPrintf("TWIC_STATUS_RESERVED = 0x%x.\r\n",
             TWIC_STATUS_RESERVED);
}

void twicUtDoEvent(void)
{
  if (TWIC_STATUS_EVENT_MESSAGE == twicIfPeekEvent()) twicIfDoEvents();
  tz1smHalOsYeild();
}

bool twicUtPeekInApiWithValue(twicConnIface_t *cif, uint8_t fidx,
                              uint8_t *aret, uint8_t (*const data)[23])
{
  uint8_t _fidx;
  twicStatus_t status;
  
  *aret = 0;
  twicIfLeCeHkTimeout(60000, fidx);
  status = twicIfIsDone(cif, &_fidx);
  if ((TWIC_STATUS_EVENT_MESSAGE == status) && (_fidx == fidx)) {
    if (TWIC_STATUS_OK == twicIfAcceptance(cif, aret, data)) return true;
  }
  return false;
}

bool twicUtPeekInApi(twicConnIface_t *cif, uint8_t fidx, uint8_t *aret)
{
  uint8_t _fidx;
  twicStatus_t status;
  
  *aret = 0;
  twicIfLeCeHkTimeout(60000, fidx);
  status = twicIfIsDone(cif, &_fidx);
  if ((TWIC_STATUS_EVENT_MESSAGE == status) && (_fidx == fidx)) {
    if (TWIC_STATUS_OK == twicIfAcceptance(cif, aret, NULL)) return true;
  }
  return false;
}

bool twicUtDbPeekInApi(twicConnIface_t *cif, uint8_t fidx, uint8_t eidx,
                       uint8_t *aret)
{
  uint8_t _fidx, _eidx;
  twicStatus_t status;
  
  *aret = 0;
  twicIfLeCeHkTimeout(60000, fidx);
  status = twicIfDbIsDone(cif, &_fidx, &_eidx, 0, 0);
  if ((TWIC_STATUS_EVENT_MESSAGE == status) && 
      (_fidx == fidx) && (_eidx == eidx)) {
    if (TWIC_STATUS_OK == twicIfDbAcceptance(cif, eidx, aret)) return true;
  }
  return false;
}

bool twicUtCheckAndDoEvent(twicStatus_t status)
{
  if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status &&
      TWIC_STATUS_WAITING_FOR_ACTIVATION != status) return false;
  twicUtDoEvent();
  return true;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.5 LE Set Advertising Parameters Command.
 * 7.8.9 LE Set Advertise Enable Command.
 * @brief LE Set Advertise enable.
 * This API is used to request the Controller to start advertising.
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t min_interval This is equivalent to the
 * Advertising_Interval_Min.
 * @param uint16_t max_interval This is equivalent to the
 * Advertising_Interval_Max.
 * @param uint8_t advertising_type This is equivalent to the Advertising_Type.
 * TWIC_ADV_TYPE_IND         :Connectable undirected advertising.
 * TWIC_ADV_TYPE_DIRECT_IND  :Connectable directed advertising.
 * TWIC_ADV_TYPE_SCAN_IND    :Scannable undirected advertising.
 * TWIC_ADV_TYPE_NONCONN_IND :Non connectable undirected advertising.
 * @param uint8_t own_address_type This is equivalent to the Own_Address_Type.
 * TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS (0)
 * TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS (1)
 * @param uint8_t direct_address_type This is equivalent to the
 * Direct_Address_Type.
 * TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS (0)
 * TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS (1)
 * @param uint64_t direct_address This is equivalent to the Direct_Address.
 * @param uint8_t advertising_channel_map This is equivalent to the
 * Advertising_Channel_Map.
 * TWIC_ADV_CHANNEL_37  (0x01) xxxxxxx1b Enable channel 37 use
 * TWIC_ADV_CHANNEL_38  (0x02) xxxxxx1xb Enable channel 38 use
 * TWIC_ADV_CHANNEL_39  (0x04) xxxxx1xxb Enable channel 39 use
 * TWIC_ADV_CHANNEL_ALL (0x07) 00000111b Default (all channels enabled)
 * @param uint8_t advertising_filter_policy This is equivalent to the
 * Advertising_Filter_Policy.
 * TWIC_ADV_FILTER_ANY_ANY     (0)
 * Allow Scan Request from Any, Allow Connect Request from Any.
 * TWIC_ADV_FILTER_WHITE_ANY   (1)
 * Allow Scan Request from White List Only, Allow Connect Request from Any.
 * TWIC_ADV_FILTER_ANY_WHITE   (2)
 * Allow Scan Request from Any, Allow Connect Request from White List Only.
 * TWIC_ADV_FILTER_WHITE_WHITE (3)
 * Allow Scan Request from White List Only, Allow Connect Request from
 * White List Only. */
#if defined(TWIC_UTIL_GA_SERVICE) || defined(TWIC_UTIL_DI_SERVICE) || \
  defined(TWIC_UTIL_HR_SERVICE) || defined(TWIC_UTIL_BP_SERVICE) ||   \
  defined(TWIC_UTIL_UD_SERVICE) || defined(TWIC_UTIL_CT_SERVICE) ||   \
  defined(TWIC_UTIL_NC_SERVICE) || defined(TWIC_UTIL_RU_SERVICE) ||   \
  defined(TWIC_UTIL_IA_SERVICE) || defined(TWIC_UTIL_HT_SERVICE) ||   \
  defined(TWIC_UTIL_UU_SERVICE)
twicStatus_t twicUtLeDiscoverable(twicConnIface_t * const cif,
                                  uint16_t min_interval, uint16_t max_interval,
                                  uint8_t advertising_type,
                                  uint8_t own_address_type,
                                  uint8_t direct_address_type,
                                  uint64_t direct_address,
                                  uint8_t advertising_channel_map,
                                  uint8_t advertising_filter_policy)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LEDISCOVERABLE, &_ar) != true;) {
    status = twicIfLeDiscoverable(cif, min_interval, max_interval,
                                  advertising_type, own_address_type,
                                  direct_address_type, direct_address,
                                  advertising_channel_map,
                                  advertising_filter_policy,
                                  sizeof(twic_util_advertising_data),
                                  twic_util_advertising_data,
                                  sizeof(twic_util_scan_resp_data),
                                  twic_util_scan_resp_data);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
#endif

twicStatus_t twicUtLeConnectionUpdate(twicConnIface_t * const cif,
                                      uint16_t conn_int_min,
                                      uint16_t conn_int_max,
                                      uint16_t slave_latency,
                                      uint16_t supervison_timeout)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LECONNECTIONUPDATE, &_ar) != true;) {
    status = twicIfLeConnectionUpdate(cif, conn_int_min, conn_int_max,
                                      slave_latency, supervison_timeout,
                                      0x20, 0x30);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerWriteCharacteristics(
  twicConnIface_t * const cif,
  const uint8_t cha, const uint16_t length, const uint8_t * const value)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTSERVERWRITECHARACTERISTICS, &_ar) != true;) {
    status = twicIfGattServerWriteCharacteristics(cif, cha, length, value);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t
twicUtGattClientExgMtu(twicConnIface_t * const cif, const uint16_t rx_mtu_size)
{
#if defined(TWIC_API_GATTCLIENTEXGMTU)
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTCLIENTEXGMTU, &_ar) != true;) {
    status = twicIfGattClientExgMtu(cif, rx_mtu_size);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
#else
  return TWIC_STATUS_OPERATION_NOT_PERMITTED;
#endif
}

twicStatus_t twicUtGattServerExgMtuResponse(
  twicConnIface_t * const cif, const uint16_t rx_mtu_size)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTSERVEREXGMTURESPONSE, &_ar) != true;) {
    status = twicIfGattServerExgMtuResponse(cif, 0, rx_mtu_size);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerCharValMultiReadOutResponse(
  twicConnIface_t * const cif, uint8_t eidx, uint8_t code)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTSERVERCHARVALMULTIREADOUTRESPONSE,
                        &_ar) != true;) {
    status = twicIfGattServerCharValMultiReadOutResponse(cif, code, eidx);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerCharValReadOutResponse(
  twicConnIface_t * const cif, uint8_t eidx, uint8_t code)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTSERVERCHARVALREADOUTRESPONSE, &_ar) != true;) {
    status = twicIfGattServerCharValReadOutResponse(cif, eidx, code);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerCharDespReadOutResponse(
  twicConnIface_t * const cif, uint8_t eidx)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTSERVERCHARDESPREADOUTRESPONSE, &_ar) != true;) {
    status = twicIfGattServerCharDespReadOutResponse(cif, eidx, 0);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerCharValWriteInResponse(
  twicConnIface_t * const cif, uint8_t eidx, uint8_t code)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTSERVERCHARVALWRITEINRESPONSE, &_ar) != true;) {
    status = twicIfGattServerCharValWriteInResponse(cif, eidx, code);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t twicUtGattServerCharDespWriteInResponse(
  twicConnIface_t * const cif, uint8_t eidx)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTSERVERCHARDESPWRITEINRESPONSE, &_ar) != true;) {
    status = twicIfGattServerCharDespWriteInResponse(cif, eidx, 0);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

uint8_t twicUtGattNotification(
  twicConnIface_t * const cif, uint8_t eidx, uint8_t *data, uint8_t len)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTNOTIFICATION, &_ar) != true;) {
    status = twicIfGattNotification(cif, eidx, len, data);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  return _ar;
}

uint8_t twicUtGattIndication(
  twicConnIface_t * const cif, uint8_t eidx, uint8_t *data, uint8_t len)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTINDICATION, &_ar) != true;) {
    status = twicIfGattIndication(cif, eidx, len, data);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  return _ar;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.1.6 Disconnect Command.
 *
 * @brief Terminate an existing connection.
 * This API is used to terminate an existing connection.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface. */
twicStatus_t twicUtLeStopAdvertising(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LESTOPADVERTISING, &_ar) != true;) {
    status = twicIfLeStopAdvertising(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.9 LE Set Advertise Enable Command.
 *
 * @brief LE Set Advertise disable.
 * This API is used to request the Controller to stop advertising.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface. */
twicStatus_t
twicUtLeDisconnect(twicConnIface_t * const cif, const twicBdaddr_t * const bd)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LEDISCONNECT, &_ar) != true;) {
    status = twicIfLeDisconnect(cif, bd);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* @brief
 * Switch the Low Power Consumption of the BLE.
 *  
 * This API enables the Low Power Consumption (LPC) of the Advertiser
 * and the Idle.  Regarding the other roles, the LPC is specified by
 * the parameter "conn_scan".  For example, if the "conn_scan" is
 * "true", the LPC is enabled when the GAP Peripheral role is
 * connected.
 *  
 * @param twicConnIface_t * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param bool conn_scan
 * If the GAP role is the Scanner, this parameter must be set with
 * "false".  This argument can be "true" if the GAP role is not
 * Scanner.
 * @param bool deep
 * If the parameter is "true", the BLE Controller will stop working
 * when it does not have any message.
 * @note
 * If the role is the Scanner, the "conn_scan" must be "false".
 * The parameter "deep" should be attentively used.
 * The parameter is supposed to be "true" when an application needs to
 * stop using the BLE for a long time.
 * Both "twicIfLeIoFinalize" and "twicIfLeIoInitialize" should be
 * invoked if the BLE again starts.*/
twicStatus_t
twicUtLeCeLowPowerMode(twicConnIface_t * const cif, bool conn_scan, bool deep)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERMODE, &_ar) != true;) {
    status = twicIfLeCeLowPowerMode(cif, true, true, conn_scan, deep);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.10 LE Set Scan Parameters Command.
 * 7.8.11 LE Set Scan Enable Command.
 *
 * @brief LE Set Scan enable.
 * This API is used to request the Controller to start scanning.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t interval This is equivalent to the LE_Scan_Interval.
 * @param uint16_t window This is equivalent to the LE_Scan_Window.
 * @param bool active This is equivalent to the LE_Scan_Type.
 * @param bool random This is equivalent to the Own_Address_Type.
 * @param bool whitelist This is equivalent to the Scanning_Filter_Policy.
 * @param bool duplicates This is equivalent to the Filter_Duplicates. */
twicStatus_t twicUtLeSetScanEnable(twicConnIface_t * const cif,
                                   uint16_t interval, uint16_t window,
                                   bool active, bool random, bool whitelist,
                                   bool duplicates)
{
#if defined(TWIC_CONFIG_ENABLE_SCAN)
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LESETSCANENABLE, &_ar) != true;) {
    status = twicIfLeSetScanEnable(cif, interval, window, active, random,
                                   whitelist, duplicates);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
#else
  (void)cif;
  (void)interval;
  (void)window;
  (void)active;
  (void)random;
  (void)whitelist;
  (void)duplicates;
  return TWIC_STATUS_OPERATION_NOT_PERMITTED;
#endif  
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.11 LE Set Scan Enable Command.
 *
 * @brief LE Set Scan disable.
 * This API is used to request the Controller to stop scanning.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface. */
twicStatus_t twicUtLeSetScanDisable(twicConnIface_t * const cif)
{
#if defined(TWIC_CONFIG_ENABLE_SCAN)
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LESETSCANDISABLE, &_ar) != true;) {
    status = twicIfLeSetScanDisable(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
#else
  (void)cif;
  return TWIC_STATUS_OPERATION_NOT_PERMITTED;
#endif  
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.12 LE Create Connection Command.
 *
 * @brief LE Create Connection.
 * This API is used to request the Controller to create a Link Layer
 * connection to a connectable advertiser.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t interval This is equivalent to the LE_Scan_Interval.
 * @param uint16_t window This is equivalent to the LE_Scan_Window.
 * @param bool whitelist This is equivalent to the Initiator_Filter_Policy.
 * @param bool peer This is equivalent to the Peer_Address_Type.
 * @param twicBdaddr_t * addr This is equivalent to the Peer_Address.
 * @param uint16_t min This is equivalent to the Conn_Interval_Min.
 * @param uint16_t max This is equivalent to the Conn_Interval_Max.
 * @param uint16_t latency This is equivalent to the Conn_Latency.
 * @param uint16_t timeout This is equivalent to the Supervision_Timeout.
 * @param bool own This is equivalent to the Own_Address_Type.
 */
twicStatus_t twicUtLeCreateConnection(twicConnIface_t * const cif,
                                      uint16_t interval, uint16_t window,
                                      bool whitelist, bool peer,
                                      twicBdaddr_t * addr, uint16_t min,
                                      uint16_t max, uint16_t latency,
                                      uint16_t timeout, bool own)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LECREATECONNECTION, &_ar) != true;) {
    status = twicIfLeCreateConnection(cif, interval, window, whitelist, peer,
                                      addr, min, max, latency, timeout, 0, 0,
                                      own);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 2].
 * Host Controller Interface Functional Specification.
 * 7.8.13 LE Create Connection Cancel Command.
 *
 * @brief LE Create Connection Cancel.
 * This API is used to request the Controller to cancel the
 * "twicUtLeCreateConnection".
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface. */
twicStatus_t twicUtLeCreateConnectionCancel(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LECREATECONNECTIONCANCEL, &_ar) != true;) {
    status = twicIfLeCreateConnectionCancel(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

twicStatus_t
twicUtLeCeLowPowerControlPinSetup(twicConnIface_t * const cif, bool pin)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERCONTROLPINSETUP, &_ar) != true;) {
    status = twicIfLeCeLowPowerControlPinSetup(cif, pin);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
                                               
/* @brief
 * Switch the Low Power Consumption (LPC) to make the data rate higher.
 *
 * This API is used to make the data rate higher instead of the LPC.
 * The delay of Host Delay relates to the data rate.
 * The Host Wakeup function uses the Host Delay Setting.
 * The Host Wakeup function must be disabled if the maximum speed is required.
 * 
 * If the comprehensive LPC of both HOST and BLE is used, by the
 * following example, it is necessary to adequately inactivate the
 * HOST.
 * 
 * High Speed (Disable the LPC):
 * twicIfLeCeConnConfigureTz1em(TZ1EM_VF_HI, TZ1EM_OP_BLE_ACTIVE, ...);
 * twicIfLeCeCnAdvConfigureTz1em(TZ1EM_VF_HI, TZ1EM_OP_BLE_ACTIVE, ...);
 * twicIfLeCeIdleConfigureTz1em(TZ1EM_VF_HI, TZ1EM_OP_BLE_ACTIVE, ...);
 * twicUtLeCeLowPower(, false, false, false);
 * 
 * Normal Speed (Enable the LPC):
 * twicUtLeCeLowPower(, true, false, false);
 * twicIfLeCeCnAdvConfigureTz1em(TZ1EM_VF_LO, TZ1EM_OP_BLE_DOZE_WRET, ...);
 * twicIfLeCeIdleConfigureTz1em(TZ1EM_VF_LO, TZ1EM_OP_BLE_DOZE_WRET, ...);
 * twicIfLeCeConnConfigureTz1em(TZ1EM_VF_LO, TZ1EM_OP_BLE_DOZE_WRET, ...);
 * twicUtLeCeLowPower(, true, true, false);
 * 
 * Please refer to the example source code for information about the above step.
 * 
 * @param twicConnIface_t * const cif
 * 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * 
 * @param bool control
 * false:Disable the Host Delay Setting.
 * true:Enable the Host Delay Setting.
 * @param bool conn
 * 
 * true:Enable the LPC of the connection when the argument "control"
 * is "true".
 * 
 * false:Disable the LPC of the connection when the argument "control"
 * is "true".
 * 
 * @param bool shutdown
 * true:Enable the shutdown feature when the argument "control" is "true".
 * false:Disable the shutdown feature when the argument "control" is "true".
 * @note
 * The argument of the parameter "conn" must not be "true" for the Scanner. */
twicStatus_t twicUtLeCeLowPower(twicConnIface_t * const cif, bool control,
                                bool conn, bool shutdown)
{
  twicStatus_t status;

  if (false == control) {
    status = twicUtLeCeLowPowerMode(cif, false, false);
    if (TWIC_STATUS_OK == status)
      status = twicUtLeCeLowPowerControlPinSetup(cif, false);
  } else {
    status = twicUtLeCeLowPowerControlPinSetup(cif, true);
    if (TWIC_STATUS_OK == status)
      status = twicUtLeCeLowPowerMode(cif, conn, shutdown);
  }
  
  return status;
}

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
twicIfLeClientCb_t twicUtCcb;
void twicUtGattClientCleanup(void)
{
  memset((void*)&twicUtCcb, 0, sizeof(twicIfLeClientCb_t));
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT).
 * 4.4.2 Discover Primary Service by Service UUID
 *
 * @brief Discover Primary Service by Service UUID.
 * This API is used by a client to discover a specific primary service
 * on a server when only the Service UUID is known.
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle The Starting Handle shall be set to 0x0001.
 * @param uint16_t end_handle The Ending Handle shall be set to 0xFFFF.
 * @param uint64_t uuid_lsb The Attribute Protocol Find By
 * Type Value Request shall be used with the Attribute Type parameter
 * set to the UUID for -A«Primary Service» and the Attribute Value set$)B
 * to the 16-bit Bluetooth UUID or 128-bit UUID for the specific
 * primary service.
 * If UUID is 7905F431-B5CE-4E99-A40F-4B1E122D00D0, this parameter
 * shall be set with argument 0xA40F4B1E122D00D0.
 * @param uint64_t uuid_msb
 * If UUID is 7905F431-B5CE-4E99-A40F-4B1E122D00D0, this parameter
 * shall be set with argument 0x7905F431B5CE4E99.
 * @param uint8_t uuid_len
 * If UUID is 7905F431-B5CE-4E99-A40F-4B1E122D00D0, this parameter
 * shall be set with argument 16.
 */
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
twicStatus_t
twicUtGattDiscoverPrimaryServiceByServiceUuid(twicConnIface_t * const cif,
                                              uint16_t start_handle,
                                              uint16_t end_handle,
                                              uint64_t uuid_lsb,
                                              uint64_t uuid_msb,
                                              uint8_t uuid_len)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(cif, TWIC_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID,
                        &_ar) != true;) {
    status = twicIfLeGattClientDiscoverPrimaryServiceByServiceUuid(
      cif, start_handle, end_handle, uuid_lsb, uuid_msb, uuid_len);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.6.1 Discover All Characteristics of a Service.
 *
 * @brief Discover Primary Service by Service UUID.
 * This API is used by a client to find all the characteristic
 * declarations within a service definition on a server when only the
 * service handle range is known. The service specified is identified
 * by the service handle range.
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle
 * The Starting Handle shall be set to starting handle of the
 * specified service.
 * @param uint16_t end_handle
 * The Ending Handle shall be set to the ending handle of the
 * specified service. */
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
twicStatus_t
twicUtGattClientDiscoverAllCharacteristics(twicConnIface_t * const cif,
                                           uint16_t start_handle,
                                           uint16_t end_handle)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTCLIENTDISCOVERALLCHARACTERISTICS, &_ar) != true;) {
    status = twicIfLeGattClientDiscoverAllCharacteristics(cif, start_handle,
                                                          end_handle);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.7.1 Discover All Characteristic Descriptors.
 *
 * @brief Discover All Characteristic Descriptors.
 * This API is used by a client to find all the characteristic
 * descriptor's Attribute Handles and Attribute Types within a
 * characteristic definition when only the characteristic handle range
 * is known. The characteristic specified is identified by the
 * characteristic handle range.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle
 * The Attribute Protocol Find Information Request shall be used with
 * the Starting Handle set to the handle of the specified
 * characteristic value + 1.
 * @param uint16_t end_handle
 * The Ending Handle set to the ending handle of the specified
 * characteristic. */
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
twicStatus_t
twicUtGattClientDiscoverAllDescriptors(twicConnIface_t * const cif,
                                       uint16_t start_handle,
                                       uint16_t end_handle)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTCLIENTDISCOVERALLDESCRIPTORS, &_ar) != true;) {
    status = twicIfLeGattClientDiscoverAllDescriptors(cif, start_handle,
                                                      end_handle);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.12.3 Write Characteristic Descriptors.
 *
 * @brief Write Characteristic Descriptors.
 * This API is used to write a characteristic descriptor value to a
 * server when the client knows the characteristic descriptor handle.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t handle
 * The Attribute Handle parameter shall be set to the characteristic
 * descriptor handle.
 * @param uint8_t length
 * The Attribute Value lenght parameter is the lenght of the new
 * Attribute Value.
 * @param uint8_t * const value
 * The Attribute Value parameter shall be set to the new
 * characteristic descriptor value. */
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
twicStatus_t
twicUtGattClientWriteCharacteristicDescriptor(twicConnIface_t * const cif,
                                              uint16_t handle, uint8_t length,
                                              uint8_t * const value)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR, &_ar) != true;) {
    status = twicIfLeGattClientWriteCharacteristicDescriptor(cif, handle,
                                                             length, value);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.9.3 Write Characteristic Value.
 *
 * @brief Write Characteristic Descriptors.
 * This API is used to write a characteristic value to a server when
 * the client knows the characteristic value handle.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t handle
 * The Attribute Handle parameter shall be set to the characteristic
 * value handle.
 * @param uint8_t length
 * The Attribute Value lenght parameter is the lenght of the new
 * Attribute Value.
 * @param uint8_t * const value
 * The Attribute Value parameter shall be set to the new
 * characteristic value. */
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
twicStatus_t
twicUtGattClientWriteCharacteristicValue(twicConnIface_t * const cif,
                                         uint16_t handle, uint8_t length,
                                         uint8_t * const value)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtPeekInApi(
         cif, TWIC_GATTCLIENTWRITECHARACTERISTICVALUE, &_ar) != true;) {
    status = twicIfLeGattClientWriteCharacteristicValue(cif, handle, length,
                                                        value);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  
#endif
