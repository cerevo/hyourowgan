/**
 * @file twic_util_init.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.x Smart
 * @version V1.0.1.FS (Free Sample - The information in this code is
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
#include "twic_util_lemng.h"

twicStatus_t twicUtLeCeInit0(twicConnIface_t *cif, uint8_t *aret, uint16_t ext)
{
#if 0
/* TX Power */
#if defined(TWIC_API_LECEDBUSWRITE)
  {
    uint8_t _ar;
    twicStatus_t status;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEDBUSWRITE, &_ar) != true;) {
      status = twicIfLeCeDbusWrite(cif, 0xD7, ext); /* TX POW -0...-32db */
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
  }
#endif
  
/* VDD12X */
#if defined(TWIC_API_LECEMEMORYWRITE)  
  {
    uint8_t _ar;
    twicStatus_t status;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEMEMORYWRITE, &_ar) != true;) {
      status = twicIfLeCeMemoryWrite(cif, 0x00060502, 0x0487);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
  }
#endif

#if defined(TWIC_API_LECEDBUSWRITE)
  {
    uint8_t _ar;
    twicStatus_t status;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEDBUSWRITE, &_ar) != true;) {
      status = twicIfLeCeDbusWrite(cif, 0x20, 0x0000);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEDBUSWRITE, &_ar) != true;) {
      status = twicIfLeCeDbusWrite(cif, 0x23, 0x0050);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEDBUSWRITE, &_ar) != true;) {
      status = twicIfLeCeDbusWrite(cif, 0x23, 0x0050);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
  }
#endif

#if defined(TWIC_API_LECEDBUSREAD)
  {
    uint8_t _ar;
    twicStatus_t status;
    uint8_t data[23];
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEDBUSREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeDbusRead(cif, 0x24);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("Addr 0x%02x, Value 0x%02x%02x\r\n", data[0], data[2], data[1]);
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEDBUSREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeDbusRead(cif, 0x25);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("Value 0x%02x%02x\r\n", data[1], data[0]);
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEDBUSREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeDbusRead(cif, 0x26);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("Value 0x%02x%02x\r\n", data[1], data[0]);
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEDBUSREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeDbusRead(cif, 0x27);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("Value 0x%02x%02x\r\n", data[1], data[0]);
  }
#endif
  
#if defined(TWIC_API_LECEMEMORYREAD)
  {
    uint8_t _ar;
    twicStatus_t status;
    uint8_t data[23];
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEMEMORYREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeMemoryRead(cif, 0x00061648);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("0x00061648 Value 0x%02x%02x\r\n", data[1], data[0]);
  }
#endif
  
#if defined(TWIC_API_LECEMEMORYWRITE)  
  {
    uint8_t _ar;
    twicStatus_t status;
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApi(cif, TWIC_LECEMEMORYWRITE, &_ar) != true;) {
      status = twicIfLeCeMemoryWrite(cif, 0x00061648, 0x0014);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
  }
#endif
#endif
  
#if defined(TWIC_API_LECEMEMORYREAD)
  {
    uint8_t _ar;
    twicStatus_t status;
    uint8_t data[23];
    /* API needs to be in the loop to wait NOP. */
    for (;twicUtPeekInApiWithValue(cif, TWIC_LECEMEMORYREAD, &_ar,
                                   &data) != true;) {
      status = twicIfLeCeMemoryRead(cif, 0x00061648);
      if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
        return status;
      twicUtDoEvent();
    }
    if (NULL != aret) *aret = _ar;
    if (_ar) return TWIC_STATUS_ERROR_IO;
    twicPrintf("0x00061648 Value 0x%02x%02x\r\n", data[1], data[0]);
  }
#endif  /* VDD12X Confirmation */

  return TWIC_STATUS_OK;
}

/* @brief
 * The API is the utility API to pack some initialization APIs of the
 * BLE Controller Extension Function.
 *
 * Please use this API after these APIs are successfully executed.
 *
 * twicIfLeIoInitialize();
 * twicIfGattInitialize();
 * twicIfGattRegistration();
 * twicIfGattCleanup();
 * twicIfLeRegisterCallback();
 *
 * Here this API is supposed to be invoked as follows.
 *  
 * twicUtLeCeInit1(&wait_ms);
 * wait(wait_ms);
 * twicUtLeCeInit2(&wait_ms);
 * wait(wait_ms);
 * twicUtLeCeInit3();
 *  
 * twicUtInitGattService();
 *
 * Please check the parameter "aret" if this API returns with
 * "TWIC_STATUS_ERROR_IO".
 *
 * Please take the specified interval of the parameter "wait_ms"
 * before the "twicUtLeCeInit2" is invoked.
 *
 * @param twicConnIface_t *cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint8_t *aret
 * This argument is the Bit Wise.
 * RPQUEUE_H4_HAL_OK                   (0x00)
 * RPQUEUE_H4_HAL_ERROR                (0x10)
 * RPQUEUE_H4_M2MSG_OK                 (0x00)
 * RPQUEUE_H4_M2MSG_UNKNOWN_DATA_TYPE  (0x02)
 * RPQUEUE_H4_M2MSG_INVALID_DATA_VALUE (0x04)
 * @param uint8_t *wait_ms
 * The necessary interval before the subsequent APIs.
 * @param twicTzbtBr_t br
 * TWIC_TZBT_2304
 * @param uint16_t pu
 * TWIC_TZBT_PUNCTUATION_MIN (High Speed:If an application needs
 * higher data rate than 2kB, the "br" is necessary to be set with
 * this value.)
 * TWIC_TZBT_PUNCTUATION (Normal)
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED
 * This API cannot be invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER Argument Error.
 * @return TWIC_STATUS_ERROR_IO The argument "aret" must be evaluated. */
twicStatus_t
twicUtLeCeInit1(twicConnIface_t *cif, uint8_t *aret, uint8_t *wait_ms,
                twicTzbtBr_t br, uint16_t pu)
{
  twicStatus_t status;
  uint8_t _ar;
  const uint8_t psb[] = {
    0x10, 0x25, 0x03, 0xd0, 0x6d, 0x0b, 0x00, 0x70, 0xb5, 0x05, 0x1c, 0x9b,
    0xf7, 0x8c, 0xfb, 0x04, 0x1c, 0x11, 0xd0, 0x60, 0x78, 0x01, 0x21, 0x09,
    0x04, 0x41, 0x18, 0x08, 0x20, 0xd7, 0xf7, 0xba, 0xf9, 0x20, 0x89, 0x00,
    0x28, 0x04, 0xd0};
  const uint8_t pspw[] = {
    0x10, 0x2C, 0xf0, 0x24, 0x05, 0x00, 0x70, 0xb5, 0x05, 0x1c, 0x64, 0xf0,
    0xb8, 0xfc, 0x04, 0x1c, 0x07, 0xd0, 0xe1, 0x68, 0x14, 0x23, 0xc8, 0x5e,
    0x4c, 0xf0, 0xf1, 0xfc, 0x20, 0x1c, 0x64, 0xf0, 0x98, 0xfc, 0x28, 0x1c,
    0x64, 0xf0, 0xfe, 0xfc, 0x70, 0xbc, 0x08, 0xbc, 0x18, 0x47};

  if (NULL != wait_ms) *wait_ms = 0;

  /* API needs to be in the loop to wait NOP. */
  for (;twicUtPeekInApi(cif, TWIC_LECEPATCHBASE, &_ar) != true;) {
    status = twicIfLeCePatchBase(cif, sizeof(psb), psb);
    if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status)
      return status;
    twicUtDoEvent();
  }
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCePatchWrite(cif, sizeof(pspw), pspw);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECEPATCHWRITE, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCePatchControl(cif, 3, true);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECEPATCHCONTROL, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCeSetBaudrate(cif, br, pu);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECESETBAUDRATE, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  if (NULL != wait_ms) *wait_ms = 100;

  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Access Profile
 * 3.2.1 Bluetooth Device Address (BD_ADDR).
 *
 * @brief
 * The API is the utility API to pack some initialization APIs of the
 * BLE Controller Extension Function.
 *
 * Please use this API after the "twicUtLeCeInit1" API is successfully executed.
 *
 * twicUtLeCeInit1(&wait_ms);
 * wait(wait_ms);
 *
 * Here this API is supposed to be invoked as follows.
 *
 * twicUtLeCeInit2(&wait_ms);
 * wait(wait_ms);
 * twicUtLeCeInit3();  
 *  
 * twicUtInitGattService();
 *
 * Please check the parameter "aret" if this API returns with
 * "TWIC_STATUS_ERROR_IO".
 *
 * Please take the specified interval of the parameter "wait_ms"
 * before the "twicUtLeCeInit3" is invoked.
 *
 * @param twicConnIface_t *cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint8_t *aret
 * This argument is the Bit Wise.
 * RPQUEUE_H4_HAL_OK                   (0x00)
 * RPQUEUE_H4_HAL_ERROR                (0x10)
 * RPQUEUE_H4_M2MSG_OK                 (0x00)
 * RPQUEUE_H4_M2MSG_UNKNOWN_DATA_TYPE  (0x02)
 * RPQUEUE_H4_M2MSG_INVALID_DATA_VALUE (0x04)
 * @param uint8_t *wait_ms
 * The necessary interval before the subsequent APIs.
 * @param uint64_t *bd_addr
 * The public Bluetooth Device Address. (Please refer to the Reference
 * materials:3.2.1 Bluetooth Device Address (BD_ADDR))
 * @param bool low_power
 * true: Initialize the Low Power Consuption. This setup is mandatory
 * if an application will use the Low Power Consumption.
 * The maximum data rate may be slower than the setup of "false", but
 * the data rate can be higher than 2kB/s even though the ATT_MTU is
 * 23bytes.
 * If the application does not use the long Connection Interval and
 * keep the connection for a long time, the parameter can be "false".
 * Please note that if this parameter is "false", the HOST Cortex-M4F
 * is not inactivated even though the BLE acts the Advertiser. The
 * TWiC is able to inactivate the HOST in the PMU RETENTION mode.
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED
 * This API cannot be invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER Argument Error.
 * @return TWIC_STATUS_ERROR_IO The argument "aret" must be evaluated.
 * @return TWIC_STATUS_ERROR_HAL Check the "tz1sm_hal.[ch]" porting. */
twicStatus_t
twicUtLeCeInit2(twicConnIface_t *cif, uint8_t *aret, uint8_t *wait_ms,
                uint64_t *bd_addr, bool low_power)
{
  twicStatus_t status;
  uint8_t _ar;

  status = twicIfLeReadBdaddr(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LEREADBDADDR, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCeFlowControl(cif, true);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECEFLOWCONTROL, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  if (false == low_power) status = twicIfLeCeHostDelay(cif, 0x0000);
  else status = twicIfLeCeHostDelay(cif, 0x000A);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECEHOSTDELAY, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;
    
  status = twicIfLeWriteBdaddr(cif, bd_addr);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LEWRITEBDADDR, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  if (false == low_power) goto change_to_tcu;
#if !defined(TWIC_LECE_LOWPOWER)
  if (true == low_power) return TWIC_STATUS_ERROR_PARAMETER;
#endif /* TWIC_LECE_LOWPOWER */

  status = twicIfLeCeLowPowerClockSetup(cif, NULL);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERCLOCKSETUP, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCeLowPowerPrimarySetup(cif, 0x00FA, 0xA);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERPRIMARYSETUP, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeCeLowPowerControlPinSetup(cif, true);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERCONTROLPINSETUP, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;
  
  status = twicIfLeCeLowPowerDiscoverableSetup(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECELOWPOWERDISCOVERABLESETUP, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  change_to_tcu:
  
  status = twicIfLeCeChangeToCm(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LECECHANGETOCM, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) {
    if (NULL != wait_ms) *wait_ms = 0;
    return TWIC_STATUS_ERROR_IO;
  }
  if (NULL != wait_ms) *wait_ms = 30;

  return TWIC_STATUS_OK;
}

/* @brief
 * The API is the utility API to pack some initialization APIs of the
 * BLE Controller Extension Function.
 *
 * Please use this API after the "twicUtLeCeInit2" API is successfully executed.
 *
 * twicUtLeCeInit2(&wait_ms);
 * wait(wait_ms);
 *
 * Here this API is supposed to be invoked as follows.
 * twicUtLeCeInit3();  
 *  
 * twicUtInitGattService();
 *
 * Please check the parameter "aret" if this API returns with
 * "TWIC_STATUS_ERROR_IO".
 *
 * @param twicConnIface_t *cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint8_t *aret
 * This parameter must be evaluated when this API returns with the
 * TWIC_STATUS_ERROR_IO.
 * Please refer to the Error Code [D] or the "LIST OF ERROR CODES [4]"
 * of the twic_interface_cb.h.
 * @param bool client
 * true:GATT Client is enabled.
 * false:GATT Client is not necessary for an application.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED
 * This API cannot be invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER Argument Error.
 * @return TWIC_STATUS_ERROR_IO The argument "aret" must be evaluated.
 * @return TWIC_STATUS_ERROR_HAL Check the "tz1sm_hal.[ch]" porting.
 *
 * @return TWIC_STATUS_UNDER_PROCESSING BLE System is busy. This API
 * must be invoked in that context if this error occurs.
 *
 * @return TWIC_STATUS_WAITING_FOR_ACTIVATION BLE System Status is not
 * active. This API must be invoked in that context if this error
 * occurs. */
twicStatus_t
twicUtLeCeInit3(twicConnIface_t * const cif, uint8_t *aret, bool client)
{
  twicStatus_t status;
  uint8_t _ar;

  status = twicIfLeInitializeDevice(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LEINITIALIZEDEVICE, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  status = twicIfLeGattServerStart(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LEGATTSERVERSTART, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  if (false == client) return TWIC_STATUS_OK;
  
  status = twicIfLeGattClientStart(cif);
  if (TWIC_STATUS_OK != status) return status;
  for (;twicUtPeekInApi(cif, TWIC_LEGATTCLIENTSTART, &_ar) != true;)
    twicUtDoEvent();
  if (NULL != aret) *aret = _ar;
  if (_ar) return TWIC_STATUS_ERROR_IO;

  return TWIC_STATUS_OK;
}
