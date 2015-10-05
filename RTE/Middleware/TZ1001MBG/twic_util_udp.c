/**
 * @file twic_util_udp.c
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
#include "twic_util_service.h"
#include "twic_util_lemng.h"
#include "twic_util_lesmp.h"

#if defined(TWIC_UTIL_UD_SERVICE)

/* Multiple piconet is not supported in this sample implementation. */
typedef struct twicUtUdpFlag {
  uint8_t congestion : 1;
  uint8_t do_inc_notification : 1;
  uint8_t do_con_indication : 1;
  uint8_t pad : 5;
} twicUtUdpFlag_t;
static twicUtUdpFlag_t flags;

static tz1smHalTimerId tid_inc_notification;
static tz1smHalTimerId tid_con_indication;

void twicUtUdpSetupService(void)
{
  const char *first_name = "TEST";
  
  /* User Data Service Descripter and Characteristic attribute data. */
  /* First Name (Read + Write) */
  memcpy(twic_util_ud_first_name, first_name, sizeof(twic_util_ud_first_name));
  /* User Index (Read) */
  memset(twic_util_ud_index, 0, sizeof(twic_util_ud_index));

  /* Database Change Increment (Read + Write + Notify) */
  memset(twic_util_ud_increment, 0, sizeof(twic_util_ud_increment));
  memset(twic_util_ud_inc_desc, 0, sizeof(twic_util_ud_inc_desc));

  /* User Control Point (Write + Indicate) */
  memset(twic_util_ud_control_point, 0x00, sizeof(twic_util_ud_control_point));
  memset(twic_util_ud_con_desc, 0, sizeof(twic_util_ud_con_desc));

  flags.congestion = false;
  flags.do_inc_notification = false;
  flags.do_con_indication = false;
}

static void twicUtUdpIncNotification(void const *arg)
{
  /* Please confirm the updated data here.
     The data will be sent soon.
     memcpy(twic_util_ud_increment,
     twic_util_udp_work_data_of_increment,
     sizeof(twic_util_ud_increment) */
  flags.do_inc_notification = true;
}

static void twicUtUdpConindication(void const *arg)
{
  /* Please confirm the updated data here.
     The data will be sent soon.
     memcpy(twic_util_ud_control_point,
     twic_util_udp_work_data_of_control_point,
     sizeof(twic_util_ud_control_point) */
  flags.do_con_indication = true;
}

TZ1SM_HAL_TIMER_INIT(TWIC_UTIL_UDP_INC_NOTIFICATION, twicUtUdpIncNotification);
TZ1SM_HAL_TIMER_INIT(TWIC_UTIL_UDP_CON_INDICATION, twicUtUdpConindication);

void twicUtUdpStopNotification(void)
{
  tz1smHalTimerStop(tid_inc_notification);
  flags.do_inc_notification = false;
}

void twicUtUdpStopIndication(void)
{
  tz1smHalTimerStop(tid_con_indication);
  flags.do_con_indication = false;
}

void twicUtUdpDescriptorChange(uint8_t eidx, const twicAttValue_t *const arg)
{
  if (EIDX_UD_INC_DESC == eidx) {
    if ((arg->value[0] & 0x1)) {
      tz1smHalTimerStart(tid_inc_notification, 1000);
    } else {
      twicUtUdpStopNotification();
    }
  } else if (EIDX_UD_CON_DESC == eidx) {
    if ((arg->value[0] & 0x2)) {
      tz1smHalTimerStart(tid_con_indication, 1000);
    } else {
      twicUtUdpStopIndication();
    }
  }
}

/* When each Eidx is written in. */
uint8_t twicUtUdpWrittenIn(twicConnIface_t * const iface, uint8_t eidx,
                           const twicAttValue_t *const arg)
{
  twicEntry_t *entry = TWIC_IF_TO_ENTRY(eidx);
  uint8_t code = 0;
  uint16_t perm_a = TWIC_PERMISSION_A0 + TWIC_PERMISSION_A1;
  uint16_t perm_e = TWIC_PERMISSION_E0 + TWIC_PERMISSION_E1;
  uint16_t perm = entry->u.attr.permissions;
  
  if (EIDX_UD_FIRST_NAME == eidx) {
    /* First Name (Read + Write) */
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        memcpy(twic_util_ud_first_name, arg->value,
               sizeof(twic_util_ud_first_name));
      else code = 0x0F;
    } else {
      code = 0x05;
    }
  } else if (EIDX_UD_INCREMENT == eidx) {
    /* Database Change Increment (Read + Write + Notify) */
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        memcpy(twic_util_ud_increment, arg->value,
               sizeof(twic_util_ud_increment));
      else code = 0x0F;
    } else {
      code = 0x05;
    }
  } else if (EIDX_UD_CONTROL_POINT == eidx) {
    /* User Control Point (Write + Indicate) */
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        memcpy(twic_util_ud_control_point, arg->value,
               sizeof(twic_util_ud_control_point));
      else code = 0x0F;      
    } else {
      code = 0x05;
    }
  }
  return code;
}

/* When each Eidx is read out. */
uint8_t twicUtUdpReadOut(twicConnIface_t * const iface, uint8_t eidx,
                         uint16_t num)
{
  twicEntry_t *entry = TWIC_IF_TO_ENTRY(eidx);
  uint16_t max = eidx + num - 1;
  uint8_t code = 0;
  uint16_t perm_a = TWIC_PERMISSION_A0;
  uint16_t perm_e = TWIC_PERMISSION_E0;
  uint16_t perm = entry->u.attr.permissions;
  
  twicLog("eidx = %d, num = %d\r\n", eidx, num);
  if (EIDX_UD_FIRST_NAME >= eidx && EIDX_UD_FIRST_NAME <= max) {
    /* First Name (Read + Write) */  
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        twicUtGattServerWriteCharacteristics(iface, eidx,
                                             sizeof(twic_util_ud_first_name),
                                             twic_util_ud_first_name);
      else code = 0x0F;
    } else {
      code = 0x05;
    }
  } else if (EIDX_UD_INDEX >= eidx && EIDX_UD_INDEX <= max) {
    /* User Index (Read) */
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        twicUtGattServerWriteCharacteristics(iface, eidx,
                                             sizeof(twic_util_ud_index),
                                             twic_util_ud_index);
      else code = 0x0F;
    } else {
      code = 0x05;
    }
  } else if (EIDX_UD_INCREMENT >= eidx && EIDX_UD_INDEX <= max) {
    /* Database Change Increment (Read + Write + Notify) */
    if ((perm_a & perm) && true == twicUtSmpFlags.keyring_is_stored) {
      if ((perm_e & perm) && true == twicUtSmpFlags.encryption_done)
        twicUtGattServerWriteCharacteristics(iface, eidx,
                                             sizeof(twic_util_ud_increment),
                                             twic_util_ud_increment);
      else code = 0x0F;
    } else {
      code = 0x05;
    }
  }
  return code;
}

void twicUtUdpTimerSetup(void)
{
  tid_inc_notification =
    tz1smHalTimerCreate(TZ1SM_HAL_TIMER(TWIC_UTIL_UDP_INC_NOTIFICATION),
                        TZ1SM_HAL_TIMER_PERIODIC, NULL);
  tid_con_indication =
    tz1smHalTimerCreate(TZ1SM_HAL_TIMER(TWIC_UTIL_UDP_CON_INDICATION),
                        TZ1SM_HAL_TIMER_PERIODIC, NULL);
}

void twicUtUdpTimerCleanup(void)
{
  tz1smHalTimerDelete(tid_inc_notification);
  tz1smHalTimerDelete(tid_con_indication);
}

void twicUtUdpRun(twicConnIface_t * const cif)
{
  uint8_t aret;
  
  if (true == flags.do_inc_notification && false == flags.congestion) {
    twicTrace();
    aret = twicUtGattNotification(cif, EIDX_UD_INCREMENT,
                                  twic_util_ud_increment,
                                  sizeof(twic_util_ud_increment));
    if (0xB3 == aret) flags.congestion = true;
    flags.do_inc_notification = false;
  }
  if (true == flags.do_con_indication && false == flags.congestion) {
    twicTrace();
    aret = twicUtGattIndication(cif, EIDX_UD_CONTROL_POINT,
                                twic_util_ud_control_point,
                                sizeof(twic_util_ud_control_point));
    if (0xB3 == aret) flags.congestion = true;
    flags.do_con_indication = false;
  }
}

void twicUtUdpCongestionCheck(void)
{
  flags.congestion = false;
}

void twicUtUdpIndicationConfirmation(const uint8_t status)
{
  flags.congestion = false;
  /* The status is not 0, if the remote system did not respond.
     It will be 0xAA or some other erro code.
     Please refer to the "LIST OF ERROR CODES [2]" in twic_interface_cb.h.
  */
}

#else

void twicUtUdpSetupService(void) { return; }
void twicUtUdpStopNotification(void) { return; }
void twicUtUdpStopIndication(void) { return; }
void twicUtUdpDescriptorChange(uint8_t eidx, const twicAttValue_t *const arg)
{ return; }
uint8_t twicUtUdpReadOut(twicConnIface_t * const iface, uint8_t eidx,
                         uint16_t num)
{ return 0; }
uint8_t twicUtUdpWrittenIn(twicConnIface_t * const iface, uint8_t eidx,
                           const twicAttValue_t *const arg)
{ return 0; }
void twicUtUdpTimerSetup(void) { return; }
void twicUtUdpTimerCleanup(void) { return; }
void twicUtUdpCongestionCheck(void) { return; }
void twicUtUdpRun(twicConnIface_t * const cif) { return; }
void twicUtUdpIndicationConfirmation(const uint8_t status) { return; }

#endif /* TWIC_UTIL_UD_SERVICE */
