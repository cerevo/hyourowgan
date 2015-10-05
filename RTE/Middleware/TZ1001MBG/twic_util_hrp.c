/**
 * @file twic_util_hrp.c
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

#if defined(TWIC_UTIL_HR_SERVICE)

/* Multiple piconet is not supported in this sample implementation. */
typedef struct twicUtHrpFlag {
  uint8_t congestion : 1;
  uint8_t do_mea_notification : 1;
  uint8_t pad : 6;
} twicUtHrpFlag_t;
static twicUtHrpFlag_t flags = {0,0,0};

static uint16_t cycle = 0;
static uint8_t bpm = 0;
static uint16_t rr_interval = 0;
static float energy_expended = 0;

#define FAKE_INTERVALS (40)
static const uint16_t fake_intervals[FAKE_INTERVALS] = {
  900, 930, 910, 901, 950, 915, 800, 700, 500, 480,
  450, 440, 460, 425, 415, 420, 431, 419, 427, 400,
  420, 425, 430, 435, 440, 445, 450, 455, 460, 465,
  470, 475, 480, 490, 500, 600, 700, 800, 900, 910,
};

static tz1smHalTimerId tid_mea_notification;
static tz1smHalTimerId tid_fake_heart;

void twicUtHrpSetupService(void)
{
  memset(twic_util_hr_measurement, 0x00, sizeof(twic_util_hr_measurement));
  memset(twic_util_hr_mea_desc, 0x00, sizeof(twic_util_hr_mea_desc));
  memset(twic_util_hr_sensor_location, 0x02,
         sizeof(twic_util_hr_sensor_location)); /*Wrist*/
  memset(twic_util_hr_control_point, 0x00, sizeof(twic_util_hr_control_point));

  flags.congestion = false;
  flags.do_mea_notification = false;
}

static void twicUtHrpMeaNotification(void const *arg)
{
  twic_util_hr_measurement[0] = 0x18; /* 1Byte (1001 LSB) */
  twic_util_hr_measurement[1] = bpm; /* 1Byte BPM */
  /* 2Byte Enery Expended */
  TWIC_SETHARF_LE(twic_util_hr_measurement + 2, (uint16_t)energy_expended);
  /* 2Byte RR interval */
  TWIC_SETHARF_LE(twic_util_hr_measurement + 4, rr_interval);
  flags.do_mea_notification = true;
}

static void twicUtHrpFakeHeart(void const *arg)
{
  const uint8_t vo2max = 37; /* VO2max:ml/kg/min */
  const float weight = 68.0; /* kg */
  const uint8_t age = 20;
  uint8_t max_heartbeat;
  uint8_t movement_intensity;
  float vo2, avo2;
  
  rr_interval = fake_intervals[cycle % FAKE_INTERVALS];
  cycle++;
  bpm = 60000 / rr_interval;

  max_heartbeat = 220 - age;
  movement_intensity = (bpm * 100) / max_heartbeat;
  vo2 = vo2max * movement_intensity / 100;
  avo2 = vo2 / 60 * weight / 1000;
  energy_expended += avo2 * 5 * (float)4.18605; /* 1kcal = 4.18605 kJ */
}

TZ1SM_HAL_TIMER_INIT(TWIC_UTIL_HRP_MEA_NOTIFICATION, twicUtHrpMeaNotification);
TZ1SM_HAL_TIMER_INIT(TWIC_UTIL_HRP_FAKE_HEART, twicUtHrpFakeHeart);

void twicUtHrpStopNotification(void)
{
  tz1smHalTimerStop(tid_mea_notification);
  flags.do_mea_notification = false;
}

void twicUtHrpDescriptorChange(uint8_t eidx, const twicAttValue_t *const arg)
{
  if (EIDX_HR_MEA_DESC == eidx) {
    if ((arg->value[0] & 0x1)) {
      tz1smHalTimerStart(tid_mea_notification, 1000);
      tz1smHalTimerStart(tid_fake_heart, 1000);
    } else {
      twicUtHrpStopNotification();
    }
  }
}

uint8_t twicUtHrpWrittenIn(uint8_t eidx, const twicAttValue_t *const arg)
{
  if (EIDX_HR_CONTROL_PO == eidx) {
    cycle = 0;
    energy_expended = 0;
  }
  return 0;
}

uint8_t twicUtHrpReadOut(twicConnIface_t * const cif, uint8_t eidx, uint16_t num)
{
  uint16_t max = eidx + num - 1;
  
  if (EIDX_HR_MEASUREMENT >= eidx && EIDX_HR_MEASUREMENT <= max) {
    twicUtGattServerWriteCharacteristics(
      cif, eidx, sizeof(twic_util_hr_measurement), twic_util_hr_measurement);
  }
  return 0;
}

void twicUtHrpTimerSetup(void)
{
  tid_mea_notification = tz1smHalTimerCreate(
    TZ1SM_HAL_TIMER(TWIC_UTIL_HRP_MEA_NOTIFICATION),
    TZ1SM_HAL_TIMER_PERIODIC, NULL);
  tid_fake_heart = tz1smHalTimerCreate(
    TZ1SM_HAL_TIMER(TWIC_UTIL_HRP_FAKE_HEART), TZ1SM_HAL_TIMER_PERIODIC, NULL);
}

void twicUtHrpTimerCleanup(void)
{
  tz1smHalTimerDelete(tid_mea_notification);
  tz1smHalTimerDelete(tid_fake_heart);
}

void twicUtHrpRun(twicConnIface_t * const cif)
{
  uint8_t aret;
  
  if (true == flags.do_mea_notification && false == flags.congestion) {
    aret = twicUtGattNotification(cif, EIDX_HR_MEASUREMENT,
                                  twic_util_hr_measurement,
                                  sizeof(twic_util_hr_measurement));
    if (0xB3 == aret) flags.congestion = true;
    flags.do_mea_notification = false;
  }
}

void twicUtHrpCongestionCheck(void)
{
  flags.congestion = false;
}

#else

void twicUtHrpSetupService(void) { return; }
void twicUtHrpStopNotification(void) { return; }
void twicUtHrpDescriptorChange(uint8_t eidx, const twicAttValue_t *const arg)
{ return; }
uint8_t twicUtHrpReadOut(twicConnIface_t * const cif, uint8_t eidx,
                         uint16_t num)
{ return 0; }
uint8_t twicUtHrpWrittenIn(uint8_t eidx, const twicAttValue_t *const arg)
{ return 0; }
void twicUtHrpTimerSetup(void) { return; }
void twicUtHrpTimerCleanup(void) { return; }
void twicUtHrpCongestionCheck(void) { return; }
void twicUtHrpRun(twicConnIface_t * const cif) { return; }

#endif /* TWIC_UTIL_HR_SERVICE */
