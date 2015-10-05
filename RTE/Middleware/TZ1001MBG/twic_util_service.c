/**
 * @file twic_util_service.c
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
#include "twic_util_lemng.h"

#if defined(TWIC_UTIL_GA_SERVICE) || defined(TWIC_UTIL_DI_SERVICE) || \
  defined(TWIC_UTIL_HR_SERVICE) || defined(TWIC_UTIL_BP_SERVICE) ||   \
  defined(TWIC_UTIL_UD_SERVICE) || defined(TWIC_UTIL_CT_SERVICE) ||   \
  defined(TWIC_UTIL_NC_SERVICE) || defined(TWIC_UTIL_RU_SERVICE) ||   \
  defined(TWIC_UTIL_IA_SERVICE) || defined(TWIC_UTIL_HT_SERVICE) ||   \
  defined(TWIC_UTIL_UU_SERVICE)

#define APP_GATT_DB_NUM (5)
typedef struct appGattAttr { 
  uint64_t value_uuid_lso;
  uint64_t value_uuid_mso;
  uint8_t value_uuid_size;
  uint8_t value_property;
  uint16_t value_permission;
  uint8_t *value;
  uint8_t value_len;
  uint16_t desp_uuid_lso;
  uint16_t desp_permission;
  uint8_t *desp;
  uint8_t desp_len;
} appGattAttr_t;
typedef struct appGattDb {
  uint64_t service_uuid_lso;
  uint64_t service_uuid_mso;
  uint8_t service_uuid_size; 
  appGattAttr_t attr[APP_GATT_DB_NUM];
} appGattDb_t;

static twicStatus_t
twicUtInitService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx,
                  appGattDb_t *db, uint8_t num)
{
  twicStatus_t status;
  uint8_t _ar;
  uint8_t service_eidx = eidx;
  uint8_t next_eidx = eidx + 1;
  uint8_t attr_eidx, desc_eidx;
  uint8_t i;
  
  for (;twicUtDbPeekInApi(cif, TWIC_LEGATTDBBEGINSERVICECREATION,
                          service_eidx, &_ar) != true;) {
    status = twicIfLeGattDbBeginServiceCreation(cif, service_eidx,
                                                db->service_uuid_lso,
                                                db->service_uuid_mso,
                                                db->service_uuid_size);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (NULL != aret) *aret = _ar; if (_ar) return TWIC_STATUS_ERROR_IO;

  for (i = 0; i < num; i++) {
    if (db->attr[i].value_uuid_lso) {
      attr_eidx = next_eidx++;

      for (;twicUtDbPeekInApi(cif, TWIC_LEGATTDBADDCHARACTERISTICS,
                              attr_eidx, &_ar) != true;) {
        status = twicIfLeGattDbAddCharacteristics(cif, service_eidx, attr_eidx,
                                                  db->attr[i].value_property,
                                                  db->attr[i].value_uuid_lso,
                                                  db->attr[i].value_uuid_mso,
                                                  db->attr[i].value_uuid_size);
        if (false == twicUtCheckAndDoEvent(status)) return status;
      }
      if (NULL != aret) *aret = _ar; if (_ar) return TWIC_STATUS_ERROR_IO;

      for (;twicUtDbPeekInApi(cif, TWIC_LEGATTDBSETCHARACTERISTICS,
                              attr_eidx, &_ar) != true;) {
        status = twicIfLeGattDbSetCharacteristics(cif, attr_eidx,
                                                  db->attr[i].value_permission,
                                                  db->attr[i].value_len,
                                                  db->attr[i].value,
                                                  db->attr[i].value_uuid_lso,
                                                  db->attr[i].value_uuid_mso,
                                                  db->attr[i].value_uuid_size);
        if (false == twicUtCheckAndDoEvent(status)) return status;
      }
      if (NULL != aret) *aret = _ar; if (_ar) return TWIC_STATUS_ERROR_IO;
    }
    if (db->attr[i].desp_uuid_lso) {
      desc_eidx = next_eidx++;

      for (;twicUtDbPeekInApi(cif, TWIC_LEGATTDBSETDESCRIPTOR, desc_eidx, &_ar)
             != true;) {
        status = twicIfLeGattDbSetDescriptor(cif, attr_eidx, desc_eidx,
                                             db->attr[i].desp_permission,
                                             db->attr[i].desp_len,
                                             db->attr[i].desp,
                                             db->attr[i].desp_uuid_lso,
                                             0, TWIC_UUID16);
        if (false == twicUtCheckAndDoEvent(status)) return status;
      }
      if (NULL != aret) *aret = _ar; if (_ar) return TWIC_STATUS_ERROR_IO;
    }
  }

  for (;twicUtDbPeekInApi(
         cif, TWIC_LEGATTDBENDSERVICECREATION, service_eidx, &_ar)
         != true;) {
    status = twicIfLeGattDbEndServiceCreation(cif, service_eidx);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (NULL != aret) *aret = _ar; if (_ar) return TWIC_STATUS_ERROR_IO;

  return TWIC_STATUS_OK;
}

#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
twicStatus_t
twicUtGattDbSetPermissions(twicConnIface_t *cif, const uint8_t entity,
                           const uint16_t permissions)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;

  for (;twicUtDbPeekInApi(cif, TWIC_LEGATTDBSETPERMISSIONS, entity, &_ar)
         != true;) {
    status = twicIfLeGattDbSetPermissions(cif, entity, permissions);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}
#endif

/* GA Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_GA_SERVICE)
uint8_t twic_util_ga_device_name[] = {TWIC_UTIL_GA_DEVICE_NAME};
uint8_t twic_util_ga_appearance[] = {TWIC_UTIL_GA_APPEARANCE};
static twicStatus_t
twicUtInitGAService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ga = { TWIC_UTIL_GA_SERVICE_SET, TWIC_UTIL_GA_SERVICE_LEN, {
      {
        0x2A00, 0, TWIC_UUID16, 0x02, 0x703, twic_util_ga_device_name,
        sizeof(twic_util_ga_device_name),
        0, 0, NULL, 0
      },
      {
        0x2A01, 0, TWIC_UUID16, 0x02, 0x703, twic_util_ga_appearance,
        sizeof(twic_util_ga_appearance),
        0, 0, NULL, 0
      },
      { 0, 0, 0, 0, 0, NULL, 0,  0, 0, NULL, 0 },
      { 0, 0, 0, 0, 0, NULL, 0,  0, 0, NULL, 0 },
      { 0, 0, 0, 0, 0, NULL, 0,  0, 0, NULL, 0 }
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &ga, 2);
}
#endif /* TWIC_UTIL_GA_SERVICE */

/* DI Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_DI_SERVICE)
uint8_t twic_util_di_manufname[] = {TWIC_UTIL_DI_MANUFNAME};
uint8_t twic_util_di_fw_version[] = {TWIC_UTIL_DI_FW_VERSION};
uint8_t twic_util_di_sw_version[] = {TWIC_UTIL_DI_SW_VERSION};
uint8_t twic_util_di_model_string[] = {TWIC_UTIL_DI_MODEL_STRING};
uint8_t twic_util_di_system_id[] = {TWIC_UTIL_DI_SYSTEM_ID};
static twicStatus_t
twicUtInitDIService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t di = { TWIC_UTIL_DI_SERVICE_SET, TWIC_UTIL_DI_SERVICE_LEN, {
      {
        0x2A29,  0, TWIC_UUID16, 0x02, 0x703, twic_util_di_manufname,
        sizeof(twic_util_di_manufname),
        0, 0, NULL, 0
      },
      {
        0x2A26, 0, TWIC_UUID16, 0x02, 0x703, twic_util_di_fw_version,
        sizeof(twic_util_di_fw_version),
        0, 0, NULL, 0
      },
      {
        0x2A28, 0, TWIC_UUID16, 0x02, 0x703, twic_util_di_sw_version,
        sizeof(twic_util_di_sw_version),
        0, 0, NULL, 0
      },
      {
        0x2A24, 0, TWIC_UUID16, 0x02, 0x703, twic_util_di_model_string,
        sizeof(twic_util_di_model_string),
        0, 0, NULL, 0
      },
      {
        0x2A23, 0, TWIC_UUID16, 0x02, 0x703, twic_util_di_system_id,
        sizeof(twic_util_di_system_id),
        0, 0, NULL, 0
      },
    }
  };
  
  return twicUtInitService(cif, aret, eidx, &di, 5);
}
#endif /* TWIC_UTIL_DI_SERVICE */

/* HR Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_HR_SERVICE)
uint8_t twic_util_hr_measurement[] = {TWIC_UTIL_HR_MEASUREMENT};
uint8_t twic_util_hr_mea_desc[] = {TWIC_UTIL_HR_MEA_DESC};
uint8_t twic_util_hr_sensor_location[] = {TWIC_UTIL_HR_SENSOR_LOCATION};
uint8_t twic_util_hr_control_point[] = {TWIC_UTIL_HR_CONTROL_POINT};
static twicStatus_t
twicUtInitHRService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t hr = { TWIC_UTIL_HR_SERVICE_SET, TWIC_UTIL_HR_SERVICE_LEN, {
      {
        0x2A37, 0, TWIC_UUID16, 0x10, 0x703, twic_util_hr_measurement,
        sizeof(twic_util_hr_measurement),
        0x2902, 0x703, twic_util_hr_mea_desc, sizeof(twic_util_hr_mea_desc)
      },
      {
        0x2A38, 0, TWIC_UUID16, 0x02, 0x703, twic_util_hr_sensor_location,
        sizeof(twic_util_hr_sensor_location),
        0, 0, NULL, 0
      },
      {
        0x2A39, 0, TWIC_UUID16, 0x08, 0x703, twic_util_hr_control_point,
        sizeof(twic_util_hr_control_point),
        0, 0, NULL, 0
      },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 }
    }
  };
  
  return twicUtInitService(cif, aret, eidx, (appGattDb_t *)&hr, 3);
}
#endif /* TWIC_UTIL_HR_SERVICE */

/* Blood Pressure Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_BP_SERVICE)
uint8_t twic_util_bp_measurement[] = {TWIC_UTIL_BP_MEASUREMENT};
uint8_t twic_util_bp_mea_desc[] = {TWIC_UTIL_BP_MEA_DESC};
uint8_t twic_util_bp_cuff_pressure[] = {TWIC_UTIL_BP_CUFF_PRESSURE};
uint8_t twic_util_bp_cuff_desc[] = {TWIC_UTIL_BP_CUFF_DESC};
uint8_t twic_util_bp_feature[] = {TWIC_UTIL_BP_FEATURE};
static twicStatus_t
twicUtInitBPService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t bp = { TWIC_UTIL_BP_SERVICE_SET, TWIC_UTIL_BP_SERVICE_LEN, {
      {
        0x2A35, 0, TWIC_UUID16, 0x20, 0x703, twic_util_bp_measurement,
        sizeof(twic_util_bp_measurement),
        0x2902, 0x703, twic_util_bp_mea_desc, sizeof(twic_util_bp_mea_desc)
      },
      {
        0x2A36, 0, TWIC_UUID16, 0x10, 0x703, twic_util_bp_cuff_pressure,
        sizeof(twic_util_bp_cuff_pressure),
        0x2902, 0x703, twic_util_bp_cuff_desc, sizeof(twic_util_bp_cuff_desc)
      },
      {
        0x2A49, 0, TWIC_UUID16, 0x02, 0x703, twic_util_bp_feature,
        sizeof(twic_util_bp_feature), 0, 0, NULL, 0
      },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 }
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &bp, 3);
}
#endif /* TWIC_UTIL_BP_SERVICE */

/* User Data Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_UD_SERVICE)
uint8_t twic_util_ud_first_name[] = {TWIC_UTIL_UD_FIRST_NAME};
uint8_t twic_util_ud_index[] = {TWIC_UTIL_UD_INDEX};
uint8_t twic_util_ud_increment[] = {TWIC_UTIL_UD_INCREMENT};
uint8_t twic_util_ud_inc_desc[] = {TWIC_UTIL_UD_INC_DESC};
uint8_t twic_util_ud_control_point[] = {TWIC_UTIL_UD_CONTROL_POINT};
uint8_t twic_util_ud_con_desc[] = {TWIC_UTIL_UD_CON_DESC};
static twicStatus_t
twicUtInitUDService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ud = { TWIC_UTIL_UD_SERVICE_SET, TWIC_UTIL_UD_SERVICE_LEN, {
      {
        0x2A8A, 0, TWIC_UUID16, 0x0A, 0x71B, twic_util_ud_first_name,
        sizeof(twic_util_ud_first_name),
        0, 0, NULL, 0
      },
      {
        0x2A9A, 0, TWIC_UUID16, 0x02, 0x719, twic_util_ud_index,
        sizeof(twic_util_ud_index), 0, 0, NULL, 0
      },
      {
        0x2A99, 0, TWIC_UUID16, 0x1A, 0x71B, twic_util_ud_increment,
        sizeof(twic_util_ud_increment),
        0x2902, 0x703, twic_util_ud_inc_desc, sizeof(twic_util_ud_inc_desc)
      },
      {
        0x2A9F, 0, TWIC_UUID16, 0x28, 0x71A, twic_util_ud_control_point,
        sizeof(twic_util_ud_control_point),
        0x2902, 0x703, twic_util_ud_con_desc, sizeof(twic_util_ud_con_desc)
      },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 }
    },
  };
  return twicUtInitService(cif, aret, eidx, &ud, 4);
}
#endif /* TWIC_UTIL_UD_SERVICE */

/* TIME Service Descripter and Characteristic attribute data. */
#if defined(TWIC_UTIL_CT_SERVICE)
uint8_t twic_util_ct_current_time[] = {TWIC_UTIL_CT_CURRENT_TIME};
uint8_t twic_util_ct_cur_desc[] = {TWIC_UTIL_CT_CUR_DESC};
uint8_t twic_util_ct_local_time_info[] = {TWIC_UTIL_CT_LOCAL_TIME_INFO};
uint8_t twic_util_ct_loc_desc[] = {TWIC_UTIL_CT_LOC_DESC};
uint8_t twic_util_ct_reference[] = {TWIC_UTIL_CT_REFERENCE};
static twicStatus_t
twicUtInitCTService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ct = { TWIC_UTIL_CT_SERVICE_SET, TWIC_UTIL_CT_SERVICE_LEN, {
      {
        0x2A2B, 0, TWIC_UUID16, 0x1A, 0x703, twic_util_ct_current_time,
        sizeof(twic_util_ct_current_time),
        0x2902, 0x703, twic_util_ct_cur_desc, sizeof(twic_util_ct_cur_desc)
      },
      {
        0x2A0F, 0, TWIC_UUID16, 0x1A, 0x703, twic_util_ct_local_time_info,
        sizeof(twic_util_ct_local_time_info),
        0x2902, 0x703, twic_util_ct_loc_desc, sizeof(twic_util_ct_loc_desc)
      },
      {
        0x2A14, 0, TWIC_UUID16, 0x02, 0x703, twic_util_ct_reference,
        sizeof(twic_util_ct_reference),
        0, 0, NULL, 0
      },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 },
      { 0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0 }
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &ct, 3);
}
#endif /* TWIC_UTIL_CT_SERVICE */

/* Next DST Change Service */
#if defined(TWIC_UTIL_NC_SERVICE)
/* Time With DST (Read) */
uint8_t twic_util_nc_time_with_dst[] = {TWIC_UTIL_NC_TIME_WITH_DST};
static twicStatus_t
twicUtInitNCService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t nc = { TWIC_UTIL_NC_SERVICE_SET, TWIC_UTIL_NC_SERVICE_LEN, {
      {
        0x2A11, 0, TWIC_UUID16, 0x02, 0x703, twic_util_nc_time_with_dst,
        sizeof(twic_util_nc_time_with_dst),
        0, 0, NULL, 0
      },
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0}
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &nc, 1);
}
#endif /* TWIC_UTIL_NC_SERVICE */

/* Reference Time Update Service */
#if defined(TWIC_UTIL_RU_SERVICE)
uint8_t twic_util_ru_control_point[] = {TWIC_UTIL_RU_CONTROL_POINT};
uint8_t twic_util_ru_state[] = {TWIC_UTIL_RU_STATE};
static twicStatus_t
twicUtInitRUService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ru = { TWIC_UTIL_RU_SERVICE_SET, TWIC_UTIL_RU_SERVICE_LEN, {
      {
        0x2A16, 0, TWIC_UUID16, 0x04, 0x703, twic_util_ru_control_point,
        sizeof(twic_util_ru_control_point),
        0, 0, NULL, 0
      },
      {
        0x2A17, 0, TWIC_UUID16, 0x02, 0x703, twic_util_ru_state,
        sizeof(twic_util_ru_state), 0, 0, NULL, 0
      },
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0}
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &ru, 2);
}
#endif /* TWIC_UTIL_RU_SERVICE */

/* Immediate Alert Service */
#if defined(TWIC_UTIL_IA_SERVICE)
uint8_t twic_util_ia_alert_level[] = {TWIC_UTIL_IA_ALERT_LEVEL};
static twicStatus_t
twicUtInitIAService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ia = { TWIC_UTIL_IA_SERVICE_SET, TWIC_UTIL_IA_SERVICE_LEN, {
      {
        0x2A06, 0, TWIC_UUID16, 0x04, 0x703, twic_util_ia_alert_level,
        sizeof(twic_util_ia_alert_level),
        0, 0, NULL, 0
      },
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0}
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &ia, 1);
}
#endif /* TWIC_UTIL_IA_SERVICE */

/* Health Thermometer Service */
#if defined(TWIC_UTIL_HT_SERVICE)
uint8_t twic_util_ht_measurement[] = {TWIC_UTIL_HT_MEASUREMENT};
uint8_t twic_util_ht_mea_desc[] = {TWIC_UTIL_HT_MEA_DESC};
uint8_t twic_util_ht_type[] = {TWIC_UTIL_HT_TYPE};
uint8_t twic_util_ht_ime_measurement[] = {TWIC_UTIL_HT_IME_MEASUREMENT};
uint8_t twic_util_ht_ime_desc[] = {TWIC_UTIL_HT_IME_DESC};
uint8_t twic_util_ht_interval[] = {TWIC_UTIL_HT_INTERVAL};
uint8_t twic_util_ht_int_desc[] = {TWIC_UTIL_HT_INT_DESC};
uint8_t twic_util_ht_valid_range[] = {TWIC_UTIL_HT_VALID_RANGE};
static twicStatus_t
twicUtInitHTService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t ht = { TWIC_UTIL_HT_SERVICE_SET, TWIC_UTIL_HT_SERVICE_LEN, {
      {
        0x2A1C, 0, TWIC_UUID16, 0x20, 0x703, twic_util_ht_measurement,
        sizeof(twic_util_ht_measurement),
        0x2902, 0x703, twic_util_ht_mea_desc, sizeof(twic_util_ht_mea_desc)
      },
      {
        0x2A1D, 0, TWIC_UUID16, 0x02, 0x703, twic_util_ht_type,
        sizeof(twic_util_ht_type),
        0, 0, NULL, 0
      },
      {
        0x2A1E, 0, TWIC_UUID16, 0x10, 0x703, twic_util_ht_ime_measurement,
        sizeof(twic_util_ht_ime_measurement),
        0x2902, 0x703, twic_util_ht_ime_desc, sizeof(twic_util_ht_ime_desc)
      },
      {
        0x2A21, 0, TWIC_UUID16, 0x2A, 0x70B, twic_util_ht_interval,
        sizeof(twic_util_ht_interval),
        0x2902, 0x703, twic_util_ht_int_desc, sizeof(twic_util_ht_int_desc)
      },    
      {
        0, 0, 0, 0, 0, NULL, 0,
        0x2906, 0x701, twic_util_ht_valid_range,
        sizeof(twic_util_ht_valid_range)
      }
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &ht, 5);
}
#endif /* TWIC_UTIL_HT_SERVICE */

/* User defined Service 128bit UUID (Experimental Temporary 128bit Service) */
#if defined(TWIC_UTIL_UU_SERVICE)
uint8_t twic_util_uu_data[] = {TWIC_UTIL_UU_DATA};
uint8_t twic_util_uu_data_desc[] = {TWIC_UTIL_UU_DATA_DESC};
static twicStatus_t
twicUtInitUUService(twicConnIface_t *cif, uint8_t *aret, uint8_t eidx)
{
  appGattDb_t uu = { TWIC_UTIL_UU_SERVICE_SET, TWIC_UTIL_UU_SERVICE_LEN, {
      {
        TWIC_UTIL_UU_CHARACTERISTICS_SET, TWIC_UTIL_UU_CHARACTERISTICS_LEN,
        0x0A, 0x703, twic_util_uu_data, sizeof(twic_util_uu_data),
        0x2902, 0x703, twic_util_uu_data_desc, sizeof(twic_util_uu_data_desc)
      },
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0},
      {0, 0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0}
    },
  };
  
  return twicUtInitService(cif, aret, eidx, &uu, 1);
}
#endif /* TWIC_UTIL_UU_SERVICE */

/* All of using entries in this TZ1K BLE. */

#if defined(TWIC_UTIL_GA_SERVICE)
/* GA Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_ga_service;
twicEntry_t twic_util_entry_ga_device_name;
twicEntry_t twic_util_entry_ga_appearance;
#endif /* TWIC_UTIL_GA_SERVICE */
#if defined(TWIC_UTIL_DI_SERVICE)
/* DI Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_di_service;
twicEntry_t twic_util_entry_di_manufname;
twicEntry_t twic_util_entry_di_fw_version;
twicEntry_t twic_util_entry_di_sw_version;
twicEntry_t twic_util_entry_di_model_string;
twicEntry_t twic_util_entry_di_system_id;
#endif /* TWIC_UTIL_DI_SERVICE */
#if defined(TWIC_UTIL_HR_SERVICE)
/* HR Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_hr_service;
twicEntry_t twic_util_entry_hr_measurement;
twicEntry_t twic_util_entry_hr_mea_desc;
twicEntry_t twic_util_entry_hr_sensor_loc;
twicEntry_t twic_util_entry_hr_control_po;
#endif /* TWIC_UTIL_HR_SERVICE */
#if defined(TWIC_UTIL_BP_SERVICE)
/* Blood Pressure Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_bp_service;
twicEntry_t twic_util_entry_bp_measurement;
twicEntry_t twic_util_entry_bp_mea_desc;
twicEntry_t twic_util_entry_bp_cuff_pressure;
twicEntry_t twic_util_entry_bp_cuff_desc;
twicEntry_t twic_util_entry_bp_feature;
#endif /* TWIC_UTIL_BP_SERVICE */
#if defined(TWIC_UTIL_UD_SERVICE)
/* User Data Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_ud_service;
twicEntry_t twic_util_entry_ud_first_name;
twicEntry_t twic_util_entry_ud_index;
twicEntry_t twic_util_entry_ud_increment;
twicEntry_t twic_util_entry_ud_inc_desc;
twicEntry_t twic_util_entry_ud_control_point;
twicEntry_t twic_util_entry_ud_con_desc;
#endif /* TWIC_UTIL_UD_SERVICE */
#if defined(TWIC_UTIL_CT_SERVICE)
/* CT Service Descripter and Characteristic attribute data. */
twicEntry_t twic_util_entry_ct_service;
twicEntry_t twic_util_entry_ct_current_time;
twicEntry_t twic_util_entry_ct_cur_desc;
twicEntry_t twic_util_entry_ct_local_time_info;
twicEntry_t twic_util_entry_ct_loc_desc;
twicEntry_t twic_util_entry_ct_reference;
#endif /* TWIC_UTIL_CT_SERVICE */
#if defined(TWIC_UTIL_NC_SERVICE)
/* Next DST Change Service */
twicEntry_t twic_util_entry_nc_service;
twicEntry_t twic_util_entry_nc_time_with_dst;
#endif /* TWIC_UTIL_NC_SERVICE */
#if defined(TWIC_UTIL_RU_SERVICE)
/* Reference Time Update Service */
twicEntry_t twic_util_entry_ru_service;
twicEntry_t twic_util_entry_ru_control_point;
twicEntry_t twic_util_entry_ru_state;
#endif /* TWIC_UTIL_RU_SERVICE */
#if defined(TWIC_UTIL_IA_SERVICE)
/* Immediate Alert Service */
twicEntry_t twic_util_entry_ia_service;
twicEntry_t twic_util_entry_ia_alert_level;
#endif /* TWIC_UTIL_IA_SERVICE */
#if defined(TWIC_UTIL_HT_SERVICE)
/* Health Thermometer Service */
twicEntry_t twic_util_entry_ht_service;
twicEntry_t twic_util_entry_ht_measurement;
twicEntry_t twic_util_entry_ht_mea_desc;
twicEntry_t twic_util_entry_ht_type;
twicEntry_t twic_util_entry_ht_ime_measurement;
twicEntry_t twic_util_entry_ht_ime_desc;
twicEntry_t twic_util_entry_ht_interval;
twicEntry_t twic_util_entry_ht_int_desc;
twicEntry_t twic_util_entry_ht_valid_range;
#endif /* TWIC_UTIL_HT_SERVICE */
#if defined(TWIC_UTIL_UU_SERVICE)
/* User defined Service 128bit UUID (Experimental Temporary 128bit Service) */
twicEntry_t twic_util_entry_uu_service;
twicEntry_t twic_util_entry_uu_data;
twicEntry_t twic_util_entry_uu_data_desc;
#endif /* TWIC_UTIL_UU_SERVICE */

/* Create the entities. */
TWIC_CONN_APP_DEF(pnet1, EIDX_NUM) = {
#if defined(TWIC_UTIL_GA_SERVICE)
/* GA Service Descripter and Characteristic attribute data. */
  &twic_util_entry_ga_service,
  &twic_util_entry_ga_device_name,
  &twic_util_entry_ga_appearance,
#endif /* TWIC_UTIL_GA_SERVICE */
#if defined(TWIC_UTIL_DI_SERVICE)
/* DI Service Descripter and Characteristic attribute data. */
  &twic_util_entry_di_service,
  &twic_util_entry_di_manufname,
  &twic_util_entry_di_fw_version,
  &twic_util_entry_di_sw_version,
  &twic_util_entry_di_model_string,
  &twic_util_entry_di_system_id,
#endif /* TWIC_UTIL_DI_SERVICE */
#if defined(TWIC_UTIL_HR_SERVICE)
/* HR Service Descripter and Characteristic attribute data. */
  &twic_util_entry_hr_service,
  &twic_util_entry_hr_measurement,
  &twic_util_entry_hr_mea_desc,
  &twic_util_entry_hr_sensor_loc,
  &twic_util_entry_hr_control_po,
#endif /* TWIC_UTIL_HR_SERVICE */
#if defined(TWIC_UTIL_BP_SERVICE)
/* Blood Pressure Service Descripter and Characteristic attribute data. */
  &twic_util_entry_bp_service,
  &twic_util_entry_bp_measurement,
  &twic_util_entry_bp_mea_desc,
  &twic_util_entry_bp_cuff_pressure,
  &twic_util_entry_bp_cuff_desc,
  &twic_util_entry_bp_feature,
#endif /* TWIC_UTIL_BP_SERVICE */
#if defined(TWIC_UTIL_UD_SERVICE)
/* User Data Service Descripter and Characteristic attribute data. */
  &twic_util_entry_ud_service,
  &twic_util_entry_ud_first_name,
  &twic_util_entry_ud_index,
  &twic_util_entry_ud_increment,
  &twic_util_entry_ud_inc_desc,
  &twic_util_entry_ud_control_point,
  &twic_util_entry_ud_con_desc,
#endif /* TWIC_UTIL_UD_SERVICE */
#if defined(TWIC_UTIL_CT_SERVICE)
/* CT Service Descripter and Characteristic attribute data. */
  &twic_util_entry_ct_service,
  &twic_util_entry_ct_current_time,
  &twic_util_entry_ct_cur_desc,
  &twic_util_entry_ct_local_time_info,
  &twic_util_entry_ct_loc_desc,
  &twic_util_entry_ct_reference,
#endif /* TWIC_UTIL_CT_SERVICE */
#if defined(TWIC_UTIL_NC_SERVICE)
/* Next DST Change Service */
  &twic_util_entry_nc_service,
  &twic_util_entry_nc_time_with_dst,
#endif /* TWIC_UTIL_NC_SERVICE */
#if defined(TWIC_UTIL_RU_SERVICE)
/* Reference Time Update Service */
  &twic_util_entry_ru_service,
  &twic_util_entry_ru_control_point,
  &twic_util_entry_ru_state,
#endif /* TWIC_UTIL_RU_SERVICE */
#if defined(TWIC_UTIL_IA_SERVICE)
/* Immediate Alert Service */
  &twic_util_entry_ia_service,
  &twic_util_entry_ia_alert_level,
#endif /* TWIC_UTIL_IA_SERVICE */
#if defined(TWIC_UTIL_HT_SERVICE)
/* Health Thermometer Service */
  &twic_util_entry_ht_service,
  &twic_util_entry_ht_measurement,
  &twic_util_entry_ht_mea_desc,
  &twic_util_entry_ht_type,
  &twic_util_entry_ht_ime_measurement,
  &twic_util_entry_ht_ime_desc,
  &twic_util_entry_ht_interval,
  &twic_util_entry_ht_int_desc,
  &twic_util_entry_ht_valid_range,
#endif /* TWIC_UTIL_HT_SERVICE */
#if defined(TWIC_UTIL_UU_SERVICE)
/* User defined Service 128bit UUID (Experimental Temporary 128bit Service) */
  &twic_util_entry_uu_service,
  &twic_util_entry_uu_data,
  &twic_util_entry_uu_data_desc,
#endif /* TWIC_UTIL_UU_SERVICE */
};

/* @brief Initialize GATT Service.
 * This API creates and initializes each GATT Service which is defined
 * in the tz1sm_config.h
 * This API can be invoked after the "twicUtLeCeInit3" succeeds.
 * The following GATT Services can be used if the each preprocessor is
 * defined.
 * GA Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_GA_SERVICE
 * DI Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_DI_SERVICE
 * HR Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_HR_SERVICE
 * Blood Pressure Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_BP_SERVICE
 * User Data Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_UD_SERVICE
 * CT Service Descripter and Characteristic attribute data.
 * TWIC_UTIL_CT_SERVICE
 * Next DST Change Service
 * TWIC_UTIL_NC_SERVICE
 * Reference Time Update Service
 * TWIC_UTIL_RU_SERVICE
 * Immediate Alert Service
 * TWIC_UTIL_IA_SERVICE
 * Health Thermometer Service
 * TWIC_UTIL_HT_SERVICE
 * User defined Service 128bit UUID (Experimental Temporary 128bit Service)
 * TWIC_UTIL_UU_SERVICE
 * TWIC_UTIL_UU_CHARACTERISTICS
 * ANCS
 * TWIC_UTIL_ANCS
 *
 * @param twicConnIface_t * const conn_iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 *
 * @return
 * TWIC_STATUS_OK: Success
 * Other: Failed */

twicStatus_t twicUtInitGattService(twicConnIface_t *const cif)
{
  uint8_t aret;
  twicStatus_t status;

  /* appDisplayTwicErrorList(); */

#if defined(TWIC_UTIL_GA_SERVICE)
  status = twicUtInitGAService(cif, &aret, EIDX_GA_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_GA_SERVICE */
#if defined(TWIC_UTIL_DI_SERVICE)
  status = twicUtInitDIService(cif, &aret, EIDX_DI_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_DI_SERVICE */
#if defined(TWIC_UTIL_HR_SERVICE)
  status = twicUtInitHRService(cif, &aret, EIDX_HR_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_HR_SERVICE */
#if defined(TWIC_UTIL_BP_SERVICE)
  status = twicUtInitBPService(cif, &aret, EIDX_BP_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_BP_SERVICE */
#if defined(TWIC_UTIL_UD_SERVICE)
  status = twicUtInitUDService(cif, &aret, EIDX_UD_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_UD_SERVICE */
#if defined(TWIC_UTIL_CT_SERVICE)
  status = twicUtInitCTService(cif, &aret, EIDX_CT_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_CT_SERVICE */
#if defined(TWIC_UTIL_NC_SERVICE)
  status = twicUtInitNCService(cif, &aret, EIDX_NC_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_NC_SERVICE */
#if defined(TWIC_UTIL_RU_SERVICE)
  status = twicUtInitRUService(cif, &aret, EIDX_RU_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_RU_SERVICE */
#if defined(TWIC_UTIL_IA_SERVICE)
  status = twicUtInitIAService(cif, &aret, EIDX_IA_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_IA_SERVICE */
#if defined(TWIC_UTIL_HT_SERVICE)
  status = twicUtInitHTService(cif, &aret, EIDX_HT_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_HT_SERVICE */
#if defined(TWIC_UTIL_UU_SERVICE)
  status = twicUtInitUUService(cif, &aret, EIDX_UU_SERVICE);
  if (TWIC_STATUS_OK != status) {
    twicLog("aret = 0x%x status = 0x%x\r\n", aret, status); twicTrace();
    goto out;
  }
#endif /* TWIC_UTIL_UU_SERVICE */

  out:
  return status;
}

#else

twicStatus_t twicUtInitGattService(twicConnIface_t *const cif)
{ return TWIC_STATUS_OK; }

#endif
