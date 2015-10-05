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

#ifndef __TWIC_UTIL_SERVICE_H__
#define __TWIC_UTIL_SERVICE_H__

/*
 * GA Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_GA_SERVICE)
#define TWIC_UTIL_GA_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_GA_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 00)
#define TWIC_UTIL_GA_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 00),
#define TWIC_UTIL_GA_DEVICE_NAME /* HRTZ1000 */   \
  0x48, 0x52, 0x54, 0x5a, 0x31, 0x30, 0x30, 0x30, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
extern uint8_t twic_util_ga_device_name[16];
#define TWIC_UTIL_GA_APPEARANCE 0x00, 0x00
extern uint8_t twic_util_ga_appearance[2];
#else
#define TWIC_UTIL_GA_SERVICE_LEN 0
#define TWIC_UTIL_GA_SERVICE_ADV
#endif /* TWIC_UTIL_GA_SERVICE */

/*
 * DI Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_DI_SERVICE)
#define TWIC_UTIL_DI_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_DI_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 0A)
#define TWIC_UTIL_DI_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 0A),
#define TWIC_UTIL_DI_MANUFNAME /* TOSHIBA */ \
  0x54, 0x4f, 0x53, 0x48, 0x49, 0x42, 0x41 
extern uint8_t twic_util_di_manufname[7];
#define TWIC_UTIL_DI_FW_VERSION 0x01
extern uint8_t twic_util_di_fw_version[1];
#define TWIC_UTIL_DI_SW_VERSION 0x01
extern uint8_t twic_util_di_sw_version[1];
#define TWIC_UTIL_DI_MODEL_STRING /* TZ1000 */ \
  0x54, 0x5a, 0x31, 0x30, 0x30, 0x30
extern uint8_t twic_util_di_model_string[6];
#define TWIC_UTIL_DI_SYSTEM_ID \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_di_system_id[8];
#else
#define TWIC_UTIL_DI_SERVICE_LEN 0
#define TWIC_UTIL_DI_SERVICE_ADV
#endif /* TWIC_UTIL_DI_SERVICE */

/*
 * HR Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_HR_SERVICE)
#define TWIC_UTIL_HR_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_HR_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 0D)
#define TWIC_UTIL_HR_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 0D),
#define TWIC_UTIL_HR_MEASUREMENT                \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_hr_measurement[6];
#define TWIC_UTIL_HR_MEA_DESC 0x00, 0x00
extern uint8_t twic_util_hr_mea_desc[2];
#define TWIC_UTIL_HR_SENSOR_LOCATION 0x00
extern uint8_t twic_util_hr_sensor_location[1];
#define TWIC_UTIL_HR_CONTROL_POINT 0x00
extern uint8_t twic_util_hr_control_point[1];
#else
#define TWIC_UTIL_HR_SERVICE_LEN 0
#define TWIC_UTIL_HR_SERVICE_ADV
#endif /* TWIC_UTIL_HR_SERVICE */

/*
 * Blood Pressure Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_BP_SERVICE)
#define TWIC_UTIL_BP_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_BP_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 10)
#define TWIC_UTIL_BP_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 10),
/* BP Measurement Value (Char Properties : Indicate)
   Systolic : xx mmHg, Diastolic : xx mmHg, MAP : xx mmHg
   Time stamp : xxth February xx:xx:xx, Pulse Rate : xx bpm
   Mesaurement Status : 0x0000
   Range : 112,80,90 mmHg : 120,90,95 mmHg" */
#define TWIC_UTIL_BP_MEASUREMENT \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_bp_measurement[12];
#define TWIC_UTIL_BP_MEA_DESC 0x00, 0x00
extern uint8_t twic_util_bp_mea_desc[2];
/* Intermediate Cuff Pressure Value (Char Properties : Nortify)
   Cuff Pressure Value : xx mmHg, Time stamp : xxth February xx:xx:xx
   Pulse Rate : xx bpm, Mesaurement Status : 0x0000
   Range : 112 mmHg : 120 mmHg */
#define TWIC_UTIL_BP_CUFF_PRESSURE \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_bp_cuff_pressure[14];
#define TWIC_UTIL_BP_CUFF_DESC 0x00, 0x00
extern uint8_t twic_util_bp_cuff_desc[];
/* Blood Pressure Feature Value (Char Properties : Read)
   (Body Movement Detection , Cuff Fit Detection,
   Irregular Pulse Detection , Pulse Rate Range Detection ,
   Measurement Position Detection Supported) */
#define TWIC_UTIL_BP_FEATURE 0x00, 0x00
extern uint8_t twic_util_bp_feature[2];
#else
#define TWIC_UTIL_BP_SERVICE_LEN 0
#define TWIC_UTIL_BP_SERVICE_ADV
#endif /* TWIC_UTIL_BP_SERVICE */

/*
 * User Data Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_UD_SERVICE)
#define TWIC_UTIL_UD_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_UD_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 1C)
#define TWIC_UTIL_UD_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 1C),
/* First Name (Read + Write) */
#define TWIC_UTIL_UD_FIRST_NAME 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ud_first_name[4];
/* User Index (Read) */
#define TWIC_UTIL_UD_INDEX 0x00
extern uint8_t twic_util_ud_index[1];
/* Database Change Increment (Read + Write + Notify) */
#define TWIC_UTIL_UD_INCREMENT 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ud_increment[4];
#define TWIC_UTIL_UD_INC_DESC 0x00, 0x00
extern uint8_t twic_util_ud_inc_desc[2];
/* User Control Point (Write + Indicate) */
#define TWIC_UTIL_UD_CONTROL_POINT 0x00, 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ud_control_point[5];
#define TWIC_UTIL_UD_CON_DESC 0x00, 0x00
extern uint8_t twic_util_ud_con_desc[2];
#else
#define TWIC_UTIL_UD_SERVICE_LEN 0
#define TWIC_UTIL_UD_SERVICE_ADV
#endif /* TWIC_UTIL_UD_SERVICE */

/*
 * CT Service Descripter and Characteristic attribute data.
 */
#if defined(TWIC_UTIL_CT_SERVICE)
#define TWIC_UTIL_CT_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_CT_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 05)
#define TWIC_UTIL_CT_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 05),
/* Current Time (Read + Notify + Write) */
#define TWIC_UTIL_CT_CURRENT_TIME \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00
extern uint8_t twic_util_ct_current_time[];
#define TWIC_UTIL_CT_CUR_DESC 0x00, 0x00
extern uint8_t twic_util_ct_cur_desc[];
/* Local Time Information (Read + Notify + Write) */
#define TWIC_UTIL_CT_LOCAL_TIME_INFO 0x00, 0x00
extern uint8_t twic_util_ct_local_time_info[];
#define TWIC_UTIL_CT_LOC_DESC 0x00, 0x00
extern uint8_t twic_util_ct_loc_desc[];
/* Reference Time Information (Read) */
#define TWIC_UTIL_CT_REFERENCE 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ct_reference[];
#else
#define TWIC_UTIL_CT_SERVICE_LEN 0
#define TWIC_UTIL_CT_SERVICE_ADV
#endif /* TWIC_UTIL_CT_SERVICE */

/*
 * Next DST Change Service.
 */
#if defined(TWIC_UTIL_NC_SERVICE)
#define TWIC_UTIL_NC_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_NC_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 07)
#define TWIC_UTIL_NC_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 07),
/* Time With DST (Read) */
#define TWIC_UTIL_NC_TIME_WITH_DST \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_nc_time_with_dst[];
#else
#define TWIC_UTIL_NC_SERVICE_LEN 0
#define TWIC_UTIL_NC_SERVICE_ADV
#endif /* TWIC_UTIL_NC_SERVICE */

/*
 * Reference Time Update Service.
 */
#if defined(TWIC_UTIL_RU_SERVICE)
#define TWIC_UTIL_RU_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_RU_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 06)
#define TWIC_UTIL_RU_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 06),
/* Time Update Control Point (Write Without Response) */
#define TWIC_UTIL_RU_CONTROL_POINT 0x00
/* Time Update State (Read) */
#define TWIC_UTIL_RU_STATE 0x00, 0x00
extern uint8_t twic_util_ru_control_point[];
extern uint8_t twic_util_ru_state[];
#else
#define TWIC_UTIL_RU_SERVICE_LEN 0
#define TWIC_UTIL_RU_SERVICE_ADV
#endif /* TWIC_UTIL_RU_SERVICE */

/*
 * Immediate Alert Service.
 */
#if defined(TWIC_UTIL_IA_SERVICE)
#define TWIC_UTIL_IA_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_IA_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 02)
#define TWIC_UTIL_IA_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 02),
/* Alert Level (Write Without Response) */
#define TWIC_UTIL_IA_ALERT_LEVEL 0x00
extern uint8_t twic_util_ia_alert_level[];
#else
#define TWIC_UTIL_IA_SERVICE_LEN 0
#define TWIC_UTIL_IA_SERVICE_ADV
#endif /* TWIC_UTIL_IA_SERVICE */

/*
 * Health Thermometer Service.
 */
#if defined(TWIC_UTIL_HT_SERVICE)
#define TWIC_UTIL_HT_SERVICE_LEN TWIC_UUID16
#define TWIC_UTIL_HT_SERVICE_SET TWIC_UTIL_UUID16_U64SET(0x18, 09)
#define TWIC_UTIL_HT_SERVICE_ADV TWIC_UTIL_UUID16(0x18, 09),
/* Temperature Measurement  Value
   xx.x degC with Time Stamp of xxth February yyyy hh:mm:ss
   Range : 30.0 degC - 100.0 degC */
#define TWIC_UTIL_HT_MEASUREMENT \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ht_measurement[12];
#define TWIC_UTIL_HT_MEA_DESC 0x00, 0x00
extern uint8_t twic_util_ht_mea_desc[2];
/* Temperature Type Value (0x02 - Body general) */
#define TWIC_UTIL_HT_TYPE 0x00
extern uint8_t twic_util_ht_type[1];
/* Intermediate Temperature Measurement Value
   37.0 degC with Time Stamp of 16th February 2015 10:00:00
   Range : 30.0 degC - 100.0 degC */
#define TWIC_UTIL_HT_IME_MEASUREMENT              \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ht_ime_measurement[12];
#define TWIC_UTIL_HT_IME_DESC 0x00, 0x00
extern uint8_t twic_util_ht_ime_desc[2];
/* Measurement Interval (Read + Authenticated Write + Indicate) */
#define TWIC_UTIL_HT_INTERVAL 0x00, 0x00
extern uint8_t twic_util_ht_interval[2];
#define TWIC_UTIL_HT_INT_DESC 0x00, 0x00
extern uint8_t twic_util_ht_int_desc[2];
/* Valid Range Descriptor (Min Value - 2 sec Max Value - 10 mins) */
#define TWIC_UTIL_HT_VALID_RANGE 0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_ht_valid_range[4];
#else
#define TWIC_UTIL_HT_SERVICE_LEN 0
#define TWIC_UTIL_HT_SERVICE_ADV
#endif /* TWIC_UTIL_HT_SERVICE */

/*
 * User defined Service 128bit UUID (Experimental Temporary 128bit Service).
 */
#if defined(TWIC_UTIL_UU_SERVICE)
#define TWIC_UTIL_UU_SERVICE_LEN TWIC_UUID128
#define TWIC_UTIL_UU_SERVICE_SET TWIC_UTIL_UUID128_U64SET(0xc6, 35, 4e, 9c, 91, 19, 4b, ea, 90, 69, 6a, 7f, 0a, e2, 87, 05)
#define TWIC_UTIL_UU_SERVICE_ADV TWIC_UTIL_UUID128(0xc6, 35, 4e, 9c, 91, 19, 4b, ea, 90, 69, 6a, 7f, 0a, e2, 87, 05),
#define TWIC_UTIL_UU_CHARACTERISTICS_LEN TWIC_UUID128
#define TWIC_UTIL_UU_CHARACTERISTICS_SET TWIC_UTIL_UUID128_U64SET(0xf8, 9a, 41, 69, 9c, 66, 4c, 00, 87, 82, 82, 86, cb, 0e, 05, 7a)
#define TWIC_UTIL_UU_CHARACTERISTICS_ADV TWIC_UTIL_UUID128(0xf8, 9a, 41, 69, 9c, 66, 4c, 00, 87, 82, 82, 86, cb, 0e, 05, 7a),
#define TWIC_UTIL_UU_DATA \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00
extern uint8_t twic_util_uu_data[20];
#define TWIC_UTIL_UU_DATA_DESC 0x00, 0x00
extern uint8_t twic_util_uu_data_desc[2];
#else
#define TWIC_UTIL_UU_SERVICE_LEN 0
#define TWIC_UTIL_UU_SERVICE_ADV
#endif /* TWIC_UTIL_HT_SERVICE */


enum APP_TWIC_EIDX {
#if defined(TWIC_UTIL_GA_SERVICE)
/* GA Service Descripter and Characteristic attribute data. */
  EIDX_GA_SERVICE = 1,
  EIDX_GA_DEVICE_NAME,
  EIDX_GA_APPEARANCE,
#endif /* TWIC_UTIL_GA_SERVICE */
#if defined(TWIC_UTIL_DI_SERVICE)
/* DI Service Descripter and Characteristic attribute data. */
  EIDX_DI_SERVICE,
  EIDX_DI_MANUFNAME,
  EIDX_DI_FW_VERSION,
  EIDX_DI_SW_VERSION,
  EIDX_DI_MODEL_STRING,
  EIDX_DI_SYSTEM_ID,
#endif /* TWIC_UTIL_DI_SERVICE */
#if defined(TWIC_UTIL_HR_SERVICE)
/* HR Service Descripter and Characteristic attribute data. */
  EIDX_HR_SERVICE,
  EIDX_HR_MEASUREMENT,
  EIDX_HR_MEA_DESC,
  EIDX_HR_SENSOR_LOC,
  EIDX_HR_CONTROL_PO,
#endif /* TWIC_UTIL_HR_SERVICE */
#if defined(TWIC_UTIL_BP_SERVICE)
/* Blood Pressure Service Descripter and Characteristic attribute data. */
  EIDX_BP_SERVICE,
  EIDX_BP_MEASUREMENT,
  EIDX_BP_MEA_DESC,
  EIDX_BP_CUFF_PRESSURE,
  EIDX_BP_CUFF_DESC,
  EIDX_BP_FEATURE,
#endif /* TWIC_UTIL_BP_SERVICE */
#if defined(TWIC_UTIL_UD_SERVICE)
/* User Data Service Descripter and Characteristic attribute data. */
  EIDX_UD_SERVICE,
  EIDX_UD_FIRST_NAME,
  EIDX_UD_INDEX,
  EIDX_UD_INCREMENT,
  EIDX_UD_INC_DESC,
  EIDX_UD_CONTROL_POINT,
  EIDX_UD_CON_DESC,
#endif /* TWIC_UTIL_UD_SERVICE */
#if defined(TWIC_UTIL_CT_SERVICE)
/* CT Service Descripter and Characteristic attribute data. */
  EIDX_CT_SERVICE,
  EIDX_CT_CURRENT_TIME,
  EIDX_CT_CUR_DESC,
  EIDX_CT_LOCAL_TIME_INFO,
  EIDX_CT_LOC_DESC,
  EIDX_CT_REFERENCE,
#endif /* TWIC_UTIL_CT_SERVICE */
#if defined(TWIC_UTIL_NC_SERVICE)
/* Next DST Change Service */
  EIDX_NC_SERVICE,
  EIDX_NC_TIME_WITH_DST,
#endif /* TWIC_UTIL_NC_SERVICE */
#if defined(TWIC_UTIL_RU_SERVICE)
/* Reference Time Update Service */
  EIDX_RU_SERVICE,
  EIDX_RU_CONTROL_POINT,
  EIDX_RU_STATE,
#endif /* TWIC_UTIL_RU_SERVICE */
#if defined(TWIC_UTIL_IA_SERVICE)
/* Immediate Alert Service */
  EIDX_IA_SERVICE,
  EIDX_IA_ALERT_LEVEL,
#endif /* TWIC_UTIL_IA_SERVICE */
#if defined(TWIC_UTIL_HT_SERVICE)
/* Health Thermometer Service */
  EIDX_HT_SERVICE,
  EIDX_HT_MEASUREMENT,
  EIDX_HT_MEA_DESC,
  EIDX_HT_TYPE,
  EIDX_HT_IME_MEASUREMENT,
  EIDX_HT_IME_DESC,
  EIDX_HT_INTERVAL,
  EIDX_HT_INT_DESC,
  EIDX_HT_VALID_RANGE,
#endif /* TWIC_UTIL_HT_SERVICE */
#if defined(TWIC_UTIL_UU_SERVICE)
/* User defined Service 128bit UUID (Experimental Temporary 128bit Service) */
  EIDX_UU_SERVICE,
  EIDX_UU_DATA,
  EIDX_UU_DATA_DESC,
#endif /* TWIC_UTIL_UU_SERVICE */
/* END */
  EIDX_END,
  EIDX_NUM = EIDX_END - 1,
};

extern twicStatus_t twicUtInitGattService(twicConnIface_t *const cif);

TWIC_EXTERN_IF(pnet1);

#endif /* __TWIC_UTIL_SERVICE_H__ */
