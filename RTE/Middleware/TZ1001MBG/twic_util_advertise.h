#ifndef __TWIC_UTIL_ADVERTISE_H__
#define __TWIC_UTIL_ADVERTISE_H__

#if defined(TWIC_UTIL_GA_SERVICE) || defined(TWIC_UTIL_DI_SERVICE) || \
  defined(TWIC_UTIL_HR_SERVICE) || defined(TWIC_UTIL_BP_SERVICE) ||   \
  defined(TWIC_UTIL_UD_SERVICE) || defined(TWIC_UTIL_CT_SERVICE) ||   \
  defined(TWIC_UTIL_NC_SERVICE) || defined(TWIC_UTIL_RU_SERVICE) ||   \
  defined(TWIC_UTIL_IA_SERVICE) || defined(TWIC_UTIL_HT_SERVICE) ||   \
  defined(TWIC_UTIL_UU_SERVICE)

#define TWIC_UTIL_ADVERTISING_SHORT_LOCAL_NAME /* TZ */ \
  0x54, 0x5a
#define TWIC_UTIL_ADVERTISING_SHORT_LOCAL_NAME_LEN 0x02
#define TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME /* TZ1000 */ \
  0x54, 0x5a, 0x31, 0x30, 0x30, 0x30
#define TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME_LEN 0x06


/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
   ADVERTISING AND SCAN RESPONSE DATA FORMAT. */

#define TWIC_UTIL_ADVERTISING_UUID16_COMPLETE_LIST_LEN  \
  TWIC_UTIL_GA_SERVICE_LEN + TWIC_UTIL_DI_SERVICE_LEN + \
  TWIC_UTIL_HR_SERVICE_LEN + TWIC_UTIL_BP_SERVICE_LEN + \
  TWIC_UTIL_UD_SERVICE_LEN + TWIC_UTIL_CT_SERVICE_LEN + \
  TWIC_UTIL_NC_SERVICE_LEN + TWIC_UTIL_RU_SERVICE_LEN + \
  TWIC_UTIL_IA_SERVICE_LEN + TWIC_UTIL_HT_SERVICE_LEN

#define TWIC_UTIL_ADVERTISING_DATA                              \
/*-----------------------------------------------------------*/ \
  0x02, /* length of this data */                               \
  0x01, /* AD type (Flags) */                                   \
  0x06, /* LE General Discoverable Mode = 0x02 */               \
/* BR/EDR Not Supported */                                      \
/* (i.e. bit 37 of LMP Extended Feature bits Page 0) = 0x04 */  \
/*-----------------------------------------------------------*/ \
  TWIC_UTIL_ADVERTISING_SHORT_LOCAL_NAME_LEN + 0x01,            \
  0x08, /* AD type (Short local name) */                        \
  TWIC_UTIL_ADVERTISING_SHORT_LOCAL_NAME,                       \
/*-----------------------------------------------------------*/ \
  TWIC_UTIL_ADVERTISING_UUID16_COMPLETE_LIST_LEN + 0x01,        \
  0x03, /* AD type (Complete list of 16-bit UUIDs available) */ \
  TWIC_UTIL_GA_SERVICE_ADV /* Generic Access Profile Service */ \
  TWIC_UTIL_DI_SERVICE_ADV /* Device Information Service */     \
  TWIC_UTIL_HR_SERVICE_ADV /* HR Service */                     \
  TWIC_UTIL_BP_SERVICE_ADV /* Blood Pressure */                 \
  TWIC_UTIL_UD_SERVICE_ADV /* User Data Service */              \
  TWIC_UTIL_CT_SERVICE_ADV /* Current time service */           \
  TWIC_UTIL_NC_SERVICE_ADV /* Next DST Change Service */        \
  TWIC_UTIL_RU_SERVICE_ADV /* Reference Time Update Service */  \
  TWIC_UTIL_IA_SERVICE_ADV /* Immediate Alert */                \
  TWIC_UTIL_HT_SERVICE_ADV /* Health Thermometer Service */

#define TWIC_UTIL_ADVERTISING_DATA_LEN                \
  TWIC_UTIL_ADVERTISING_SHORT_LOCAL_NAME_LEN +        \
  TWIC_UTIL_ADVERTISING_UUID16_COMPLETE_LIST_LEN + 0x07
    
#define TWIC_UTIL_SCAN_RESP_DATA_TX                             \
  0x02, /* length of this data */                               \
  0x00, /* AD type (TX Power Level) */                          \
  0x00  /* 0dB (-127...127 = 0x81...0x7F) */                    
#define TWIC_UTIL_SCAN_RESP_DATA_CLN                            \
  TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME_LEN + 0x01,         \
  0x09, /* AD type (Complete local name) */                     \
  TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME

#if defined(TWIC_UTIL_UU_SERVICE)
#define TWIC_UTIL_SCAN_RESP_DATA                                \
  TWIC_UTIL_SCAN_RESP_DATA_TX, TWIC_UTIL_SCAN_RESP_DATA_CLN,    \
/*-----------------------------------------------------------*/ \
  0x11, /* length of this data */                               \
  0x07, /* AD type (Complete list of 128bit UUIDs available) */ \
  TWIC_UTIL_UU_CHARACTERISTICS_ADV
#define TWIC_UTIL_SCAN_RESP_DATA_LEN                    \
  TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME_LEN + 0x17
#else /* !(TWIC_UTIL_UU_SERVICE) */
#define TWIC_UTIL_SCAN_RESP_DATA                            \
  TWIC_UTIL_SCAN_RESP_DATA_TX, TWIC_UTIL_SCAN_RESP_DATA_CLN
#define TWIC_UTIL_SCAN_RESP_DATA_LEN                    \
  TWIC_UTIL_ADVERTISING_COMPLETE_LOCAL_NAME_LEN + 0x05
#endif /* TWIC_UTIL_UU_SERVICE */

extern uint8_t twic_util_advertising_data[TWIC_UTIL_ADVERTISING_DATA_LEN];
extern uint8_t twic_util_scan_resp_data[TWIC_UTIL_SCAN_RESP_DATA_LEN];

#endif

#endif /* __TWIC_UTIL_ADVERTISE_H__ */
