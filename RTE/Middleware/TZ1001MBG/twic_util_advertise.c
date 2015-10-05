#include "twic_interface.h"
#include "twic_util_macro.h"
#include "twic_util_service.h"
#include "twic_util_advertise.h"

/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
   ADVERTISING AND SCAN RESPONSE DATA FORMAT. */

#if defined(TWIC_UTIL_GA_SERVICE) || defined(TWIC_UTIL_DI_SERVICE) || \
  defined(TWIC_UTIL_HR_SERVICE) || defined(TWIC_UTIL_BP_SERVICE) ||   \
  defined(TWIC_UTIL_UD_SERVICE) || defined(TWIC_UTIL_CT_SERVICE) ||   \
  defined(TWIC_UTIL_NC_SERVICE) || defined(TWIC_UTIL_RU_SERVICE) ||   \
  defined(TWIC_UTIL_IA_SERVICE) || defined(TWIC_UTIL_HT_SERVICE) ||   \
  defined(TWIC_UTIL_UU_SERVICE)

uint8_t twic_util_advertising_data[] = {TWIC_UTIL_ADVERTISING_DATA};
uint8_t twic_util_scan_resp_data[] = {TWIC_UTIL_SCAN_RESP_DATA};

#endif
