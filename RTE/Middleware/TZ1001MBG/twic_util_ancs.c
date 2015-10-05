/**
 * @file twic_util_ancs.c
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
#include "twic_util_ancs.h"

#if defined(TWIC_UTIL_ANCS)

/* Multiple piconet is not supported in this sample implementation. */
twicUtAncs_t twicUtAncs;

static void twicUtAncsNcPrimaryServiceByUuid(
  const void * const cif, const uint8_t status, const bool next,
  const uint16_t error_handle, const uint16_t attribute_handle,
  const uint16_t end_group_handle)
{
  twicTrace();
  twicPrintf("status:0x%02x\r\n", status);
  twicPrintf("next:%d\r\n", next);
  twicPrintf("error_handle:0x%04x\r\n", error_handle);
  twicPrintf("attribute_handle:0x%04x\r\n", attribute_handle);
  twicPrintf("end_group_handle:0x%04x\r\n", end_group_handle);

  if (0 == status && 0 == error_handle) {
    twicUtAncs.start_finding_characteristics = true;
  } else {
    twicUtAncs.start_finding_characteristics = false;
  }
  twicUtAncs.start_handle = attribute_handle;
  twicUtAncs.end_handle = end_group_handle;
  twicUtAncs.notification_source_char_val_handle = 0;
  twicUtAncs.control_point_char_val_handle = 0;
  twicUtAncs.data_source_char_val_handle = 0;
  twicUtAncs.data_source_char_desp_handle = 0;
}

static void twicUtAncsNcAllCharOfService(
  const void * const cif, const uint8_t status, const bool next,
  const uint16_t error_handle, const uint16_t attribute_handle,
  const uint8_t char_properties, const uint16_t char_value_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
  twicUuid_t ancs_uuid;

  twicTrace();
  twicPrintf("status:0x%02x\r\n", status);
  twicPrintf("next:%d\r\n", next);
  twicPrintf("error_handle:0x%04x\r\n", error_handle);
  twicPrintf("attribute_handle:0x%04x\r\n", attribute_handle);
  twicPrintf("char_properties:0x%02x\r\n", char_properties);
  twicPrintf("char_value_handle:0x%04x\r\n", char_value_handle);
  twicPrintf("uuid_msb:0x%llx\r\n", uuid_msb);
  twicPrintf("uuid_lsb:0x%llx\r\n", uuid_lsb);
  twicPrintf("uuid_len:0x%02x\r\n", uuid_len);

  twicSetUuid((uint8_t *)&ancs_uuid.uu, uuid_lsb, uuid_msb);
  twicUtPrintVal("UUID", uuid_len, (uint8_t *)&ancs_uuid.uu);

  /* Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD
   * (notifiable) */
  if (0x9FBF120D630142D9 == uuid_msb && 0x8C5825E699A21DBD == uuid_lsb) {
    twicPrintf("FOUND ANCS Notification Source characteristics value.\r\n");
    twicUtAncs.notification_source_char_val_handle = char_value_handle;
    twicUtAncs.notification_source_char_desp_handle = 0;
    twicUtAncs.start_finding_notification_source_descriptor = true;
  }
  /* Control Point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9
   * (writeable with response) */
  if (0x69D1D8F345E149A8 == uuid_msb && 0x98219BBDFDAAD9D9 == uuid_lsb) {
    twicPrintf("FOUND ANCS Control Point characteristics value.\r\n");
    twicUtAncs.control_point_char_val_handle = char_value_handle;
  }
  /* Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable) */
  if (0x22EAC6E924D64BB5 == uuid_msb && 0xBE44B36ACE7C7BFB == uuid_lsb) {
    twicPrintf("FOUND ANCS Data Source characteristics value.\r\n");
    twicUtAncs.data_source_char_val_handle = char_value_handle;
    twicUtAncs.data_source_char_desp_handle = 0;
    twicUtAncs.start_finding_data_source_descriptor = true;
  }
}

static void twicUtAncsNcNsAllCharDescriptors(
  const void * const cif, const uint8_t status, const bool next,
  const uint16_t error_handle, const uint16_t attribute_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
  twicUuid_t ancs_uuid;

  twicTrace();
  twicPrintf("status:0x%02x\r\n", status);
  twicPrintf("next:%d\r\n", next);
  twicPrintf("error_handle:0x%04x\r\n", error_handle);
  twicPrintf("attribute_handle:0x%04x\r\n", attribute_handle);
  twicPrintf("uuid_msb:0x%llx\r\n", uuid_msb);
  twicPrintf("uuid_lsb:0x%llx\r\n", uuid_lsb);
  twicPrintf("uuid_len:0x%02x\r\n", uuid_len);
  twicSetUuid((uint8_t *)&ancs_uuid.uu, uuid_lsb, uuid_msb);
  twicUtPrintVal("UUID", uuid_len, (uint8_t *)&ancs_uuid.uu);

  if (0x2902 == uuid_lsb) {
    twicPrintf("FOUND CHAR DESP of ANCS Notification Source.\r\n");
    twicUtAncs.notification_source_char_desp_handle = attribute_handle;
    twicUtAncs.notification_source_char_desp_value[0] = 0x01;
    twicUtAncs.notification_source_char_desp_value[1] = 0x00;
    twicUtAncs.ready_to_subscribe_notification_source = true;
    twicUtAncs.start_writing_desp_notification_source = true;
  }
}

static void twicUtAncsNcDsAllCharDescriptors(
  const void * const cif, const uint8_t status, const bool next,
  const uint16_t error_handle, const uint16_t attribute_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
  twicUuid_t ancs_uuid;

  twicTrace();
  twicPrintf("status:0x%02x\r\n", status);
  twicPrintf("next:%d\r\n", next);
  twicPrintf("error_handle:0x%04x\r\n", error_handle);
  twicPrintf("attribute_handle:0x%04x\r\n", attribute_handle);
  twicPrintf("uuid_msb:0x%llx\r\n", uuid_msb);
  twicPrintf("uuid_lsb:0x%llx\r\n", uuid_lsb);
  twicPrintf("uuid_len:0x%02x\r\n", uuid_len);
  twicSetUuid((uint8_t *)&ancs_uuid.uu, uuid_lsb, uuid_msb);
  twicUtPrintVal("UUID", uuid_len, (uint8_t *)&ancs_uuid.uu);

  if (0x2902 == uuid_lsb) {
    twicPrintf("FOUND CHAR DESP of ANCS Data Source.\r\n");
    twicUtAncs.data_source_char_desp_handle = attribute_handle;
    twicUtAncs.data_source_char_desp_value[0] = 0x01;
    twicUtAncs.data_source_char_desp_value[1] = 0x00;
    twicUtAncs.ready_to_subscribe_data_source = true;
    twicUtAncs.start_writing_desp_data_source = true;
  }
}

static void twicUtAncsNcNsCharDespWResponse(const void * const cif,
                                            const uint8_t status)
{
  twicTrace();
  twicPrintf("Ns Subscription Status:0x%02x\r\n", status);
  /* iOS7/iOS8 iPodTouch and iPhone6 OK. */
  if (0 == status) {
    twicPrintf("OK.\r\n");
  } else if (0x05 == status) {
    twicPrintf("Failed due to Insufficient Authentication.\r\n");
    twicUtAncs.start_writing_desp_notification_source = true; /* retry */
  } else if (0x0F == status) {
    twicPrintf("Failed due to Insufficient Encryption.\r\n");
    twicUtAncs.start_writing_desp_notification_source = true; /* retry */
  } else {
    twicPrintf("Failed due to unexpected reason.\r\n");
    twicUtAncs.start_writing_desp_notification_source = true; /* retry */
  }
}

static void twicUtAncsNcDsCharDespWResponse(const void * const cif,
                                            const uint8_t status)
{
  twicTrace();
  twicPrintf("Ss Subscription Status:0x%02x\r\n", status);
  /* iOS7/iOS8 iPodTouch and iPhone6 OK. */
  if (0 == status) {
    twicPrintf("OK.\r\n");
  } else if (0x05 == status) {
    twicPrintf("Failed due to Insufficient Authentication.\r\n");
    twicUtAncs.start_writing_desp_data_source = true; /* retry */
  } else if (0x0F == status) {
    twicPrintf("Failed due to Insufficient Encryption.\r\n");
    twicUtAncs.start_writing_desp_data_source = true; /* retry */
  } else {
    twicPrintf("Failed due to unexpected reason.\r\n");
    twicUtAncs.start_writing_desp_data_source = true; /* retry */
  }
}

static void twicUtAncsNcCpCharValueWResponse(const void * const cif,
                                             const uint8_t status)
{
  twicTrace();
  twicPrintf("Writed to the control point: status:0x%02x\r\n", status);
  /* iOS7/iOS8 iPodTouch and iPhone6 OK. */
  if (0 == status) {
    twicPrintf("OK.\r\n");
  } else if (0x05 == status) {
    twicPrintf("Failed due to Insufficient Authentication.\r\n");
    twicUtAncs.start_writing_control_point = true; /* retry */
  } else if (0x0F == status) {
    twicPrintf("Failed due to Insufficient Encryption.\r\n");
    twicUtAncs.start_writing_control_point = true; /* retry */
  } else {
    twicPrintf("Failed due to unexpected reason.\r\n");
    twicUtAncs.start_writing_control_point = true; /* retry */
  }
}

static void twicUtAncsPrepareNc(twicConnIface_t * const cif)
{
  twicStatus_t status;
  
  if (true == twicUtAncs.start_finding_service) {
    if (NULL != twicUtCcb.primary_service_by_uuid) return;
    twicUtCcb.primary_service_by_uuid = &twicUtAncsNcPrimaryServiceByUuid;
    /* The Apple Notification Center Service is a primary service
     * whose service UUID is 7905F431-B5CE-4E99-A40F-4B1E122D00D0. */
    twicLog("'Discover Primary Service By ServiceUuid'.\r\n");
    status = twicUtGattDiscoverPrimaryServiceByServiceUuid(
      cif, 0x0001, /* start handle */ 0xFFFF, /* end_handle */
      0xA40F4B1E122D00D0, /* uuid_lso */ 0x7905F431B5CE4E99, /* mso */ 16);
    twicUtAncs.start_finding_service = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in DiscoverPrimaryServiceByServiceUuid.\r\n");
    }
  }
  if (true == twicUtAncs.start_finding_characteristics) {
    if (NULL != twicUtCcb.all_char_of_service) return;
    twicUtCcb.all_char_of_service = &twicUtAncsNcAllCharOfService;
    twicLog("'Discover All Characteristics'.\r\n");
    status = twicUtGattClientDiscoverAllCharacteristics(
      cif, twicUtAncs.start_handle, twicUtAncs.end_handle);
    twicUtAncs.start_finding_characteristics = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in DiscoverAllCharacteristics.\r\n");
    }
  }
  if (true == twicUtAncs.start_finding_notification_source_descriptor) {
    if (NULL != twicUtCcb.all_char_descriptors) return;
    twicUtCcb.all_char_descriptors = &twicUtAncsNcNsAllCharDescriptors;
    twicLog("'Discover All Descriptors'.\r\n");
    status = twicUtGattClientDiscoverAllDescriptors(
      cif, twicUtAncs.notification_source_char_val_handle + 1,
      twicUtAncs.notification_source_char_val_handle + 1);
    twicUtAncs.start_finding_notification_source_descriptor = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in DiscoverAllDescriptors of Ns.\r\n");
    }
  }
  if (true == twicUtAncs.start_finding_data_source_descriptor) {
    if (NULL != twicUtCcb.all_char_descriptors) return;
    twicUtCcb.all_char_descriptors = &twicUtAncsNcDsAllCharDescriptors;
    twicLog("'Discover All Descriptors'.\r\n");
    status = twicUtGattClientDiscoverAllDescriptors(
      cif, twicUtAncs.data_source_char_val_handle + 1,
      twicUtAncs.data_source_char_val_handle + 1);
    twicUtAncs.start_finding_data_source_descriptor = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in DiscoverAllDescriptors of Ds.\r\n");
    }
  }
}

static void twicUtAncsNcSubscription(twicConnIface_t * const cif)
{
  twicStatus_t status;
  
  if (false == twicUtAncs.ready_to_subscribe_notification_source ||
      false == twicUtAncs.ready_to_subscribe_data_source) return;
  
  if (true == twicUtAncs.start_writing_desp_notification_source) {
    if (NULL != twicUtCcb.char_desp_writein_response) return;
    twicUtCcb.char_desp_writein_response = &twicUtAncsNcNsCharDespWResponse;
    twicLog("'Subscribe Notification Source'.\r\n");
    status = twicUtGattClientWriteCharacteristicDescriptor(
      cif, twicUtAncs.notification_source_char_desp_handle,
      2, twicUtAncs.notification_source_char_desp_value);
    twicUtAncs.start_writing_desp_notification_source = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in WriteCharacteristicDescriptor (0x%x).\r\n", status);
    }
  } else if (true == twicUtAncs.start_writing_desp_data_source) {
    if (NULL != twicUtCcb.char_desp_writein_response) return;
    twicUtCcb.char_desp_writein_response = &twicUtAncsNcDsCharDespWResponse;
    twicLog("'Subscribe Data Source'.\r\n");
    status = twicUtGattClientWriteCharacteristicDescriptor(
      cif, twicUtAncs.data_source_char_desp_handle,
      2, twicUtAncs.data_source_char_desp_value);
    twicUtAncs.start_writing_desp_data_source = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in WriteCharacteristicDescriptor (0x%x).\r\n", status);
    }
  }
}

/* @brief Obtained Primary Service By UUID
 *
 * Application program must register this API to the
 * 'primary_service_by_uuid' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'primary_service_by_uuid' are same.
 * Please just only place this API to the 'primary_service_by_uuid'.
 */
void twicUtAncsPrimaryServiceByUuidCb(const void * const cif,
                                      const uint8_t status, const bool next,
                                      const uint16_t error_handle,
                                      const uint16_t attribute_handle,
                                      const uint16_t end_group_handle)
{
  if (NULL == twicUtCcb.primary_service_by_uuid) return;
  twicUtCcb.primary_service_by_uuid(cif, status, next, error_handle,
                                    attribute_handle, end_group_handle);
  if (false == next) twicUtCcb.primary_service_by_uuid = NULL;
}

/* @brief Obtained All Characteristics of a Service.
 *
 * Application program must register this API to the
 * 'all_char_of_service' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'all_char_of_service' are same.
 * Please just only place this API to the 'all_char_of_service'.
 */
void twicUtAncsAllCharOfServiceCb(const void * const cif, const uint8_t status,
                                  const bool next, const uint16_t error_handle,
                                  const uint16_t attribute_handle,
                                  const uint8_t char_properties,
                                  const uint16_t char_value_handle,
                                  const uint64_t uuid_lsb,
                                  const uint64_t uuid_msb,
                                  const uint8_t uuid_len)
{
  if (NULL == twicUtCcb.all_char_of_service) return;
  twicUtCcb.all_char_of_service(cif, status, next, error_handle,
                                attribute_handle, char_properties,
                                char_value_handle, uuid_lsb, uuid_msb,
                                uuid_len);
  if (false == next) twicUtCcb.all_char_of_service = NULL;
}

/* @brief Obtained All Characteristics Descriptors of a Service.
 *
 * Application program must register this API to the
 * 'all_char_descriptors' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'all_char_descriptors' are same.
 * Please just only place this API to the 'all_char_descriptors'.
 */
void twicUtAncsAllCharDescriptorsCb(const void * const cif,
                                    const uint8_t status, const bool next,
                                    const uint16_t error_handle,
                                    const uint16_t attribute_handle,
                                    const uint64_t uuid_lsb,
                                    const uint64_t uuid_msb,
                                    const uint8_t uuid_len)
{
  if (NULL == twicUtCcb.all_char_descriptors) return;
  twicUtCcb.all_char_descriptors(cif, status, next, error_handle,
                                 attribute_handle, uuid_lsb, uuid_msb,
                                 uuid_len);
  if (false == next) twicUtCcb.all_char_descriptors = NULL;
}

static void twicUtAncsChangeSmpStatus(const uint8_t status)
{
  twicPrintf("status:0x%02x\r\n", status);
  if (0 == status) {
    twicPrintf("Write-in Success.\r\n");
    twicUtAncs.start_pairing_request = false;
    twicUtAncs.start_encryption = false;
  } else if (0x05 == status) {
    twicPrintf("Write-in was failed ");
    twicPrintf("due to Insufficient Authentication.\r\n");
    twicUtAncs.start_pairing_request = true;
    twicUtAncs.start_encryption = false;
  } else if (0x0F == status) {
    twicPrintf("Write-in was failed ");
    twicPrintf("due to Insufficient Encryption.\r\n");
    twicUtAncs.start_pairing_request = false;
    twicUtAncs.start_encryption = true;
  } else {
    twicPrintf("Write-in was failed due to unexpected reason.\r\n");
    twicUtAncs.start_pairing_request = true;
    twicUtAncs.start_encryption = true;
  }
}

/* @brief Response of the written characteristic descriptor.
 *
 * Application program must register this API to the
 * 'char_desp_writein_response' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'char_desp_writein_response' are same.
 * Please just only place this API to the 'char_desp_writein_response'.
 */
void twicUtAncsCharDespWriteinResponseCb(const void * const cif,
                                         const uint8_t status)
{
  twicUtAncsChangeSmpStatus(status);
  if (NULL == twicUtCcb.char_desp_writein_response) return;
  twicUtCcb.char_desp_writein_response(cif, status);
  twicUtCcb.char_desp_writein_response = NULL;
}

/* @brief Response of the written characteristic value.
 *
 * Application program must register this API to the
 * 'char_value_writein_response' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'char_value_writein_response' are same.
 * Please just only place this API to the 'char_value_writein_response'.
 */
void twicUtAncsCharValueWriteinResponseCb(const void * const cif,
                                          const uint8_t status)
{
  twicUtAncsChangeSmpStatus(status);
  if (NULL == twicUtCcb.char_value_writein_response) return;
  twicUtCcb.char_value_writein_response(cif, status);
  twicUtCcb.char_value_writein_response = NULL;
}

/* @brief This API will invoke either the Notification Source Cb or
 *        the Data Source Cb.
 *
 * Application program must register this API to the
 * 'notification_received' function of the 'twicIfLeClientCb_t'.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 *
 * @param const uint8_t status
 * 0xA5 Remote Data Length Error (Data from remote greater then MTU Size)
 * 0x00 No Error.
 *
 * @param const uint16_t char_value_handle
 * ATT Characteristics Value Handle of either the Notification Source
 * or Data Source.
 *
 * @param const twicAttValue_t *const resp
 * The octets length and the data.
 */
void twicUtAncsNcNotificationCb(const void * const cif, const uint8_t status,
                                const uint16_t char_value_handle,
                                const twicAttValue_t *const resp)
{
  twicTrace();
  twicPrintf("status:0x%02x\r\n", status);
  twicPrintf("char_value_handle:0x%04x\r\n", char_value_handle);
  twicUtPrintVal("VALUE", resp->parameter_length, resp->value);

  if (0xA5 == status) {
    twicPrintf("Remote Data Length Error.\r\n");
    return;
  }
  
  if (twicUtAncs.notification_source_char_val_handle == char_value_handle) {
    twicUtAncsNcNsCb(cif, resp);
  } else if (twicUtAncs.data_source_char_val_handle == char_value_handle) {
    twicUtAncsNcDsCb(cif, resp);
  }
}

/* @brief Setup the Notification Consumer.
 *
 * This API setups the Notification Consumer.
 * - First of all the variables of this ANCS sample implementation
 *   shall be cleaned up by this API.
 * - This API can be invoked in the twicIfLeCb_t::connection_complete.
 *   Please refer to the app_sample_profile of the example_ANCS.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
void twicUtAncsSetupNc(twicConnIface_t * const cif)
{
  (void)cif;
  memset((void*)&twicUtAncs, 0, sizeof(twicUtAncs_t));

  twicUtCcb.primary_service_by_uuid = NULL;
  twicUtCcb.all_char_of_service = NULL;
  twicUtCcb.all_char_descriptors = NULL;
  twicUtCcb.all_char_descriptors = NULL;
  twicUtCcb.char_desp_writein_response = NULL;
  twicUtCcb.char_value_writein_response = NULL;
}

/* @brief Start subscribing the Notification Source and Data Source.
 *
 * This API start finding the following ATT handles and subscribing
 * the Notification Source and Data Source.
 *
 * This API is supposed to be invoked from when a central TZ1000
 * starts subscribing the Notification Source and Data Source right
 * after a connection is completed.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
void twicUtAncsStartNc(twicConnIface_t * const cif)
{
  (void)cif;
  if (true == twicUtAncs.ready_to_subscribe_notification_source &&
      true == twicUtAncs.ready_to_subscribe_data_source) {
    twicUtAncs.notification_source_char_desp_value[0] = 0x01;
    twicUtAncs.notification_source_char_desp_value[1] = 0x00;
    twicUtAncs.data_source_char_desp_value[0] = 0x01;
    twicUtAncs.data_source_char_desp_value[1] = 0x00;
    twicUtAncs.start_writing_desp_notification_source = true;
    twicUtAncs.start_writing_desp_data_source = true;
  } else {
    twicUtAncs.start_finding_service = true;
  }
}

/* @brief Stop subscribing the Notification Source and Data Source.
 *
 * This API stop subscribing the Notification Source and Data Source
 * by writing to the Descriptor. 
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
void twicUtAncsStopNc(twicConnIface_t * const cif)
{
  (void)cif;
  twicUtAncs.notification_source_char_desp_value[0] = 0x00;
  twicUtAncs.notification_source_char_desp_value[1] = 0x00;
  twicUtAncs.data_source_char_desp_value[0] = 0x00;
  twicUtAncs.data_source_char_desp_value[1] = 0x00;
  twicUtAncs.start_writing_desp_notification_source = true;
  twicUtAncs.start_writing_desp_data_source = true;
}

/* @brief Write to the Control Point.
 *
 * This API write the specified data by the parameter 'data' to the
 * Control Point.
 *
 * When this API processing is completed characteristic write value
 * call-back API is invoked.
 * void (*char_value_writein_response)( const void * const connif,
 * const uint8_t status)
 *
 * Application program must register this API
 * "twicUtAncsCharDespWriteinResponseCb" to the
 * 'char_desp_writein_response' function of the 'twicIfLeClientCb_t'.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 *
 * @param const uint8_t * const data
 * The data to be written to the Control Point.
 *
 * @param const uint8_t length
 * The octets length of the argument 'data'.
 */
twicStatus_t twicUtAncsWriteToControlPoint(twicConnIface_t * const cif,
                                           uint8_t * const data,
                                           const uint8_t length)
{
  twicStatus_t status;
  
  (void)cif;
  if (false == twicUtAncs.ready_to_subscribe_notification_source ||
      false == twicUtAncs.ready_to_subscribe_data_source) {
    return TWIC_STATUS_OPERATION_NOT_PERMITTED;
  }

  if (true == twicUtAncs.start_writing_control_point) {
    if (NULL != twicUtCcb.char_value_writein_response)
      return TWIC_STATUS_UNDER_PROCESSING;
    twicUtCcb.char_value_writein_response = &twicUtAncsNcCpCharValueWResponse;
    twicLog("Write to the Control Point.\r\n");
    status = twicUtGattClientWriteCharacteristicValue(
      cif, twicUtAncs.control_point_char_val_handle, length, data);
    twicUtAncs.start_writing_control_point = false;
    if (TWIC_STATUS_OK != status) {
      twicLog("Failure in WriteCharacteristicDescriptor (0x%x).\r\n", status);
    }
  }

  return status;
}

/* @brief The process of the this ANCS sample implementation.
 *
 * Application program must periodically invoke this API to process the ANCS.
 * Please refer to the 'app_sample_profile.c' of the 'example_ANCS'
 * about how to invoke this API.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
void twicUtAncsRunNc(twicConnIface_t * const cif)
{
  twicUtAncsPrepareNc(cif);
  twicUtAncsNcSubscription(cif);
}

#else

void twicUtAncsNcNsCb(const void * const cif, const twicAttValue_t *const resp)
{
  (void)cif;
  (void)resp;
}
void twicUtAncsNcDsCb(const void * const cif, const twicAttValue_t *const resp)
{
  (void)cif;
  (void)resp;
}
void twicUtAncsPrimaryServiceByUuidCb(const void * const cif,
                                      const uint8_t status, const bool next,
                                      const uint16_t error_handle,
                                      const uint16_t attribute_handle,
                                      const uint16_t end_group_handle)
{
  (void)cif;
  (void)status;
  (void)next;
  (void)error_handle;
  (void)attribute_handle;
  (void)end_group_handle;
}
void twicUtAncsAllCharOfServiceCb(const void * const cif,
                                  const uint8_t status,
                                  const bool next, const uint16_t error_handle,
                                  const uint16_t attribute_handle,
                                  const uint8_t char_properties,
                                  const uint16_t char_value_handle,
                                  const uint64_t uuid_lsb,
                                  const uint64_t uuid_msb,
                                  const uint8_t uuid_len)
{
  (void)cif;
  (void)status;
  (void)next;
  (void)error_handle;
  (void)attribute_handle;
  (void)char_properties;
  (void)char_value_handle;
  (void)uuid_lsb;
  (void)uuid_msb;
  (void)uuid_len;
}
void twicUtAncsAllCharDescriptorsCb(const void * const cif,
                                    const uint8_t status, const bool next,
                                    const uint16_t error_handle,
                                    const uint16_t attribute_handle,
                                    const uint64_t uuid_lsb,
                                    const uint64_t uuid_msb,
                                    const uint8_t uuid_len)
{
  (void)cif;
  (void)status;
  (void)next;
  (void)error_handle;
  (void)attribute_handle;
  (void)uuid_lsb;
  (void)uuid_msb;
  (void)uuid_len;
}
void twicUtAncsCharDespWriteinResponseCb(const void * const cif,
                                         const uint8_t status)
{
  (void)cif;
  (void)status;
}
void twicUtAncsCharValueWriteinResponseCb(const void * const cif,
                                          const uint8_t status)
{
  (void)cif;
  (void)status;
}
void twicUtAncsNcNotificationCb(const void * const cif, const uint8_t status,
                                const uint16_t char_value_handle,
                                const twicAttValue_t *const resp)
{
  (void)cif;
  (void)status;
  (void)char_value_handle;
  (void)resp;
}
void twicUtAncsSetupNc(twicConnIface_t * const cif)
{
  (void)cif;
}
void twicUtAncsStopNc(twicConnIface_t * const cif)
{
  (void)cif;
}
void twicUtAncsStartNc(twicConnIface_t * const cif)
{
  (void)cif;
}
void twicUtAncsRunNc(twicConnIface_t * const cif)
{
  (void)cif;
}

#endif
