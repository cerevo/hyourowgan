/**
 * @file twic_util_ancs.h
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

#ifndef __TWIC_UTIL_ANCS_H__
#define __TWIC_UTIL_ANCS_H__

/* Multiple piconet is not supported in this sample implementation. */
typedef struct twicUtAncs {
  uint8_t start_finding_service : 1;
  uint8_t start_finding_characteristics : 1;
  uint8_t start_finding_notification_source_descriptor : 1;
  uint8_t start_finding_data_source_descriptor : 1;
  
  uint8_t start_writing_desp_notification_source : 1;
  uint8_t start_writing_desp_data_source : 1;
  uint8_t start_writing_control_point : 1;
  uint8_t ready_to_subscribe_notification_source : 1;

  uint8_t ready_to_subscribe_data_source : 1;
  uint8_t start_pairing_request : 1;
  uint8_t start_encryption : 1;
  uint8_t reserved : 5;

  uint16_t data_source_char_desp_handle;
  
  uint16_t start_handle;
  uint16_t end_handle;

  uint16_t notification_source_char_val_handle;
  uint16_t control_point_char_val_handle;

  uint16_t data_source_char_val_handle;
  uint16_t notification_source_char_desp_handle;

  uint8_t notification_source_char_desp_value[2];
  uint8_t data_source_char_desp_value[2];
} twicUtAncs_t;
extern twicUtAncs_t twicUtAncs;

/* Reference Manual : The Apple Notification Center Service. */
/* Notification Source characteristic. */
typedef TZ1K_PACKED_HDR struct {
  uint8_t event_id;
  /* This field informs the accessory whether the given iOS
   * notification was added, modified, or removed. The enumerated
   * values for this field are defined in EventID Values (page 16). */
  uint8_t event_flags;
  /* A bitmask whose set bits inform an NC of specificities with the
   * iOS notification. For example, if an iOS notification is
   * considered "important", the NC may want to display a more
   * aggressive user interface (UI) to make sure the user is properly
   * alerted. The enumerated bits for this field are defined in
   * event_fags (page 16). */
  uint8_t category_id;
  /* A numerical value providing a category in which the iOS
   * notification can be classified. The NP will make a best effort to
   * provide an accurate category for each iOS notification. The
   * enumerated values for this field are defined in CategoryID Values
   * (page 15). */
  uint8_t category_count;
  /* The current number of active iOS notifications in the given
   * category. For example, if two unread emails are sitting in a
   * user's email inbox, and a new email is pushed to the user's iOS
   * device, the value of CategoryCount is 3. */
  uint32_t notification_uid;
  /* A 32-bit numerical value that is the unique identifier (UID) for
   * the iOS notification. This value can be used as a handle in
   * commands sent to the Control Point characteristic to interact
   * with the iOS notification. */
} TZ1K_PACKED_FTR twicUtAncsNsc_t; /* Notification Source characteristic */

/* Reference Manual : The Apple Notification Center Service. */
/* event_id Values */
#define TWICUTANCS_EVENT_ID_NOTIFICATION_ADDED    (0)
#define TWICUTANCS_EVENT_ID_NOTIFICATION_MODIFIED (1)
#define TWICUTANCS_EVENT_ID_NOTIFICATION_REMOVED  (2)
#define TWICUTANCS_RESERVED_EVENTID_VALUES_XTO255 (3)

/* Reference Manual : The Apple Notification Center Service. */
/* event_flags */
#define TWICUTANCS_EVENT_FLAG_SILENT          (1 << 0)
#define TWICUTANCS_EVENT_FLAG_IMPORTANT       (1 << 1)
#define TWICUTANCS_EVENT_FLAG_PRE_EXISTING    (1 << 2)
#define TWICUTANCS_EVENT_FLAG_POSITIVE_ACTION (1 << 3)
/* A positive action exists and is associated with this iOS notification. */
#define TWICUTANCS_EVENT_FLAG_NEGATIVE_ACTION (1 << 4)
/* A negative action exists and is associated with this iOS notification. */

/* Reference Manual : The Apple Notification Center Service. */
/* category_id Values */
#define TWICUTANCS_CATEGORY_ID_OTHER (0)
#define TWICUTANCS_CATEGORY_ID_INCOMING_CALL (1)
#define TWICUTANCS_CATEGORY_ID_MISSED_CALL (2)
#define TWICUTANCS_CATEGORY_ID_VOICEMAIL (3)
#define TWICUTANCS_CATEGORY_ID_SOCIAL (4)
#define TWICUTANCS_CATEGORY_ID_SCHEDULE (5)
#define TWICUTANCS_CATEGORY_ID_EMAIL (6)
#define TWICUTANCS_CATEGORY_ID_NEWS (7)
#define TWICUTANCS_CATEGORY_ID_HEALTH_AND_FITNESS (8)
#define TWICUTANCS_CATEGORY_ID_BUSINESS_AND_FINANCE (9)
#define TWICUTANCS_CATEGORY_ID_LOCATION (10)
#define TWICUTANCS_CATEGORY_ID_ENTERTAINMENT (11)
#define TWICUTANCS_RESERVED_CATEGORY_ID_VALUES_XTO255 (12)

/* Reference Manual : The Apple Notification Center Service. */
/* Get Notification Attributes. */
/* The Get Notification Attributes command allows an NC to retrieve
 * the attributes of a specific iOS notification. */
typedef TZ1K_PACKED_HDR struct {
  uint8_t command_id;
  /* Should be set to 0 (command_id_get_notification_attributes). */  
  uint32_t notification_uid;
  /* The 32-bit numerical value representing the UID of the iOS
   * notification for which the client wants information. */
  uint8_t *attribute_ids;
  /* A list of attributes that the NC wants to retrieve. Some
   * attributes may need to be followed by a 16-bit. Example: [Attr ID
   * 1][Attr ID 2][Attr ID 2 LEN_LSO][Attr ID 2 LEN_MSO][Attr ID 2
   * Values] */
} TZ1K_PACKED_FTR twicUtAncsGnac_t; /* Get Notification Attributes command */

typedef TZ1K_PACKED_HDR struct {
  uint8_t id;
  uint16_t length;    
  uint8_t *value;
} TZ1K_PACKED_FTR twicUtAncsAttributeList_t;

/* Reference Manual : The Apple Notification Center Service. */
/* Get Notification Attributes. */
/* A response to a Get Notification Attributes command. */
typedef TZ1K_PACKED_HDR struct {
  uint8_t command_id;
  /* Set to 0 (command_id_get_notification_attributes). */
  uint32_t notification_uid;
  /* The 32-bit numerical value that is the UID of the iOS
   * notification the following attributes correspond to. */
  twicUtAncsAttributeList_t *head;
  /* A list of AttributeIDs/16-bit Length/Attribute tuples. An
   * attribute is always a string whose length in bytes is provided in
   * the tuple, but which is not NULL-terminated. If a requested
   * attribute is empty or missing for the iOS notification, its
   * length is set to 0. */
} TZ1K_PACKED_FTR twicUtAncsRgnac_t;
/* NOTE: If the response is larger than the negotiated GATT Maximum
 * Transmission Unit (MTU), it is split into multiple fragments by the
 * NP. The NC must recompose the response by splicing each
 * fragment. The response is complete when the complete tuples for
 * each requested attribute has been received. */

/* Reference Manual : The Apple Notification Center Service. */
/* Get App Attributes. */
/* A Get App Attributes command contains the following information. */
typedef TZ1K_PACKED_HDR struct {
  uint8_t command_id; 
  /* Should be set to 1 (command_id_get_app_attributes). */
  uint8_t *app_identifier;
  /* The string identifier of the app the client wants information
   * about. This string must be NULL-terminated. */
  twicUtAncsAttributeList_t *head;
  /* A list of AttributeIDs/16-bit Length/Attribute tuples. An
   * attribute is always a string whose length in bytes is provided in
   * the tuple, but which is not NULL-terminated. If a requested
   * attribute is empty or missing for the app, its length is set to
   * 0. */
} TZ1K_PACKED_FTR twicUtAncsGaac_t;
/* As with a response to a Get Notification Attributes command, if the
 * response to a Get App Attributes command is larger than the
 * negotiated GATT Maximum Transmission Unit (MTU), it is split into
 * multiple fragments by the NP. The NC must recompose the response by
 * splicing each fragment. The response is complete when the complete
 * tuples for each requested attribute has been received. */

/* Reference Manual : The Apple Notification Center Service. */
/* Perform Notification Action. */
typedef TZ1K_PACKED_HDR struct {
  uint8_t command_id;
  /* Set to 2 (command_id_perform_notification_action). */
  uint32_t notification_uid;
  /* A 32-bit numerical value representing the UID of the iOS
   * notification on which the client wants to perform an action. */
  uint8_t action_id;
  /* The desired action the NC wants to be performed on the iOS
     notification. */
} TZ1K_PACKED_FTR twicUtAncsPna_t;

/* Reference Manual : The Apple Notification Center Service. */
/* command_id Values */
#define TWICUTANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES (0)
#define TWICUTANCS_COMMAND_ID_GET_APP_ATTRIBUTES (1)
#define TWICUTANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION (2)
#define TWICUTANCS_RESERVED_COMMANDID_VALUES_XTO255 (3)

/* Reference Manual : The Apple Notification Center Service. */
/* notification_attribute_id Values */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER (0)
/* UTF-8 strings. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_TITLE (1)
/* Needs to be followed by a 2-bytes max length parameter. UTF-8 strings. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_SUBTITLE (2)
/* Needs to be followed by a 2-bytes max length parameter. UTF-8 strings. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_MESSAGE (3)
/* Needs to be followed by a 2-bytes max length parameter.
 * The format of the notification_attribute_id_message_size constant
 * is a string that represents the integral value of the message
 * size. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_MESSAGE_SIZE (4)
/* UTF-8 strings. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_DATE (5)
/* The format of the notification_attribute_id_date constant is a
 * string that uses the Unicode Technical Standard (UTS) #35 date
 * format pattern yyyyMMdd'T'HHmmSS. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_POSITIVE_ACTION_LABEL (6)
/* The label used to describe the positive action that can be
performed on the iOS notification. UTF-8 strings. */
#define TWICUTANCS_NOTIFICATION_ATTRIBUTE_ID_NEGATIVE_ACTION_LABEL (7)
/* The label used to describe the negative action that can be
performed on the iOS notification. UTF-8 strings. */
#define TWICUTANCS_RESERVED_NOTIFICATION_ATTRIBUTE_ID_VALUES_XTO255 (8)
/* UTF-8 strings. */

/* Reference Manual : The Apple Notification Center Service. */
/* action_id Values */
#define TWICUTANCS_ACTION_ID_POSITIVE (0)
#define TWICUTANCS_ACTION_ID_NEGATIVE (1)
#define TWICUTANCS_RESERVED_ACTION_ID_VALUES_XTO255 (2)

/* Reference Manual : The Apple Notification Center Service. */
/* app_attribute_id_Values */
#define TWICUTANCS_APP_ATTRIBUTE_ID_DISPLAY_NAME (0)
#define TWICUTANCS_RESERVED_APP_ATTRIBUTE_ID_VALUES_XTO255 (1)

/* Reference Manual : The Apple Notification Center Service. */
/* Error Codes */
/* When writing to the Control Point characteristic, an NC may receive
 * the following ANCS-specific error codes: */
#define TWICUTANCS_UNKNOWN_COMMAND (0xA0)
/* The commandID was not recognized by the NP. */
#define TWICUTANCS_INVALID_COMMAND (0xA1)
/* The command was improperly formatted. */
#define TWICUTANCS_INVALID_PARAMETER (0xA2)
/* One of the parameters (for example, the NotificationUID) does not refer to an existing object on the NP. */
#define TWICUTANCS_ACTION_FAILED (0xA3)
/* The action was not performed. */
/* If the NP replies with an error, it will not generate any GATT
 * notification on the Data Source characteristic for the
 * corresponding command. */

/* @brief Obtained Primary Service By UUID
 *
 * Application program must register this API to the
 * 'primary_service_by_uuid' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'primary_service_by_uuid' are same.
 * Please just only place this API to the 'primary_service_by_uuid'.
 */
extern void
twicUtAncsPrimaryServiceByUuidCb(const void * const cif,
                                 const uint8_t status, const bool next,
                                 const uint16_t error_handle,
                                 const uint16_t attribute_handle,
                                 const uint16_t end_group_handle);

/* @brief Obtained All Characteristics of a Service.
 *
 * Application program must register this API to the
 * 'all_char_of_service' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'all_char_of_service' are same.
 * Please just only place this API to the 'all_char_of_service'.
 */
extern void
twicUtAncsAllCharOfServiceCb(const void * const cif, const uint8_t status,
                             const bool next, const uint16_t error_handle,
                             const uint16_t attribute_handle,
                             const uint8_t char_properties,
                             const uint16_t char_value_handle,
                             const uint64_t uuid_lsb, const uint64_t uuid_msb,
                             const uint8_t uuid_len);

/* @brief Obtained All Characteristics Descriptors of a Service.
 *
 * Application program must register this API to the
 * 'all_char_descriptors' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'all_char_descriptors' are same.
 * Please just only place this API to the 'all_char_descriptors'.
 */
extern void
twicUtAncsAllCharDescriptorsCb(const void * const cif, const uint8_t status,
                               const bool next, const uint16_t error_handle,
                               const uint16_t attribute_handle,
                               const uint64_t uuid_lsb, const uint64_t uuid_msb,
                               const uint8_t uuid_len);

/* @brief Response of the written characteristic descriptor.
 *
 * Application program must register this API to the
 * 'char_desp_writein_response' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'char_desp_writein_response' are same.
 * Please just only place this API to the 'char_desp_writein_response'.
 */
extern void twicUtAncsCharDespWriteinResponseCb(const void * const cif,
                                                const uint8_t status);

/* @brief Response of the written characteristic value.
 *
 * Application program must register this API to the
 * 'char_value_writein_response' function of the 'twicIfLeClientCb_t'.
 *
 * @note
 * The parameters of this API and the 'char_value_writein_response' are same.
 * Please just only place this API to the 'char_value_writein_response'.
 */
extern void twicUtAncsCharValueWriteinResponseCb(const void * const cif,
                                                 const uint8_t status);

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
extern void
twicUtAncsNcNotificationCb(const void * const cif, const uint8_t status,
                           const uint16_t char_value_handle,
                           const twicAttValue_t *const resp);

/* @brief Notification Source Cb
 *
 * This function is invoked when the Notification Source is received
 * from the iOS Peripheral Device.
 * Application program must provide this function.
 * Please refer to the example_ANCS for information about example
 * implementation.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * 
 * @param const twicAttValue_t *const resp
 * The octets length and the data.
 * To parse the message, the "twicUtAncsNsc_t" Notification Source
 * characteristic can be used.
 * twicUtAncsNsc_t *nsc = (twicUtAncsNsc_t *)resp->value
 */
extern void
twicUtAncsNcNsCb(const void * const cif, const twicAttValue_t *const resp);

/* @brief Data Source Cb
 *
 * This function is invoked when the Data Source is received
 * from the iOS Peripheral Device.
 * Application program must provide this function.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * 
 * @param const twicAttValue_t *const resp
 * The octets length and the data.
 */
extern void
twicUtAncsNcDsCb(const void * const cif, const twicAttValue_t *const resp);

/* @brief Setup the Notification Consumer.
 *
 * This API setups the Notification Consumer.
 * First of all the variables of this ANCS sample implementation
 * shall be cleaned up by this API.
 * This API can be invoked in the twicIfLeCb_t::connection_complete.
 * Please refer to the app_sample_profile of the example_ANCS.
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
extern void twicUtAncsSetupNc(twicConnIface_t * const cif);

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
extern void twicUtAncsStartNc(twicConnIface_t * const cif);

/* @brief Stop subscribing the Notification Source and Data Source.
 *
 * This API stop subscribing the Notification Source and Data Source
 * by writing to the Descriptor. 
 *
 * @param const void * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 */
extern void twicUtAncsStopNc(twicConnIface_t * const cif);

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
extern void twicUtAncsRunNc(twicConnIface_t * const cif);

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
extern twicStatus_t
twicUtAncsWriteToControlPoint(twicConnIface_t * const cif,
                              uint8_t * const data, const uint8_t length);

#endif /* __TWIC_UTIL_ANCS_H__ */
