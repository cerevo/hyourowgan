/**
 * @file twic_util_lesmp.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0 Smart
 * @version V2.0.0.FS (Free Sample - The information in this code is
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
#include "twic_util_lesmp.h"

#if defined(TWIC_UTIL_LESMP)

extern void twicUtStoreBondingInformation(const bool address_type_random,
                                          const twicBdaddr_t *const identity,
                                          const bool erase,
                                          twicUtKeyring_t *keyring);
extern bool twicUtReadBondingInformation(twicUtKeyring_t *keyring);

/* Multiple piconet is not supported in this sample implementation. */
twicUtSmpFlags_t twicUtSmpFlags; /* One set of keys only. */

static twicStk_t short_term;
static twicUtKeyring_t keyring;

static void print_stkgenmethod(const twicStkGenMethod_t method)
{
  switch(method) {
  case TWIC_JUST_WORK_UNAUTHENTICATED:
    twicPrintf("TWIC_JUST_WORK_UNAUTHENTICATED\r\n");
    break;
  case TWIC_DISPLAY_RESPONDER_INPUT_AUTHENTICATED:
    twicPrintf("TWIC_DISPLAY_RESPONDER_INPUT_AUTHENTICATED\r\n");
    break;
  case TWIC_INPUT_RESPONDER_DISPLAY_AUTHENTICATED:
    twicPrintf("TWIC_INPUT_RESPONDER_DISPLAY_AUTHENTICATED\r\n");
    break;
  case TWIC_INPUT_RESPONDER_INPUT_AUTHENTICATED:
    twicPrintf("TWIC_INPUT_RESPONDER_INPUT_AUTHENTICATED\r\n");
    break;
  }
}

static void print_pairingfeature(const twicPairingFeature_t *const p)
{
  switch (p->io_capability) {
  case TWIC_SMP_IO_CAPABILITY_DISPLAY_ONLY :
    twicPrintf("TWIC_SMP_IO_CAPABILITY_DISPLAY_ONLY\r\n");
    break;
  case TWIC_SMP_IO_CAPABILITY_DISPLAY_YES_NO :
    twicPrintf("TWIC_SMP_IO_CAPABILITY_DISPLAY_YES_NO\r\n");
    break;
  case TWIC_SMP_IO_CAPABILITY_KEYBOARD_ONLY :
    twicPrintf("TWIC_SMP_IO_CAPABILITY_KEYBOARD_ONLY\r\n");
    break;
  case TWIC_SMP_IO_CAPABILITY_NOINPUT_NO_OUTPUT :
    twicPrintf("TWIC_SMP_IO_CAPABILITY_NOINPUT_NO_OUTPUT\r\n");
    break;
  case TWIC_SMP_IO_CAPABILITY_KEYBOARD_DISPLAY :
    twicPrintf("TWIC_SMP_IO_CAPABILITY_KEYBOARD_DISPLAY\r\n");
    break;
  }

  if (true == p->oob_data_present) twicPrintf("OOB DATA PRESENT\r\n");
  else twicPrintf("OOB DATA NOT PRESENT\r\n");

  twicPrintf("auth_req_bonding:%d\r\n", p->auth_req_bonding);
  twicPrintf("auth_req_mitm_protection:%d\r\n", p->auth_req_mitm_protection);
  twicPrintf("max_enc_key_size:%d\r\n", p->max_enc_key_size);

  twicPrintf("init_key_dist_enckey:%d\r\n", p->init_key_dist_enckey);
  twicPrintf("init_key_dist_idkey:%d\r\n", p->init_key_dist_idkey);
  twicPrintf("init_key_dist_sign:%d\r\n", p->init_key_dist_sign);

  twicPrintf("resp_key_dist_enckey:%d\r\n", p->resp_key_dist_enckey);
  twicPrintf("resp_key_dist_idkey:%d\r\n", p->resp_key_dist_idkey);
  twicPrintf("resp_key_dist_sign:%d\r\n", p->resp_key_dist_sign);
}

static void print_reasoncode(const twicSmReasonCode_t reason)
{
  switch (reason) {
  case TWIC_STATUS_OK :
    twicPrintf("OK\r\n");
    break;
  case TWIC_PASSKEY_ENTRY_FAILED :
    twicPrintf("TWIC_PASSKEY_ENTRY_FAILED\r\n");
    break;
  case TWIC_OOB_NOT_AVAILABLE :
    twicPrintf("TWIC_OOB_NOT_AVAILABLE\r\n");
    break;
  case TWIC_AUTHENTICATION_REQUIREMENTS :
    twicPrintf("TWIC_AUTHENTICATION_REQUIREMENTS\r\n");
    break;
  case TWIC_CONFIRM_VALUE_FAILED :
    twicPrintf("TWIC_CONFIRM_VALUE_FAILED\r\n");
    break;
  case TWIC_PAIRING_NOT_SUPPORTED :
    twicPrintf("TWIC_PAIRING_NOT_SUPPORTED\r\n");
    break;
  case TWIC_ENCRYPTION_KEY_SIZE :
    twicPrintf("TWIC_ENCRYPTION_KEY_SIZE\r\n");
    break;
  case TWIC_COMMAND_NOT_SUPPORTED :
    twicPrintf("TWIC_COMMAND_NOT_SUPPORTED\r\n");
    break;
  case TWIC_UNSPECIFIED_REASON :
    twicPrintf("TWIC_UNSPECIFIED_REASON\r\n");
    break;
  case TWIC_REPEATED_ATTEMPTS :
    twicPrintf("TWIC_REPEATED_ATTEMPTS\r\n");
    break;
  default:
    twicTrace();
    twicLog("reason = 0x%x\r\n", reason);
    break;
  }
}

static void print_key_type(const uint8_t key_type)
{
  switch (key_type) {
  case 0x01 : twicPrintf("Short Term Key(STK)\r\n"); break;
  case 0x02 : twicPrintf("Long Term Key(LTK)\r\n"); break;
  default: twicTrace(); break;
  }
}

static void print_encryption(const bool encryption_enable)
{
  if (true == encryption_enable) twicPrintf("Encryption enable\r\n");
  else twicPrintf("Encryption disable\r\n");
}

static void print_authinfo(const twicAuthInfo_t * const p)
{
  twicPrintf("remote_ltk_received:%d\r\n", p->remote_ltk_received);
  twicPrintf("remote_irk_received:%d\r\n", p->remote_irk_received);
  twicPrintf("remote_csrk_received:%d\r\n", p->remote_csrk_received);
  twicPrintf("local_ltk_sent:%d\r\n", p->local_ltk_sent);
  twicPrintf("local_irk_sent:%d\r\n", p->local_irk_sent);
  twicPrintf("local_csrk_sent:%d\r\n", p->local_csrk_sent);
  twicPrintf("bonding_enabled:%d\r\n", p->bonding_enabled);
  twicPrintf("mitm_enabled:%d\r\n", p->mitm_enabled);
}

static void print_bdaddr_type(const bool random)
{
  if (false == random) twicPrintf("Public address\r\n");
  else twicPrintf("Random address\r\n");
}

/* Multiple piconet is not supported in this sample implementation. */
void twicUtSmpSetup(void)
{
  twicUtSmpFlags.keyring_is_stored = false;
  twicUtSmpFlags.r_pairing_confirmation = false;
  twicUtSmpFlags.r_send_bonding_information = false;
  twicUtSmpFlags.r_kb_respond_passkey = false;
  twicUtSmpFlags.r_dp_respond_passkey = false;  
  twicUtSmpFlags.i_bonding_information = false;
  twicUtSmpFlags.i_send_bonding_information = false;
  twicUtSmpFlags.encryption_done = false;
}

void twicUtSmpResolvablePrivacy(const void * const cif,
                                const twicPrivacy_t *const resp)
{
  twicTrace();
  twicPrintf("status:0x%02x\r\n", resp->status);
  twicUtPrintBdAddr(resp->bd.address);
  twicUtPrintVal("IRK", 16, resp->irk.key);
  memcpy(keyring.local.irk.key, resp->irk.key, 16);
}

void twicUtSmpResolvedPrivacy(const void * const cif,
                              const twicPrivacy_t *const resp)
{
  twicTrace();
  twicPrintf("status:0x%02x\r\n", resp->status);
  twicUtPrintBdAddr(resp->bd.address);
  twicUtPrintVal("IRK", 16, resp->irk.key);
}

static void pairing_acceptance_sent(const void * cif, const uint8_t status)
{
  twicLog("status = %d\r\n", status);
}

static void smpr_stk_generation_method(const void * cif,
                                       const uint8_t status,
                                       const twicStkGenMethod_t method)
{
  twicLog("status = %d\r\n", status);
  print_stkgenmethod(method);
}

static void smpi_stk_generation_method(const void * cif,
                                       const uint8_t status,
                                       const twicStkGenMethod_t method)
{
  twicLog("status = %d\r\n", status);
  print_stkgenmethod(method);
}

static void smpr_pairing_demand(const void * cif,
                                const twicPairingFeature_t *const resp)
{
  twicTrace();
  twicUtSmpFlags.r_pairing_confirmation = true;
  print_pairingfeature(resp);
}

static void smpi_security_request(const void * const cif,
                                  const bool bonded_device,
                                  const bool auth_req_bonding,
                                  const bool auth_req_mitm_protection)
{
  twicTrace();
  twicPrintf("bonded_device:%d\r\n", bonded_device);
  twicPrintf("auth_req_bonding:%d\r\n", auth_req_bonding);
  twicPrintf("auth_req_mitm_protection:%d\r\n", auth_req_mitm_protection);
}

static void smpi_pairing_response(const void * const cif, const uint8_t status,
                                  const twicPairingFeature_t *const resp)
{
  twicLog("status = %d\r\n", status);
  print_pairingfeature(resp);
}

static void smpr_input_passkey(const void * cif)
{
  twicTrace();
  twicUtSmpFlags.r_kb_respond_passkey = true;  
}
static void smpi_input_passkey(const void * cif) { twicTrace(); }
static void smpr_display_passkey(const void * const cif)
{
  twicTrace();
  twicUtSmpFlags.r_dp_respond_passkey = true;  
}

static void smpi_display_passkey(const void * const cif) { twicTrace(); }

static void smpr_stk_generated(const void * cif, const twicStk_t *const stk)
{
  twicTrace();
  memcpy(short_term.key, stk->key, 16);
  twicUtPrintVal("STK", 16, short_term.key);
}

static void smpi_stk_generated(const void * cif, const twicStk_t *const stk)
{
  twicTrace();
  memcpy(short_term.key, stk->key, 16);
  twicUtPrintVal("STK", 16, short_term.key);
}

static void
smpr_pairing_failed(const void * cif, const twicSmReasonCode_t reason)
{
  twicTrace();
  print_reasoncode(reason);
}

static void
smpi_pairing_failed(const void * cif, const twicSmReasonCode_t reason)
{
  twicTrace();
  print_reasoncode(reason);
}

static void
smpr_encryption_info(const void * cif, const twicLtk_t *const ltk)
{
  twicTrace();
  memcpy(keyring.remote.ltk.key, ltk->key, 16);
  twicUtPrintVal("LTK", 16, keyring.remote.ltk.key);
}

static void
smpi_encryption_info(const void * cif, const twicLtk_t *const ltk)
{
  twicTrace();
  memcpy(keyring.remote.ltk.key, ltk->key, 16);
  twicUtPrintVal("LTK", 16, keyring.remote.ltk.key);
}

static void smpr_master_identification(const void * cif,
                                       const twicEdiv_t *const ediv,
                                       const twicRand_t *const rand)
{
  twicTrace();
  memcpy(keyring.remote.ediv.value, ediv->value, 2);
  memcpy(keyring.remote.rand.value, rand->value, 8);
  twicUtPrintVal("EDIV", 2, keyring.remote.ediv.value);
  twicUtPrintVal("RAND", 8, keyring.remote.rand.value);
}

static void smpi_master_identification(const void * cif,
                                       const twicEdiv_t *const ediv,
                                       const twicRand_t *const rand)
{
  twicTrace();
  memcpy(keyring.remote.ediv.value, ediv->value, 2);
  memcpy(keyring.remote.rand.value, rand->value, 8);
  twicUtPrintVal("EDIV", 2, keyring.remote.ediv.value);
  twicUtPrintVal("RAND", 8, keyring.remote.rand.value);
}

static void smpr_encryption_change(const void * const cif,
                                   const twicSmReasonCode_t reason,
                                   const uint8_t key_type,
                                   const bool encryption_enable,
                                   const uint8_t encryption_key_size)
{
  twicTrace();
  print_reasoncode(reason);
  print_key_type(key_type);
  print_encryption(encryption_enable);
  twicUtPrintVal("Eencryption key size", 1, &encryption_key_size);
  twicUtSmpFlags.encryption_done = encryption_enable;
}

static void smpi_encryption_change(const void * const cif,
                                   const twicSmReasonCode_t reason,
                                   const uint8_t key_type,
                                   const bool encryption_enable,
                                   const uint8_t encryption_key_size)
{
  twicTrace();
  print_reasoncode(reason);
  print_key_type(key_type);
  print_encryption(encryption_enable);
  twicUtPrintVal("Eencryption key size", 1, &encryption_key_size);
  twicUtSmpFlags.encryption_done = encryption_enable;
}

static void
smpr_encryption_info_sent(const void * cif, const twicLtk_t *const ltk)
{
  twicTrace();
  memcpy(keyring.local.ltk.key, ltk->key, 16);
  twicUtPrintVal("LTK", 16, keyring.local.ltk.key);
}

static void
smpi_encryption_info_sent(const void * cif, const twicLtk_t *const ltk)
{
  twicTrace();
  memcpy(keyring.local.ltk.key, ltk->key, 16);
  twicUtPrintVal("LTK", 16, keyring.local.ltk.key);
}

static void smpr_master_identification_sent(const void * cif,
                                            const twicEdiv_t *const ediv,
                                            const twicRand_t *const rand)
{
  twicTrace();
  memcpy(keyring.local.ediv.value, ediv->value, 2);
  memcpy(keyring.local.rand.value, rand->value, 8);
  twicUtPrintVal("EDIV", 2, keyring.local.ediv.value);
  twicUtPrintVal("RAND", 8, keyring.local.rand.value);
}

static void smpi_master_identification_sent(const void * cif,
                                            const twicEdiv_t *const ediv,
                                            const twicRand_t *const rand)
{
  twicTrace();
  memcpy(keyring.local.ediv.value, ediv->value, 2);
  memcpy(keyring.local.rand.value, rand->value, 8);
  twicUtPrintVal("EDIV", 2, keyring.local.ediv.value);
  twicUtPrintVal("RAND", 8, keyring.local.rand.value);
}

static void
smpr_encryption_key_refresh_complete(const void * cif,
                                     twicSmReasonCode_t reason,
                                     const uint8_t key_type,
                                     const uint8_t encryption_key_size)
{
  twicTrace();
  print_reasoncode(reason);
  print_key_type(key_type);
  twicUtPrintVal("Eencryption key size", 1, &encryption_key_size);
}

static void
smpi_encryption_key_refresh_complete(const void * cif,
                                     twicSmReasonCode_t reason,
                                     const uint8_t key_type,
                                     const uint8_t encryption_key_size)
{
  twicTrace();
  print_reasoncode(reason);
  print_key_type(key_type);
  twicUtPrintVal("Eencryption key size", 1, &encryption_key_size);
}

static void smpr_pairing_complete_event(const void * cif,
                                        const uint8_t status,
                                        const twicAuthInfo_t bits)
{
  twicLog("status:%d\r\n", status);
  print_authinfo(&bits);
}

static void smpi_pairing_complete_event(const void * cif,
                                        const uint8_t status,
                                        const twicAuthInfo_t bits)
{
  twicLog("status:%d\r\n", status);
  print_authinfo(&bits);
}

static void smpr_stk_session_request_reply_event(const void * const cif,
                                                 const uint8_t status,
                                                 const twicStk_t *const stk)
{
  twicLog("status:%d\r\n", status);
  memcpy(short_term.key, stk->key, 16);
  twicUtPrintVal("STK", 16, short_term.key);
}

static void smpr_ltk_session_request_reply_event(const void * const cif,
                                                 const uint8_t status,
                                                 const twicLtk_t *const ltk)
{
  twicLog("status:%d\r\n", status);
  memcpy(keyring.local.ltk.key, ltk->key, 16);
  twicUtPrintVal("LTK", 16, keyring.local.ltk.key);
}

static void
smpr_identity_information(const void * cif, const twicIrk_t *const irk)
{
  twicTrace();
  memcpy(keyring.remote.irk.key, irk->key, 16);
  twicUtPrintVal("IRK", 16, keyring.remote.irk.key);
}

static void
smpi_identity_information(const void * cif, const twicIrk_t *const irk)
{
  twicTrace();
  memcpy(keyring.remote.irk.key, irk->key, 16);
  twicUtPrintVal("IRK", 16, keyring.remote.irk.key);
}

static void
smpr_identity_address_information(const void * cif,
                                  const bool address_type_random,
                                  const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
}

static void
smpi_identity_address_information(const void * cif,
                                  const bool address_type_random,
                                  const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
}

static void
smpr_signing_information(const void * cif, const twicCsrk_t *const csrk)
{
  twicTrace();
  memcpy(keyring.remote.csrk.key, csrk->key, 16);
  twicUtPrintVal("CSRK", 16, keyring.remote.csrk.key);
}

static void
smpi_signing_information(const void * cif, const twicCsrk_t *const csrk)
{
  twicTrace();
  memcpy(keyring.remote.csrk.key, csrk->key, 16);
  twicUtPrintVal("CSRK", 16, keyring.remote.csrk.key);
}

static void
smpr_identity_information_sent(const void * cif, const twicIrk_t *const irk)
{
  twicTrace();
  memcpy(keyring.local.irk.key, irk->key, 16);
  twicUtPrintVal("IRK", 16, keyring.local.irk.key);
}

static void
smpi_identity_information_sent(const void * cif, const twicIrk_t *const irk)
{
  twicTrace();
  memcpy(keyring.local.irk.key, irk->key, 16);
  twicUtPrintVal("IRK", 16, keyring.local.irk.key);
}

static void
smpr_identity_address_information_sent(const void * cif,
                                       const bool address_type_random,
                                       const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
}

static void
smpi_identity_address_information_sent(const void * cif,
                                       const bool address_type_random,
                                       const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
}

static void
smpr_signing_information_sent(const void * cif, const twicCsrk_t *const csrk)
{
  twicTrace();
  memcpy(keyring.local.csrk.key, csrk->key, 16);
  twicUtPrintVal("CSRK", 16, keyring.local.csrk.key);
}

static void
smpi_signing_information_sent(const void * cif, const twicCsrk_t *const csrk)
{
  twicTrace();
  memcpy(keyring.local.csrk.key, csrk->key, 16);
  twicUtPrintVal("CSRK", 16, keyring.local.csrk.key);
}

static void smpr_oob_information(const void * cif) { twicTrace(); }
static void smpi_oob_information(const void * cif) { twicTrace(); }

static void smpr_store_bonding_information(const void * cif,
                                           const bool address_type_random,
                                           const twicBdaddr_t *const identity,
                                           const bool erase)
{
  uint8_t _erase = (true == erase) ? 1 : 0;
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
  if (false == erase) {
    twicUtSmpFlags.keyring_is_stored = true;
  }
  else twicUtSmpFlags.keyring_is_stored = false;
  twicUtPrintVal("Erase?", 1, &_erase);
  twicUtStoreBondingInformation(address_type_random, identity, erase, &keyring);
}

static void smpi_store_bonding_information(const void * cif,
                                           const bool address_type_random,
                                           const twicBdaddr_t *const identity,
                                           const bool erase)
{
  uint8_t _erase = (true == erase) ? 1 : 0;
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
  if (false == erase) twicUtSmpFlags.keyring_is_stored = true;
  else twicUtSmpFlags.keyring_is_stored = false;
  twicUtPrintVal("Erase?", 1, &_erase);
  twicUtStoreBondingInformation(address_type_random, identity, erase, &keyring);
}

/* inquiry the bonding information of remote bd. */
static void smpr_inquiry_bonding_information(const void * cif,
                                             const bool address_type_random,
                                             const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
  twicUtSmpFlags.r_send_bonding_information = true;
}

static void smpi_inquiry_bonding_information(
  const void * cif, const bool address_type_random,
  const twicBdaddr_t *const identity)
{
  twicTrace();
  print_bdaddr_type(address_type_random);
  twicUtPrintBdAddr(identity->address);
  twicUtSmpFlags.i_send_bonding_information = true;
}

static void
smpr_bonding_state(const void * cif, const twicSmReasonCode_t reason)
{
  twicTrace();
  print_reasoncode(reason);
}

static void
smpi_bonding_state(const void * cif, const twicSmReasonCode_t reason)
{
  twicTrace();
  print_reasoncode(reason);
}

const twicIfLeSmpRCb_t rcb = {
  /* twicIfLeSmSlvSecurityRequest
     void (*pairing_demand)(const twicPairingFeature_t *const resp);
  */
  &smpr_pairing_demand,
  /* When pairing_demand is received, invokes twicIfLeSmSlvPairingConfirm
     void (*pairing_acceptance_sent)(const uint8_t status);
  */
  &pairing_acceptance_sent,
  /* void (*pairing_failed)(const twicSmReasonCode_t reason);
   */
  &smpr_pairing_failed,
  /* void (*stk_generation_method)(
     const uint8_t status, const twicStkGenMethod_t method);
  */
  &smpr_stk_generation_method,
  /* void (*input_passkey)(void);
   */
  &smpr_input_passkey,
  /* void (*display_passkey)(const void * const connif);
   */
  &smpr_display_passkey,
  /* void (*stk_generated)(const twicStk_t *const stk);
   */
  &smpr_stk_generated,
  /* void (*encryption_info)(const twicLtk_t *const ltk);
   */
  &smpr_encryption_info,
  /* void (*master_identification)(
     const twicEdiv_t *const ediv, const twicRand_t *const rand);
  */
  &smpr_master_identification,
  /* void (*encryption_info_sent)(const twicLtk_t *const ltk);
   */
  &smpr_encryption_info_sent,
  /* void (*master_identification_sent)(
    const twicEdiv_t *const ediv, const twicRand_t *const rand);
  */
  &smpr_master_identification_sent,
  /* void (*encryption_change)(
     const void * const connif, const twicSmReasonCode_t reason,
     const uint8_t key_type, const bool encryption_enable,
     const uint8_t encryption_key_size);
  */
  &smpr_encryption_change,
  /* void (*encryption_key_refresh_complete)(
     twicSmReasonCode_t reason, const uint8_t key_type,
     const uint8_t encryption_key_size);
  */
  &smpr_encryption_key_refresh_complete,
  /* void (*pairing_complete_event)(
     const uint8_t status, const twicAuthInfo_t bits);
  */
  &smpr_pairing_complete_event,
  /* void (*stk_session_request_reply_event)(
     const void * const connif, const uint8_t status,
     const twicStk_t *const stk);
  */
  &smpr_stk_session_request_reply_event,
  /* void (*ltk_session_request_reply_event)(
     const void * const connif, const uint8_t status,
     const twicLtk_t *const ltk);
  */
  &smpr_ltk_session_request_reply_event,
  /* void (*identity_information)(const twicIrk_t *const irk);
   */
  &smpr_identity_information,
  /* void (*identity_address_information)(
    const bool address_type_random, const twicBdaddr_t *const identity);
  */
  &smpr_identity_address_information,
  /* void (*signing_information)(const twicCsrk_t *const csrk);
   */
  &smpr_signing_information,
  /* void (*identity_information_sent)(const twicIrk_t *const irk);
   */
  &smpr_identity_information_sent,
  /* void (*identity_address_information_sent)(
     const bool address_type_random, const twicBdaddr_t *const identity);
  */
  &smpr_identity_address_information_sent,
  /* void (*signing_information_sent)(const twicCsrk_t *const csrk);
   */
  &smpr_signing_information_sent,
  /* void (*oob_information)(void);
   */
  &smpr_oob_information,
  /* void (*store_bonding_information)(
     const bool address_type_random, const twicBdaddr_t *const identity,
     const bool erase);
  */
  &smpr_store_bonding_information,
  /* void (*inquiry_bonding_information)(
     const bool address_type_random, const twicBdaddr_t *const identity);
  */
  &smpr_inquiry_bonding_information,
  /* void (*bonding_state)(const twicSmReasonCode_t reason);
   */
  &smpr_bonding_state,
};
  
const twicIfLeSmpICb_t icb = {
  /* void (*pairing_response)(
     const void * const connif, const uint8_t status,
     const twicPairingFeature_t *const resp);
  */
  &smpi_pairing_response,
  /* void (*security_request)(
     const void * const connif, const bool bonded_device,
     const bool auth_req_bonding, const bool auth_req_mitm_protection);
  */
  &smpi_security_request,
  /* void (*pairing_failed)(
     const void * const connif, const twicSmReasonCode_t reason);
  */
  &smpi_pairing_failed,
  /* void (*stk_generation_method)(
     const void * const connif, const uint8_t status,
     const twicStkGenMethod_t method);
  */
  &smpi_stk_generation_method,
  /* void (*input_passkey)(const void * const connif);
   */
  &smpi_input_passkey,
  /* void (*display_passkey)(const void * const connif);
   */
  &smpi_display_passkey,
  /* void (*stk_generated)(const void * const connif,
     const twicStk_t *const stk);
  */
  &smpi_stk_generated,
  /* void (*encryption_info)(
     const void * const connif, const twicLtk_t *const remote_ltk);
  */
  &smpi_encryption_info,
  /* void (*master_identification)(
     const void * const connif, const twicEdiv_t *const remote_ediv,
     const twicRand_t *const remote_rand);
  */
  &smpi_master_identification,
  /* void (*encryption_info_sent)(
     const void * const connif, const twicLtk_t *const local_ltk);
  */
  &smpi_encryption_info_sent,
  /* void (*master_identification_sent)(
     const void * const connif, const twicEdiv_t *const local_ediv,
     const twicRand_t *const local_rand);
  */
  &smpi_master_identification_sent,
  /* void (*encryption_change)(
     const void * const connif, const twicSmReasonCode_t reason,
     const uint8_t key_type, const bool encryption_enable,
     const uint8_t encryption_key_size);
  */
  &smpi_encryption_change,
  /* void (*encryption_key_refresh_complete)(
     const void * const connif, const twicSmReasonCode_t reason,
     const uint8_t key_type, const uint8_t encryption_key_size);
  */
  &smpi_encryption_key_refresh_complete,
  /* void (*pairing_complete_event)(
     const void * const connif, const uint8_t status,
     const twicAuthInfo_t bits);
  */
  &smpi_pairing_complete_event,
  /* void (*identity_information)(
     const void * const connif, const twicIrk_t *const remote_irk);
  */
  &smpi_identity_information,
  /* void (*identity_address_information)(
     const void * const connif, const bool remote_address_type_random,
     const twicBdaddr_t *const remote_identity);
  */
  &smpi_identity_address_information,
  /* void (*signing_information)(
     const void * const connif, const twicCsrk_t *const remote_csrk);
  */
  &smpi_signing_information,
  /* void (*identity_information_sent)(
     const void * const connif, const twicIrk_t *const local_irk);
  */
  &smpi_identity_information_sent,
  /* void (*identity_address_information_sent)(
     const void * const connif, const bool local_address_type_random,
     const twicBdaddr_t *const local_identity);
  */
  &smpi_identity_address_information_sent,
  /* void (*signing_information_sent)(
     const void * const connif, const twicCsrk_t *const local_csrk);
  */
  &smpi_signing_information_sent,
  /* void (*oob_information)(const void * const connif);
   */
  &smpi_oob_information,
  /* void (*store_bonding_information)(
     const void * const connif, const bool remote_address_type_random,
     const twicBdaddr_t *const remote_identity, const bool erase);
  */
  &smpi_store_bonding_information,
  /* void (*inquiry_bonding_information)(
     const void * const connif, const bool address_type_random,
     const twicBdaddr_t *const remote_identity);
  */
  &smpi_inquiry_bonding_information,
  /* void (*bonding_state)(
     const void * const connif, const twicSmReasonCode_t reason);
  */
  &smpi_bonding_state,
};

static twicStatus_t
twicUtLeSmSlvBondingInformationReply(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(
         cif, TWIC_LESMSLVBONDINGINFORMATIONREPLY, &_ar) != true;) {
    status = twicIfLeSmSlvBondingInformationReply(
      cif, &keyring.remote.ediv, &keyring.remote.rand,
      &keyring.remote.ltk, &keyring.remote.irk, &keyring.remote.csrk,
      &keyring.local.ediv, &keyring.local.rand, &keyring.local.ltk,
      &keyring.local.irk, &keyring.local.csrk, 7);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

static twicStatus_t
twicUtLeSmSlvBondingInformationNegativeReply(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(
         cif, TWIC_LESMSLVBONDINGINFORMATIONNEGATIVEREPLY, &_ar) != true;) {
    status = twicIfLeSmSlvBondingInformationNegativeReply(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 3.5.1 Pairing Request.
 * 3.5.2 Pairing Response.
 * 3.5.3 Pairing Confirm.
 * 3.6.7 Security Request.
 *
 * The API is concatenated to the callback below.
 * void (*pairing_demand)(const twicPairingFeature_t *const resp)
 * The callback API will be invoked when the Security Request is
 * issued from the local device. This callback is also invoked when
 * Pairing Request is received.  To Perform the Pairing Response and
 * the Pairing Confirmation, the API should be invoked when the
 * callback function occures.  The elements of the Pairing Response
 * and the Pairing Confirmation are set in this API.  Please refer to
 * the "3.5.2 Pairing Response' for the IO Capability, OOB data flag,
 * AuthReq, Maximum Encryption Key Size, Initiator Key Distribution,
 * Responder Key Distribution and the Confirm value which is
 * internally resolved.  The API can be modified when an application
 * needs to change the elements. */
static twicStatus_t twicUtLeSmSlvPairingConfirm(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LESMSLVPAIRINGCONFIRM, &_ar) != true;) {
    status = twicIfLeSmSlvPairingConfirm(
      cif,
      TWIC_UTIL_LESMP_SLV_IO_CAP, /* io_capability */
      false, /* oob_data_flag_auth_present */
      true,  /* auth_req_bonding */
      false, /* auth_req_mitm_protection, */
      TWIC_UTIL_SLV_MAX_ENC_KEY_SIZE,  /* max_enc_key_size */
      true,  /* init_key_dist_enckey */
      true,  /* init_key_dist_idkey */
      true,  /* init_key_dist_sign */
      true,  /* resp_key_dist_enckey */
      true,  /* resp_key_dist_idkey */
      true); /* resp_key_dist_sign */
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

static twicStatus_t twicUtLeSmMasPairingRequest(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LESMMASPAIRINGREQUEST, &_ar) != true;) {
    status = twicIfLeSmMasPairingRequest(
      cif,
      TWIC_UTIL_LESMP_MAS_IO_CAP, /* io_capability */
      false, /* oob_data_flag_auth_present */
      true,  /* auth_req_bonding */
      false, /* auth_req_mitm_protection, */
      TWIC_UTIL_MAS_MAX_ENC_KEY_SIZE,  /* max_enc_key_size */
      true,  /* init_key_dist_enckey */
      true,  /* init_key_dist_idkey */
      true,  /* init_key_dist_sign */
      true,  /* resp_key_dist_enckey */
      true,  /* resp_key_dist_idkey */
      true); /* resp_key_dist_sign */
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

static twicStatus_t twicUtLeSmMasStartEncryption(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  twicLog("start encryption\r\n");
  for (;twicUtPeekInApi(cif, TWIC_LESMMASSTARTENCRYPTION, &_ar) != true;) {
    status = twicIfLeSmMasStartEncryption(
      cif, &keyring.remote.ediv, &keyring.remote.rand, &keyring.remote.ltk, 7);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}  

/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 2.4.6 Slave Security Request.
 *
 * The slave device may request security by transmitting a Security
 * Request to the master. When a master device receives a Security
 * Request it may encrypt the link, initiate the pairing procedure, or
 * reject the request. */
static twicStatus_t
twicUtLeSmSlvSecurityRequest(twicConnIface_t * const cif,
                             const bool auth_req_mitm_protection)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LESMSLVSECURITYREQUEST, &_ar) != true;) {
    status = twicIfLeSmSlvSecurityRequest(cif, true, auth_req_mitm_protection);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* passkey: 000,000 to 999,999. ex)
            123456 is 0x0001E240 = {0x40, 0xE2, 0x01, 0x00} */
static twicStatus_t
twicUtLeSmSlvKbPasskeyEntryReply(twicConnIface_t * const cif,
                                 twicPasskeyEntry_t * passkey)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LESMSLVKBPASSKEYREPLY, &_ar) != true;) {
    status = twicIfLeSmSlvKbPasskeyEntryReply(cif, passkey);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

static twicStatus_t
twicUtLeSmSlvKbPasskeyEntryNegativeReply(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif,TWIC_LESMSLVKBPASSKEYNEGATIVEREPLY,&_ar) != true;) {
    status = twicIfLeSmSlvKbPasskeyEntryNegativeReply(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* passkey: 000,000 to 999,999. ex)
            123456 is 0x0001E240 = {0x40, 0xE2, 0x01, 0x00} */
static twicStatus_t
twicUtLeSmSlvDpPasskeyEntryReply(twicConnIface_t * const cif,
                                 twicPasskeyEntry_t * passkey)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif, TWIC_LESMSLVDPPASSKEYREPLY, &_ar) != true;) {
    status = twicIfLeSmSlvDpPasskeyEntryReply(cif, passkey);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

static twicStatus_t
twicUtLeSmSlvDpPasskeyEntryNegativeReply(twicConnIface_t * const cif)
{
  twicStatus_t status = TWIC_STATUS_OK;
  uint8_t _ar;
  
  for (;twicUtPeekInApi(cif,TWIC_LESMSLVDPPASSKEYNEGATIVEREPLY,&_ar) != true;) {
    status = twicIfLeSmSlvDpPasskeyEntryNegativeReply(cif);
    if (false == twicUtCheckAndDoEvent(status)) return status;
  }
  if (_ar) return TWIC_STATUS_ERROR_IO;
  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 3.5.1 Pairing Request.
 *
 * The initiator starts the Pairing Feature Exchange by invoking this
 * API to send a Pairing Request command to the responding device.
 *
 * The element of the Pairing Request Packet is set in this API by the
 * "twicUtLeSmMasPairingRequest".  Please refer to the Reference
 * materials "3.5.1 Pairing Request" for the element.  Each
 * application is supposed to change the parameter of the element by
 * modifying this API and the "twicUtLeSmMasPairingRequest" if the
 * application needs other specifications.
 *
 * The element which an application is able to change is below.
 *
 * io_capability : Table 3.3 defines the values which are used when
 * exchanging IO capabilities (see Section 2.3.2).
 *
 * oob_data_present : Table 3.4 defines the values which are used when
 * indicating whether OOB authentication data is available (see
 * Section 2.3.3).
 *
 * auth_req_bonding : This field that indicates the type of bonding
 * being requested by the responding device as defined in Table 3.5.
 *
 * auth_req_mitm_protection : 1-bit flag that is set to one if the
 * device is requesting MITM protection, otherwise it shall be set to
 * 0. A device sets the MITM flag to one to request an Authenticated
 * security property for STK.
 *
 * max_enc_key_size : This value defines the maximum encryption key
 * size in octets that the device can support. The maximum key size
 * shall be in the range 7 to 16 octets.
 *
 * The Initiator Key Distribution field defines which keys the
 * initiator shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Initiator Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * init_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * init_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * init_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 *
 * The Responder Key Distribution field defines which keys the
 * responder shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Responder Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * resp_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * resp_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * resp_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 */
void twicUtSmpMasPairingRequest(twicConnIface_t * const cif)
{
  twicLog("'Mas Pairing Request'.\r\n");
  twicUtSmpFlags.keyring_is_stored = false;
  twicUtSmpFlags.encryption_done = false;
  if (TWIC_STATUS_OK != twicUtLeSmMasPairingRequest(cif)) {
    twicLog("Failure in SmMasPairingRequest.\r\n");
    return;
  }
  twicLog("'Mas Start Encryption'.\r\n");
  if (TWIC_STATUS_OK != twicUtLeSmMasStartEncryption(cif)) {
    twicLog("Failure in SmMasStartEncryption.\r\n");
    return;
  }
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 6.6 START ENCRYPTION.
 * 
 * This API is issued by an ppplication on the Master device to
 * encrypt the link using LTK. After the link is encrypted the
 * callback is invoked.
 *
 * void (*encryption_change)(
 *   const void * const connif, const twicSmReasonCode_t reason,
 *   const uint8_t key_type, const bool encryption_enable,
 *   const uint8_t encryption_key_size);
 */
void twicUtSmMasStartEncryption(twicConnIface_t * const cif)
{
  twicUtSmpFlags.encryption_done = false;
  if (false == twicUtSmpFlags.keyring_is_stored) {
    twicLog("'Mas Pairing Request'.\r\n");
    if (TWIC_STATUS_OK != twicUtLeSmMasPairingRequest(cif)) {
      twicLog("Failure in SmMasPairingRequest.\r\n");
    }
  }
  twicLog("'Mas Start Encryption'.\r\n");
  if (TWIC_STATUS_OK != twicUtLeSmMasStartEncryption(cif)) {
    twicLog("Failure in SmMasStartEncryption.\r\n");
  }
}

/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 2.4.6 Slave Security Request.
 *
 * The slave device may request security by transmitting a Security
 * Request to the master. When a master device receives a Security
 * Request it may encrypt the link, initiate the pairing procedure, or
 * reject the request.
 *
 * The Security Request command includes the required security
 * properties. A security property of MITM protection required shall
 * only be set if the slave's IO capabilities would allow the Passkey
 * Entry association model to be used or out of band authentication
 * data is available.
 *
 * This implementation does not expect the MITM protection. The
 * "auth_req_mitm_protection" can be set "true" if the MITM protection
 * is required.
 */
void twicUtSmpSlvSecurityRequest(twicConnIface_t * const cif)
{
  const bool auth_req_mitm_protection = false;

  if (true == twicUtSmpFlags.keyring_is_stored) {
    twicLog("Keyring is already stored.\r\n");
    return;
  }
  twicUtSmpFlags.encryption_done = false;
  twicLog("Issue 'Slv Security Request'.\r\n");
  twicUtLeSmSlvSecurityRequest(cif, auth_req_mitm_protection);
}

void twicUtSmpRun(twicConnIface_t * const cif)
{
  twicPasskeyEntry_t passkey;
  
  if (true == twicUtSmpFlags.r_send_bonding_information) {
    if (false == twicUtSmpFlags.keyring_is_stored &&
        true == twicUtReadBondingInformation(&keyring)) {
      twicUtSmpFlags.keyring_is_stored = true;
    }
    if (true == twicUtSmpFlags.keyring_is_stored) {
      twicLog("'Bonding Information Reply'.\r\n");
      twicUtLeSmSlvBondingInformationReply(cif);
      twicUtSmpFlags.r_send_bonding_information = false;
      twicLog("'Bonding Information Reply (Done)'.\r\n");
    } else {
      twicLog("'Bonding Information Negative Reply'.\r\n");
      twicUtLeSmSlvBondingInformationNegativeReply(cif);
      twicUtSmpFlags.r_send_bonding_information = false;
      twicLog("'Bonding Information Negative Reply (Done)'.\r\n");
    }
  }
  if (true == twicUtSmpFlags.r_pairing_confirmation) {
    twicLog("'Pairing Confirm'.\r\n");
    twicUtLeSmSlvPairingConfirm(cif);
    twicUtSmpFlags.r_pairing_confirmation = false;
    twicLog("'Pairing Confirm (Done)'.\r\n");    
  }
  if (true == twicUtSmpFlags.r_kb_respond_passkey) {
    twicLog("'Input Kb Passkey'.\r\n");
    if (true == twicUtReadPasskeyEntry(&passkey)) {
      twicUtLeSmSlvKbPasskeyEntryReply(cif, &passkey);
    } else {
      twicUtLeSmSlvKbPasskeyEntryNegativeReply(cif);
    }
    twicUtSmpFlags.r_kb_respond_passkey = false;
    twicLog("'Input Kb Passkey (Done)'.\r\n");
  }
  if (true == twicUtSmpFlags.r_dp_respond_passkey) {
    twicLog("'Input Dp Passkey'.\r\n");
    if (true == twicUtReadPasskeyEntry(&passkey)) {
      twicUtLeSmSlvDpPasskeyEntryReply(cif, &passkey);
    } else {
      twicUtLeSmSlvDpPasskeyEntryNegativeReply(cif);
    }
    twicUtSmpFlags.r_dp_respond_passkey = false;
    twicLog("'Input Dp Passkey (Done)'.\r\n");
  }
}

#endif /* TWIC_UTIL_LESMP */
