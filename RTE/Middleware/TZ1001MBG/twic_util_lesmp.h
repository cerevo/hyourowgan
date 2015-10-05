/**
 * @file twic_util_lesmp.h
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

#if defined(TWIC_UTIL_LESMP)

#ifndef __TWIC_UTIL_LESMP_H__
#define __TWIC_UTIL_LESMP_H__

typedef struct {
  twicEdiv_t ediv;
  twicRand_t rand;
  twicLtk_t ltk;
  twicIrk_t irk;
  twicCsrk_t csrk;
} twicUtSmpKeys_t;

typedef struct {
  twicUtSmpKeys_t local;
  twicUtSmpKeys_t remote;
} twicUtKeyring_t;

typedef struct {
  uint8_t keyring_is_stored : 1;
  uint8_t r_pairing_confirmation : 1;
  uint8_t r_send_bonding_information : 1;
  uint8_t r_kb_respond_passkey : 1;
  uint8_t r_dp_respond_passkey : 1;
  uint8_t i_bonding_information : 1;
  uint8_t i_send_bonding_information : 1;
  uint8_t encryption_done : 1;
} twicUtSmpFlags_t;

extern twicUtSmpFlags_t twicUtSmpFlags;

extern const twicIfLeSmpRCb_t rcb;
extern const twicIfLeSmpICb_t icb;

extern void twicUtSmpSetup(void);
extern void twicUtSmpResolvablePrivacy(const void * const cif,
                                       const twicPrivacy_t *const resp);
extern void twicUtSmpResolvedPrivacy(const void * const cif,
                                     const twicPrivacy_t *const resp);
extern void twicUtSmpMasPairingRequest(twicConnIface_t * const cif);
extern void twicUtSmMasStartEncryption(twicConnIface_t * const cif);
extern void twicUtSmpSlvSecurityRequest(twicConnIface_t * const cif);
extern void twicUtSmpRun(twicConnIface_t * const cif);

/* @brief Store the bonding information.
 *
 * This API must be provided by such application which need the
 * authentication and the encryption to store the bonding information.
 * If the "erase" is the "true", please remove all the "Copied keyring".
 * Otherwise, please have and store the soft copy of the "keyring".
 * @param [out] const bool address_type_random
 * A public device address or a random address.
 * @param[out] const twicBdaddr_t *const identity
 * Bluetooth device address.
 * @param[out] const bool erase
 * Elimination or maintenance of the keyring.
 * @param[out] twicUtKeyring_t *keyring
 * Keyring.
 */
extern void twicUtStoreBondingInformation(const bool address_type_random,
                                          const twicBdaddr_t *const identity,
                                          const bool erase,
                                          twicUtKeyring_t *keyring);
/* @brief Read the bonding information.
 *
 * This API must be provided by such application which need the
 * authentication and the encryption to store the bonding information.
 *
 * @param [out] twicUtKeyring_t *keyring
 * Keyring
 * @return true
 * The keyring exists
 * @return false
 * The keyring does not exist
 */
extern bool twicUtReadBondingInformation(twicUtKeyring_t *keyring);

/* @brief Read the passkey entry.
 *
 * This API must be provided by such application which need the
 * authentication and the encryption to store the bonding information.
 *
 * Copy the passkey to the argument "pass" if the application allows
 * to input it.  If grant the requirement of the passkey entry, please
 * return with "true".  Otherwise please return with "false".
 *
 * @param [out] twicPasskeyEntry_t *pass	Passkey Entry
 * @return true	Apply and Grant the passkey entry
 * @return false	Refuse the requirement
 * @note
 * 000,000 to 999,999. ex) 123456 is 0x0001E240 = {0x40, 0xE2, 0x01, 0x00}
 */
extern bool twicUtReadPasskeyEntry(twicPasskeyEntry_t *pass);

#endif /* __TWIC_UTIL_LESMP_H__ */

#endif /* TWIC_UTIL_LESMP */
