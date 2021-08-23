/* ==========================================================
 * arduino_wrapper.h - Function to wrap itsdk to arduino env
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2020
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 * 
 *
 * ==========================================================
 */


#ifndef __ARDUINO_WRAPPER_H__
#define __ARDUINO_WRAPPER_H__

#ifdef __cplusplus
extern "C"{
#endif


#include <it_sdk/sigfox/sigfox.h>


#define ITSDK_SX1276_TCXO_VCC_BANK	 		__BANK_A					   // SX1276 GPIO FOR Activating TCXO
#define ITSDK_SX1276_TCXO_VCC_PIN	 		__LP_GPIO_8
#define ITSDK_SX1276_DIO_4_BANK	 	 		__BANK_B					   // SX1276 GPIO 4 => CAD Detected / Pll lock
#define ITSDK_SX1276_DIO_4_PIN	 	 		__LP_GPIO_7                    // Not the right configuration @TODO

// forward reference type
typedef struct sigfox_api_s sigfox_api_t;

// function types
typedef uint8_t	(sigfox_api_getCurrentRegion_t)(sigfox_api_t *pInterface, uint32_t * region);
typedef uint8_t (sigfox_api_getDeviceId_t)(sigfox_api_t *pInterface, uint32_t * devId);
typedef uint8_t (sigfox_api_getInitialPac_t)(sigfox_api_t *pInterface, uint8_t * pac);
typedef uint8_t (sigfox_api_getDeviceKey_t)(sigfox_api_t *pInterface, uint8_t * key);
typedef uint8_t (sigfox_api_getTxPower_t)(sigfox_api_t *pInterface, int8_t * power);
typedef void 	(sigfox_api_printLog_t)(sigfox_api_t *pInterface, char * msg);
// Return the deviceId into the given parameter
// Return the PAC parameter: a 8 Bytes array, buffer is provided by the caller
// Return the KEY parameter: a 16 Bytes array, buffer is provided by the caller
// Return the Tx power (-127 for zone default)
// Method to print a log

// Function wrapper
struct sigfox_api_s {
 	// Get the current REGION (based on LoRaWan region) - see config_defines.h
	sigfox_api_getCurrentRegion_t *getCurrentRegion;
	// Return the deviceId into the given parameter
	sigfox_api_getDeviceId_t *getDeviceId;
	// Return the PAC parameter: a 8 Bytes array, buffer is provided by the caller
	sigfox_api_getInitialPac_t *getInitialPac;
	// Return the KEY parameter: a 16 Bytes array, buffer is provided by the caller
	sigfox_api_getDeviceKey_t *getDeviceKey;
	// Return the Tx power (-127 for zone default)
	sigfox_api_getTxPower_t *getTxPower;
    // Method to print a log
    sigfox_api_printLog_t *printLog;
    // Eeprom baseAddress - reserve 16Bytes for sigfox from this address
    uint32_t  eepromBase;
    // Device ID has been setup as encrypted
    bool      isEncrypted;
};

// Overrided functions
itsdk_sigfox_init_t sigfox_setup(sigfox_api_t * api);
void sigfox_loop();

extern sigfox_api_t * __api;

#ifdef __cplusplus
}
#endif

#endif