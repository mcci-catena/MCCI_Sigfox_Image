/* ==========================================================
 * sigfox.h - Sigfox communication abstraction layer
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
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

#include <it_sdk/config_defines.h>
#include <it_sdk/sigfox/sigfox.h>

#ifndef IT_SDK_SIGFOX_API_H_
#define IT_SDK_SIGFOX_API_H_


// Function wrapper
typedef struct {
 	// Get the current REGION (based on LoRaWan region) - see config_defines.h
	uint8_t	(*getCurrentRegion)(uint32_t * region);
	// Return the deviceId into the given parameter
	uint8_t (*getDeviceId)(uint32_t * devId);
	// Return the PAC parameter: a 8 Bytes array, buffer is provided by the caller
	uint8_t (*getInitialPac)(uint8_t * pac);
	// Return the KEY parameter: a 16 Bytes array, buffer is provided by the caller
	uint8_t (*getDeviceKey)(uint8_t * key);
	// Return the Tx power (-127 for zone default)
	uint8_t (*getTxPower)(int8_t * power);
} sigfox_api_t;

// Overrided functions
itsdk_sigfox_init_t sigfox_setup(sigfox_api_t * api);
void sigfox_loop();


#endif
