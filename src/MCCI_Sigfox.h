/* ==========================================================
 * MCCI_Sigfox.h - C++ Class for managing Sigfox communications 
 * Project : MCCI Sigfox Image
 * ----------------------------------------------------------
 * Created on: 6 oct. 2020
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
#ifndef __MCCI_SIGFOX_HXX_
#define  __MCCI_SIGFOX_HXX_
#include <Arduino.h>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <it_sdk/itsdk.h>
#include <arduino_wrapper.h>

#ifdef __cplusplus
}
#endif

#define REGION_RC1  (uint32_t)__LORAWAN_REGION_EU868
#define REGION_RC2  (uint32_t)__LORAWAN_REGION_US915
#define REGION_RC3  (uint32_t)__LPWAN_REGION_JP923
#define REGION_RC4  (uint32_t)__LPWAN_REGION_AU915
#define REGION_RC5  (uint32_t)__LPWAN_REGION_KR920

#define DEFAULT_TXPOWER  SIGFOX_DEFAULT_POWER

typedef enum {
    MCCSIG_SUCCESS = 0,
    MCCSIG_NOTINITIALIZED,
    MCCSIG_DOWNLINK_RECEIVED,
    MCCSIG_TRANSMIT_ERROR
} mcci_sigfox_response_e;

class MCCI_Sigfox {

    protected:
        sigfox_api_t   sigfoxApiWrapper;
        sigfox_api_t * sigfoxApiWrapperInUse;

    public:
        // String init
        MCCI_Sigfox(
            char *      devId,          // Device id string, must be 8 hex Char long (32bits even with leading 0)
            char *      pac,            // Device pac string, must be 16 hex chars
            char *      key,            // Device key string, muct be 32 hex chars
            uint32_t    region,         // Sigfox region REGION_RCx
            uint32_t    eepromBase      // Eprom starting address to store Sigfox Data - reserve 24 Bytes from this one
        );
        // Binary init
        MCCI_Sigfox(
            uint32_t   devId,           // Device Id
            uint8_t  * pac,             // Device Pac in a uint8_t[8]
            uint8_t  * key,             // Device Key in a uint8_t[16]
            uint32_t   region,          // Sigfox region REGION_RCx
            uint32_t   eepromBase       // Eprom starting address to store Sigfox Data - reserve 24 Bytes from this one
        );
        // Full Api init
        MCCI_Sigfox(sigfox_api_t * api);

        // Update the configuration - only works with non full api initilization
        mcci_sigfox_response_e setLogger( HardwareSerial * serial );
        mcci_sigfox_response_e setTxPower( int8_t power );
        boolean isReady();

        // Read the configuration
        uint8_t     getCurrentRC();
        uint32_t    getDeviceId();
        void        getInitialPac(uint8_t * pac);
        int16_t     getLastRssi();
        uint16_t    getLastSeqId();
        void        switchToPublicKey();
        void        switchToPrivateKey();
        void        printSigfoxVersion();

        // Transmission
        mcci_sigfox_response_e sendBit(boolean bitValue);
        mcci_sigfox_response_e sendBitWithAck(boolean bitValue,uint8_t * downlinkBuffer);
        mcci_sigfox_response_e sendFrame(uint8_t * buffer, uint8_t size);
        mcci_sigfox_response_e sendFrameWithAck(uint8_t * buffer, uint8_t size, uint8_t * downlinkBuffer);

    private:
        boolean   __initOK;

        uint8_t   convertToHexByte(char * s);
        void      initFromInternalVars();
        boolean   isValidRegion(uint32_t region);

        uint8_t   __getCurrentRegion(uint32_t * region);
        uint8_t   __getDeviceId(uint32_t * devId);
        uint8_t   __getInitialPac(uint8_t * pac);
        uint8_t   __getDeviceKey(uint8_t * key);
        uint8_t   __getTxPower(int8_t * power);
        void      __printLog( char * msg );
};


#endif //  __MCCI_SIGFOX_HXX_