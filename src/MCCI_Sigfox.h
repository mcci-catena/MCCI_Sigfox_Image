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
} mcci_sigfox_response_e;

class MCCI_Sigfox {

    protected:
        sigfox_api_t   sigfoxApiWrapper;
        sigfox_api_t * sigfoxApiWrapperInUse;

    public:
        MCCI_Sigfox(
            char *   devId,
            char *   pac,
            char *   key, 
            uint32_t region
        );

        MCCI_Sigfox(
            uint32_t   devId,
            uint8_t  * pac,
            uint8_t  * key, 
            uint32_t    region
        );
        MCCI_Sigfox(sigfox_api_t * api);

    private:
        uint32_t  __devid;
        uint8_t   __pac[8];
        uint8_t   __key[16];
        uint32_t  __region;
        int8_t    __txPower;
        HardwareSerial * __logger;


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