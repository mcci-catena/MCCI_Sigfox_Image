/* ==========================================================
 * MCCI_Sigfox.cpp - C++ Class for managing Sigfox communications 
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
#include "MCCI_Sigfox.h"

// Key offuscation, this is nothing but better than nothing.
#define PROTKEY 0xA4

/** ----------------------------------------------------------------------------------------------------
 *  WRAPPER TO C CODE
 *  ----------------------------------------------------------------------------------------------------
 */

struct {
    uint32_t  * devid;
    uint8_t   * pac;
    uint8_t   * key;
    uint32_t  * region;
    int8_t    * txPower;
    HardwareSerial * logger;
}  varWrapper_s;


/**
 * Internal wrapper for the sigfox API
 */
extern "C" uint8_t _getCurrentRegion(uint32_t * region) {
   *region = *varWrapper_s.region;;
   return 0;
}


extern "C" uint8_t _getDeviceId(uint32_t * devId) {
   *devId = *varWrapper_s.devid;
   return 0;
}

extern "C" uint8_t _getInitialPac(uint8_t * pac) {
  bcopy(varWrapper_s.pac,pac,8);
  return 0;
}

extern "C" uint8_t _getDeviceKey(uint8_t * key) {
  for ( int i = 0 ; i < 16 ; i++)
     key[i] = varWrapper_s.key[i] ^ PROTKEY;
  return 0;
}

extern "C" uint8_t _getTxPower(int8_t * power) {
  *power = *varWrapper_s.txPower;
  return 0;
}

extern "C" void _printLog( char * msg ) {
    if( varWrapper_s.logger != NULL ) {
        varWrapper_s.logger->print(msg);
    }
}

/** ----------------------------------------------------------------------------------------------------
 *  CONSTRUCTION
 *  ----------------------------------------------------------------------------------------------------
 */

/** 
 * Constructor from Strings
 * MCCI_Sigfox( "01415DEE", "01020304050060708", "0102030405060708090A0B0D0E0F", MCCI_Sigfox.REGION_RC2);
 */
MCCI_Sigfox::MCCI_Sigfox(
    char *   devId,
    char *   pac,
    char *   key, 
    uint32_t  region
) {
    __initOK = false;

   // check for size
   if (
        strlen(devId) != 8
     || strlen(pac) != 16
     || strlen(key) != 32
   ) return;

    // convert input
    __devid = 0;
    for ( int i = 0 ; i < 8 ; i+=2 ) {
        __devid <<= 8;
        __devid = this->convertToHexByte(&devId[i]);
    }
    for ( int i = 0 ; i < 8 ; i++ ) {
        __pac[i] = this->convertToHexByte(&pac[2*i]);
    }
    for ( int i = 0 ; i < 16 ; i++ ) {
        __key[i] = this->convertToHexByte(&key[2*i]);
        __key[i] ^= PROTKEY;
    }
    __region = region;
    this->initFromInternalVars();
}

/**
 * Constructor from binary
 * uint8_t pac = {...}
 * uint8_t key = {...}
 * MCCI_Sigfox( 0x01415DEE, pac, key, MCCI_Sigfox.REGION_RC2);
 */
MCCI_Sigfox::MCCI_Sigfox(
    uint32_t   devId,
    uint8_t  * pac,
    uint8_t  * key, 
    uint32_t    region
) {
    __initOK = false;
    __devid = devId;
    bcopy(pac,__pac,8);
    for ( int i = 0 ; i < 16 ; i++) {
        __key[i] = key[i] ^ PROTKEY;
    }
    __region = region;
    this->initFromInternalVars();
}

/**
 * Constructor from an api
 */
MCCI_Sigfox::MCCI_Sigfox(sigfox_api_t * api) {
    sigfoxApiWrapperInUse = api;
    __initOK = true;
}


/**
 * In this init scheme, the settings comes from the internal value and the
 * function wrapper is internal also
 */
void MCCI_Sigfox::initFromInternalVars() {
    // check the region
    if ( !isValidRegion(__region) ) return;
    __txPower = DEFAULT_TXPOWER;
    __logger = NULL;

    varWrapper_s.devid = &__devid;
    varWrapper_s.pac = __pac;
    varWrapper_s.key = __key;
    varWrapper_s.region = &__region;
    varWrapper_s.txPower = &__txPower;
    varWrapper_s.logger = __logger;

    sigfoxApiWrapper.getCurrentRegion = _getCurrentRegion;
    sigfoxApiWrapper.getDeviceId = _getDeviceId;
    sigfoxApiWrapper.getInitialPac = _getInitialPac;
    sigfoxApiWrapper.getDeviceKey = _getDeviceKey;
    sigfoxApiWrapper.getTxPower = _getTxPower;
    sigfoxApiWrapper.printLog = _printLog;

    sigfoxApiWrapperInUse = &sigfoxApiWrapper;
    __initOK = true;
}


/** 
 * Return tru when the given region has a valid & supported value
 */
boolean MCCI_Sigfox::isValidRegion( uint32_t region) {
    switch(region) {
        case REGION_RC1:
        case REGION_RC2:
        case REGION_RC3:
        case REGION_RC4:
        case REGION_RC5:
            return true;
        default:
            return false;
    }
}

uint8_t MCCI_Sigfox::convertToHexByte( char * s ) {
    uint8_t v = 0;
    for ( int i = 0 ; i < 2 ; i++ ) {
        v <<= 4;
        if ( s[i] >= '0' && s[i] <= '9' ) v+= s[i] - '0';
        else if ( s[i] >= 'A' && s[i] <= 'F' ) v+= s[i] - 'A';
        else if ( s[i] >= 'a' && s[i] <= 'f' ) v+= s[i] - 'a';
    }
    return v;
}


/** ----------------------------------------------------------------------------------------------------
 *  CHANGE SETTING MADE AT INIT
 *  ----------------------------------------------------------------------------------------------------
 */