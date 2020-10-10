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
    uint32_t  devid;
    uint8_t   pac[8];
    uint8_t   key[16];
    uint32_t  region;
    int8_t    txPower;
    HardwareSerial * logger;
    uint32_t  eepromBase;
}  varWrapper_s;


/**
 * Internal wrapper for the sigfox API
 */
extern "C" uint8_t _getCurrentRegion(uint32_t * region) {
   *region = varWrapper_s.region;
   return 0;
}


extern "C" uint8_t _getDeviceId(uint32_t * devId) {
   *devId = varWrapper_s.devid;
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
  *power = varWrapper_s.txPower;
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
 * MCCI_Sigfox( "01415DEE", "01020304050060708", "0102030405060708090A0B0D0E0F", REGION_RC2, 0x8080000);
 */
MCCI_Sigfox::MCCI_Sigfox(
    char *      devId,          // Device id string, must be 8 hex Char long (32bits even with leading 0)
    char *      pac,            // Device pac string, must be 16 hex chars
    char *      key,            // Device key string, muct be 32 hex chars
    uint32_t    region,         // Sigfox region REGION_RCx
    uint32_t    eepromBase      // Eprom starting address to store Sigfox Data - reserve 24 Bytes from this one
) {
    __initOK = false;

   // check for size
   if (
        strlen(devId) != 8
     || strlen(pac) != 16
     || strlen(key) != 32
   ) return;
    if ( eepromBase < 0x8080000 || eepromBase > (0x8080000 + 6*1024 - 24) ) {
        return;
    }

    // convert input
    varWrapper_s.devid = 0;
    for ( int i = 0 ; i < 4 ; i++ ) {
        varWrapper_s.devid <<= 8;
        varWrapper_s.devid += this->convertToHexByte(&devId[2*i]);
    }
    for ( int i = 0 ; i < 8 ; i++ ) {
        varWrapper_s.pac[i] = this->convertToHexByte(&pac[2*i]);
    }
    for ( int i = 0 ; i < 16 ; i++ ) {
        varWrapper_s.key[i] = this->convertToHexByte(&key[2*i]);
        varWrapper_s.key[i] ^= PROTKEY;
    }
    varWrapper_s.region = region;
    varWrapper_s.eepromBase = eepromBase;

    this->initFromInternalVars();
}

/**
 * Constructor from binary
 * uint8_t pac = {...}
 * uint8_t key = {...}
 * MCCI_Sigfox( 0x01415DEE, pac, key, REGION_RC2, 0x8080000);
 */
MCCI_Sigfox::MCCI_Sigfox(
    uint32_t   devId,           // Device Id
    uint8_t  * pac,             // Device Pac in a uint8_t[8]
    uint8_t  * key,             // Device Key in a uint8_t[16]
    uint32_t   region,          // Sigfox region REGION_RCx
    uint32_t   eepromBase       // Eprom starting address to store Sigfox Data - reserve 24 Bytes from this one
) {
    __initOK = false;
    if ( eepromBase < 0x8080000 || eepromBase > (0x8080000 + 6*1024 - 24) ) {
        return;
    }
    varWrapper_s.devid = devId;
    bcopy(pac,varWrapper_s.pac,8);
    for ( int i = 0 ; i < 16 ; i++) {
        varWrapper_s.key[i] = key[i] ^ PROTKEY;
    }
    varWrapper_s.region = region;
    varWrapper_s.eepromBase = eepromBase;
    this->initFromInternalVars();
}

/**
 * Constructor from an api
 */
MCCI_Sigfox::MCCI_Sigfox(sigfox_api_t * api) {
    sigfoxApiWrapperInUse = api;
    if ( sigfox_setup(sigfoxApiWrapperInUse) == SIGFOX_INIT_SUCESS ) {
       __initOK = true;
    }
}


/**
 * In this init scheme, the settings comes from the internal value and the
 * function wrapper is internal also
 */
void MCCI_Sigfox::initFromInternalVars() {
    // check the region
    if ( !isValidRegion(varWrapper_s.region) ) return;
    varWrapper_s.txPower = DEFAULT_TXPOWER;
    varWrapper_s.logger = NULL;
  
    sigfoxApiWrapper.getCurrentRegion = _getCurrentRegion;
    sigfoxApiWrapper.getDeviceId = _getDeviceId;
    sigfoxApiWrapper.getInitialPac = _getInitialPac;
    sigfoxApiWrapper.getDeviceKey = _getDeviceKey;
    sigfoxApiWrapper.getTxPower = _getTxPower;
    sigfoxApiWrapper.printLog = _printLog;
    sigfoxApiWrapper.eepromBase = varWrapper_s.eepromBase;

    sigfoxApiWrapperInUse = &sigfoxApiWrapper;
    if ( sigfox_setup(sigfoxApiWrapperInUse) == SIGFOX_INIT_SUCESS ) {
       __initOK = true;
    }
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
        else if ( s[i] >= 'A' && s[i] <= 'F' ) v+= s[i] - 'A' + 10;
        else if ( s[i] >= 'a' && s[i] <= 'f' ) v+= s[i] - 'a' + 10;
    }
    return v;
}


/** ----------------------------------------------------------------------------------------------------
 *  CHANGE SETTING MADE AT INIT
 *  Only when you did not used the full api constructor
 *  ----------------------------------------------------------------------------------------------------
 */

mcci_sigfox_response_e MCCI_Sigfox::setLogger( HardwareSerial * serial ) {
    if ( __initOK ) {
        varWrapper_s.logger = serial;
        return MCCSIG_SUCCESS;
    }
    return MCCSIG_NOTINITIALIZED;
}

mcci_sigfox_response_e MCCI_Sigfox::setTxPower( int8_t power ) {
    if ( __initOK ) {
        varWrapper_s.txPower = power;
        return MCCSIG_SUCCESS;
    }
    return MCCSIG_NOTINITIALIZED;
}

boolean MCCI_Sigfox::isReady() {
    return __initOK;
}

/** ----------------------------------------------------------------------------------------------------
 *  CONFIGURATION
 *  API to access the current configuration
 *  ----------------------------------------------------------------------------------------------------
 */
uint8_t MCCI_Sigfox::getCurrentRC() {
    uint8_t rc = 0;
    itsdk_sigfox_getCurrentRcz(&rc);
    return rc;
}

uint32_t MCCI_Sigfox::getDeviceId() {
    itsdk_sigfox_device_is_t devId;
    itsdk_sigfox_getDeviceId(&devId);
    return (uint32_t)devId;
} 

void MCCI_Sigfox::getInitialPac(uint8_t * pac) {
    itsdk_sigfox_getInitialPac(pac);
}

int16_t MCCI_Sigfox::getLastRssi() {
    int16_t rssi;
    itsdk_sigfox_getLastRssi(&rssi);
    return rssi;
}

uint16_t MCCI_Sigfox::getLastSeqId() {
    uint16_t seq;
    itsdk_sigfox_getLastSeqId(&seq);
    return seq;
}

void MCCI_Sigfox::switchToPublicKey() {
    itsdk_sigfox_switchPublicKey();
}

void MCCI_Sigfox::switchToPrivateKey() {
    itsdk_sigfox_switchPrivateKey();
}

void MCCI_Sigfox::printSigfoxVersion() {
    if ( sigfoxApiWrapperInUse->printLog != NULL ) {
        uint8_t * libStr;
        itsdk_sigfox_getSigfoxLibVersion(&libStr);
        sigfoxApiWrapperInUse->printLog((char *)libStr);
        sigfoxApiWrapperInUse->printLog("\r\n");
    }
}

int8_t MCCI_Sigfox::getTxPower() {
    int8_t power;
    itsdk_sigfox_getTxPower(&power);
    return power;
}

/** ----------------------------------------------------------------------------------------------------
 *  COMMUNICATIONS
 *  API to execute communications with Sigfox
 *  ----------------------------------------------------------------------------------------------------
 */

mcci_sigfox_response_e MCCI_Sigfox::sendBit(boolean bitValue) {
    return sendBitWithAck(bitValue,NULL);
}

mcci_sigfox_response_e MCCI_Sigfox::sendBitWithAck(boolean bitValue,uint8_t * downlinkBuffer) {
    itdsk_sigfox_txrx_t ret;
    bool ack = (downlinkBuffer!=NULL)?true:false;
    ret = itsdk_sigfox_sendBit(bitValue,2,SIGFOX_SPEED_DEFAULT,SIGFOX_POWER_DEFAULT,ack,downlinkBuffer);
    switch (ret) {
        case SIGFOX_TRANSMIT_SUCESS:
        case SIGFOX_TXRX_NO_DOWNLINK:
            return MCCSIG_SUCCESS;
        case SIGFOX_TXRX_DOWLINK_RECEIVED:
            return MCCSIG_DOWNLINK_RECEIVED;
        case SIGFOX_TXRX_ERROR:
        default:
            return MCCSIG_TRANSMIT_ERROR;    
    }
}

mcci_sigfox_response_e MCCI_Sigfox::sendFrame(uint8_t * buffer, uint8_t size) {
    return sendFrameWithAck(buffer,size,NULL);
}

mcci_sigfox_response_e MCCI_Sigfox::sendFrameWithAck(uint8_t * buffer, uint8_t size, uint8_t * downlinkBuffer) {
    itdsk_sigfox_txrx_t ret;
    bool ack = (downlinkBuffer!=NULL)?true:false;
    ret = itsdk_sigfox_sendFrame(buffer,size,2,SIGFOX_SPEED_DEFAULT,SIGFOX_POWER_DEFAULT,PAYLOAD_ENCRYPT_NONE,ack,downlinkBuffer);
    switch (ret) {
        case SIGFOX_TRANSMIT_SUCESS:
        case SIGFOX_TXRX_NO_DOWNLINK:
            return MCCSIG_SUCCESS;
        case SIGFOX_TXRX_DOWLINK_RECEIVED:
            return MCCSIG_DOWNLINK_RECEIVED;
        case SIGFOX_TXRX_ERROR:
        default:
            return MCCSIG_TRANSMIT_ERROR;    
    }
}

