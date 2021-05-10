/*

Module:  Catena_Sigfox_wrapper.cpp

Function:
	The wrapper to obtain sigfox parameters from the FRAM

Copyright notice:
	See accompanying LICENSE file.

Author:
	Dhinesh Kumar Pitchai, MCCI Corporation	January 2021

*/

#ifdef ARDUINO_ARCH_STM32

#include <Catena_Sigfox_wapper.h>
#include <arduino_wrapper.h>

/* the global instance pointer */
MCCI_Catena_Sigfox *MCCI_Catena_Sigfox::pSigfox = NULL;

#define REGION_1  1
#define REGION_2  2
#define REGION_3  3
#define REGION_4  4
#define REGION_5  5

/**
 * Internal wrapper for the sigfox API
 */

bool MCCI_Catena_Sigfox::begin()
    {
    MCCI_Catena_Sigfox::pSigfox = this; 

    pSigfox->isReady();

    return true;
    }

void MCCI_Catena_Sigfox::loop()
	{
	sigfox_loop();
	}

MCCI_Catena_Sigfox::MCCI_Catena_Sigfox()
    {

    }

/****************************************************************************\
|
|	Getting provisioning info
|
\****************************************************************************/

bool MCCI_Catena_Sigfox::GetDevID(
    uint8_t *pBuf
    )
    {
    SigfoxConfiguringInfo  SigfoxInfo;

    MCCI_Catena_Sigfox * const pSigfox = MCCI_Catena_Sigfox::GetInstance();

    if (! pSigfox->GetSigfoxConfiguringInfo(&SigfoxInfo))
        return false;

    memcpy(
        pBuf,
        &SigfoxInfo.DevID,
        sizeof(SigfoxInfo.DevID)
        );

    return true;
    }

bool MCCI_Catena_Sigfox::GetPAC(
    uint8_t *pBuf
    )
    {
    SigfoxConfiguringInfo  SigfoxInfo;

    MCCI_Catena_Sigfox * const pSigfox = MCCI_Catena_Sigfox::GetInstance();

    if (! pSigfox->GetSigfoxConfiguringInfo(&SigfoxInfo))
        return false;

    int j = 7;
    for(int i = 0; i < sizeof(SigfoxInfo.PAC); i++)
        {
        pBuf[j] = SigfoxInfo.PAC[i];
        --j;
        }

    return true;
    }

bool MCCI_Catena_Sigfox::GetKey(
    uint8_t *pBuf
    )
    {
    SigfoxConfiguringInfo  SigfoxInfo;

    MCCI_Catena_Sigfox * const pSigfox = MCCI_Catena_Sigfox::GetInstance();

    if (! pSigfox->GetSigfoxConfiguringInfo(&SigfoxInfo))
        return false;

    memcpy(
        pBuf,
        SigfoxInfo.Key,
        sizeof(SigfoxInfo.Key)
        );

    return true;
    }

bool MCCI_Catena_Sigfox::GetRegion(
    uint32_t *pBuf
    )
    {
    SigfoxConfiguringInfo  SigfoxInfo;

    MCCI_Catena_Sigfox * const pSigfox = MCCI_Catena_Sigfox::GetInstance();

    if (! pSigfox->GetSigfoxConfiguringInfo(&SigfoxInfo))
        return false;

    switch (SigfoxInfo.Region)
    {
    case REGION_1:
        *pBuf = REGION_RC1;
        break;
    case REGION_2:
        *pBuf = REGION_RC2;
        break;
    case REGION_3:
        *pBuf = REGION_RC3;
        break;
    case REGION_4:
        *pBuf = REGION_RC4;
        break;
    case REGION_5:
        *pBuf = REGION_RC5;
        break;
    
    default:
        break;
    }

    return true;
    }

bool MCCI_Catena_Sigfox::GetEncryption(
    uint8_t *pBuf
    )
    {
    SigfoxConfiguringInfo  SigfoxInfo;

    MCCI_Catena_Sigfox * const pSigfox = MCCI_Catena_Sigfox::GetInstance();

    if (! pSigfox->GetSigfoxConfiguringInfo(&SigfoxInfo))
        return false;

    memcpy(
        pBuf,
        &SigfoxInfo.Encryption,
        sizeof(SigfoxInfo.Encryption)
        );

    return true;
    }

#endif // ARDUINO_ARCH_STM32