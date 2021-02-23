/*

Module:  Catena_Sigfox_wrapper.h

Function:
	The wrapper to obtain sigfox parameters from the FRAM

Copyright notice:
	See accompanying LICENSE file.

Author:
	Dhinesh Kumar Pitchai, MCCI Corporation	January 2021

*/

#ifdef ARDUINO_ARCH_STM32

#ifndef __CATENA_SIGFOX_WRAPPER_H__
#define __CATENA_SIGFOX_WRAPPER_H__

#include <Arduino.h>
#include <MCCI_Sigfox.h>
#ifndef _MCCIADK_ENV_H_
# include <mcciadk_env.h>
#endif

class MCCI_Catena_Sigfox : public MCCI_Sigfox
        {
    public:
        // information provided for Sigfox provisioning
        struct SigfoxConfiguringInfo
            {
            uint8_t         Key[16];
            uint8_t         PAC[8];
            uint8_t         DevID[4];
            uint8_t         Region;
            uint8_t         Encryption;
            };

        struct ConfiguringInfo
            {
            SigfoxConfiguringInfo  *pInfo;
            };

        struct ConfiguringTable
            {
            const ConfiguringInfo  *pInfo;
            unsigned                nInfo;
            };


        /*
        || we only support a single instance, but we don't name it. During
        || begin processing, we register, then we can find it.
        */
        static MCCI_Catena_Sigfox *GetInstance()
                {
                return MCCI_Catena_Sigfox::pSigfox;
                }

        /*
        || the constructor.
        */
        MCCI_Catena_Sigfox();

        /*
        || the begin function. Call this to start things -- the constructor
        || does not!
        */
        bool begin(void);

        /*
        || the function to call from your loop()
        */
        void loop(void);

        bool GetDevID(
                uint8_t *pBuf
                );

        bool GetPAC(
            uint8_t *pBuf
            );

        bool GetKey(
            uint8_t *pBuf
            );

        bool GetRegion(
            uint32_t *pBuf
            );

        bool GetEncryption(
            uint8_t *pBuf
            );

        // return true iff network seems to be provisioned.  Make
        // it virtual so it can be overridden if needed.
        virtual bool IsProvisioned(void)
            {
            return this->GetSigfoxConfiguringInfo(nullptr);
            }

protected:
        // you should provide a function that returns
        // configuring info from persistent storage.
        virtual bool GetSigfoxConfiguringInfo(
                        SigfoxConfiguringInfo *pConfiguringInfo
                        )
                {
                // if not provided, default zeros buf and returns false.
                if (pConfiguringInfo)
                    {
                    memset(
                        pConfiguringInfo,
                        0,
                        sizeof(*pConfiguringInfo)
                        );
                    }
                return false;
                }

private:
        // this is a 'global' -- it gives us a way to bootstrap
        // back into C++ from the LMIC code.
        static MCCI_Catena_Sigfox *pSigfox;
};

#endif

#endif // ARDUINO_ARCH_STM32