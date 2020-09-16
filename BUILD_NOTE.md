# How to integrate ItSDK/Sigfox in MCCI SDK

## Needed libraries

You can get the libraries from ST or directly from this repository:
https://github.com/disk91/itsdk-example-sigfox-sx1276/tree/master/Drivers/LIBS
- libSgfxAddonV020_CM0_GCC.a - contains the additional function for sigfox test. Not mandatory.
- libSgfxCmacV100_CM0_GCC.a - contains some functions used for HMAC & encryption (useful) and wrappers to access the ID, PAC, KEY with Secure Element.
- libSgfxCoreV231_CM0_GCC.a - contains the sigfox core protocol implementation
- libSgfxSTModemSx1276V123_CM0_GCC.a - contains the underlying Sx1276 functions

**Some remarks about these libraries:**
- libSgfxCore come from Sigfox
	- It uses an allocated memory space 
- libSgfxSTModem comes from ST
	- It have large static memory buffer including
		* SpiTxBuffer - 3.12KB

The arduino project needs to include these libraries during compilation time with the following order:
1. libSgfxSTModem
2. libSgfxCore
3. libSgfxCmac
4. libSgfxAddon

### What’ve made to manage this:
- Add the 4 library files into [hardware/stm32/2.7.0/system/Middlawares/SIGFOX](hardware/stm32/2.7.0/system/Middlewares/SIGFOX) directory
- modify [platform.txt](hardware/stm32/2.7.0/platform.txt) file
```
compiler.arm.cmsis.ldflags="-L{runtime.tools.CMSIS-4.5.0.path}/CMSIS/Lib/GCC/" -l{build.cmsis_lib_gcc} "-L{build.system.path}/Middlewares/SIGFOX/" -lSgfxSTModemSx1276V123_CM0_GCC -lSgfxCoreV231_CM0_GCC -lSgfxCmacV100_CM0_GCC -lSgfxAddonV020_CM0_GCC
```

## Needed headers

From itsdk project [Inc/drivers/sx1276](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/drivers/sx1276)
* __hw.h__ - common inclusion limiting the modification of the orignial files
* __sfgx_credentials.h__ - api supporting the encryption / hmac functions
* __sgfx_sx1276_drivers.h__ - sx1276 lowlevel function API
* __sigfox_lowlevel.h__ - sx1276 lowlevel function API
* __sigfox_sx1276.h__ - sx1276 lowlevel function API
* __sx1276.h__ - sx1276 lowlevel API (common with LoRaWan)
* __sx1276Regs-Fsk.h__ - sx1276 driver for FSK communications
* __sx1276Regs-LoRa.h__ - sx1276 driver for LoRa communications
Moved in [hardware/stm32/2.7.0/system/Middlewares/SIGFOX](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/)

The organization of these files is not fully clean. in my implementation I’ve tried to no change the ST organization to be able to follow the future ST modifications if some.

From itsdk project [Inc/drivers/sigfox](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/drivers/sigfox)
* __mcu_api.h__ - the sigfox core api with system features like memory allocation, timer use…
* __rf_api.h__ - the sigfox core api with radio layer
* __se_api.h__ - the sigfox core api with secure element (ID, PAC, KEY storage)
* __se_nvm.h__ - the sigfox core api with non volatile storage (for Sequence ID…)
* __sigfox_api.h__ - the sigfox end-user API (what the sigfox core is exposing)
* __sigfox_types.h__ - type abstraction
* __st_sigfox_api.h__ - rest of the st api simplified a lot
Moved in [hardware/stm32/2.7.0/system/Middlewares/SIGFOX](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/)

Extra dependencies
* [Inc/drivers/lorawan](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/drivers/lorawan) - a common file with LoRaWan stack with some defines.


What I’ve made to manage this:
I’ve created a corresponding tree structure in [hardware/stm32/2.7.0/system/Middelwares/SIGFOX](hardware/stm32/2.7.0/system/Middlewares/SIGFOX) directory
* Inc
	* drivers
		* sx1276
		* sigfox

Then I’ve added the Include directory in the [platform.txt](hardware/stm32/2.7.0/platform.txt)
```
compiler.stm.extra_include="-I{build.source.path}" "-I{build.core.path}/avr" "-I{build.core.path}/stm32" "-I{build.core.path}/stm32/USB" "-I{build.system.path}/Drivers/{build.series}_HAL_Driver/Inc/" "-I{build.system.path}/Drivers/{build.series}_HAL_Driver/Src/" "-I{build.system.path}/{build.series}/" "-I{build.system.path}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" "-I{build.system.path}/Middlewares/ST/STM32_USB_Device_Library/Core/Src" "-I{build.system.path}/Middlewares/SIGFOX/Inc"
```

## Compatibility with ITSDK
To simplify the IT_SDK integration I also add the compatibility file in [hardware/stm32/2.7.0/system/Middelwares/SIGFOX/Inc/it_sdk](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk):

### configuration files
* config.h - configure IdSdk - here we have some constant for the ABZ and Sigfox
* config_defines.h - Itsdk list of constant values associated with configuration
* configSigfox.h - contains the configuration related to Sx1276, somes depends on the board, let see later how to manage it ___TODO___

### submodules
* time
	* in [Inc/it_sdk/time](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/time)
	* time management functions
		* time.h - manage time
		* timer.h - manage software timer

* eeprom
	* in [Inc/it_sdk/eeprom](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/eeprom)
	* manage state & configurations
		* eeprom.h - basic eeprom access functions
		* sdk_config.h - manage the NVM configuration storage
		* sdk_state.h - manage the volatile configuration (device state)

* logger
	* in [Inc/it_sdk/logger](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/logger)
		* error.h - list of driver error for tracing
		* logger.h - primitive to trace debug / info / warn / error

* encrypt
	* in [Inc/it_sdk/encrypt](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/encrypt)
	* manage the AES encryption fro HMAC

* wrappers.h / itsdk.h
	* in [Inc/it_sdk/](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/)
	* set of functions implemented mostly on the underlaying layer (stm32 layer of the sdk)

* lowpower
	* [Inc/it_sdk/lowpower](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/lowpower)
	* header for power management - just used for a lowpower delay (can be normal delay)

* sigfox
	* [Inc/it_sdk/sigfox](hardware/stm32/2.7.0/system/Middlewares/SIGFOX/Inc/it_sdk/sigfox)
	* High level headers for sigfox - just used for device Id type

## Source files
The Sigfox source files are located in [hardware/stm32/2.7.0/cores/arduino/stm32/SIGFOX](hardware/stm32/2.7.0/cores/arduino/stm32/SIGFOX)
* murata_cmwx1zzabz.c - low level board integration for ABZ
* sigfox_credentials.c - integration with the encryption layer
* sigfox_lib_api.c - implement the sigfox mcu_api
* sigfox_lowlevel.c - low level radio api implementation
* sigfox_sx1276.c - sx1276 api implementation
* sx1276.c - sx1276 low level driver
* sigfox.c - high level API for end-user
* gpio_wrapper.c - implement all the function related to gpio and irq
* time.c - software timer
* timeServer.c - sigfox/lorawan wrapper with software timers.
* spi.c - driver with SPI


## Sigfox Configuration

***TODO***
- We have different parameters to setup Sigfox
	* deviceID - unique device ID (32b)
	* devicePAC - device PAC initially assigned to the device ID 
	* deviceKEY - device secret KEY, to be protected
	* seqId - current transmission sequence ID 
	* currentRegion - current radio configuration (zone)
- ID, PAC, KEY, currentRC are set by the user and injected into the Sigfox stack (driver needs Read)
- seqId must be stored in the NVM (driver needs RW)

API expected
```C
// Function wrapper
typedef struct {
 	// Get the current REGION (based on LoRaWan region) - see config_defines.h
	uint8_t	(*getCurrentRegion)(uint32_t * region);
	// Return the deviceId into the given parameter
	uint8_t (*getDeviceId)(uint32_t * devId);
	// Return the PAC parameter: a 8 Bytes array
	uint8_t (*getInitialPac)(uint8_t * pac);
	// Return the KEY parameter: a 16 Bytes array
	uint8_t (*getDeviceKey)(uint8_t * kyy);
	// Return the PAC parameter: a 8 Bytes array
	uint8_t (*getCurrentSeqId)(uint16_t * seqId);
	// Return the KEY parameter: a 16 Bytes array
	uint8_t (*setCurrentSeqId)(uint16_t seqId);
	// Return the Tx power (-127 for zone default)
	uint8_t (*getTxPower)(int8_t * power);

} sigfox_api_t;
```

- IRQ Handler
	* the irq handler is defined in gpio_wrapper.c
	* it is also defined in stm32/interrupt.cpp
	* this needs to be improved of the interrup mechanism need to be wrapped a different way

- Function to wrap with Ardunio functions
```C

``



