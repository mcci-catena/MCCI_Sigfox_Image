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

The libraies have been added in [src/cortex-m0](src/cortex-m0). The file [library.poperties](library.poperties) references them.

## ItSDK Sigfox / murata driver stack

### Headers

From itsdk project [Inc/drivers/sx1276](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/drivers/sx1276)
* __hw.h__ - common inclusion limiting the modification of the orignial files
* __sfgx_credentials.h__ - api supporting the encryption / hmac functions
* __sgfx_sx1276_drivers.h__ - sx1276 lowlevel function API
* __sigfox_lowlevel.h__ - sx1276 lowlevel function API
* __sigfox_sx1276.h__ - sx1276 lowlevel function API
* __sx1276.h__ - sx1276 lowlevel API (common with LoRaWan)
* __sx1276Regs-Fsk.h__ - sx1276 driver for FSK communications
* __sx1276Regs-LoRa.h__ - sx1276 driver for LoRa communications
Moved in [src/drivers/sx1276](src/drivers/sx1276/)

The organization of these files is not fully clean. in my implementation I’ve tried to no change the ST organization to be able to follow the future ST modifications if some.

From itsdk project [Inc/drivers/sigfox](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/drivers/sigfox)
* __mcu_api.h__ - the sigfox core api with system features like memory allocation, timer use…
* __rf_api.h__ - the sigfox core api with radio layer
* __se_api.h__ - the sigfox core api with secure element (ID, PAC, KEY storage)
* __se_nvm.h__ - the sigfox core api with non volatile storage (for Sequence ID…)
* __sigfox_api.h__ - the sigfox end-user API (what the sigfox core is exposing)
* __sigfox_types.h__ - type abstraction
* __st_sigfox_api.h__ - rest of the st api simplified a lot
Moved in [src/drivers/sigfox](src/drivers/sigfox/)

Some of the dependencies are common with the lowan stack, even if they are not useful, they wtill exist to limit the modification of the oraginal ST and Semtech file to be able to track the future modifications.
from itsdk project [Inc/drivers/lorawan](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/drivers/lorawan) 
* __phy/radio.h__ - some needed structure
* __systime.h__ - time function
* __timeServer.h__ - timer function
* __trace.h__ - log function
* __util_console.h__ - log function
* __utilities.h__ - some macro
* __utilities_conf.h__ - some defines / nothing really interesting
* __vcom.h__ - noting really interesting

### source file
* murata_cmwx1zzabz.c - low level board integration for ABZ
* sigfox_credentials.c - integration with the encryption layer
* sigfox_lib_api.c - implement the sigfox mcu_api
* sigfox_lowlevel.c - low level radio api implementation
* sigfox_sx1276.c - sx1276 api implementation
* sx1276.c - sx1276 low level driver
* timeServer.c - Semtech API for software timer
* sigfox.c - high level API for end-user
* logger.c - implement the logging level control
* aes128ctr.c - aes wrapper to tiny-AES-c library
* timer.c - implement software timers with callback
* gpio_wrapper.c - Gpio driver
* spi.c - SPI driver

## ItSDK modules integration

### Headers from itsdk to make the system integration
To simplify the IT_SDK integration I also add the compatibility file in [src/it_sdk](src/it_sdk):

### configuration files
* [config.h](src/it_sdk/config.h) - configure IdSdk - here we have some constant for the ABZ and Sigfox to configure the sifox / murata driver and the it_sdk files
* [configSigfox.h](src/it_sdk/configSigfox.h) - contains the configuration related to Sx1276, somes depends on the board, let see later how to manage it ___TODO___

## itSdk sdk general hearders
* [config_defines.h](src/it_sdk/config_defines.h) - Itsdk list of constant values associated with configuration
* [itsdk.h](src/it_sdk/itsdk.h) - main sdk header, include some of the other and contains some generic defines
* [wrappers.h](src/it_sdk/wrappers.h) - low level mcu api (i2c, time, spi, gpio headers)

### submodules
* time
  * in [src/it_sdk/time](src/it_sdk/time)
  * time management functions
	* time.h - manage time
	* time.c - implementation
	* timer.h - manage software timer
	* timer.c - implementation
* eeprom
  * in [src/it_sdk/eeprom](src/it_sdk/eeprom)
  * The implementation for this module will have to be made in the Arduino library - See arduino wrapper
  * manage state & configurations
    * eeprom.h - basic eeprom access functions
	* sdk_config.h - manage the NVM configuration storage
	* sdk_state.h - manage the volatile configuration (device state)

* logger
  * in [src/it_sdk/logger](src/it_sdk/logger)
  * The implementation for this module will have to be made in the Arduino library - See arduino wrapper
    * error.h - list of driver error for tracing
	* logger.h - primitive to trace debug / info / warn / error

* encrypt
  * in [src/it_sdk/encrypt](src/it_sdk/encrypt)
  * manage the AES encryption for HMAC
  * It uses a external lib __tiny-AES-c__ and wrap the function of this lib with itsdk 

* lowpower
  * [src/it_sdk/lowpower](src/it_sdk/lowpower)
  * header for power management - just used for a lowpower delay (can be normal delay)
  * The implementation for this module will have to be made in the Arduino library - See arduino wrapper

* sigfox
  * [src/it_sdk/sigfox](src/it_sdk/sigfox)
  * High level API for sigfox
  * This file have been modified and simplified to fit with the Arduino implementation

* stm32
  * [src/stm32l_sdk](src/stm32l_sdk)
  * Implement the harwdare interface with STM32, contains the GPIO and SPI drivers used by the driver
	I've prefered to use SPI driver from itsdk instead of making a wrapper with the Wire lib on arduino as the SPI, running with DMA for murata/sigfox is complex to make working and including this in a standard Arduinon driver sounds complicated.
	from itsdk project [Inc/stm32l_sdk/spi](https://github.com/disk91/stm32-it-sdk/tree/master/Inc/stm32l_sdk/spi), moved in [src/drivers/stm32](src/drivers/stm32/)
      * __spi.c__ - spi driver header
	SPI reserve TIM2. So this timer can't be used by the Arduino sketch
	The SPI driver override the HAL_SPI_TXCpltCallback and HAL_SPI_TxHalfCpltCallback functions it means the following interrupt must be set and handled by Arduino layer
	```C
	void DMA1_Channel2_3_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(&hdma_spi1_tx);
	}
	```

	The GPIO driver could be more easy to port but, it was more conveniant to reuse it directly as the Interrupt mechanism is really different than Ardunio one.
  * Here we have some missing integration to make with the Arduino layer:
	- **GPIO**
    	- We need to have the Interrupt handler called by Arduino
		```C 
  		void gpio_Callback(uint16_t GPIO_Pin)
		```
		The different needed IRQ Handler are:
		```C
		void EXTI0_1_IRQHandler(void)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);		// DIO2
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);		// DIO1
		}
		void EXTI4_15_IRQHandler(void)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);		// DIO0
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);		// DIO3
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_??);		// DIO4
		}
		``` 
        - We need to inject the correct GPIO setup by setting the right configuration when it can change from a board to another. Currently this is Hardcoded fro Catena-4618 in arduino_wrapper.h.
		```C
		#define ITSDK_SX1276_TCXO_VCC_BANK	 		__BANK_A					   // SX1276 GPIO FOR Activating TCXO
		#define ITSDK_SX1276_TCXO_VCC_PIN	 		__LP_GPIO_8
		#define ITSDK_SX1276_DIO_4_BANK	 	 		__BANK_?					   // SX1276 GPIO 4 => CAD Detected / Pll lock
		#define ITSDK_SX1276_DIO_4_PIN	 	 		__LP_GPIO_?
		```


## Arduino integration

### Internal API
* I've made a [src/arduino_wrapper.c](src/arduino_wrapper.C) file to implement the Arduino version of the ItSDK modules not imported. In this file, some of the implementation is finalyzed, some has been mocked to be able to compile but need to be rewritten.
* from module time.h
```C
void itsdk_delayMs(uint32_t ms); 	// implemented
uint64_t itsdk_time_get_ms(); 		// implemented
```
* from module lowpower.h
```C
uint32_t lowPower_delayMs(uint32_t duration); // implemented full-power (acceptable) no significative saving expected
```
* from module encrypt.h
```C
void itsdk_encrypt_cifferKey(uint8_t * key, int len);	// implemented as doing nothing
void itsdk_encrypt_unCifferKey(uint8_t * key, int len); // implemented as doing nothing
```
* from wrappers.h
```C
uint16_t adc_getVdd(); // Mocked - Todo
int16_t adc_getTemperature(); // Mocked - Todo
uint32_t itsdk_getIrqMask(); // implemented
void itsdk_setIrqMask(uint32_t mask); // implemented
void itsdk_enterCriticalSection(); // implemented
void itsdk_leaveCriticalSection(); // implemented 
```

* from module eeprom.h
```C
itsdk_sigfox_init_t itsdk_sigfox_getSeNvmOffset(uint32_t * offset);	// implemented
itsdk_sigfox_init_t itsdk_sigfox_getNvmOffset(uint32_t * offset); // implemented
uint32_t itdt_align_32b(uint32_t v); // implemented
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len);	// Mocked - Todo
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len); // Mocked - Todo
```

* from module errors.h
```C
itsdk_error_ret_e itsdk_error_noreport(uint32_t error);  // Currently do nothing - Todo 
```

* from module logger.h
```C
void serial1_print(char * msg); // implemented - do nothing
void serial2_print(char * msg); // implemented - do nothing
void debug_print(debug_print_type_e lvl, char * msg); // implemented - call the function given on sigfox driver setup
```

### End-User API
* When the Sigfox setup is call, some parameters are needed
   * deviceID - unique device ID (32b)
   * devicePAC - device PAC initially assigned to the device ID 
   * deviceKEY - device secret KEY, to be protected
   * currentRegion - current radio configuration (zone)
   * txPower - transmission power
   * printLog - print a	Log line function, NULL if no log expected

	Currently a structure with the following API is pass to the driver to inject these parameters
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
		// Return the Tx power (-127 for zone default)
		uint8_t (*getTxPower)(int8_t * power);
		// Method to print a log
    	void (*printLog)(char * msg);
	} sigfox_api_t;
	```