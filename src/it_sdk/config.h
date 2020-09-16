/**
 * Compatibility file for Itsdk inclusion
 */
#include <stm32l0xx_hal_gpio.h>
#include <stm32l0xx_hal_dma.h>
#include <stm32l0xx_hal_spi.h>
#include <stm32l0xx_hal_tim.h>
#include <stm32l0xx_hal_rcc.h>

#include <it_sdk/itsdk.h>
#include <it_sdk/config_defines.h>

#ifndef IT_SDK_CONFIG_H_
#define IT_SDK_CONFIG_H_

#define ITSDK_WITH_SIGFOX_LIB 		__ENABLE
#define ITSDK_SIGFOX_LIB 			__SIGFOX_SX1276
#define ITSDK_PLATFORM				__PLATFORM_STM32L0
#define ITSDK_CONFIGURATION_MODE	__CONFIG_STATIC							// Update it later
#define ITSDK_WITH_SECURESTORE		__DISABLE
#define ITSDK_SIGFOX_NVM_SOURCE		__SFX_NVM_NONE

#define ITSDK_WITH_ERROR_RPT		__DISABLE								// Enable the Error reporting code. The allow to store error code in the EEPROM
#define ITSDK_ERROR_USE_EPROM		__ENABLE								//  Error reports are stored in the EEPROM
#define ITSDK_WITH_ERROR_EXTENTION	__DISABLE								//  Add an application extension for error code in configError.h file
#define ITSDK_ERROR_BLOCKS			4										//  Max number of error block / 1 block stores 1 error and needs 8 Byte for storage.
																			//  The first block is header

#define ITSDK_WITH_SPI				__SPI_ENABLED							// Use SPI (inludes the strutures)
#define ITSDK_SPI_HANDLER_TYPE		SPI_HandleTypeDef						// The name of the Spi structure to be used for the targeted MCU
#define ITSDK_SPI_TIMEOUT			100										// SPI transaction timeout in ms
#define ITSDK_WITH_HW_TIMER			0										// Disable HW Timer code
#define ITSDK_TIMER_SLOTS			7										// Maximum number of SOFT TIMER available in parallel - 0 disable SOFT TIMER code

// Some catena specific defines 
#define ITSDK_WITHOUT_AUTORELOADPRELOAD										// The Hal version seems different

// Sigfox driver state
typedef	struct {
	bool					initialized;
	uint8_t					current_power;
} sigfox_t;


extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

#include <it_sdk/configSigfox.h>

#endif // IT_SDK_CONFIG_H_