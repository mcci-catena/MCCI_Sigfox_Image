/* ==========================================================
 * config.h - Configure ItSDK and Murata driver
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

/**
 * Stm32 drivers
 */
#include <stm32l0xx_hal_gpio.h>
#include <stm32l0xx_hal_dma.h>
#include <stm32l0xx_hal_spi.h>
#include <stm32l0xx_hal_tim.h>
#include <stm32l0xx_hal_rcc.h>

#include <it_sdk/config_defines.h>
#include <arduino_wrapper.h>

#ifndef IT_SDK_CONFIG_H_
#define IT_SDK_CONFIG_H_


#define ITSDK_WITH_SIGFOX_LIB 		__ENABLE
#define ITSDK_SIGFOX_LIB 			__SIGFOX_SX1276
#define ITSDK_PLATFORM				__PLATFORM_STM32L0
#define ITSDK_DEVICE				__DEVICE_STM32L072XX					// Specific device
#define ITSDK_CONFIGURATION_MODE	__CONFIG_STATIC							// Update it later
#define ITSDK_LORAWAN_ENCRYPTION	__PAYLOAD_ENCRYPT_NONE
#define ITSDK_WITH_SECURESTORE		__DISABLE
#define ITSDK_SIGFOX_NVM_SOURCE		__SFX_NVM_NONE

#define ITSDK_WITH_ERROR_RPT		__DISABLE								// Enable the Error reporting code. The allow to store error code in the EEPROM
#define ITSDK_ERROR_USE_EPROM		__ENABLE								//  Error reports are stored in the EEPROM
#define ITSDK_WITH_ERROR_EXTENTION	__DISABLE								//  Add an application extension for error code in configError.h file
#define ITSDK_ERROR_BLOCKS			4										//  Max number of error block / 1 block stores 1 error and needs 8 Byte for storage.
																			//  The first block is header
#define ITSDK_WITH_GPIO_HANDLER		__DISABLE								// Disable the internal GPIO Handler, when disable you need to map it manually


#define ITSDK_WITH_SPI				__SPI_ENABLED							// Use SPI (inludes the strutures)
#define ITSDK_SPI_HANDLER_TYPE		SPI_HandleTypeDef						// The name of the Spi structure to be used for the targeted MCU
#define ITSDK_SPI_TIMEOUT			100										// SPI transaction timeout in ms
#define ITSDK_WITH_HW_TIMER			0										// Disable HW Timer code
#define ITSDK_TIMER_SLOTS			7										// Maximum number of SOFT TIMER available in parallel - 0 disable SOFT TIMER code


#define ITSDK_LOGGER_CONF			0x000F									// All logs on debug (see logger.c) (File/Serial1/Serial2/Debug)
#define ITSDK_LOGGER_WITH_SEG_RTT	__DISABLE								// disable SEGGER RTT trace driver for DEBUG interface
#define ITSDK_LOGGER_MODULE			( \
									  __LOG_MOD_NONE		\
									| __LOG_MOD_LOWSIGFOX   \
									| __LOG_MOD_STKSIGFOX   \
									| __LOG_MOD_STKLORA     \
									| __LOG_MOD_STIMER      \
									| __LOG_MOD_LOWLORADBG  \
									| __LOG_MOD_LOWLORAINF  \
									)										// list the module to be activated in log see config_defines.h


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