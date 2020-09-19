/* ==========================================================
 * arduino_wrapper.c - Function to wrap itsdk to arduino env
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2020
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

#include <it_sdk/itsdk.h>
#include <drivers/sigfox/sigfox_api.h>
#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/logger/error.h>

/** ==============================================================================================
 * ITSDK configuration and state.
 * Some of these structures are needed by the Sifgoc driver.
 */
// State
itsdk_state_t itsdk_state = {0};
itsdk_configuration_nvm_t itsdk_config = { 0 };
// functions wrapper from the end-user level
sigfox_api_t * __api = NULL;


/** ==============================================================================================
 * TIME function
 * Implement the needed time function
 */

// wait for a given number of Ms
void itsdk_delayMs(uint32_t ms) {
	delay(ms);
}

/**
 * wait for a given time in Ms switching the MCU in low power mode during that time
 * You can also decide to stay Up, this is a couple of seconds of saving on callback
 * and a potential 1s saving during uplink.
 *  @TODO
 * */
uint32_t lowPower_delayMs(uint32_t duration) {
	delay(duration);
}

// get the time in ms since start with 32b millis overflow management
uint64_t __last_millis = 0;
uint64_t __loops_millis = 0;
uint64_t itsdk_time_get_ms() {
	uint32_t m = millis();
	if ( m < __last_millis ) __loops_millis++;
	__last_millis = m;
	return __loops_millis << 32 + m;
}


/** ==============================================================================================
 * Internal key protection
 * Usually in ITSDK, the keys are not manipulated in clear test, so there are some just in time
 * decryption function in addition of a stronger encrypted storage. 
 * Here we do not have this feature so the function just do nothing.
 */

// Encrypt a given key
void itsdk_encrypt_cifferKey(uint8_t * key, int len) {

}

// Decryt the given key ( usually this is a simple encryption revesible by the same operation)
void itsdk_encrypt_unCifferKey(uint8_t * key, int len) {
	itsdk_encrypt_cifferKey(key,len);
}


/** ==============================================================================================
 * MCU sensors
 * Sigfox libs requires some information from the usual MCU sensors
 */

/**
 * @TODO
 * Get the internal VCC in mV
 * This is mandatory for OOB frame confirming downlink, according to the Sigfox protocol
 */
uint16_t adc_getVdd() {
	return 3300;
}

/**
 * @TODO
 * Get the temperature (car be internal MCU Temp) scale 0.01 °C
 * This is mandatory for OOB frame confirming downlink, according to the Sigfox protocol
 */
int16_t adc_getTemperature() {
	return 2000;
}

/** ==============================================================================================
 * NVM access - Sigfox API needs 4+5 Bytes of NVM memory to store some internal data including the
 * Sequence Id
 * As STM32 only supports 32b alignement for eeprom R/W, the real space occupied is 4+8 = 12 BYTES
 * The following functions are needed to support this requirement
 */

/**
 * Return the offset of the NVM area for Sigfox Secure Element
 */
itsdk_sigfox_init_t itsdk_sigfox_getSeNvmOffset(uint32_t * offset) {
	itsdk_sigfox_getNvmOffset(offset);
	int size = itdt_align_32b(SFX_NVMEM_BLOCK_SIZE);
	*offset += size;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox
 */
itsdk_sigfox_init_t itsdk_sigfox_getNvmOffset(uint32_t * offset) {
	*offset = 0;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Align a value on the next 32b value
 * This is for NVM size whare offset need to be aligned
 * on 32b values
 */
uint32_t itdt_align_32b(uint32_t v) {
	if ( (v & 3) != 0 ) {
		v &= 0xFFFFFFFC;
		v += 4;
	}
	return v;
}


// Mocked version
static uint8_t __fakeNvm[12];

/**
 * Read the NVM, Bank will be 0, offset will start at 0 and offset+len will be in the
 * 0..12 NVM buffer expected by the sigfox library.
 * The result is a byte stream in data array passed by the caller. It reads len Byte
 * @TODO
 * */
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len) {
	
	uint8_t  * _data = (uint8_t *)data;
	// mock = use ram to store it
	for ( int i = 0 ; i < len ; i++ ) {
		_data[i] = __fakeNvm[offset+i];
	}

}

/**
 * Write the NVM, Bank will be 0, offset will start at 0 and offset+len will be in the
 * 0..12 NVM buffer expected by the sigfox library.
 * As a result the Byte stream in data array will be written innNVM
 * @TODO
 * */
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len) {

	uint8_t  * _data = (uint8_t *)data;
	// mock = use ram to store it
	for ( int i = 0 ; i < len ; i++ ) {
		__fakeNvm[offset+i] = _data[i];
	}

}

/** ***********************************************************************************
 *  ERROR TRACING
 *  In a normal case, this should write an error where ever you want - in itsdk it's written
 *  in the NVM for later investigation but it can be written in the console.
 *  Fatal error is infinite loop, subject to watchdog and reboot.
 * */

// Some missing functions
itsdk_error_ret_e itsdk_error_noreport(uint32_t error) {
	if ( (error & ITSDK_ERROR_LEVEL_FATAL ) == ITSDK_ERROR_LEVEL_FATAL ) while(1);
	return ITSDK_ERROR_SUCCESS;
}


/** ***********************************************************************************
 *  CRITICAL SECTION
 *  Disable interrupt when entering in a critical section
 * */

/**
 * Get the IRQ Mask
 */
uint32_t itsdk_getIrqMask() {
	return __get_PRIMASK();
}

/**
 * Set / Restore the IRQ Mask
 */
void itsdk_setIrqMask(uint32_t mask) {
	__set_PRIMASK(mask);
}
/**
 * Enter a critical section / disable interrupt
 */
static uint32_t __interrupt_mask;
void itsdk_enterCriticalSection() {
	__interrupt_mask = itsdk_getIrqMask();
	//__disable_irq();
	__set_PRIMASK(1);	// allows to capture but not execute the interruption appearing during the critical section execution
}

/**
 * Restore the initial irq mask
 * to leave a critical secqtion
 */
void itsdk_leaveCriticalSection() {
	itsdk_setIrqMask(__interrupt_mask);
}

/** ***********************************************************************************
 *  LOG MANAGEMENT
 *  Assuming only one function will be used to manage the logging (debug)
 *  Other are just declared empty
 * */

void serial1_print(char * msg) {}
void serial2_print(char * msg) {}

/**
 * Print a debug/info/warn/fatal message
 * MOCKED? - use a callback function from the end-user level
 * */
void debug_print(debug_print_type_e lvl, char * msg) {
	if ( __api != NULL && __api->printLog != NULL ) __api->printLog(msg);
}

/** ***********************************************************************************
 *  HARDWARE INIT
 *  NEEDED IRQ HANDLER @TODO
 *  - DMA1_Channel2_3_IRQHandler(void)
 *  - GPIO - need to call void gpio_Callback(uint16_t GPIO_Pin) and clear irqs
 * */
// Hardware layer
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;


/**
 * Resume the SPI configuration
 * */
void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi1);
}

/**
 * Init the harwdare related to Sigfox stack
 * */
void init_hardware(void) {

  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  MX_SPI1_Init();

  // @TODO Paul
  // - Init du hardware complet

}


void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(spiHandle->Instance==SPI1)
    {
		__HAL_RCC_SPI1_CLK_ENABLE();
		
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration    
			 PB3     ------> SPI1_SCK
			PA7     ------> SPI1_MOSI
			PA6     ------> SPI1_MISO 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* SPI1 DMA Init */
		/* SPI1_TX Init */
		hdma_spi1_tx.Instance = DMA1_Channel3;
		hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
		hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_tx.Init.Mode = DMA_NORMAL;
		hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_spi1_tx);
		__HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);
	}
}

