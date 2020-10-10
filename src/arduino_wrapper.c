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
#include "interrupt.h"

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
	uint32_t maxDur = itsdk_stimer_nextTimeoutMs();
	if ( maxDur < duration ) {
		duration = maxDur;
	}
	delay(duration);
	return 0;
}

// get the time in ms since start with 32b millis overflow management
uint64_t __last_millis = 0;
uint64_t __loops_millis = 0;
uint64_t itsdk_time_get_ms() {
	uint64_t m = millis();
	if ( m < __last_millis ) __loops_millis++;
	__last_millis = m;
	return (__loops_millis << 32) + m;
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
 * We request to reserve 16Bytes: the 4 first byte at base are a Magic number to ensure the zone has been 
 * initialized.
 */

static uint32_t __eepromBase;
#define __eepromMagic 0xDD1155CC
#define EEPROM_START_ADDR 0
#define EEPROM_SIZE (6*1024)
#define FLASH_PEKEY1               ((uint32_t)0x89ABCDEFU) /*!< Flash program erase key1 */
#define FLASH_PEKEY2               ((uint32_t)0x02030405U) /*!< Flash program erase key: used with FLASH_PEKEY2
                                                               to unlock the write access to the FLASH_PECR register and
                                                               data EEPROM */

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
	*offset = __eepromBase+4;
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

/**
 * Read the NVM, Bank will be 0, offset will start at 0 and offset+len will be in the
 * 0..12 NVM buffer expected by the sigfox library.
 * The result is a byte stream in data array passed by the caller. It reads len Byte
 * @TODO
 * */
uint32_t __eepromRead(uint32_t addr) {
	return (*(volatile uint32_t*)addr);
}

bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len) {

	uint8_t  * _data = (uint8_t *)data;
	uint32_t   _eepromAddr;
	uint32_t   v;

	_eepromAddr = (uint32_t)(EEPROM_START_ADDR+offset);
	if ( (_eepromAddr & 0x3) != 0 ) {
	    return false;
	}

	// Read data
	for (int i = 0; i < len; i += 4) {
		v = __eepromRead(_eepromAddr);
		_data[i]=(v & 0xFF000000) >> 24;
		if ( i+1 < len) _data[i+1]=(v & 0x00FF0000) >> 16;
		if ( i+2 < len) _data[i+2]=(v & 0x0000FF00) >> 8;
		if ( i+3 < len) _data[i+3]=(v & 0x000000FF);
		_eepromAddr+=4;
	}
	return true;
}

/**
 * Write the NVM, Bank will be 0, offset will start at 0 and offset+len will be in the
 * 0..12 NVM buffer expected by the sigfox library.
 * As a result the Byte stream in data array will be written innNVM
 * @TODO
 * */
bool __eepromWrite(uint32_t addr, uint32_t v) {

	uint16_t tmout = 10000;
	while ( (FLASH->SR & FLASH_SR_BSY) && tmout) tmout--;
	if ( tmout == 0 ) return false;

	// Clear the FTDW bit (data will be erased before write if it non zero)
	FLASH->PECR &= (uint32_t)(~(uint32_t)FLASH_PECR_FIX);

	*(volatile uint32_t *)addr = v;

	while ( (FLASH->SR & FLASH_SR_BSY) && tmout) tmout--;
	if ( tmout == 0 ) return false;

	return true;
}
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len) {
	uint8_t *  _data = (uint8_t *)data;
	uint32_t   _eepromAddr;
	uint32_t   v;

	_eepromAddr = (uint32_t)(EEPROM_START_ADDR+offset);
	if ( (_eepromAddr & 0x3) != 0 ) {
	    return false;
	}

	// Unlock EEPROM
	if (FLASH->PECR & FLASH_PECR_PELOCK) {
			FLASH->PEKEYR = FLASH_PEKEY1;
			FLASH->PEKEYR = FLASH_PEKEY2;
	}
	// Copy data
	for (int i = 0; i < len; i += 4) {
		v = _data[i] << 24;
		v+= (i+1 < len)?_data[i+1]<<16:0;
		v+= (i+2 < len)?_data[i+2]<<8:0;
		v+= (i+3 < len)?_data[i+3]:0;
		if (v != __eepromRead(_eepromAddr)) __eepromWrite(_eepromAddr,v);
		_eepromAddr+=4;
	}
	// Lock EEPROM
	FLASH->PECR |= FLASH_PECR_PELOCK;
	return true;
}

/** ***********************************************************************************
 *  ERROR TRACING
 *  In a normal case, this should write an error where ever you want - in itsdk it's written
 *  in the NVM for later investigation but it can be written in the console.
 *  Fatal error is infinite loop, subject to watchdog and reboot.
 * */

// Some missing functions
itsdk_error_ret_e itsdk_error_noreport(uint32_t error) {
	log_error("***\r\n");
	log_error("*** Error 0x%08X ***\r\n",error);
	log_error("***\r\n");
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
 *  GPIO WRAPPER
 *  The way the GPIO Interrupt are implemented is different and we have to manage
 *  the integration with different layers:
 *  - Arduino capture the interrupt and call a different callback function per gpio as the pin number is not transmitted
 *  - This callback will call the generic ItSDK handler indicating the pin number
 *  - The generic ItSDK handler call the desired function
 * */

static void gpio_interrupt_dio0(void) {
	if (ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE)
	  gpio_Callback(ITSDK_SX1276_DIO_0_PIN);
}
static void gpio_interrupt_dio1(void) {
	if ( ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE ) 
	gpio_Callback(ITSDK_SX1276_DIO_1_PIN);
}
static void gpio_interrupt_dio2(void) {
	if ( ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE ) 
	gpio_Callback(ITSDK_SX1276_DIO_2_PIN);
}
static void gpio_interrupt_dio3(void) {
	if ( ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE ) 
	gpio_Callback(ITSDK_SX1276_DIO_3_PIN);
}
static void gpio_interrupt_dio4(void) {
	if ( ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE ) 
	gpio_Callback(ITSDK_SX1276_DIO_4_PIN);
}


/** ***********************************************************************************
 *  HARDWARE INIT
 *  NEEDED IRQ HANDLER @TODO
 *  - DMA1_Channel2_3_IRQHandler(void)
 *  - GPIO - need to call void gpio_Callback(uint16_t GPIO_Pin) and clear irqs
 * */
// Hardware layer
TIM_HandleTypeDef htim2 = {0};
SPI_HandleTypeDef hspi1 = {0};
DMA_HandleTypeDef hdma_spi1_tx = {0};


/**
 * Resume the SPI configuration
 * */
void MX_SPI1_Init(void)
{
  bzero(&hspi1,sizeof(SPI_HandleTypeDef));
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
  if ( HAL_SPI_Init(&hspi1) != HAL_OK ) {
	  log_error("Problem during SPI initialization\r\n");
  }
  HAL_NVIC_DisableIRQ(SPI1_IRQn);
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
		bzero(&hdma_spi1_tx,sizeof(DMA_HandleTypeDef));
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

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/**
 * Timer 2 configuration
 * */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  bzero(&htim2,sizeof(TIM_HandleTypeDef));	
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  #ifndef ITSDK_WITHOUT_AUTORELOADPRELOAD
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  #endif
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    while(1);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    while(1);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    while(1);
  }

  HAL_NVIC_DisableIRQ(TIM2_IRQn);	// because the arduino MspInit enable it
  __HAL_RCC_TIM2_CLK_ENABLE();
}

/**
 * Init the harwdare related to Sigfox stack
 * */
GPIO_TypeDef * getPortFromBankId(uint8_t bankId);
void init_hardware(uint32_t eepromBase) {

  // init eeprom memory zone
  __eepromBase = eepromBase;
  uint32_t v = __eepromRead(__eepromBase);
  if ( v != __eepromMagic ) {
  	  if (FLASH->PECR & FLASH_PECR_PELOCK) {
			FLASH->PEKEYR = FLASH_PEKEY1;
			FLASH->PEKEYR = FLASH_PEKEY2;
	  }
	  __eepromWrite(__eepromBase,__eepromMagic);
	  FLASH->PECR |= FLASH_PECR_PELOCK;
	  uint8_t zero[12];
	  bzero(zero,12);
	  uint32_t offset;
	  itsdk_sigfox_getSeNvmOffset(&offset);
	  _eeprom_write(0,offset,zero,20);
  }

  // GPIO Irq handler registration
  // The way GPIO Interrupts works between Arduino and ITSDK is a bit different and hard to match without chnaging the Arduino interrupt lib
  // So my choice is to register all the needed interrupt to register inside Arduino the callback function within the ItSdk interrupt handler forever
  // Then the ItSDK Gpio wrapper will manage the irq configuration
  // Set analog to not trigger any interrupt before ful configuration
  if ( ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE )
    stm32_interrupt_enable_forC(getPortFromBankId(ITSDK_SX1276_DIO_0_BANK), ITSDK_SX1276_DIO_0_PIN, gpio_interrupt_dio0, GPIO_MODE_ANALOG);
  if ( ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE )
	stm32_interrupt_enable_forC(getPortFromBankId(ITSDK_SX1276_DIO_1_BANK), ITSDK_SX1276_DIO_1_PIN, gpio_interrupt_dio0, GPIO_MODE_ANALOG);
  if ( ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE )
	stm32_interrupt_enable_forC(getPortFromBankId(ITSDK_SX1276_DIO_2_BANK), ITSDK_SX1276_DIO_2_PIN, gpio_interrupt_dio0, GPIO_MODE_ANALOG);
  if ( ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE )
	stm32_interrupt_enable_forC(getPortFromBankId(ITSDK_SX1276_DIO_3_BANK), ITSDK_SX1276_DIO_3_PIN, gpio_interrupt_dio0, GPIO_MODE_ANALOG);
  if ( ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE )
    stm32_interrupt_enable_forC(getPortFromBankId(ITSDK_SX1276_DIO_4_BANK), ITSDK_SX1276_DIO_4_PIN, gpio_interrupt_dio0, GPIO_MODE_ANALOG);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  SX1276InitLowPower();				// configure all the sx1276 GPIO as low power


  // DMA Init
  MX_DMA_Init();
  // SPI Init
  MX_SPI1_Init();
  // TIMER2 Init
  MX_TIM2_Init();
 
  __HAL_DMA_DISABLE_IT(hspi1.hdmatx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hspi1.hdmatx, DMA_IT_TC);
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx) );
  __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx) );

}

