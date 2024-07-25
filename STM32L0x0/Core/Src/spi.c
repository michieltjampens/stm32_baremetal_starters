/**
 * Class with methods to use the I2C1 peripheral on the STM32L0x0 device
 */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "spi.h"

/* Private variables ---------------------------------------------------------*/

uint32_t lastResult;

/* *********************************** S E T U P *************************************************** */
/**
 * Brief Init the SPI peripheral and the used GPIO's
 */
__INLINE void SPI_Configure(void){

	SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN);  // Enable the peripheral clock of GPIOA
	SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN);  // Enable the peripheral clock of GPIOB
	/*
	 * PA4 - NSS  - AF0
	 * PA5 - SCK  - AF0
	 * PA6 - MISO - AF0
	 * PA7 - MOSI - AF0
	 *
	 * PB0 - DRDY
	 */
	/* Use AF for MISO,MOSI,SCLK with AF0 */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE4|GPIO_MODER_MODE5|GPIO_MODER_MODE6|GPIO_MODER_MODE7
			, GPIO_MODER_MODE4_1|GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1); //Use AF
	GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (0x00000FF0)) | (0 << (5 * 4)) | (0 << (6 * 4)) | (0 << (7 * 4)); // Pick AF0 (SPI)

	/* Alter to high speed */
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEED5|GPIO_OSPEEDER_OSPEED6|GPIO_OSPEEDER_OSPEED7
				, GPIO_OSPEEDER_OSPEED5_1|GPIO_OSPEEDER_OSPEED6_1|GPIO_OSPEEDER_OSPEED7_1); //Use High Speed
	// A4-A7 are at push pull which is default

	/* Set B0 to an input */
	MODIFY_REG(GPIOB->MODER,GPIO_MODER_MODE0,0);

	/* A4 (SS), A6 (MISO) and B0 need a pull up */
	MODIFY_REG(GPIOA->PUPDR,GPIO_PUPDR_PUPD4|GPIO_PUPDR_PUPD6, GPIO_PUPDR_PUPD4_0|GPIO_PUPDR_PUPD6_0);
	MODIFY_REG(GPIOB->PUPDR,GPIO_PUPDR_PUPD0|GPIO_PUPDR_PUPD6, GPIO_PUPDR_PUPD0_0);

	/* Enable clock */
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN); // Enable the peripheral clock SPI

	/* Configure the serial clock baud rate using the BR[2:0] bits.*/
	SET_BIT(SPI1->CR1,SPI_CR1_BR_2); // 1MHz
	/* SPI_CR1_BR_0 = 5MHz but not consistent?
	 * SPI_CR1_BR_1 = 2MHz
	 * 0+1          = 1MHz
	 * SPI_CR1_BR_2 = 500kHz consistent
	 * 2+0			= 250kHz
	 * 2+1          = 125kHz
	 *
	 */
	/* Configure the CPOL and CPHA bits combination to define one of the four
	relationships between the data transfer and the serial clock. */
	CLEAR_BIT(SPI1->CR1,SPI_CR1_CPOL); // Clock is idle low	
	CLEAR_BIT(SPI1->CR1,SPI_CR1_CPHA);

	/* Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
	BIDIOE (RXONLY and BIDIMODE can't be set at the same time). */

	/* Configure the LSBFIRST bit to define the frame format */
	CLEAR_BIT(SPI1->CR1,SPI_CR1_LSBFIRST); // 0 -> MSB first

	/* Configure the CRCEN and CRCEN bits if CRC is needed (while SCK clock signal is at idle state). */
	CLEAR_BIT(SPI1->CR1,SPI_CR1_CRCEN); // 0 -> No CRC needed

	/* Configure SSM and SSI . */
	CLEAR_BIT(SPI1->CR1,SPI_CR1_SSM);  // 0 -> MCU is master, NSS output enable

	/* Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on
	NSS if master is configured to prevent MODF error). */
	SET_BIT(SPI1->CR1,SPI_CR1_MSTR); // 1 -> Master mode

	/* Set the DFF bit to configure the data frame format (8 or 16 bits).*/
	CLEAR_BIT(SPI1->CR1,SPI_CR1_DFF); // 0 -> 8-bit data frame format is selected for transmission/reception

	/* Write to SPI_CR2 register */
	/* Configure SSOE  */
	SET_BIT(SPI1->CR2,SPI_CR2_SSOE); //MCU is master, NSS output enable


	SET_BIT(SPI1->CR1,SPI_CR1_SPE); // Enable it (pulls down SS because of SSM/SSOE)

	while( !(SPI1->CR1 &SPI_CR1_SPE) ); // Wait for it to be enabled?

	uint32_t tickOld=Tick;
	while( Tick - tickOld < 10 ); // Tick is 1ms

}

/* General stuff */
uint8_t exchangeByte( uint8_t value ){
	SPI1->DR = value; // Read a byte
	while( !(SPI1->SR & SPI_SR_TXE) ); // Wait for buffer
	while( !(SPI1->SR & SPI_SR_RXNE) );
	return SPI1->DR; // clear RXNE flag
}
