/*
 * spi.h
 *
 *  Created on: Jul 18, 2024
 *      Author: Michiel
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
/* Includes ------------------------------------------------------------------*/
#include "./stm32l0xx.h"
/* Private includes ----------------------------------------------------------*/

/* Shared Variables ------------------------------------------------------------*/
extern __IO uint32_t Tick;
extern __IO uint16_t error;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void SPI_Configure(void);
uint8_t exchangeByte( uint8_t value );

#endif /* INC_SPI_H_ */
