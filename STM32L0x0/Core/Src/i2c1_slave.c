/**
 * Class with methods to use the I2C1 peripheral on the STM32L0x0 device
 */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "i2c1_slave.h"

/* Private variables ---------------------------------------------------------*/
uint8_t i2c_recBuffer[8];
uint8_t i2c_cnt=0x00;

/* *********************************** S E T U P *************************************************** */
/**
 * Brief Init the I2C1 peripheral and the used GPIO's for use as slave
 */
__INLINE void I2C1_Configure_Slave(void){
  /* GPIO Setup, PB6=SCL and PB7=SDA */
  SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN);  // Enable the peripheral clock of GPIOB
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE6|GPIO_MODER_MODE7, GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1); //Use AF  
  // AF1 for I2C
  MODIFY_REG( GPIOA->AFR[1], GPIO_AFRH_AFSEL6|GPIO_AFRH_AFSEL7, (1 << GPIO_AFRH_AFSEL6_Pos) | (1 << GPIO_AFRH_AFSEL7_Pos) );
  
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7); // Open drain

  SET_BIT(RCC->APB1ENR,RCC_APB1ENR_I2C1EN); // Enable the peripheral clock I2C1 do this before GPIO's

  /* Configure I2C1 as slave */
  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ADDRIE; 		   // Periph enable, address match interrupt enable 
  I2C1->OAR1 |= (uint32_t)(I2C1_OWN_ADDRESS << 1); // 7-bit address = 0x5A (see .h) 
  SET_BIT( I2C1->OAR1, I2C_OAR1_OA1EN ); 		   // Enable own address 1 */
  SET_BIT( I2C1->CR1,I2C_CR1_RXIE | I2C_CR1_TXIE); // Enable Receive and transmit interrupt

  /* Configure Interrupts */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  SET_BIT(I2C1->CR1,I2C_CR1_PE); 		// Enable I2C peripheral
  while((I2C1->CR1 & I2C_CR1_PE) == 0); // Wait for it to be enabled
}

/* *************************************** I R Q **************************************************** */
void I2C1_IRQHandler(void){

	  if(I2C1->ISR & I2C_ISR_RXNE){ // Receive buffer not empty
		  i2c_recBuffer[i2c_cnt] = I2C1->RXDR;
		  i2c_cnt++;
		  if( i2c_cnt == 8 ){ // prevent overflow
			  i2c_cnt=0;
		  }
		  // Do something?
	  }else if((I2C1->ISR & I2C_ISR_ADDR) == I2C_ISR_ADDR){
		  I2C1->ICR |= I2C_ICR_ADDRCF; /* Clear address match flag */
		  /* Check if transfer direction is read (slave transmitter) */
		  if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR){
			  I2C1->CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
		  }
	  }else if((I2C1->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS){
	  		  I2C1->CR1 &=~ I2C_CR1_TXIE; /* Disable transmit IT */
	  }else if(I2C1->ISR & I2C_ISR_NACKF){ /* NACK Received*/
		  SET_BIT(I2C1->ICR, I2C_ICR_NACKCF); // Clear flag
	  }else if(I2C1->ISR & I2C_ISR_STOPF){
		  SET_BIT(I2C1->ICR, I2C_ICR_STOPCF); // Clear flag
	  }else if(I2C1->ISR & I2C_ISR_BERR ){ // misplaced Start or STOP condition
		  SET_BIT(I2C1->ICR, I2C_ICR_BERRCF);
		  error=ERROR_I2C_BERR;
	  }else if(I2C1->ISR & I2C_ISR_ARLO ){ // Arbitration lost
		  SET_BIT(I2C1->ICR, I2C_ICR_ARLOCF);
		  error=ERROR_I2C_ARLO;
	  }else if(I2C1->ISR & I2C_ISR_PECERR){ // PEC Error in reception
		  SET_BIT(I2C1->ICR, I2C_ICR_PECCF);
		  error=ERROR_I2C_PECERR;
	  }else if(I2C1->ISR & I2C_ISR_TIMEOUT){  // Timeout or tLOW detection flag
		  SET_BIT(I2C1->ICR, I2C_ICR_TIMOUTCF);
		  error=ERROR_I2C_TIMEOUT;
		  /* Check address match */
	  }else{
	    error = ERROR_I2C; /* Report an error */
	    NVIC_DisableIRQ(I2C1_IRQn); /* Disable I2C2_IRQn */
	  }
}
