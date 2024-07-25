/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/lpuart1.h"
#include "../inc/main.h"

/* Shared variables ----------------------------------------------------------*/
__IO uint8_t cmdReady;
uint8_t freeSpace_USART1 = CIRCULAR-1;
uint8_t irqStatus=IDLE;

/* Private variables ---------------------------------------------------------*/

/* Circular buffer for UART OUT*/
uint8_t outputBuffer_USART2[CIRCULAR];
uint8_t *outTemp_USART2;
uint8_t *outStart_USART2;
uint8_t *outEnd_USART2;
uint8_t *outHead_USART2;
uint8_t *outTail_USART2;

/* Circular buffer for UART IN*/
uint8_t inputBuffer_USART2[64];
uint8_t *inputTemp_USART2;
uint8_t *ui_read_USART2;
uint8_t *ui_write_USART2;
uint8_t *ui_head_USART2;
uint8_t *ui_tail_USART2;


void LPUART1_Configure(){
    /* Initialise the circular buffers */
    ui_read_USART2 = &inputBuffer_USART2[0];
    ui_write_USART2 =   &inputBuffer_USART2[0];
    ui_head_USART2 =  &inputBuffer_USART2[0];
    ui_tail_USART2 =  &inputBuffer_USART2[64-1];

    outStart_USART2 = &outputBuffer_USART2[0];
    outEnd_USART2  =   &outputBuffer_USART2[0];
    outHead_USART2 =  &outputBuffer_USART2[0];
    outTail_USART2 =  &outputBuffer_USART2[CIRCULAR-1];

    LPUART1_Configure_GPIO();
    LPUART1_Configure_Setup();

    cmdReady=0;
}
void LPUART1_Configure_GPIO(void){
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* GPIO configuration for USART1 signals */
  /* (1) Select Alternate Function (AF) mode (b10) on PA0 and PA1 */
  /* Moder LSB nibble: 1111 -> 1010 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0|GPIO_MODER_MODE1))\
                 | (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
  /* (2) AF4 for LPUSART1 signals, the line is explained in detail below it */
  GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (0x00000FF0)) | (6 << (0 * 4)) | (6 << (1 * 4));

  /* Extra info                                                                         */
  /* For the alternate functions, check the datasheet 'Alternate functions'             */
  /* AFR[0]=pin 0 to 7, AFR[1]=pin 8 to 15, each pin has 4 bits for selection           */
  /* So AFR[0] &~(0x00000FF0) = AFR[0] & 0xFFFFF00F -> Reset the bits for PA0 & PA1     */
  /* 4 << (0 * 4) -> Value 4 shifted one nibble to get to PA0 position  -> 0x00000004   */
  /* 4 << (1 * 4) -> Value 4 shifted two nibbles to get to PA1 position -> 0x00000040   */
}
void LPUART1_Configure_Setup(void){
	uint32_t tickstart;

    /* Enable the peripheral clock USART1 */
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN;
    /* Configure USART1 */
    /* System clock is 16MHz (1MHz oversampling by 16), 19200 baud (both already divided by 100) */
    LPUART1->BRR = (256*160000)/192;//19200??

    LPUART1->CR2 |= USART_CR2_SWAP; /* Swap RX and TX this needs to be set before CR1_UE is set */

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    /* Extra info                                 */
    /* USART2->CR1 = USART2 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_UE = USART Enable                */

    /* polling idle frame Transmission */
    tickstart = Tick;
    while((LPUART1->ISR & USART_ISR_TC) != USART_ISR_TC){
    	if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			error = ERROR_USART_TIMEOUT;
			return;
		}
    }
	LPUART1->ICR |= USART_ICR_TCCF;  /* Clear TC flag  (no bit for receive) */
	LPUART1->CR1 |= USART_CR1_TCIE | USART_CR1_RXNEIE; /* Enable Transmission Complete and receive interrupt */

	/* Configure Interrupt */
	/* Set priority for USART1_IRQn */
	NVIC_SetPriority(LPUART1_IRQn, 0);
	/* Enable USART1_IRQn */
	NVIC_EnableIRQ(LPUART1_IRQn);
}

void LPUART1_writeByte( uint8_t data ){
	if( irqStatus & IDLE ){// Meaning buffer not in use
		irqStatus = BUSY;  // Show that it's in use now
		if( outEnd_USART2==outStart_USART2 ){ // Buffer is empty
			LPUART1->TDR = data; // Write the byte
			return; // Finished here
		}else{ // Buffer isn't empty but the irq is in idle so TXE empty, get it to work
			LPUART1->TDR = *outStart_USART2++;
			if (outStart_USART2 == outTail_USART2) {  // So never write on the tail!
				outStart_USART2 = outHead_USART2;     // End reached, back to head
			}
		}
	}
	// At this point the irq is busy with 'something'
	if( outEnd_USART2+1 == outStart_USART2 || (outStart_USART2==outHead_USART2 && outEnd_USART2+1==outTail_USART2 )){
		// Getting in here means that adding a byte would overflow the buffer, so wait a bit
		irqStatus |= WAITING; // Put up the flag that we are waiting for a byte to transfer
		while( irqStatus & WAITING );
	}
	*outEnd_USART2++ = data;
	if (outEnd_USART2 == outTail_USART2) // So never write on the tail!
		outEnd_USART2 = outHead_USART2;  // End reached, back to head
}
/**
 * Brief Send elements from an array in the send buffer till a 0x00 is found
 * Param
 * 		buffer -> Pointer to the array
 * RetVal
 * 		None
 */
void LPUART1_writeText( const char *buffer ){
    while( *buffer != 0x00){
    	LPUART1_writeByte(*buffer++);
    }
}
/**
 * Brief Send the given amount of elements from an array
 * Param
 * 		buffer -> Pointer to the array
 * 		length -> amount of elements to add
 * RetVal
 * 		None
 */
void LPUART1_writeByteArray( uint8_t *buffer, uint8_t length ){
    while( length != 0x00){
    	LPUART1_writeByte(*buffer++);
        length--;
    }
}
void LPUART1_writeNullEndedArray( uint8_t *buffer ){
    while( *buffer != 0x00 )
    	LPUART1_writeByte(*buffer++);
}
void LPUART1_SendArrayReversed( uint8_t *buffer, uint8_t length ){
	buffer = buffer+length-1; // point to the end instead of the start
    while( length != 0x00){
    	LPUART1_writeByte(*buffer--);
        length--;
    }
}

/**
 * Brief Convert a 16bit number to hex ascii and send it
 * Param
 * 		nr -> The number to send
 */
void LPUART1_writeHexWord( uint16_t nr ){
	uint8_t data[7]={'0','x','0','0',0,0,0};
	uint8_t tmp;
	uint8_t index=5;
	if( nr <= 0xFF ){
		index=3;
	}
	while( index > 1 ){
		tmp = nr%16;
		if(tmp>9){
			data[index]= tmp+55;
		}else{
			data[index] = tmp + '0';
		}
		nr -= tmp;
		nr /= 16;
		index --;
	}
	LPUART1_writeNullEndedArray(data);
}
void LPUART1_writeHexQuad( uint32_t nr ){
	uint8_t data[11]={'0','x','0','0','0','0','0','0','0','0',0};
	uint8_t tmp;
	uint8_t index=9;
	while( index > 1 ){
		tmp = nr%16;
		if(tmp>9){
			data[index]= tmp+55;
		}else{
			data[index] = tmp + '0';
		}
		nr -= tmp;
		nr /= 16;
		index --;
	}
	LPUART1_writeNullEndedArray(data);
}
void LPUART1_writeHexWordNoPrefix( uint16_t nr ){
	uint8_t data[5]={'0','0','0','0',0};
	uint8_t tmp;
	uint8_t index=3;

	while( nr!=0 ){
		tmp = nr%16;
		if(tmp>9){
			data[index]= tmp+55;
		}else{
			data[index] = tmp + '0';
		}
		nr /= 16;
		index --;
	}
	LPUART1_writeNullEndedArray(data);
}
void LPUART1_writeHexByteNoPrefix( uint8_t nr ){
	uint8_t tmp;

	tmp = nr%16;
	if(tmp>9){
		tmp = tmp + 55;
	}else{
		tmp = tmp + '0';
	}

	nr /= 16;
	if(nr>9){
		LPUART1_writeByte( nr+55); // 65 is ascii value of A
	}else{
		LPUART1_writeByte( nr + '0');
	}
	LPUART1_writeByte(tmp);
}
void LPUART1_writeHexByteArrayNoPrefix( uint8_t *nrs, uint8_t length ){
	uint8_t tmp;

	while( length != 0x00){
		tmp = *nrs/16;
		if(tmp>9){
			LPUART1_writeByte(tmp+55);
		}else{
			LPUART1_writeByte(tmp + '0');
		}
		tmp = *nrs%16;
		if(tmp>9){
			LPUART1_writeByte(tmp+55);
		}else{
			LPUART1_writeByte(tmp + '0');
		}
        nrs++;
        length--;
    }
}
void LPUART1_writeDec( uint16_t nr ){
	uint8_t data[6]={'0',0,0,0,0,0};
	uint8_t index=0;
	if( nr==0 ){
		LPUART1_writeByte('0');
	}else{
		while( nr!=0 ){
			data[index++]=nr%10+'0';
			nr/=10;
		}
		LPUART1_SendArrayReversed(data,index);
	}
}
void LPUART1_write32bitDec( uint32_t nr ){
	uint8_t data[10]={'0',0,0,0,0,0,0,0,0,0};
	uint8_t index=0;
	if( nr==0 ){
		LPUART1_writeByte('0');
	}else{
		while( nr!=0 ){
			data[index++]=nr%10+'0';
			nr/=10;
		}
		LPUART1_SendArrayReversed(data,index);
	}
}
void LPUART1_Transfer_Buffer( void ){
    uint8_t rec[16];
    uint8_t a;

    if( ui_read_USART2 == ui_write_USART2) // This shouldn't happen, but somehow does
    	return;

    /* Clean the buffer that will be used */
    for( a=0;a<16;a++)
        rec[a]=0x00;

    a = 0x00;
    /* Move the data from the circular buffer to the local one */
    inputTemp_USART2 = ui_write_USART2;                             // Remember the current endstop for the circular buffer,because other might use it in ISR
    if (ui_read_USART2 > inputTemp_USART2) {                     // If the 'end' is closer to the beginning of the buffer than the 'start'
        do{
            rec[a++] = *ui_read_USART2++;
        }while( ui_read_USART2 != ui_tail_USART2 && a < CIRCULAR+5);  // Repeat till the 'start' has reached the end of the buffer
        ui_read_USART2 = ui_head_USART2;                       // Because the end was reached, continue from the start of the buffer
    }
    do{
        rec[a++] = *ui_read_USART2++;
    }while( ui_read_USART2 < inputTemp_USART2 && a < CIRCULAR+5);      // Repeat the 'start' is the same as the 'end'

    executeCommand( rec );
}
void LPUART1_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    // Check if interrupt is due to transmit complete
    if( LPUART1->ISR & USART_ISR_TXE ){
    	if( outStart_USART2 != outEnd_USART2 ){ // meaning still data to send
			 LPUART1->TDR = *outStart_USART2++; // This also clears the complete flag
			 freeSpace_USART1++;
			 irqStatus=BUSY; // Indicate that the irq is working on the buffer
			 if (outStart_USART2 == outTail_USART2) { // So never write on the tail!
				 outStart_USART2 = outHead_USART2;    // End reached, back to head
			 }
		 }else{ // No more data to send, just clear the flag
			 LPUART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
			 irqStatus=IDLE;
		 }
    }
    if( LPUART1->ISR & USART_ISR_TC ){
    	LPUART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((LPUART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE){ // ISR for received data
        recChar = (uint8_t)(LPUART1->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *ui_write_USART2++ = recChar;
            if (ui_write_USART2 == ui_tail_USART2) { // So never write on the tail!
                ui_write_USART2 = ui_head_USART2;
            }
        }else if(recChar==0x00){
        	//ok=0x00;
        }else if( recChar == '\n'){
        }else if( recChar == '\r'){
        	cmdReady ++;
        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //LPUART1_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(LPUART1_IRQn); /* Disable USART2_IRQn */
    }
}
uint8_t LPUART1_hasCmd(){
	if( cmdReady != 0){
		cmdReady=0;
		return 1;
	}
	return 0;
}
