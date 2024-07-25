/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Shared Variables  ---------------------------------------------------------*/
__IO uint32_t Tick;
__IO uint16_t error;

/* Private variables ---------------------------------------------------------*/
uint16_t delay;
uint16_t heartBeats=0;

int main(void){
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32l0xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32l0xx.c file
    */
	
    SysTick_Config(2000); /* 1ms config */
    SystemClock_Config();
    SysTick_Config(1600); /* 0.1ms config */

	init();

	while (1) {
		
	}
}

/**
  * Brief   This function configures the system clock @16MHz and voltage scale 1
  *         assuming the registers have their reset value before the call.
  *         POWER SCALE   = RANGE 1
  *         SYSTEM CLOCK  = PLL MUL8 DIV2
  *         PLL SOURCE    = HSI/4
  *         FLASH LATENCY = 0
  * Param   None
  * Retval  None
  */
__INLINE void SystemClock_Config(void){

  uint32_t tickstart;
  
  /* (1) Enable power interface clock */
  RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
  
  /* (2) Select voltage scale 1 (1.65V - 1.95V) i.e. (01)  for VOS bits in PWR_CR */
  PWR->CR = (PWR->CR & ~(PWR_CR_VOS)) | PWR_CR_VOS_0;
  
  /* (3) Enable HSI divided by 4 in RCC->CR */
  RCC->CR |= RCC_CR_HSION | RCC_CR_HSIDIVEN;
  
  tickstart = Tick;
    
  /* (4) Wait for HSI ready flag and HSIDIV flag */
   while ((RCC->CR & (RCC_CR_HSIRDY |RCC_CR_HSIDIVF)) != (RCC_CR_HSIRDY |RCC_CR_HSIDIVF)){
    if ((Tick - tickstart ) > HSI_TIMEOUT_VALUE){
      error = ERROR_HSI_TIMEOUT; /* Report an error */
      return;
    }      
  } 
  /* (5) Set PLL on HSI, multiply by 8 and divided by 2 */
  RCC->CFGR |= RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLDIV2;
  
  /* (6) Enable the PLL in RCC_CR register */
  RCC->CR |= RCC_CR_PLLON; 
  
  tickstart = Tick;
  
  /* (7) Wait for PLL ready flag */
  while ((RCC->CR & RCC_CR_PLLRDY)  == 0){
    if ((Tick - tickstart ) > PLL_TIMEOUT_VALUE){
      error = ERROR_PLL_TIMEOUT; /* Report an error */
      return;
    }      
  }
  
  /* (8) Select PLL as system clock */
  RCC->CFGR |= RCC_CFGR_SW_PLL; /* (8) */
  tickstart = Tick;
  
  /* (9) Wait for clock switched on PLL */
  while ((RCC->CFGR & RCC_CFGR_SWS_PLL)  == 0){
    if ((Tick - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE){
      error = ERROR_CLKSWITCH_TIMEOUT; /* Report an error */
      return;
    }      
  }
}

/* ********************************************************************************************************** */
/* *********************************** A P P L I C A T I O N  C O D E *************************************** */
/* ********************************************************************************************************** */
void init(void){

    configure_IO();
	// Enable UART1
    LPUART1_Configure();
	// Enable I2C1 as a slave
    I2C1_Configure_Slave();
	// Configure SPI1
    SPI_Configure();

    delay = 20; // wait a bit
    while(delay!=0){} // 20ms delay
}

void configure_IO(void){
  
  /* Enable the peripheral clock of GPIOA, GPIOB, GPIOC */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;

  // B5 as output for heart beat
  /* MODIFY_REG( REG, CLEAR_MASK, SET_MASK ) */
  MODIFY_REG( GPIOB->MODER, GPIOB->MODER & ~(GPIO_MODER_MODE5), GPIO_MODER_MODE5_0);
}

/******************************************************************************/
/*            Cortex-M0 Plus Processor Interrupt Handlers                    */
/******************************************************************************/

void EXTI4_15_IRQHandler(void){
	if (EXTI->PR & (EXTI_PR_PIF4)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF4); // Writing a 1 clears it
	}
	if (EXTI->PR & (EXTI_PR_PIF6)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF6); // Writing a 1 clears it
	}
	if (EXTI->PR & (EXTI_PR_PIF7)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF7); // Writing a 1 clears it
	}
}
/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/
/* This function handles SysTick Handler.  */
void SysTick_Handler(void){
    Tick++;

    heartBeats++;
    if( heartBeats == 10000 ){ // Every 1s
    	HEART_ON;
    	heartBeats=0;
    }else if( heartBeats == 1000 ){ // Stay one for 100ms
    	HEART_OFF;
    }
    if(delay>0 )
    	delay--;
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
