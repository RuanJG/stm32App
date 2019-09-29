#include "stm32f10x_conf.h"
#include "main_config.h"
#include "iap.h"
#include "uart.h"
#include "can1.h"
#include "bsp.h"
#include "systick.h"
#include "hw_config.h"
#include "app.h"



void bsp_stm32f10x_hsi()
{
		return ;
	
		//using HSI to PLL to 48M, when SystemCoreClockUpdate() , check the SystemCoreClock != 72000000 , will know whether it is failed.
		RCC_DeInit();
		RCC_HSICmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET){};
		
		//加上这两句才能到64M
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  
    FLASH_SetLatency(FLASH_Latency_2);  
		
		RCC_HCLKConfig(RCC_SYSCLK_Div1);     
    RCC_PCLK1Config(RCC_HCLK_Div2);  
    RCC_PCLK2Config(RCC_HCLK_Div1);  
		RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
		RCC_ADCCLKConfig(RCC_PCLK2_Div4);
		
		//8M to PLL need to div2 , and then go to the PLL mul , which can be adjust to be more higher freq.
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);//4*12 = 48M , can use usb.
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};
			
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
}

void bsp_stm32f10x_SetSysClock(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
	
	if( HSE_VALUE == 0 ){
		bsp_stm32f10x_hsi();
		return ;
	}
	
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

 
		// HSI/HSE -> DIV/MUL -> PLLCLK -> SYSCLK -> AHB DIV -> HCLK-> AHB bus, flash , DMA , FCLK
		//                              -> USB                      -> /8 -> systick
		//                                                          -> /2 = PCLK1 (APB1 CLK) max(36MHZ) (! TIM2-7 : if( PCLK1.div != 1 ) CLK = 2*PCLK1 ; else CLK = PCLK1)
		//                                                          -> /1 = PCLK2 (APB2 CLK) max (72M) (! TIM1 : if( PCLK2.div != 1 ) CLK = 2*PCLK2 ; else CLK = PCLK2)
		//                                                                        -> PCLK2 /2/4/8 -> ADC_CLK (ADC1,ADC2 max 14M) 
		/* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK/2 */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
		
		/* PLLCLK /1.5 =  USBCLK  */
		RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    //ADC CLK
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	  
	  
#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
        
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }
    
    // clear bits
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
		

		if( HSE_VALUE == 8000000 ){
			/* PLL configuration: PLLCLK = 8M/1 * 9 = 72 MHz */ 
			RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9);
		}else if ( HSE_VALUE == 16000000 ) {
			/* PLL configuration: PLLCLK = 16M/2 * 9 = 72 MHz */ 
			RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9);
		}
		
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
    /*  PLL configuration:  = (HSE / 2) * 6 = 24 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL6);
		
#else    
    //clear bits
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		
		if( HSE_VALUE == 8000000 )
		{
			//8M HSE :  8M/1 * 9 = 72M
			RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_PLLMULL9);
		}else if ( HSE_VALUE == 16000000 ) {
			//16M HSE:  16M/2 *9 = 72M
			RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_Div2 | RCC_CFGR_PLLMULL9);
		}
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { 
		bsp_stm32f10x_hsi();
  }
}




void bsp_stm32f10x_SystemInit (void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#ifndef STM32F10X_CL
  RCC->CFGR &= (uint32_t)0xF8FF0000;
#else
  RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif /* STM32F10X_CL */   
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

#ifdef STM32F10X_CL
  /* Reset PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEBFFFFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;      
#else
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
#endif /* STM32F10X_CL */
    
#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
  #ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl(); 
  #endif /* DATA_IN_ExtSRAM */
#endif 

  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */

  bsp_stm32f10x_SetSysClock();
	SystemCoreClockUpdate();


//Vector Table base offset field.This value must be a multiple of 0x200.
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | 0; /* Vector Table Relocation in Internal SRAM. */
#else
  SCB->VTOR = FLASH_BASE | 0; /* Vector Table Relocation in Internal FLASH. */
#endif 
}










Uart_t *pConsoleUart = CONSOLE_NONE_TYPE;
volatile int console_type=0 ;
volatile int console_cmd_index;
static unsigned char * console_cmd_buffer;
int console_cmd_buffer_size;

void console_init(int type, void * pridata )
{
	if( type == CONSOLE_USB_TYPE  && BOARD_USING_USB == 1 ){
		console_type |= type;
	}
	
	if( (type == CONSOLE_UART_TYPE) && (pridata != NULL) ){
		console_type |= type;
		pConsoleUart = (Uart_t*) pridata;
	}
	
	console_cmd_index = -1;
	console_cmd_buffer = NULL;
	console_cmd_buffer_size = 0;
}

int console_cmd_config( unsigned char *buffer , int size )
{
	console_cmd_index = -1;
	console_cmd_buffer = buffer;
	console_cmd_buffer_size = size;
}

int console_cmd_parse( unsigned char data )
{
	int len;
	
	if( console_cmd_index >= console_cmd_buffer_size )  console_cmd_index = -1;
	
	// get start
	if( console_cmd_index == -1 ){
		if( data != '/' ) return 0;
		console_cmd_index = 0;
		return 0;
	}
	
	//get end
	if( data == '/' ){
		len = console_cmd_index;
		console_cmd_index = -1;
		return len;
	}else{
		console_cmd_buffer[console_cmd_index++] = data;
		return 0;
	}

}


// return cmd length
int console_cmd_check()
{
	unsigned char data;
	
	if( console_cmd_buffer_size <= 0 ) return 0;
	
	if( (console_type & CONSOLE_UART_TYPE) != 0   &&   pConsoleUart != NULL ){
		if( 1 == Uart_Get(pConsoleUart,&data,1) ){
			return console_cmd_parse( data ) ;
		}
	}else{
		if( (console_type & CONSOLE_USB_TYPE) != 0){
			if( 1 == USB_RxRead( &data, 1 ) ){
				return console_cmd_parse( data );
			}
		}
	}
}

#if !BOARD_SPECIAL_FPUT
int fputc(int ch, FILE *f)
{
	//TODO lock ?
	
	if( (console_type & CONSOLE_UART_TYPE) != 0   &&   pConsoleUart != NULL ){
		Uart_Put( pConsoleUart, (unsigned char*)&ch, 1);
	}
	
	if( (console_type & CONSOLE_USB_TYPE) != 0){
		USB_TxWrite( (unsigned char*)&ch, 1);
	}
	return (ch);
}
#endif














void bsp_init()
{
	
#if BOARD_PRIVATE_SETUP_CLK
	app_SystemInit();
#elif BOARD_COMMON_SETUP_CLK
	#if (defined STM32F10X_HD) || (defined STM32F10X_MD)
	bsp_stm32f10x_SystemInit();
	#else
	#error No Clock setup to define
	#endif
#else
	SystemInit();
#endif
	
#if BOARD_HAS_IAP
	iap_config_vect_table();
#endif
	
	NVIC_PriorityGroupConfig(CUSTOM_SYSTICK_IRQ_PRIORITY);
	
	//if no this setting , flash will very slow
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
#if BOARD_USING_SYSTICK
	systick_init(BOARD_SYSTICK_FREQ); 
#endif

#if BOARD_USING_USB
	USB_Config();
#endif
	
}

void bsp_event()
{
#if BOARD_USING_SYSTICK
	systick_event();
#endif
	
}

void bsp_deinit()
{
#if ( BOARD_IAP == 1 ) || ( BOARD_HAS_IAP == 1 )
	  if( 0 != IAP_PORT_CAN1 )
			CAN_DeInit(CAN1);
	
	#if BOARD_IAP
	  if( 0 != IAP_PORT_UART )
			USART_DeInit(IAP_UARTDEV);
	#endif
	
	#if IAP_PORT_USB
		USB_Deinit();
	#endif
#endif
	
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	
	systick_deinit();
}






