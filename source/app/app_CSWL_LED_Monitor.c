#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "flashRW.h"

#if BOARD_CSWL_LED_MONITOR


volatile int debug_en = 1;
#define _LOG(X...) if( debug_en ) printf(X)

Uart_t Uart1;
Uart_t Uart3;
#define PC_UART &Uart1
#define IAP_UART &Uart1
#define CONSOLE_UART &Uart3


static void app_SetSysClock(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
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
		
		//8M to PLL need to div8 , and then go to the PLL mul , which can be adjust to be more higher freq.
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);//4*12 = 48M , can use usb.
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};
			
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
  }
}




void app_SystemInit (void)
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
    

  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
  app_SetSysClock();
	
	SystemCoreClockUpdate();


//Vector Table base offset field.This value must be a multiple of 0x200.
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | 0; /* Vector Table Relocation in Internal SRAM. */
#else
  SCB->VTOR = FLASH_BASE | 0; /* Vector Table Relocation in Internal FLASH. */
#endif 
}





void Uart_USB_SWJ_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	Uart_Configuration (&Uart1, USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#if 0
	//USART2								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart2, USART2, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
#endif

#if 1
	//USART3								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart3, USART3, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	
#if BOARD_HAS_IAP
	#if IAP_PORT_USB 
	iap_init_in_usb();
	#endif
	#if IAP_PORT_CAN1
	iap_init_in_can1();
	#endif
	#if  IAP_PORT_UART
	iap_init_in_uart( IAP_UART );
	#endif
#endif

	console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	
}


#define CAPTURE_IDEL			0
#define CAPTURE_STARTED		1
#define CAPTURE_FINISH		2
volatile unsigned int capture_time_ms	= 5 ;  // [ 0.1ms, 600ms ]
volatile unsigned int capture_1to6_tickers = 0;
volatile unsigned int capture_1to6_periods = 0;

volatile unsigned int capture_7to12_tickers = 0;
volatile unsigned int capture_7to12_periods = 0;

unsigned int capture_timer_period = 0xffff;
volatile int capture_counter_status = CAPTURE_IDEL;


void capture_counter_timer_init(TIM_TypeDef *timer)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = capture_timer_period-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(timer, DISABLE);
	
	//TIM_ITRxExternalClockConfig(timer,TIM_TS_ETRF);
	TIM_ETRClockMode2Config(timer, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	//TIM_SelectInputTrigger(timer, TIM_TS_ETRF);
	
	TIM_ClearITPendingBit(timer, TIM_IT_Update);
	TIM_SetCounter(timer, 0);
	//TIM_Cmd(timer, ENABLE);
	
}


void capture_counter_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	
	capture_counter_timer_init(TIM2);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	capture_counter_timer_init(TIM1);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseStructure.TIM_Period = (100*capture_time_ms)-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	
	// Enable the Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update );
		capture_1to6_periods++;
	}
}

void TIM1_UP_IRQHandler()
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update );
		capture_7to12_periods++;
	}
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM1, DISABLE);
		
		capture_1to6_tickers = TIM_GetCounter(TIM2);
		capture_7to12_tickers = TIM_GetCounter(TIM1); 
		capture_1to6_tickers += ( capture_1to6_periods * capture_timer_period );
		capture_7to12_tickers += ( capture_7to12_periods * capture_timer_period );
		capture_counter_status = CAPTURE_FINISH;
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
		TIM_Cmd(TIM3, DISABLE);
	}
	
}


void capture_counter_stop()
{
	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);
	capture_counter_status = CAPTURE_IDEL;
}

void capture_counter_start()
{
	capture_counter_stop();
	
	capture_1to6_tickers = 0;
	capture_1to6_periods = 0;
	capture_7to12_tickers = 0;
	capture_7to12_periods = 0;
	capture_counter_status = CAPTURE_STARTED;
	
	TIM_SetCounter(TIM2, 0);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_SetCounter(TIM1, 0);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_Cmd(TIM1, ENABLE);
	
	TIM_SetCounter(TIM3, 0);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM3, ENABLE);
}

int capture_counter_check(int *freq1_6, int *freq_7_12)
{
	if( capture_counter_status == CAPTURE_FINISH )
	{
		*freq1_6 = capture_1to6_tickers *1000 / capture_time_ms ;// HZ
		*freq_7_12 = capture_7to12_tickers *1000 / capture_time_ms ;// HZ
		capture_counter_stop();
		return 1;
	}
	return 0;
}










#define S0(X) GPIO_WriteBit( GPIOB, GPIO_Pin_1 , X)
#define S1(X) GPIO_WriteBit( GPIOB, GPIO_Pin_0 , X)
#define S2(X) GPIO_WriteBit( GPIOA, GPIO_Pin_6 , X)
#define S3(X) GPIO_WriteBit( GPIOA, GPIO_Pin_5 , X)
#define E0(X) GPIO_WriteBit( GPIOA, GPIO_Pin_4 , X)

void tc3200_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	E0(0);
	S1(1);
	S2(1);
	S3(0);
	S0(1);
}











void capture_init()
{
	capture_counter_init();
	
	//TODO tsc3200 init
	tc3200_gpio_init();

	capture_counter_start();
}

void capture_loop()
{
	int freq1=0,freq2=0;
	
	//TODO init the tsc3200 
	
	
	if( 1 == capture_counter_check(&freq1, &freq2) ){
		systick_delay_ms(50);
		_LOG("%d   ,  %d\n", freq1,freq2);
		capture_counter_start();
	}
	
}



void cmd_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = Uart_Get(CONSOLE_UART,buffer,1);// USB_RxRead( buffer, 8 );
	
	if( len == 0 ) return;

	if( buffer[0] == '1' ){

	}else if( buffer[0] == '0' ){

	}
}




void app_init()
{
	Uart_USB_SWJ_init();
	capture_init();
	if( SystemCoreClock != 72000000 ){
		while(1);
	}
}




volatile int led = 0;
void app_event()
{
	capture_loop();
	cmd_even();
}

#endif //BOARD_CSWL_LED_MONITOR
