#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "pwm.h"
#include "switcher.h"


#if BOARD_UC_SHAVER_SAMPLE


#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;


#define CONSOLE_UART &Uart1
#define IAP_UART &Uart1



#define WashingLed_On() GPIO_WriteBit(GPIOB, GPIO_Pin_15, 1)
#define WashingLed_Off() GPIO_WriteBit(GPIOB, GPIO_Pin_15, 0)

#define ShaverLed_On() GPIO_WriteBit( GPIOC, GPIO_Pin_13 , 0);
#define ShaverLed_Off() GPIO_WriteBit( GPIOC, GPIO_Pin_13 , 1);

void WashingLed_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	WashingLed_Off();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	ShaverLed_Off();
}





void Uart_init()
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

	Uart_Configuration (&Uart1, USART1, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
/*
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
*/
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

}





#define ADC_SAMPLE_COUNT 32  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
#define ADC_SAMPLE_CHANNEL_COUNT 1
static unsigned short escADCConvert[ADC_SAMPLE_COUNT][ADC_SAMPLE_CHANNEL_COUNT];
volatile int adc_updated;

void ADC_Configuration ()
{										 
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC_SAMPLE_CHANNEL_COUNT;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// 239.5 sampleTime => (239.5+12.5)/12M = 21us; 21*ADC_SAMPLE_COUNT * 3 = ADC_SAMPLE_COUNT * 63us = 1.890ms each group
	// 71.5 smapleTime => 7us; 7us*ADC_SAMPLE_COUNT*3 = ADC_SAMPLE_COUNT* 21us = 0.630 ms 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);//ADC_SampleTime_71Cycles5	
 
	// set ADC channel for temperature sensor
	//ADC_TempSensorVrefintCmd(ENABLE);
	
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);
 
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_CHANNEL_COUNT*ADC_SAMPLE_COUNT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&escADCConvert;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);


  /* Enable DMA channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_DAM1_IRQ_PREPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_DAM1_IRQ_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	adc_updated = 0;

}

void DMA1_Channel1_IRQHandler(void)
{
 if(DMA_GetITStatus(DMA1_IT_TC1))
 {
	 adc_updated = 1;
	 DMA_ClearITPendingBit(DMA1_IT_GL1);
 }
}


unsigned short Cali_Adc_Value(int id)
{
	int i;
	u32 sensor = 0; 	
	for(i=0;i<ADC_SAMPLE_COUNT;i++)
	{
		sensor += escADCConvert[i][0];
	}
	sensor = sensor>>5;
	return sensor;
}



int is_charging()
{
	int mv;
	
	while( adc_updated == 0) ;
	
	mv=Cali_Adc_Value(0)*3300/4095;
	if( mv > 1500 )
		return 1;
	else
		return 0;
}



#define MOTOR_FREQ  50
#define MOTOR_PERIOD 1000
#define MOTOR_STOP_PWM  0
#define MOTOR_START_PWM 100

void shaver_set_pwm(unsigned short pwm)
{
	pwm_set( TIM3 , 2, pwm );
}

unsigned short shaver_get_pwm()
{
	return pwm_get(TIM3, 2);
}


void pump_set_pwm(unsigned short pwm)
{
	pwm_set( TIM2 , 3, pwm );
}

unsigned short pump_get_pwm()
{
	return pwm_get(TIM2, 3);
}


void pwm_output_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	
	
	//shaver
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	pwm_init( TIM3, 2, MOTOR_PERIOD , MOTOR_FREQ , 1, 0);
	shaver_set_pwm( MOTOR_STOP_PWM );
	
	
	//pump
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	pwm_init( TIM2, 3, MOTOR_PERIOD , MOTOR_FREQ , 1, 0);
	pump_set_pwm( MOTOR_STOP_PWM );

}




#define WASHING_INTERVEL_MS 10
#define WASHING_LOOP_MAX_COUNT 34  // washing period = 7s , washing time = count * 7s
volatile int washing_mode;
systick_time_t washing_timer;
volatile int washing_step;
volatile int washing_ms;
volatile int washing_loop_counter;

void washing_start()
{
	washing_mode = 1;
	washing_step = 0;
	washing_ms = 0;
	washing_loop_counter = 0;
	shaver_set_pwm(MOTOR_STOP_PWM);
	pump_set_pwm(MOTOR_STOP_PWM);
	WashingLed_On();
}

void washing_stop()
{
	washing_mode = 0;
	shaver_set_pwm(MOTOR_STOP_PWM);
	pump_set_pwm(MOTOR_STOP_PWM);
	WashingLed_Off();
	washing_step = 0;
	washing_ms = 0;
}

void washing_init()
{
	washing_stop();
	washing_step = 0;
	systick_init_timer( &washing_timer, WASHING_INTERVEL_MS );
}

void washing_event()
{
	if( washing_mode == 0 ) return;
	
	if( 0 == systick_check_timer( &washing_timer ) ) return; 
	
	
	switch( washing_step )
	{
		case 0:
			washing_ms += WASHING_INTERVEL_MS;
			if( washing_ms > 500 )
			{
				washing_step++;
				washing_ms = 0;
				washing_loop_counter = 0;
				pump_set_pwm(MOTOR_START_PWM);
			}
			break;
			
		case 1:
			washing_ms += WASHING_INTERVEL_MS;
			if( washing_ms == 3000) 
			{
				pump_set_pwm(MOTOR_STOP_PWM);
			}else if( washing_ms ==4000  )
			{
				shaver_set_pwm(MOTOR_START_PWM);
			}
			if( washing_ms ==6000 )
			{
				shaver_set_pwm(MOTOR_STOP_PWM);
			}
			if( washing_ms ==7000 )
			{
				washing_loop_counter++;
				if( washing_loop_counter < WASHING_LOOP_MAX_COUNT ){
					pump_set_pwm(MOTOR_START_PWM);
					washing_ms = 0;
				}else{
					washing_step++;
				}
			}
			break;
		
		case 2:
			//finish
			washing_stop();
			break;
		
		default:
			washing_stop();
			break;
	}
	
}



void safe_checking_event()
{	
	int charging ;
	
	charging  =  is_charging() ;
	
	if( charging ==1 ||  washing_mode == 0 )
	{
		if( pump_get_pwm() != MOTOR_STOP_PWM )
		{
			pump_set_pwm(MOTOR_STOP_PWM);
			_LOG("ERROR: pump unexpection running \n");
		}
		if( charging ==1 && shaver_get_pwm() != MOTOR_STOP_PWM )
		{
			shaver_set_pwm(MOTOR_STOP_PWM);
			ShaverLed_Off();
		}
		
		if( washing_mode ==1 ){
			washing_stop();
		}
	}
}




systick_time_t heart_led_timer;
volatile int washingLed_flash;
volatile int washingLed_flash_ms;

void heart_led_init()
{
	systick_init_timer(&heart_led_timer, 100);
	washingLed_flash = 0;
	washingLed_flash_ms = 0;
	
}

void heart_led_start()
{
	washingLed_flash_ms = 0;
	washingLed_flash =1;
}

void heart_led_stop()
{
	//washingLed_flash_ms = 0;
	washingLed_flash =0;
}



void heart_led_even()
{
	if( washingLed_flash == 0 ) return;
	
	if( systick_check_timer( &heart_led_timer ) ){
		washingLed_flash_ms+=100;
			
		if( washingLed_flash_ms > 1000  && washingLed_flash_ms%200 == 0 ){
			WashingLed_On();
		}else{
			WashingLed_Off();
		}
		
		
		if( washingLed_flash_ms >= 3000 ){
			washingLed_flash = 0;
			WashingLed_On();
		}
		
		
	}
}





#define SW_INTERVAL_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher startButton;
volatile unsigned int pressMs;

void switch_key_press_handler()
{
	pressMs = systick_get_ms();
	heart_led_start();
}

void switch_key_release_handler()
{
	volatile unsigned int releaseMs;
	
	heart_led_stop();
	
	if( washing_mode == 1 ){
		washing_stop();
		return;
	}
	
	releaseMs = systick_get_ms() - pressMs ;
	
	//if( releaseMs > 3000 && releaseMs < 10000 ){
	if( washingLed_flash_ms >= 3000 ){
		//start/stop washing
		if( is_charging() == 1 )
			return;
	
		washing_start();
		
	}else if( releaseMs > 10 ){
		//start/stop shaver
		if( is_charging() == 1 )
			return;
		
		if( shaver_get_pwm() != MOTOR_STOP_PWM ) {
			shaver_set_pwm(MOTOR_STOP_PWM);
			ShaverLed_Off();
		}else{
			shaver_set_pwm(MOTOR_START_PWM);
			ShaverLed_On();
		}
		WashingLed_Off();
	}else{
		_LOG("ERROR: unexpect even in key release  press:%d-->release:%d\n",pressMs,systick_get_ms());
		WashingLed_Off();
	}
	
}


void switch_init()
{
	switcher_init( &startButton, SW_INTERVAL_COUNT, 1 , 0 , GPIOB, GPIO_Pin_6, switch_key_press_handler , switch_key_release_handler);

	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &startButton );

}




void app_init()
{

	WashingLed_init();
	// uart debug
	Uart_init();
	
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

	//console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	
	ADC_Configuration ();
	switch_init();
	pwm_output_init();
	washing_init();
	heart_led_init();

}


void app_event()
{
	switch_even();
	washing_event();
	safe_checking_event();
	heart_led_even();
}

#endif