#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "pwm.h"
#if BOARD_IR_TESTBOARD


#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

#define CURRENT_UART &Uart3
#define PC_UART &Uart1
#define CONSOLE_UART &Uart1
#define IAP_UART &Uart1



#define RedLed_On() GPIO_WriteBit(GPIOB, GPIO_Pin_6, 0)
#define RedLed_Off() GPIO_WriteBit(GPIOB, GPIO_Pin_6, 1)
#define GreenLed_On() GPIO_WriteBit(GPIOB, GPIO_Pin_7, 0)
#define GreenLed_Off() GPIO_WriteBit(GPIOB, GPIO_Pin_7, 1)



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



void change_pwm_persen( int persen )
{
	//pwm_restart( TIM4 , 3 , 10*persen );
	pwm_set( TIM4 , 3 , 10*persen );
}

void pwm_output_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	pwm_init( TIM4, 3, 1000 , 50 , 1, 0 );
	change_pwm_persen(20);
}


/*
int pwm_capture_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if( 0 > pwm_input( TIM3 , 1000, 25 , TIM_Channel_2) ){
		_LOG("pwm input init error \n");
		return -1;
	}
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	return 0;
}

volatile uint16_t period,duty;
volatile int pwm_capture_availed = 0;

void TIM3_IRQHandler(void)
{
	if( SET == TIM_GetITStatus(TIM3, TIM_IT_CC2 ) )
	{
		period = TIM_GetCapture2(TIM3);
		if( period > 0 )
		{
			duty = TIM_GetCapture1(TIM3);
			pwm_capture_availed = 1;
		}else{
			//break here
			period = 0;
			duty = 0;
			pwm_capture_availed = 0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}else if( SET == TIM_GetITStatus(TIM3 , TIM_IT_Update) ){
		period = TIM_GetCapture2(TIM3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}else{
		//break here
		TIM_ClearITPendingBit(TIM3, 0xFF);
	}
}
 
 
void pwm_check_event()
{
	int diff;
	
	if( pwm_capture_availed ){
		pwm_capture_availed = 0;
		//_LOG("pwm: %d,%d\n", period,duty);
		diff = duty*2 - period;
		diff = diff > 0 ? diff:(-1*diff);
		if( diff < 5 ){
			GreenLed_On();
			_LOG("1");
		}else{
			GreenLed_Off();
			_LOG("0");
		}
	}else{
		GreenLed_Off();
	}
}
 */





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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
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
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5);//ADC_SampleTime_71Cycles5	
 
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




/*
*  get adc value
*		if ok , return db ( db > 0 )
*  	else  ,  return error ( -1 )
*/
int get_vcc()
{
	while( adc_updated == 0) ;
	return Cali_Adc_Value(0);
}

volatile int start_pwm_check_flag = 0;
volatile int pwm_check_false_counter = 0;
volatile int adc_sum,adc_countL,adc_countH, adc_mid;
volatile int output_pwm_persen = 20;
systick_time_t pwm_timer;

//INTERVAL_MS * SAMPLE_COUNT = 400 ms , 50HZ=20ms a period, so we will sum 200ms/20ms=10 period .
#define INTERVAL_MS 1 
#define SAMPLE_COUNT 100 


void pwm_adc_init()
{
	ADC_Configuration ();
	systick_init_timer( &pwm_timer, INTERVAL_MS);
	adc_sum = 0;
	adc_countL = 0;
	adc_countH = 0;
	adc_mid = 10; // will change auto
	output_pwm_persen = 20;
	start_pwm_check_flag = 0;
	pwm_check_false_counter = 0;
	
	GreenLed_Off();
}

void pwm_adc_event()
{
	int adc, persen,diff;
	
	//if( start_pwm_check_flag == 0 ) return;
	if( 0 == systick_check_timer( &pwm_timer ) ) return ;
		
	adc = get_vcc();
	if( adc == -1 ) return;
	//_LOG("v=%d\n", adc );

	if( adc > adc_mid  ){
		adc_sum ++;
		adc_countH ++;
	}else if( adc >=0 ){
		adc_countL ++;
		adc_sum ++;
	}else{
		;
	}
	if( adc_sum < SAMPLE_COUNT ) return ;
	
	// compare
	//_LOG("mid=%d,H=%d,L=%d\n",adc_mid,adc_countH,adc_countL);
	persen = adc_countH ; // adc_countH*100/adc_sum
	diff = persen - output_pwm_persen;
	diff = diff>0 ? diff:(-1*diff);
	if( diff <= 3 ){
		if( output_pwm_persen == 80 ){
			_LOG("OK\n");
			GreenLed_On();
			RedLed_Off();
			start_pwm_check_flag = 0;
			pwm_check_false_counter = 0;
			output_pwm_persen = 20;
		}else{
			output_pwm_persen += 30;
		}
	}else{
		
		pwm_check_false_counter ++;
		
		if( pwm_check_false_counter >= 3 ){
			_LOG("NG\n");
			GreenLed_Off();
			RedLed_On();
			output_pwm_persen = 20;
			pwm_check_false_counter = 0;
		}
		
		//reset adc_mid
		change_pwm_persen(100);
		systick_delay_us(40000);
		adc_mid = get_vcc();
		_LOG("H=%d,",adc_mid);
		change_pwm_persen(0);
		systick_delay_us(40000);
		adc_mid += get_vcc();
		_LOG("S=%d,",adc_mid);
		adc_mid = adc_mid >> 1;
		_LOG("m=%d\n",adc_mid);
	}
	change_pwm_persen(output_pwm_persen);
	//systick_delay_us(2000);
	get_vcc();
	
	
	// reinit
	adc_sum = 0;
	adc_countL = 0;
	adc_countH = 0;
	
}









void gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB , GPIO_InitStructure.GPIO_Pin);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA , GPIO_InitStructure.GPIO_Pin);
	
	
	// led pass false
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB , GPIO_InitStructure.GPIO_Pin);
	
}

void numberled_toggle()
{
	static int step = 0;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
	GPIO_WriteBit(GPIOA, GPIO_Pin_5,0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_3,0);
	switch ( step ){
		case 0:
			GPIO_SetBits(GPIOB, GPIO_Pin_4);
			//GPIO_SetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		case 1:
			//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15 );
			GPIO_SetBits(GPIOA,GPIO_Pin_6);
			break;
		case 2:
			//GPIO_SetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4  | GPIO_Pin_6 | GPIO_Pin_8);
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			break;
		case 3:
			GPIO_SetBits(GPIOA, GPIO_Pin_15);
			//GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		case 4:
			GPIO_SetBits(GPIOA,  GPIO_Pin_8);
			//GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		case 5:
			//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB,GPIO_Pin_14  );
			break;
		case 6:
			//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB, GPIO_Pin_15);
			break;
		case 7:
			//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB, GPIO_Pin_13 );
			GPIO_WriteBit(GPIOA, GPIO_Pin_5,1);
			GPIO_WriteBit(GPIOB, GPIO_Pin_3,0);
			break;
		case 8:
			//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB, GPIO_Pin_13 );
			GPIO_WriteBit(GPIOA, GPIO_Pin_5,0);
			GPIO_WriteBit(GPIOB, GPIO_Pin_3,1);
			break;
		case 9:
			GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		case 10:
			GPIO_WriteBit(GPIOA, GPIO_Pin_5,1);
			GPIO_WriteBit(GPIOB, GPIO_Pin_3,1);
			break;
		default:
			step = 0;
			break;
		
	}

	step++;
	step %= 11;
	
}

void barled_toggle()
{
	static int step = 0;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 );
	
	switch ( step ){
		case 0:
			GPIO_SetBits(GPIOA,GPIO_Pin_0 );
			break;
		case 1:
			GPIO_SetBits(GPIOA,GPIO_Pin_1 );
			break;
		case 2:
			GPIO_SetBits(GPIOA,GPIO_Pin_2 );
			break;
		case 3:
			GPIO_SetBits(GPIOA,GPIO_Pin_3 );
			break;
		case 4:
			GPIO_SetBits(GPIOA,GPIO_Pin_4 );
			break;
		default:
			step = 0;
			break;
		
	}
	step++;
	step %= 5;
}






static int sw_value;
static int sw_counter;
static int sw_on_tag;
static int sw_pressed;
#define SW_MAX_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
systick_time_t sw_timer;


void switch_det_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	sw_value = 0;
	sw_counter = 0;
	sw_on_tag = 0;
	sw_pressed = 0;
	
	systick_init_timer( &sw_timer, 10 );
}

void switch_det_event()
{
	
	if( 0 == systick_check_timer( &sw_timer ) )
		return;
	
	sw_value +=  GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_5 );
	sw_counter++;
	
	if( sw_counter >= SW_MAX_COUNT ){

		//if( sw_value >= SW_MAX_COUNT ){
		if( sw_value == 0 ){
			sw_on_tag = 1;
		}else {
			sw_on_tag = 0;
		}
		sw_value = 0;
		sw_counter = 0;
		
		
		if( sw_pressed == 0 ){
			sw_pressed = sw_on_tag;
		}else{
			if( sw_on_tag == 0 ){
				sw_pressed = 2;
				//or
				//sw_pressed = 0;
				//sw_press_handler();
			}
		}
	}
}

int switch_is_pressed()
{
	if( sw_pressed == 2 ){
		sw_pressed = 0;
		return 1;
	}
	return 0;
}

int switch_is_on()
{
	return sw_on_tag;
}






volatile int suspend_mode = 0;
volatile int key_press = 0;

void EXTI_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void enter_sleep()
{
	GreenLed_Off();
	RedLed_Off();
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);//进入停止模式
}

void wake_up()
{
	SystemInit();//时钟初始化
	RedLed_On();
}

void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		key_press = 1;
		if( suspend_mode == 1 ){
			wake_up();
			suspend_mode = 0;
			return;
		}else{
			suspend_mode = 1;
			enter_sleep();
		}
	}
}

int is_key_press()
{
	if( key_press == 1 ){
		key_press = 0;
		return 1;
	}
	return 0;
}






systick_time_t led_timer;
systick_time_t led2_timer;

void app_init()
{
	// led io
	gpio_init();
	// uart debug
	Uart_init();
	
#if BOARD_HAS_IAP
	if( IAP_PORT_USB == 1 ) iap_init_in_usb();
	else if( IAP_PORT_CAN1 == 1 )iap_init_in_can1();
	else if( IAP_PORT_UART == 1) iap_init_in_uart( IAP_UART );
#endif

	//console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	
	
	// IR detect 
	pwm_output_init();
	pwm_adc_init();
	// suspend button
	//EXTI_init();
	
	// start ui led board display
	systick_init_timer( &led_timer, 400);
	systick_init_timer( &led2_timer, 400);
	//numberled_toggle();
	//GPIO_SetBits(GPIOA,GPIO_Pin_4 |GPIO_Pin_3 |GPIO_Pin_2 |GPIO_Pin_1|GPIO_Pin_0  );
	
	switch_det_init();
}


void app_event()
{
	if( 1 == systick_check_timer( &led_timer) ){
		barled_toggle();
	}
	if( 1 == systick_check_timer( &led2_timer) ){
		numberled_toggle();
	}

	switch_det_event();
	if( switch_is_pressed() )
		start_pwm_check_flag = 1;
	
	pwm_adc_event();
}

#endif