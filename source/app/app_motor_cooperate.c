#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "pwm.h"
#include "LCD1602.h"
#include "hw_config.h"
#include "switcher.h"
#if BOARD_MOTOR_COOPERATE


#define _LOG(X...) if( 1 ) printf(X);



volatile char current_point = 0 ; // 0 motor1 round ; 1 motor1 speed ; 2 motor2 round; 3 motor2 speed



Uart_t Uart1;
//Uart_t Uart2;
//Uart_t Uart3;

#define CONSOLE_UART &Uart1
#define IAP_UART &Uart1



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

	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);



}






#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif
#define CONFIG_ADDRESS (0x8010000-FLASH_PAGE_SIZE)
#define CONFIG_SOTRE_SIZE FLASH_PAGE_SIZE

// the size of struct config should be n*4Byte
struct config {
	int config_avaliable;
	int motor1_speed;
	int motor1_circle_count;
	int motor1_direction;
	int motor2_speed;
	int motor2_circle_count;
	int motor2_direction;
};
volatile struct config g_config;
systick_time_t auto_save_config_timer;

void config_read()
{
	volatile int * p;
	__IO int* addr;
	int i, int_count;
	
	addr = (__IO int*) CONFIG_ADDRESS;
	int_count = sizeof( struct config )/sizeof( int );	
	p = (volatile int*)&g_config;
	for( i=0; i< int_count  ; i++ ){
		*(p+i) = *(__IO int*) addr;
		addr++;
	}
	
}

int config_save()
{
	int timeout, i, res;
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	volatile int * data = (volatile int *) &g_config;
	int addr = CONFIG_ADDRESS;
	int int_count = sizeof( struct config )/sizeof( int );
	
	g_config.config_avaliable = 1;
		
	FLASHStatus = FLASH_COMPLETE;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	res = 0;
	timeout = 10;
	while( timeout-- > 0)
	{
		FLASHStatus = FLASH_ErasePage(addr);
		if( FLASHStatus == FLASH_COMPLETE ){
			res = 1;
			break;
		}
	}
	
	if( res == 1 )
	{
		for ( i = 0; i< int_count ; i++ )
		{
			res = 0;
			timeout = 10;
			while( timeout-- > 0)
			{
				FLASHStatus = FLASH_ProgramWord( addr+4*i, *(data+i) ) ;//FLASH_ProgramOptionByteData(IAP_TAG_ADDRESS,tag);
				if( FLASHStatus == FLASH_COMPLETE ){
					res = 1;
					break;
				}
			}
			if( res == 0 ) break;
		}
	}

	FLASH_Lock();
	
	return res;
}


void config_init()
{
	//if no this setting , flash will very slow
	//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	g_config.config_avaliable = 0;
	config_read();
	if( g_config.config_avaliable != 1 ){
		// never init
		g_config.config_avaliable = 1;
		g_config.motor1_circle_count = 0;
		g_config.motor1_direction = 0;
		g_config.motor1_speed = 0;
		g_config.motor2_circle_count = 0;
		g_config.motor2_direction = 0;
		g_config.motor2_speed = 0;
		
		config_save();
	}
	systick_init_timer( &auto_save_config_timer , 1000 );
}

void config_auto_save_even()
{
	if( systick_check_timer( & auto_save_config_timer ) && g_config.config_avaliable == 2){
		config_save();
	}
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
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
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);//ADC_SampleTime_71Cycles5	
 
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


unsigned short Cali_Adc_Value()
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




#define ACCEL_MAX 100
#define ACCEL_MIN 0
volatile int adc_accel;
int accel_check_even()
{
	int accel;
	
	if( adc_updated == 0 ){
		return adc_accel;
	}
	accel = Cali_Adc_Value();
	_LOG("adc=%d\n",accel);
	accel = accel*3300/4096;
	adc_accel = (ACCEL_MAX - ACCEL_MIN)*accel/ (3300/2);
	_LOG("accel=%d\n",adc_accel);
}






#define ONE_ROUND_FREQ 400  //xi feng 
#define MAX_FREQ 4000   // limit=[0,200khz],  20Khz=3000 r/min;  4000Hz=600r/min
#define LIMIT_MAX_SPEED 180 // round/min
#define RATE_ACCLE		300
#define DERATE_ACCLE	400

#define LIMIT_MAX_FREQ 1200 // (LIMIT_MAX_SPEED / 60) * ONE_ROUND_FREQ   PS: to be <= MAX_FREQ
#define LIMIT_MIN_FREQ 1
#define LIMIT_MAX_ROUND_COUNT 99
#define LIMIT_MIN_ROUND_COUNT 1
#define MOTOR_LOOP_INTERVAL_MS 10
#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR_DIRECTION_RIGHT 	Bit_RESET
#define MOTOR_DIRECTION_LEFT 		Bit_SET
#define MOTOR_EN_ON  Bit_RESET
#define MOTOR_EN_OFF Bit_SET




float deaccle;

struct motor {
	unsigned short expect_speed;
	unsigned short expect_freq;
	unsigned short current_freq;
	unsigned short id;
	unsigned int   current_pulse;
	unsigned int   expect_pulse;
	unsigned int   rate_reduce_pulse_point;
	float					 deaccel_freq;
	float          accel_freq;
	float					 accel;
	int						 status;
	//pwm pin
	GPIO_TypeDef* pGPIOx;
	uint16_t pGPIO_pin;
	//pwm
	unsigned char channel;
	TIM_TypeDef* timer ;
	//dir pin
	GPIO_TypeDef* dGPIOx;
	uint16_t dGPIO_pin;
	//en pin
	GPIO_TypeDef* eGPIOx;
	uint16_t eGPIO_pin;
};

volatile struct motor motor1;
volatile struct motor motor2;
systick_time_t motor_timer;


void motor_set_direct(volatile struct motor *motorx , unsigned char direction) 
{
	GPIO_WriteBit( motorx->dGPIOx , motorx->dGPIO_pin, (BitAction) direction);
}

void motor_en(volatile struct motor *motorx , unsigned char en)
{
	GPIO_WriteBit( motorx->eGPIOx , motorx->eGPIO_pin, (BitAction) en);
}




#define ADC_MIN 50
#define ADC_MAX 3500
volatile int adc_freq ;

int adc_freq_update()
{
	int adc;
	if( adc_updated == 0 ){
		return adc_freq;
	}
	adc = Cali_Adc_Value();
	//_LOG("adc=%d\n",adc);

	adc = adc > ADC_MAX ? ADC_MAX:adc;
	adc = adc < ADC_MIN ? ADC_MIN:adc;
	
	// adc_freq/LIMIT_MAX_FREQ = (adc-ADC_MIN)/(ADC_MAX-ADC_MIN)
	adc_freq = LIMIT_MAX_FREQ*(adc-ADC_MIN)/(ADC_MAX-ADC_MIN) ;
	//_LOG("adc_freq=%d\n",adc_freq);
}



int motor_rate(volatile struct motor *motorx, unsigned short freq)
{
	unsigned short period;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if( freq < LIMIT_MIN_FREQ ){
		// stop 
		TIM_Cmd(motorx->timer, DISABLE);
		#if 0
		GPIO_InitStructure.GPIO_Pin=motorx->pGPIO_pin;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
		GPIO_Init(motorx->pGPIOx, &GPIO_InitStructure);
		GPIO_WriteBit(motorx->pGPIOx,motorx->pGPIO_pin, Bit_RESET);
		#endif
		//pwm_set( motorx->timer, motorx->channel, 0);
		_LOG("Motor%d set freq(%d)\n", motorx->id, freq);
		
	}else if( freq <= LIMIT_MAX_FREQ) {
		//set freq
		period = SystemCoreClock / (motorx->timer->PSC+1) / freq; 
		TIM_SetAutoreload( motorx->timer , period );
		pwm_set(motorx->timer , motorx->channel , 10);
		TIM_Cmd(motorx->timer, ENABLE);
		#if 0
		GPIO_InitStructure.GPIO_Pin=motorx->pGPIO_pin;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
		GPIO_Init(motorx->pGPIOx, &GPIO_InitStructure);
		#endif
		
		_LOG("Motor%d set freq(%d), start...\n", motorx->id, freq);
	}else{
		_LOG("Motor%d set freq(%d) error\n", motorx->id, freq); 
		return -1;
	}
	
	motorx->current_freq = freq;
	return 0;
}


void motor_hw_init(volatile struct motor *motorx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//pwm
	GPIO_InitStructure.GPIO_Pin=motorx->pGPIO_pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(motorx->pGPIOx, &GPIO_InitStructure);
	//default pwm=0 , set freq at max  ; motor controler freq 0-200KHZ , limit at [0,1000] hz
	pwm_init(motorx->timer, motorx->channel, 20 ,  MAX_FREQ , 1 , 1); 
	TIM_ARRPreloadConfig( motorx->timer, ENABLE);
	switch( motorx->channel ){
	case 1:
		TIM_OC1PreloadConfig( motorx->timer, TIM_OCPreload_Enable);
		break;
	case 2:
		TIM_OC2PreloadConfig( motorx->timer, TIM_OCPreload_Enable);
		break;
	case 3:
		TIM_OC3PreloadConfig( motorx->timer, TIM_OCPreload_Enable);
		break;
	case 4:
		TIM_OC4PreloadConfig( motorx->timer, TIM_OCPreload_Enable);
		break;
	}

	//en pin
	GPIO_InitStructure.GPIO_Pin=motorx->eGPIO_pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(motorx->eGPIOx, &GPIO_InitStructure);
	motor_en( motorx, MOTOR_EN_OFF);
	
	//dir pin
	GPIO_InitStructure.GPIO_Pin=motorx->dGPIO_pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(motorx->dGPIOx, &GPIO_InitStructure);
	//motor_set_direct( motorx, motorx->direction);
	
	motor_rate(motorx, 0);
}

void TIM2_IRQHandler()
{
	if( SET == TIM_GetITStatus(TIM2, TIM_IT_CC3 ) )
	{
		motor2.current_pulse++;
		if( motor2.current_pulse >= motor2.expect_pulse ){
			motor2.current_pulse = motor2.expect_pulse;
			//motor_rate( &motor2, 0 );
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	}else{
		//break here
		TIM_ClearITPendingBit(TIM2, 0xFF);
	}
}

void TIM3_IRQHandler()
{
	//if( SET == TIM_GetITStatus(TIM3 , TIM_IT_Update) )
	if( SET == TIM_GetITStatus(TIM3, TIM_IT_CC2 ) )
	{
		motor1.current_pulse++;
		if( motor1.current_pulse >= motor1.expect_pulse ){
			motor1.current_pulse = motor1.expect_pulse;
			//motor_rate( &motor1, 0 );
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}else{
		//break here
		TIM_ClearITPendingBit(TIM3, 0xFF);
	}
}

void motor_rate_check_even( volatile struct motor *motorx )
{
	int freq, exfreq;
	float accel;
	
	// has been expect speed
	if( motorx->current_freq == motorx->expect_freq ){
		motorx->accel = 0;
		return;
	}
	
	accel = motorx->expect_freq == 0 ? motorx->deaccel_freq:motorx->accel_freq ;
	
	// add or reduce speed
	exfreq = motorx->expect_freq;

	if( motorx->current_freq < exfreq )
	{
		
		motorx->accel += (accel*MOTOR_LOOP_INTERVAL_MS) ;
		freq = motorx->accel; // 去小数
		motorx->accel -= freq;
		
		freq += motorx->current_freq;// add rate
		freq = (freq < exfreq) ? freq: exfreq;
	}else{
		
		motorx->accel += (accel *MOTOR_LOOP_INTERVAL_MS) ;
		freq = motorx->accel; // 去小数
		motorx->accel -= freq;
		
		freq = motorx->current_freq - freq;// reduce rate
		freq = (freq >= exfreq) ? freq: exfreq;
	}
	
	
	//if( freq != motorx->current_freq &&  freq >= LIMIT_MIN_FREQ && freq <= LIMIT_MAX_FREQ ){
	if( freq != motorx->current_freq ) {
		motor_rate( motorx, freq );
	}else{
		//call error
	}
	
	
}



void motor_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	motor1.id = MOTOR1_ID;
	motor1.expect_speed = 0;
	motor1.expect_freq = 0;
	motor1.current_freq = 0;
	motor1.expect_pulse = 0;
	motor1.current_pulse = 0;
	//pwm pin
	motor1.pGPIOx = GPIOA;
	motor1.pGPIO_pin = GPIO_Pin_7;
	//pwm
	motor1.channel = 2;
	motor1.timer = TIM3;
	//dir pin
	motor1.dGPIOx = GPIOA ;
	motor1.dGPIO_pin = GPIO_Pin_6;
	//en pin
	motor1.eGPIOx = GPIOA;
	motor1.eGPIO_pin = GPIO_Pin_15;
	
	
	
	motor2.id = MOTOR2_ID;
	motor2.expect_speed = 0;
	motor2.expect_freq = 0;
	motor2.current_freq = 0;
	motor2.expect_pulse = 0;
	motor2.current_pulse = 0;
	//pwm pin
	motor2.pGPIOx = GPIOA;
	motor2.pGPIO_pin = GPIO_Pin_2;
	//pwm
	motor2.channel = 3;
	motor2.timer = TIM2;
	//dir pin
	motor2.dGPIOx = GPIOA ;
	motor2.dGPIO_pin = GPIO_Pin_1;
	//en pin
	motor2.eGPIOx = GPIOA;
	motor2.eGPIO_pin = GPIO_Pin_3;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO , ENABLE);
	motor_hw_init(&motor1);
	motor_hw_init(&motor2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_CAN1_IRQ_PREPRIORITY; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_CAN1_IRQ_SUBPRIORITY; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	systick_init_timer( &motor_timer , MOTOR_LOOP_INTERVAL_MS );
	
}



void motor_start(volatile struct motor *motorx , unsigned int expect_freq , unsigned int expect_pulse , unsigned char direction)
{
	//加速度是  RATE_ACCLE/MOTOR_LOOP_INTERVAL_MS
	// s = v0 * t + 0.5* a*t*t ; t= (v1-v0)/RATE_ACCLE/MOTOR_LOOP_INTERVAL_MS
	
	//t = v/a;  
	// s = v*t/2  ==>  s = v*(v/a)/2 = (v*v)/(a*2); 
	float accel = 40; // p/ss //expect_freq*expect_freq/ONE_ROUND_FREQ ;// p/s*s
	unsigned int pulse;
	
	
	// running
	//if( motorx->current_pulse < motorx->expect_pulse ){
	//	_LOG("motor%d start : is running , error\n", motorx->id);
	//	return;
	//}
	
	// reinit parameter
	if( expect_freq > LIMIT_MAX_FREQ || expect_freq < LIMIT_MIN_FREQ ){
		_LOG("motor%d start : freq %d error\n", motorx->id, expect_freq);
		return;
	}
	if( expect_pulse > (LIMIT_MAX_ROUND_COUNT*ONE_ROUND_FREQ) || expect_pulse < (LIMIT_MIN_ROUND_COUNT*ONE_ROUND_FREQ) ){
		_LOG("motor%d start : round count error\n", motorx->id);
		return;
	}
	

	motorx->accel_freq = RATE_ACCLE/1000.0 ;//MOTOR_LOOP_INTERVAL_MS*a/1000; //=RATE_ACCLE;
	motorx->deaccel_freq = DERATE_ACCLE/1000.0;
	motorx->accel = 0;
	
	motorx->expect_freq = expect_freq;
	//motorx->current_freq = 0;
	
	motorx->expect_pulse = expect_pulse;
	motorx->current_pulse = 0;
	

	motor_set_direct( motorx, direction);
	motor_en( motorx, MOTOR_EN_ON);

		
	_LOG("Start : expect_pulse=%d, expect_freq = %d, accel=%f\n", expect_pulse, expect_freq, motorx->accel_freq );
	//systick_delay_us(500000);
}


void motor_stop(volatile struct motor *motorx)
{
	unsigned int pulse;
	
	_LOG("Stoped ... \n");
	
	if( motorx->current_freq == 0 ) return;
	
	motorx->accel = 0;
	motorx->expect_freq = 0;
	
	while( motorx->current_freq > 0 ){
		if( systick_check_timer( &motor_timer ) ){
			motor_rate_check_even(&motor1);
			motor_rate_check_even(&motor2);
		}
		systick_delay_ms( MOTOR_LOOP_INTERVAL_MS );
	}
	
	_LOG("Stoped ... \n");
	
}



volatile int motor_manual_status = 0;

void motor_loop()
{
	adc_freq_update();
	
	if( adc_freq > 0 ){
		if(  motor_manual_status  == 0 ){
			motor_start(&motor1 , adc_freq , g_config.motor1_circle_count * ONE_ROUND_FREQ , g_config.motor1_direction);
			motor_manual_status = 1;
		}
	}else{
		if( motor1.current_freq == 0 )  motor_manual_status = 0;
	}
	
	if( motor1.current_pulse == motor1.expect_pulse ){
		motor1.expect_freq = 0;
	}else{
		motor1.expect_freq = adc_freq;
	}
	
	
	if( systick_check_timer( &motor_timer ) ){
		motor_rate_check_even(&motor1);
		motor_rate_check_even(&motor2);
	}
}












#define M1_SPEED_DATA_X 0
#define M1_SPEED_DATA_Y 11 
#define M1_ROUND_DATA_X 0
#define M1_ROUND_DATA_Y 6
#define M1_DERECTION_X	0
#define M1_DERECTION_Y	15

#define M2_SPEED_DATA_X 1
#define M2_SPEED_DATA_Y 11 
#define M2_ROUND_DATA_X 1
#define M2_ROUND_DATA_Y 6
#define M2_DERECTION_X	1
#define M2_DERECTION_Y	15

#define lcd_update_m1_round( )		_LOG( "r1=%d\n",g_config.motor1_circle_count);lcd_display_number( M1_ROUND_DATA_X , M1_ROUND_DATA_Y , g_config.motor1_circle_count, 2 )
#define lcd_update_m1_speed( )		_LOG( "s1=%d\n",g_config.motor1_speed);lcd_display_number( M1_SPEED_DATA_X, M1_SPEED_DATA_Y , g_config.motor1_speed, 3)
#define lcd_update_m2_round( )		_LOG( "r2=%d\n",g_config.motor2_circle_count);lcd_display_number( M2_ROUND_DATA_X , M2_ROUND_DATA_Y , g_config.motor2_circle_count, 2)
#define lcd_update_m2_speed( )		_LOG( "s2=%d\n",g_config.motor2_speed);lcd_display_number( M2_SPEED_DATA_X, M2_SPEED_DATA_Y , g_config.motor2_speed, 3 )
void lcd_update_m1_direction()
{
	if( g_config.motor1_direction==MOTOR_DIRECTION_RIGHT ){
		_LOG( "d1=right\n"); 
		LCD1602_Write(M1_DERECTION_X, M1_DERECTION_Y, 'R');
	}else{
		_LOG( "d1=left\n"); 
		LCD1602_Write(M1_DERECTION_X, M1_DERECTION_Y, 'L');
	}
}

void lcd_update_m2_direction()
{
	if( g_config.motor2_direction==MOTOR_DIRECTION_RIGHT ){
		_LOG( "d2=right\n"); 
		LCD1602_Write(M2_DERECTION_X, M2_DERECTION_Y, 'R');
	}else{
		_LOG( "d2=left\n"); 
		LCD1602_Write(M2_DERECTION_X, M2_DERECTION_Y, 'L');
	}
}

unsigned char lcd_data[2][16] = {
	{'M','1',' ',' ','R',':',' ',' ',' ','S',':',' ',' ',' ',' ',' '},
	{'M','2',' ',' ','R',':',' ',' ',' ','S',':',' ',' ',' ',' ',' '},
};



void lcd_display_number( int x, int y, int number, int count)
{
	for( ; count >0 ; count--)
	{
		LCD1602_Write(x, y+count-1, '0'+ number%10 );
		number/=10;
	}
}

void lcd_update_mouse()
{
	switch( current_point ){
		case 0 :
			LCD1602_SetMouse( M1_ROUND_DATA_X, M1_ROUND_DATA_Y+1);
			break;
			
		case 1:
			LCD1602_SetMouse( M1_SPEED_DATA_X, M1_SPEED_DATA_Y+2);
			break;
			
		case 2 :
			LCD1602_SetMouse( M1_DERECTION_X, M1_DERECTION_Y);
			break;
			
		case 3:
			LCD1602_SetMouse( M2_ROUND_DATA_X, M2_ROUND_DATA_Y+1);
			break;
		case 4:
			LCD1602_SetMouse( M2_SPEED_DATA_X, M2_SPEED_DATA_Y+2);
			break;
		
		case 5:
			LCD1602_SetMouse( M2_DERECTION_X, M2_DERECTION_Y);
			break;
		
		default:
			current_point = 0;
			break;
	}
}


void lcd_init()
{
	int x,y;
	LCD1602_Init();
	
	for( x=0; x<2 ; x++){
		for( y=0; y< 16; y++)
			LCD1602_Write( x, y, lcd_data[x][y] );
	}
	
	if( g_config.config_avaliable ){
		lcd_update_m1_round();
		lcd_update_m1_speed();
		lcd_update_m2_round();
		lcd_update_m2_speed();
	}

	lcd_update_m2_direction();
	lcd_update_m1_direction();
	
	lcd_update_mouse();
}






void round_add_key_toggle_handler()
{
	switch( current_point ){
		case 0 :
			if( g_config.motor1_circle_count < LIMIT_MAX_ROUND_COUNT ){
				g_config.motor1_circle_count++;
				g_config.config_avaliable = 2;
			}
			lcd_update_m1_round();
			break;
			
		case 1:
			g_config.motor1_speed = g_config.motor1_speed+2 > LIMIT_MAX_SPEED ? LIMIT_MAX_SPEED: g_config.motor1_speed+2 ;
			g_config.config_avaliable = 2;
			lcd_update_m1_speed();
			break;
			
		case 2 :
			g_config.motor1_direction = g_config.motor1_direction == MOTOR_DIRECTION_LEFT ? MOTOR_DIRECTION_RIGHT:MOTOR_DIRECTION_LEFT;
			lcd_update_m1_direction();
			g_config.config_avaliable = 2;
			break;
			
		case 3:
			if( g_config.motor2_circle_count < LIMIT_MAX_ROUND_COUNT ){
				g_config.motor2_circle_count++;
				g_config.config_avaliable = 2;
			}
			lcd_update_m2_round();
			break;
		case 4:
			g_config.motor2_speed = g_config.motor2_speed+2 > LIMIT_MAX_SPEED ? LIMIT_MAX_SPEED: g_config.motor2_speed+2 ;
			g_config.config_avaliable = 2;
			lcd_update_m2_speed();
			break;
		
		case 5:
			g_config.motor2_direction = g_config.motor2_direction == MOTOR_DIRECTION_LEFT ? MOTOR_DIRECTION_RIGHT:MOTOR_DIRECTION_LEFT;
			lcd_update_m2_direction();
			g_config.config_avaliable = 2;
			break;
		
		default:
			current_point = 0;
			break;
	}
}

void round_reduce_key_toggle_handler()
{
	switch( current_point ){
		case 0 :
			if( g_config.motor1_circle_count > 0 ){
				g_config.motor1_circle_count--;
				g_config.config_avaliable = 2;
			}
			lcd_update_m1_round();
			break;
			
		case 1:
			g_config.motor1_speed = g_config.motor1_speed-2 > 0 ? g_config.motor1_speed-2 : 0 ;
			g_config.config_avaliable = 2;
			lcd_update_m1_speed();
			break;
			
		case 2 :
			g_config.motor1_direction = g_config.motor1_direction == MOTOR_DIRECTION_LEFT ? MOTOR_DIRECTION_RIGHT:MOTOR_DIRECTION_LEFT;
			lcd_update_m1_direction();
			g_config.config_avaliable = 2;
			break;
			
		case 3:
			if( g_config.motor2_circle_count > 0 ){
				g_config.motor2_circle_count--;
				g_config.config_avaliable = 2;
			}
			lcd_update_m2_round();
			break;
			
		case 4:
			g_config.motor2_speed = g_config.motor2_speed-2 > 0 ? g_config.motor2_speed-2 : 0 ;
			g_config.config_avaliable = 2;
			lcd_update_m2_speed();
			break;
		
		case 5:
			g_config.motor2_direction = g_config.motor2_direction == MOTOR_DIRECTION_LEFT ? MOTOR_DIRECTION_RIGHT:MOTOR_DIRECTION_LEFT;
			lcd_update_m2_direction();
			g_config.config_avaliable = 2;
			break;
		
		default:
			current_point = 0;
			break;
	}
}

void start_key_press_handler()
{
	if( g_config.config_avaliable == 2 )
	{
			config_save();
	}
	
	
	if( motor1.current_pulse >=  motor1.expect_pulse  && motor2.current_pulse >=  motor2.expect_pulse ){
		_LOG("start motor1 speed=%d, round=%d\n", g_config.motor1_speed , g_config.motor1_circle_count * ONE_ROUND_FREQ );
		motor_start( &motor1, g_config.motor1_speed * ONE_ROUND_FREQ /60 , g_config.motor1_circle_count * ONE_ROUND_FREQ , g_config.motor1_direction);
		_LOG("start motor2 speed=%d, round=%d\n", g_config.motor2_speed , g_config.motor2_circle_count * ONE_ROUND_FREQ );
		motor_start( &motor2, g_config.motor2_speed * ONE_ROUND_FREQ /60 , g_config.motor2_circle_count * ONE_ROUND_FREQ , g_config.motor2_direction);
	}else{ 
		_LOG("stop motor1\n");
		motor_stop( &motor1 );
		_LOG("stop motor2\n");
		motor_stop( &motor2 );
	}
	/*
	if( motor2.current_pulse >=  motor2.expect_pulse ){
		_LOG("start motor2 speed=%d, round=%d\n", g_config.motor2_speed , g_config.motor2_circle_count * ONE_ROUND_FREQ );
		motor_start( &motor2, g_config.motor2_speed * ONE_ROUND_FREQ /60 , g_config.motor2_circle_count * ONE_ROUND_FREQ );
	}else{ 
		_LOG("stop motor2\n");
		motor_stop( &motor2 );
	}
	*/
}

void switch_key_press_handler()
{
	current_point++;
	current_point %= 6;
	lcd_update_mouse();

}



#define SW_INTERVAL_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher round_add_key,round_reduce_key,start_key,switch_key;

void switch_init()
{
	switcher_init( &round_add_key, SW_INTERVAL_COUNT, 1 , 0 , GPIOA, GPIO_Pin_5, round_add_key_toggle_handler , NULL);
	switcher_init( &round_reduce_key, SW_INTERVAL_COUNT,1, 0 , GPIOA, GPIO_Pin_4 , round_reduce_key_toggle_handler, NULL);
	switcher_init( &start_key , SW_INTERVAL_COUNT,1, 0 , GPIOB, GPIO_Pin_0, start_key_press_handler ,NULL);
	switcher_init( &switch_key, SW_INTERVAL_COUNT,1, 0 , GPIOB, GPIO_Pin_1, switch_key_press_handler ,NULL);
	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &round_add_key  );
	switcher_interval_check( &round_reduce_key  );
	switcher_interval_check( &start_key );
	switcher_interval_check( &switch_key );
}























systick_time_t led_timer;

void heart_led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	systick_init_timer(&led_timer, 500);
	
}

void heart_led_even()
{
	static BitAction led = Bit_RESET;
	
	if( systick_check_timer( &led_timer ) ){
		GPIO_WriteBit( GPIOC, GPIO_Pin_13 , led);
		led = led==Bit_RESET ? Bit_SET: Bit_RESET ;
		
		//accel_check_even();
	}
}



void cmd_even()
{
	static char buffer[4]={0};
	static char index = 0;
	
	//if( Uart_Get( CONSOLE_UART , (unsigned char*) &buffer[index], 1 ) ){
	if( USB_RxRead((unsigned char*) &buffer[index], 1 ) ){
		if( buffer[index] == ';' ){
			if( index == 1 ){
				switch( buffer[index -1 ]){
					case '+':
						_LOG("add \n");
						round_add_key_toggle_handler();
					break;
					case '-':
						_LOG("reduce \n");
						round_reduce_key_toggle_handler();
					break;
					case 's':
						_LOG("start \n");
						start_key_press_handler();
					break;
					case 'w':
						_LOG("switch \n");
						switch_key_press_handler();
					break;
				}
			}
			index = 0;
			return;
		}
		index++;
		index = index % 4;
	}
}






void app_init()
{
	Uart_init();

#if BOARD_HAS_IAP
	if( IAP_PORT_USB == 1 ) iap_init_in_usb();
	else if( IAP_PORT_CAN1 == 1 )iap_init_in_can1();
	else if( IAP_PORT_UART == 1) iap_init_in_uart( IAP_UART );
#endif
	//console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	
	config_init();
	motor_init();
	lcd_init();
	switch_init();
	heart_led_init();
	
	ADC_Configuration ();
}


void app_event()
{
	motor_loop();
	switch_even();
	heart_led_even();
	cmd_even();
	config_auto_save_even();
}


#endif
