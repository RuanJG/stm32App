#include "stm32f10x_conf.h"
#include "../board/main_config.h"
#include "../board/bsp.h"
#include "stm32f10x.h"

#define ADC_SAMPLE_COUNT 34  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
#define ADC_SAMPLE_CHANNEL_COUNT 1
static unsigned short escADCConvert[ADC_SAMPLE_COUNT][ADC_SAMPLE_CHANNEL_COUNT];


void ADC_Configuration ()
{										 
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC_SAMPLE_CHANNEL_COUNT;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// 239.5 sampleTime => (239.5+12.5)/12M = 21us; 21*ADC_SAMPLE_COUNT * 3 = ADC_SAMPLE_COUNT * 63us = 1.890ms each group
	// 71.5 smapleTime => 7us; 7us*ADC_SAMPLE_COUNT*3 = ADC_SAMPLE_COUNT* 21us = 0.630 ms 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);//ADC_SampleTime_71Cycles5	
 
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
	
	

}

void DMA1_Channel1_IRQHandler(void)
{
 if(DMA_GetITStatus(DMA1_IT_TC1))
 {
	 DMA_ClearITPendingBit(DMA1_IT_GL1);
 }
}


unsigned short Cali_Adc_Value(int id)
{
	int i;
	u32 max=0,min=0,tmp;
	u32 sensor = 0; 
	
	max = escADCConvert[0][id];	
	min = escADCConvert[0][id];
	for(i=0;i<ADC_SAMPLE_COUNT;i++)
	{
		tmp = escADCConvert[i][id];
		if( tmp > max ) max = tmp;
		if( tmp < min ) min = tmp;
		sensor += tmp;
	}
	sensor -= (max+min);
	sensor = sensor/(ADC_SAMPLE_COUNT-2);
	//sensor = sensor>>5;
	return sensor;
}





int get_voice_db()
{
	unsigned short adc;
	int db;
	
	
	adc = Cali_Adc_Value(0);
	
	//db = vol/10 ==> (adc*4096/3300) / 10
	db = (adc*4096)/33000;
	
	return db;
	
}



#define LED_STATUS_ID 1
#define LED_ERROR_ID 2
#define LED_PASS_ID 3
#define LED_VOICE_ERROR_ID 4
#define LED_CURRENT_ERROR_ID 5

#define LED_ON_CMD 1
#define LED_OFF_CMD 2
#define LED_CHECK_CMD 3

int led_ctrl(int id, char cmd)
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	int onValue;

	switch( id ){
		case LED_STATUS_ID:
			gpio = GPIOB; pin = GPIO_Pin_3; onValue = 0;
			break;
		case LED_ERROR_ID:
			gpio = GPIOB; pin = GPIO_Pin_4; onValue = 0;
			break;
		case LED_PASS_ID:
			gpio = GPIOB; pin = GPIO_Pin_5; onValue = 0;
			break;
		case LED_VOICE_ERROR_ID:
			gpio = GPIOB; pin = GPIO_Pin_6; onValue = 0;
			break;
		case LED_CURRENT_ERROR_ID:
			gpio = GPIOB; pin = GPIO_Pin_7; onValue = 0;
			break;		
		
	}
	
	if( cmd == LED_ON_CMD ){
		if( onValue == 0 )
			GPIO_ResetBits( gpio, pin );
		else
			GPIO_SetBits( gpio, pin);
		return 0;
		
	}else if( cmd == LED_OFF_CMD ){
		if( onValue == 0 )
			GPIO_SetBits( gpio, pin);		
		else
			GPIO_ResetBits( gpio, pin );
		return 0;
		
	}else if( cmd == LED_CHECK_CMD ){
		return onValue == GPIO_ReadOutputDataBit( gpio, pin );
		
	}else{
		return -1;
	}
}

void led_on(int id)
{
	led_ctrl( id, LED_ON_CMD );
}

void led_off(int id)
{
	led_ctrl( id, LED_OFF_CMD );
}

int is_led_on(int id)
{
	return led_ctrl( id, LED_CHECK_CMD );	
}

void led_toggle(int id)
{
	if( is_led_on(id)){
		led_off(id);
	}else{
		led_on(id);
	}
}


void led_pin_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	led_off(LED_ERROR_ID);
	led_off(LED_PASS_ID);
	led_off(LED_VOICE_ERROR_ID);
	led_off(LED_CURRENT_ERROR_ID);
	led_on(LED_STATUS_ID);
	
}




static int sw_value;
static int sw_counter;
static int sw_on_tag;
#define SW_MAX_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
systick_time_t sw_timer;

void switch_det_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	sw_value = 0;
	sw_counter = 0;
	sw_on_tag = 0;
	
	systick_init_timer( &sw_timer, 10 );
	
}

void switch_det_event()
{
	
	if( 0 == systick_check_timer( &sw_timer ) )
		return;
	
	sw_value +=  GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_0 );
	sw_counter++;
	
	if( sw_counter >= SW_MAX_COUNT ){
		sw_counter = 0;
	
		if( sw_value == 0 ){
			sw_on_tag = 1;
		}else {
			sw_on_tag = 0;
		}
	}
	
}



void app_init()
{
	ADC_Configuration ();
	led_pin_config();
	switch_det_config();
	
}


void app_event()
{
	switch_det_event();
}

