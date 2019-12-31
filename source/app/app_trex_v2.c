#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#if BOARD_Trex_V2


#define MACHINE_OLD 0  // old machine switch define is difference
#define USE_SERVER2 1 //just start thought current detection instead of swtich

#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

//#define CONSOLE_UART &Uart3
#define PC_UART &Uart3
#define IAP_UART &Uart3
#define METER_UART &Uart1
#define CURRENT_UART &Uart1

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

	Uart_Configuration (&Uart1, USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
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
	
}










#define ADC_SAMPLE_COUNT 34  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
#define ADC_SAMPLE_CHANNEL_COUNT 1
volatile unsigned short escADCConvert[ADC_SAMPLE_COUNT][ADC_SAMPLE_CHANNEL_COUNT];
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


void userStation_log( char * str );

/*
*  get adc value
*		if ok , return db ( db > 0 )
*  	else  ,  return error ( -1 )
*/
int get_voice_db()
{
	unsigned short adc;
	int db;
	char buffer[32]={0};
	
	if( adc_updated == 0 )
		return -1;
	
	adc = Cali_Adc_Value(0);
	adc_updated = 0;
	
	//_LOG("adc=%d\n",adc);
	//db = vol/10 ==> (adc*3300/4096) / 10
	//db = (adc*2933)/40960;
	//db = (adc*3344)/40960;
	
	//db = (( adc*3300/4096 ) / 3162 ) *94; // 94: 1Pa=94db ; 3162: 3162mV/Pa 
	//db = adc*3300*94*2/4096/3801; // adc*0.02395
#if MACHINE_OLD
	db = adc*0.080664; // vol/10 ==> (adc*3304/4096) / 10
#else
	db = adc*0.039848;  //db = (( adc*3300/4096 ) / 3162 ) *94; // 94: 1Pa=94db ; 3162: 3162mV/Pa 
#endif
	//_LOG("db=%d",db);
	
	//db = (adc*3300)/4096;
	//sprintf(buffer,"db=%d",db);
	//userStation_log( buffer );

	//return db;
	return adc;
	
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
			gpio = GPIOB; pin = GPIO_Pin_8; onValue = 0;
			break;
		case LED_ERROR_ID:
			gpio = GPIOB; pin = GPIO_Pin_9; onValue = 0;
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_8 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	led_off(LED_ERROR_ID);
	led_off(LED_PASS_ID);
	led_off(LED_VOICE_ERROR_ID);
	led_off(LED_CURRENT_ERROR_ID);
	led_off(LED_STATUS_ID);
	
}













volatile int sw_value;
volatile int sw_counter;
volatile int sw_on_tag;
#define SW_MAX_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
systick_time_t sw_timer;

void switch_det_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;GPIO_Mode_AIN
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	sw_value = 0;
	sw_counter = 0;
	sw_on_tag = 0;
	
	systick_init_timer( &sw_timer, 10 );
	
}

void switch_det_event()
{
	
	if( 0 == systick_check_timer( &sw_timer ) )
		return;
	
	sw_value +=  GPIO_ReadInputDataBit( GPIOA, GPIO_Pin_5 );
	sw_counter++;
	
	if( sw_counter >= SW_MAX_COUNT ){
		//_LOG("sw=%d\n",sw_value);
#if MACHINE_OLD
		if( sw_value >= SW_MAX_COUNT ){
#else
		if( sw_value == 0 ){
#endif
			sw_on_tag = 1;
		}else {
			sw_on_tag = 0;
		}
		sw_value = 0;
		sw_counter = 0;
	}
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
	float current_max;
	float current_min;
	int db_max;
	int db_min;
	int machine_no;
	float db_factor;
};
volatile struct config g_config;


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
		g_config.current_max = 0.5 ; //A
		g_config.current_min = 0.35 ; //A
		g_config.db_max = 65;
		g_config.db_min = 30;
		g_config.machine_no = 'X';
		g_config.db_factor = 0.080664;
		config_save();
	}
}











// packget  head tag 
#define USER_DATA_TAG	1
#define USER_LOG_TAG	2
#define USER_START_TAG	3
#define USER_CONFIG_TAG	4
#define USER_CMD_CHMOD_TAG 4
#define USER_CMD_CURRENT_MAXMIN_TAG 5
#define USER_CMD_VOICE_MAXMIN_TAG 6
#define USER_CMD_GET_MAXMIN_TAG 7
#define USER_ACK_TAG 8
#define USER_CMD_SET_SW_STATUS_TAG 9
#define USER_CMD_SET_MACHINE_NO 10 

//packget body : result error value
#define USER_RES_CURRENT_FALSE_FLAG 1
#define USER_RES_VOICE_FALSE_FLAG 2
#define USER_RES_ERROR_FLAG 4

//work_mode value
#define USER_MODE_ONLINE 1
#define USER_MODE_OFFLINE 2
#define USER_MODE_SLAVER 3 


Uart_t *userUart=NULL;
protocol_t encoder;
protocol_t decoder;
volatile unsigned char userStation_mode;


#define ENABLE_REPORT_RESEND 0
#define REPORT_MAX_DATA_COUNT 100
volatile unsigned char report_buffer[REPORT_MAX_DATA_COUNT][15];
volatile unsigned int report_buffer_index;
volatile unsigned int report_buffer_sent_index;
volatile char report_listen_tag;
systick_time_t report_listen_timer;

volatile unsigned int report_ms;



void userStation_init()
{
	userUart = PC_UART;
	userStation_mode = USER_MODE_ONLINE;
	protocol_init( &encoder );
	protocol_init( &decoder );
	
	report_listen_tag = 0;
}

int userStation_send( unsigned char *data , int len)
{
	if( 1 == protocol_encode( &encoder, data, len ) ){
		Uart_Put_Sync( userUart , encoder.data, encoder.len );
		//USB_TxWrite( encoder.data, encoder.len );
		return 1;
	}else{
		return 0;
	}
}


unsigned char buffer[ 256 ];
void userStation_log( char * str ){
	
	if( strlen( str ) > sizeof(buffer) )
		return;
	
	buffer[0] = USER_LOG_TAG;
	memcpy( &buffer[1], str , strlen(str) );
	
	if( 0 == userStation_send( buffer, strlen(str)+1 ) ){
		_LOG("userStation_log: encode false");
	}
	
}



void userStation_report_resend_start()
{
	report_listen_tag = 1;
	systick_init_timer( &report_listen_timer , 500);
}


void userStation_reset_report()
{
	report_ms = systick_get_ms();
	report_buffer_index = 0;
	report_buffer_sent_index = 0;
	report_listen_tag = 0;
}

unsigned int userStation_get_report_ms()
{
	unsigned int ms;
	
	ms = systick_get_ms() - report_ms ;
	return ms;
}

void userStation_report(int db, float current, int switch_state)
{
	//tag[0]+db[0]db[1]db[2]db[3]+current[....]+work_current[0]+error[0]  ; work_counter is the count of the records
	
	unsigned int ms;
	
	if( report_buffer_index >= REPORT_MAX_DATA_COUNT ){ 
		if( switch_state == 0 )
		{
			report_buffer_index = REPORT_MAX_DATA_COUNT-1;
			if( report_buffer_sent_index >= report_buffer_index )
				report_buffer_sent_index = report_buffer_index;
		}else{
			return ;
		}
	}
	
	ms = userStation_get_report_ms();
	
	report_buffer[report_buffer_index][0]= USER_DATA_TAG;
	memcpy( &report_buffer[report_buffer_index][1], (unsigned char*) &ms, 4 ) ;
	memcpy( &report_buffer[report_buffer_index][5], (unsigned char*) &db, 4 ) ;
	memcpy( &report_buffer[report_buffer_index][9], (unsigned char*) &current , 4 ); 
	report_buffer[report_buffer_index][13]= report_buffer_index;
	report_buffer[report_buffer_index][14] = switch_state;
	
	//_LOG("report0: db=%d, current=%f , count=%d, sw=%d\n", db, current, report_buffer_index ,switch_state);
	//_LOG("report1: db=%d, current=%f , count=%d, sw=%d\n", db, current, report_buffer[report_buffer_index][13] ,report_buffer[report_buffer_index][14]);
	
	
	report_buffer_index++;
	
	
}


void userStation_report_send_event()
{
	
	if( report_listen_tag == 1 )
	{
		//in resend loop
		if( report_buffer_index > report_buffer_sent_index && 1 == systick_check_timer( &report_listen_timer ) )
		{
			_LOG("userStation_report_send_event: timeout->resend\n");
			userStation_log("userStation_report_send_event: timeout->resend \n");
			if( 0 == userStation_send( &report_buffer[report_buffer_sent_index][0], 15 ) ){
				_LOG("userStation_report_send_event: send false\n");
				userStation_log("userStation_report_send_event: send false\n");
			}
		}
		
	}else{
		
		if( report_buffer_index > report_buffer_sent_index ) 
		{
			
			if( 0 == userStation_send( &report_buffer[report_buffer_sent_index][0], 15 ) ){
				_LOG("userStation_report_send_event: send false\n");
				userStation_log("userStation_report_send_event: send false\n");
			}
			userStation_report_resend_start();
			//_LOG("send: index=%d, sw=%d\n", report_buffer[report_buffer_sent_index][13] ,report_buffer[report_buffer_sent_index][14]);
			
			report_buffer_sent_index++;
		}
		
	}
}



void userStation_send_config()
{
	unsigned char buffer[23];
	
	buffer[0]= USER_CONFIG_TAG;
	buffer[1] = g_config.config_avaliable; // 1 available
	memcpy( &buffer[2], (unsigned char*) &g_config.current_max, 4 ) ;
	memcpy( &buffer[6], (unsigned char*) &g_config.current_min , 4 ); 
	memcpy( &buffer[10], (unsigned char*) &g_config.db_max, 4 ) ;
	memcpy( &buffer[14], (unsigned char*) &g_config.db_min , 4 ); 
	buffer[18] = g_config.machine_no;
	memcpy( &buffer[19], (unsigned char*) &g_config.db_factor , 4 ); 
	
	if( 0 == userStation_send( buffer, sizeof(buffer) ) ){
		_LOG("send config false\n");
		userStation_log("send config false\n");
	}
}



void userStation_handleMsg( unsigned char *data, int len)
{
	int dbmin,dbmax;
	float cmax,cmin,dbfactor;
	unsigned char buffer[32]={0};
	
	switch( data[0] ){
		case USER_CMD_CHMOD_TAG:
			if( len == 2 ){
				userStation_mode = data[1];
			}
			break;
			
		case USER_CMD_CURRENT_MAXMIN_TAG:
			if( len == 9 ){
				memcpy( (char*)&cmax, data+1, 4);
				memcpy( (char*)&cmin, data+5, 4);
				g_config.current_max = cmax;
				g_config.current_min = cmin;
				config_save();
				userStation_send_config();
				sprintf((char*)buffer,"current=[%fA,%fA]",g_config.current_min, g_config.current_max);
				userStation_log( (char*) buffer );
			}
			break;
		
		case USER_CMD_VOICE_MAXMIN_TAG:
			if( len == 13 ){
				memcpy( (char*)&dbmax, data+1, 4);
				memcpy( (char*)&dbmin, data+5, 4);
				memcpy( (char*)&dbfactor, data+9, 4);
				g_config.db_max = dbmax;
				g_config.db_min = dbmin;
				g_config.db_factor = dbfactor;
				config_save();
				userStation_send_config();
				sprintf((char*)buffer,"db=[%d,%d]",g_config.db_min, g_config.db_max);
				userStation_log( (char*) buffer );
			}
			break;
			
		case USER_CMD_GET_MAXMIN_TAG:
			userStation_send_config();
			break;
		
		case USER_ACK_TAG:
			if( report_listen_tag == 1 )
				report_listen_tag = 0;
			break;
			
		case USER_CMD_SET_SW_STATUS_TAG:
			if( len == 2 ){
				if( userStation_mode == USER_MODE_SLAVER ){
					sw_on_tag = data[1];
				}
			}
			break;
			
		case USER_CMD_SET_MACHINE_NO:
			if( len == 2 ){
				g_config.machine_no = data[1];
				_LOG("set Machine No =%c\n",data[1]);
				config_save();
				userStation_send_config();
			}
			break;
		
		default:
			break;
	}
}


void userStation_listen_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = Uart_Get( userUart, buffer, 8 );
	//len = USB_RxRead ( buffer, 8 );
	for( i=0 ; i<len; i++ ){
		if( 1 == protocol_parse( &decoder, buffer[i] ) ){
			userStation_handleMsg( decoder.data, decoder.len );
		}
	}
	
	userStation_report_send_event();
}





#define USING_MINIMETER 1
#define USING_VICTORMETER 0



#if USING_MINIMETER

/*
出厂时仪表波特率是9600,可以自行设置
每台设备出厂时通迅地址都是0x00

A:单次读取设备数据(设备接收到此命令后,向主机发送1次数据后回到等待状态)
0x88+0xAE+设备地址+0x11  实例88AE0011

B:连续读取设备数据(设备接收到此命令后,将以每秒6组的速度向主机连续发送数据)
0x88+0xAE+设备地址+0x21

C:停止设备输出数据,此命令不管当前工作的是哪一台设备,都将停止输出
0x88+0xAE+任意字节+0x01

D:设备返回给主机(PC)数据格式
0xFA+0xFB+设备地址+数据字节0+数据字节1+数据字节2+数据字节3+校验字节
数据由4个字节组成(注意:靠左的是低字节和习惯上是反的),接收后需要转换成浮点数才能对应设备显示数据.校验字节是4个数据字节相加的结果(进位舍去)
*/
typedef struct miniMeterCoder {
	unsigned char packet[8];
	unsigned char index;
	unsigned char addr;
	float 				value;
}miniMeterCoder_t;

void miniMeterCoder_init(miniMeterCoder_t * coder)
{
	int i;
	for( i=0; i< sizeof(coder->packet); i++)
		coder->packet[i]=0;
	coder->index = 0;
	coder->addr = 0;
}

int miniMeterCoder_prase( miniMeterCoder_t * coder, unsigned char data )
{
	unsigned char crc;
	unsigned char *value ;
	
	if( coder->index == 0  && data != 0xFA ) return 0;
	if( coder->index == 1 && data != 0xFB ) { coder->index = 0; return 0; }
	if( coder->index == 2  && data != coder->addr ) { coder->index = 0; return 0; }
	
	coder->packet[ coder->index++ ] = data;
	
	if( coder->index < sizeof(coder->packet) ) return 0;
	
	//check crc
	coder->index = 0;
	crc = coder->packet[3]+coder->packet[4]+coder->packet[5]+coder->packet[6];
	if( crc == coder->packet[7] ) {
		//coder->value = *((float*) &coder->packet[3] );
		value = (unsigned char *)&coder->value;
		value[0] = coder->packet[6];
		value[1] = coder->packet[5];
		value[2] = coder->packet[4];
		value[3] = coder->packet[3];
		return 1;
	}
	
	return -1;
}



miniMeterCoder_t currentMeter;
volatile int meter_started = 0;

void miniMeter_init()
{
	miniMeterCoder_init( &currentMeter );
}


void miniMeter_clear()
{
	
	Uart_Clear_Rx(METER_UART);
	
}


void miniMeter_toggle()
{
	unsigned char packget[]={0x88,0xAE,0x00,0x11};
	
	Uart_Put(METER_UART, packget, sizeof( packget) );
}




void miniMeter_test_even()
{
	unsigned char packget[8];
	int i,count;
	char buffer[16] = {0};

	count = Uart_Get( METER_UART , packget, sizeof( packget ) );
	if( count == 0 ) return;
	
	for( i=0; i< count ; i++){
		if( 1 == miniMeterCoder_prase( &currentMeter , packget[i] ) ){
			sprintf(buffer,"%3.5fV",currentMeter.value);
			userStation_log(buffer);
			//_LOG( "%.5fV" , currentMeter.value);
		}
	}
}




int service_getCurrent( float *A)
{
	unsigned char data;
	int i,count,res;
	
	res = 0;

	for( i=0,count=10; i< count ; i++)
 {
		if( 1 == Uart_Get( METER_UART , &data, 1 ) ) 
		{
			res = miniMeterCoder_prase( &currentMeter , data );
			if( 1 == res )
		  {
				*A = currentMeter.value;
				break;
			}else if( -1 == res ){
				//error
				break;
			}else{
				//continue
			}
		}else{
			// no uart data , break;
			res = 0;
			break;
		}
	}
	return res;
}



int miniMeter_start()
{
	unsigned char packget[]={0x88,0xAE,0x00,0x21};
	float A;
	int retry = 5;
	
	
	if ( meter_started == 1 ) return 1;
	miniMeter_clear();
	
	while( 0 < retry-- ){
		Uart_Put(METER_UART, packget, sizeof( packget) );
		systick_delay_ms(300);
		if( 1 == service_getCurrent( &A) ){
			meter_started = 1;
			return 1;
		}
	}
	
	return 0;
}

void miniMeter_stop()
{
	unsigned char packget[]={0x88,0xAE,0x00,0x01};

	//return ;
	
	if ( meter_started == 0 ) return;
	meter_started = 0;
	//Uart_Put(METER_UART, packget, sizeof( packget) );
}


#endif




#if USING_VICTORMETER

Uart_t *victor8165Uart;

#define VICTOR8165_BUFFSIZE 32
volatile unsigned char victor8165_buf[VICTOR8165_BUFFSIZE];
volatile unsigned char victor8165_inited = 0;
systick_time_t victor8165_cmd_timer;

int victor8165_cmd( char * cmd  )
{
	int i,count,len;
	
	while( systick_check_timer( &victor8165_cmd_timer ) == 0 ){
		systick_delay_us(500);
	}
	Uart_Clear_Rx( victor8165Uart );
	//while( 1 == Uart_Get( victor8165Uart, &victor8165_buf[0] , 1) );
	
	i = strlen(cmd);
	Uart_Put_Sync( victor8165Uart, (unsigned char *)cmd, i );
	//_LOG("cmd:%s",cmd);
	
	//get result , 1000ms timeout;
	count = 0;
	len = 0;
	for( i=0; i< 500 ; i++ ){
		count = Uart_Get( victor8165Uart, &victor8165_buf[len] , VICTOR8165_BUFFSIZE - len -1);
		len += count;
		if( len > 0 && victor8165_buf[len-1]==0x0A ){
			//systick_delay_us(50000);
			victor8165_buf[len]= 0;
			systick_init_timer( &victor8165_cmd_timer , 10); // next cmd need to wait 50ms
			return len;
		}
		if( count > 0 ) i--;
		systick_delay_us( 1000 );
	}
	
	//Uart_Put( victor8165Uart, (unsigned char *)"\n", 1 );
	//systick_delay_us( 100000 );
	_LOG("cmd:%s  timeout\n",cmd);
	systick_init_timer( &victor8165_cmd_timer , 100); // next cmd need to wait 100ms
	
	return 0;
}

int victor8165_check(const char* cmd, const char *result)
{
	int len;
	int i;
	
	len = victor8165_cmd( (char*) cmd);
	if( len == strlen(result) && 0 == strncmp(result,(char*) victor8165_buf , len) ){
		return 1;
	}
	
	/*
	for( i=0; i< len; i++ ){
		
		if( result[i] != victor8165_buf[i] ){
			userStation_log( "x,");
			return 0;
		}else{
			userStation_log( "1,");
		}
	}
	
	return 1;
	*/
	
	userStation_log( "check :\n");
	userStation_log((char*)cmd);
	userStation_log((char*)victor8165_buf);
	return 0;
}


void victor8165_init()
{
	int retry,res;
	
	victor8165Uart = CURRENT_UART;
	systick_init_timer( &victor8165_cmd_timer , 10);
	
	//set DCI mode
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "ADC\n");
		if( 1 == victor8165_check( "FUNC1?\n", "ADC\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set DCI mode ok\n");
		userStation_log("init: DCI ok");
	}else{
		_LOG("victor8165_init: set DCI mode false\n");
		userStation_log("init: DCI false");
		return;
	}
	
	//set Rate
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "RATE M\n");
		if( 1 == victor8165_check( "RATE?\n", "M\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set Rate M mode ok\n");
		userStation_log("init: Rate ok");
	}else{
		_LOG("victor8165_init: set Rate M mode false\n");
		userStation_log("init: Rate false");
		return;
	}
	
	//set range AUTO
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "RANGE 5\n");
		if( 1 == victor8165_check( "RANGE1?\n", "6\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set AUTO mode ok\n");
		userStation_log("init: Auto ok");
	}else{
		_LOG("victor8165_init: set AUTO mode false\n");
		userStation_log("init: Auto false");
		return;
	}
	
	victor8165_inited = 1;
	
}


int service_getCurrent( float *A)
{
	int retry, len;
	
	//victor8165_check( "FUNC1?\n", "ADC\n");
	//if( victor8165_inited == 0 ||  0 == victor8165_check( "FUNC1?\n", "ADC\n") ){
	if( victor8165_inited == 0 ){
		_LOG("victor8165_getCurrent: need to init\n");
		victor8165_inited = 0;
		victor8165_init();
		return 0;
	}
	
	
	for( retry = 0; retry < 2; retry++ ){
		len = victor8165_cmd( "MEAS1?\n");
		if( len > 0 ) 
			break;
	}
	
	if( len == 0 ){
		return 0;
	}
	
	victor8165_buf[len] = 0; // set string end tag
	sscanf((const char*)victor8165_buf , "%e;\n",  A);
	//*mA = A*1000;
	
	return 1;
}


#endif





systick_time_t led_timer;
systick_time_t collect_timer;
systick_time_t work_timer;
volatile unsigned char server_status; // 0: idel ,  1:cail

#define START_BY_DETECT_METER 1

void server_init()
{
	server_status =0;
	#if USING_VICTORMETER 
	victor8165_init();
	#endif
	#if USING_MINIMETER
	miniMeter_init();
	#endif
}

void server_start()
{
	unsigned char tag;
	
	userStation_reset_report();
	
	//delay for start read data
	systick_init_timer( &work_timer, 2500);


	tag = USER_START_TAG;	
	userStation_send( &tag , 1 );
	userStation_send_config();
	
	
	#if USING_VICTORMETER 
	if( 0 == victor8165_check( "FUNC1?\n", "ADC\n") ){
		_LOG("victor8165_getCurrent: need to init\n");
		victor8165_inited = 0;
		victor8165_init();
	}
	#endif
}

void server_stop()
{
	int i,error;
	float current;
	int db;
	
	
	#if USING_MINIMETER
	miniMeter_stop();
	#endif
	
	userStation_report( 0 , 0 ,0 );
	
}

void server_runtime()
{	
	int res;
	volatile int db;
	float current;
	
	if( 0 == systick_check_timer( &work_timer ) )
		return ;
	
	
	#if USING_MINIMETER
	if( 0 == miniMeter_start() ){
		_LOG(" miniMeter start error \n");
		userStation_log(" miniMeter start error \n");
	}
	#endif
	


	//get current by uart
	res = service_getCurrent( &current);
	if( -1 == res ){
		userStation_log("read current false");
		_LOG("read current false\n");
		
	}else if( 1 == res ){
		
		while( 1 ){
			db  = get_voice_db();
			if( db > 0 ) break;
		}
		userStation_report( db , current , sw_on_tag);
		
	}
	
	//will run this func after 10ms
	systick_init_timer( &work_timer, 5);
}

void server_event()
{
	if( sw_on_tag == 1 ){
		
		if( server_status == 0 ){
			server_start();
			server_status = 1;
		}
		if( server_status == 1 )
			server_runtime();
		
	}else{
		
		if( server_status == 1 ){
			server_stop();
			server_status = 0;
		}else if( server_status == 2){
			// has call server_stop , as collection has finish earlier
			server_status = 0;
		}else{
			server_status = 0;
		}
		
	}	
}



volatile int server2_start_tag;
float server2_current;
volatile int server2_db;

void server2_init()
{
	server2_start_tag =0;
	
	#if USING_VICTORMETER 
	victor8165_init();
	#endif
	#if USING_MINIMETER
	miniMeter_init();
	#endif
	
}

void server2_event()
{
	int res;
	unsigned char tag;
	float current;
	
	#if USING_MINIMETER
	if( 0 == miniMeter_start() ){
		_LOG(" miniMeter start error \n");
		userStation_log(" miniMeter start error \n");
	}
	#endif
	
	res = service_getCurrent( &current);
	if( -1 == res ){
		userStation_log("read current false");
		_LOG("read current false\n");
	}else if( 1 == res ){
		//get a new data
		if( current >= 0.1 )
		{
			//motor is running
			server2_current = current;
			while( 1 ){
				server2_db  = get_voice_db();
				if( server2_db > 0 ) break;
			}
			if( server2_start_tag == 0 ){
				//start reporting
				server2_start_tag = 1;
				userStation_reset_report();
				tag = USER_START_TAG;	
				userStation_send( &tag , 1 );
				userStation_send_config();
				userStation_log("report start ...");
				_LOG("read current false\n");
			}
			userStation_report( server2_db , server2_current , 1);

		}else{
			// waiting or stop
			if( server2_start_tag == 1)
			{
				server2_start_tag = 0;
				userStation_report( server2_db , server2_current , 0);
				userStation_log("report end ...");
			}
		}
		
	}else{
		// no data
	}
}



systick_time_t heart_led_timer;

void heart_led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	systick_init_timer(&heart_led_timer, 1000);
	
}

void heart_led_even()
{
	static BitAction led = Bit_RESET;
	//static unsigned char tag = 0;
	//char buffer[32];
	
	if( systick_check_timer( &heart_led_timer ) ){
		GPIO_WriteBit( GPIOC, GPIO_Pin_13 , led);
		led = led==Bit_RESET ? Bit_SET: Bit_RESET ;
		//sprintf(buffer,"%d=sw:%d,ss:%d",tag++,sw_on_tag,server_status);
		//userStation_log(buffer);
		//tag %= 10;
	}		
}




systick_time_t user_station_loop_test_timer;

void user_station_loop_test_init()
{
	systick_init_timer(&user_station_loop_test_timer, 200);
	
}
volatile int testcount=0;

void user_station_loop_test_even()
{
	unsigned char tag;

	if( userStation_mode == USER_MODE_OFFLINE && systick_check_timer( &user_station_loop_test_timer ) ){
		if( testcount >= 50 ){
			userStation_report( 40, 0.444444 ,0 ); 
			testcount = 0;
		}else if( testcount == 0 ){
			//tag = USER_START_TAG;	
			//userStation_send( &tag , 1 );
			systick_delay_us(500000);
			userStation_reset_report();
			testcount ++;	
		}else{
			userStation_report( 40, 0.444444 ,1 );
			testcount ++;			
		}
	}		
}


void cmd_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = USB_RxRead( buffer, 8 );
	
	if( len == 0 ) return;
		if( buffer[0] == '1' ){
			//sw_on_tag = 1;
			//userStation_report( 40, 0.44445 , 0, 0 );
		}else if( buffer[0] == '0' ){
			//sw_on_tag = 0;
			//userStation_report( 40, 0.444444 , 50, 0 ); 
		}
}




void app_init()
{
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
	led_pin_config();
	heart_led_init();
	switch_det_config();
	userStation_init();
#if USE_SERVER2
	server2_init();
#else
	server_init();
#endif
	config_init();
	user_station_loop_test_init();
	//systick_delay_us(1000000);
	//miniMeter_start();

}





void app_event()
{
	#if 1
	if( userStation_mode == USER_MODE_ONLINE ){
		switch_det_event();
	}
	
	if( userStation_mode != USER_MODE_OFFLINE ){
		#if USE_SERVER2
		server2_event();
		#else
		server_event();
		#endif
	}
	
	userStation_listen_even();
	heart_led_even();
	
	//miniMeter_test_even();
	user_station_loop_test_even();
	//cmd_even();
	#endif
}

#endif