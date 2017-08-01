#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"

#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

#define CURRENT_UART &Uart3
#define PC_UART &Uart2
#define CONSOLE_UART &Uart1
#define IAP_UART &Uart2


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

#if 0
	//CAN
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
									
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	Can1_Configuration_mask(0, CAN1_ID, CAN_ID_STD, 0x1ff , CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_5tq, 4);
#endif
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);



	Uart_config_console( CONSOLE_UART );

#if BOARD_HAS_IAP
	if( 1 == IAP_PORT_UART)
		iap_init_in_uart( IAP_UART );

	if ( 1== IAP_PORT_CAN1 )
		iap_init_in_can1();
#endif

	
}










#define ADC_SAMPLE_COUNT 34  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
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




/*
*  get adc value
*		if ok , return db ( db > 0 )
*  	else  ,  return error ( -1 )
*/
int get_voice_db()
{
	unsigned short adc;
	int db;
	
	if( adc_updated == 0 )
		return -1;
	
	adc = Cali_Adc_Value(0);
	adc_updated = 0;
	
	//_LOG("adc=%d\n",adc);
	//db = vol/10 ==> (adc*3300/4096) / 10
	//db = (adc*2933)/40960;
	db = (adc*3344)/40960;
	
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
	led_off(LED_STATUS_ID);
	
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
		//_LOG("sw=%d\n",sw_value);
		if( sw_value >= SW_MAX_COUNT ){
			sw_on_tag = 1;
		}else {
			sw_on_tag = 0;
		}
		sw_value = 0;
		sw_counter = 0;
	}
}



// packget  head tag 
#define USER_DATA_TAG	1
#define USER_LOG_TAG	2
#define USER_START_TAG	3
#define USER_CMD_CHMOD_TAG 4
#define USER_CMD_CURRENT_MAXMIN_TAG 5
#define USER_CMD_VOICE_MAXMIN_TAG 6

//packget body : result error value
#define USER_RES_CURRENT_FALSE_FLAG 1
#define USER_RES_VOICE_FALSE_FLAG 2
#define USER_RES_ERROR_FLAG 4

//work_mode value
#define USER_MODE_ONLINE 1
#define USER_MODE_OFFLINE 2



Uart_t *userUart=NULL;
protocol_t encoder;
protocol_t decoder;
volatile unsigned char userStation_mode;
float MAX_ALARM_CURRENT_VALUE = 0.5; //A
float MIN_ALARM_CURRENT_VALUE = 0.4; //A
int MAX_ALARM_VOICE_VALUE = 65 ;  //db
int MIN_ALARM_VOICE_VALUE = 20 ;  //db

void userStation_init()
{
	userUart = PC_UART;
	userStation_mode = USER_MODE_ONLINE;
	protocol_init( &encoder );
	protocol_init( &decoder );
	
}

int userStation_send( unsigned char *data , int len)
{
	if( 1 == protocol_encode( &encoder, data, len ) ){
		Uart_Put_Sync( userUart , encoder.data, encoder.len );
		return 1;
	}else{
		return 0;
	}
}



void userStation_report(int db, float current, int work_counter, int error)
{
	//tag[0]+db[0]db[1]db[2]db[3]+current[....]+work_current[0]+error[0]  ; work_counter is the count of the records
	
	unsigned char buffer[11];
	
	buffer[0]= USER_DATA_TAG;
	memcpy( &buffer[1], (unsigned char*) &db, 4 ) ;
	memcpy( &buffer[5], (unsigned char*) &current , 4 ); 
	buffer[9]= work_counter;
	buffer[10] = error;
	

	if( 0 == userStation_send( buffer, sizeof(buffer) ) ){
		_LOG("userStation_report: report false\n");
	}
	
	_LOG("report: db=%d, current=%f , count=%d, error=%d\n", db, current, work_counter ,error);
}

void userStation_log( char * str ){
	unsigned char buffer[ 32 ];
	
	if( strlen( str ) > 30 )
		return;
	
	buffer[0] = USER_LOG_TAG;
	memcpy( &buffer[1], str , strlen(str) );
	
	if( 0 == userStation_send( buffer, strlen(str)+1 ) ){
		_LOG("userStation_log: encode false");
	}
	
}


void userStation_handleMsg( unsigned char *data, int len)
{
	int dbmin,dbmax;
	float cmax,cmin;
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
				MAX_ALARM_CURRENT_VALUE = cmax;
				MIN_ALARM_CURRENT_VALUE = cmin;
				_LOG("set current max = %f\n", cmax);
				sprintf((char*)buffer,"current=[%f,%f]",cmin, cmax);
				userStation_log( (char*) buffer );
			}
			break;
		
		case USER_CMD_VOICE_MAXMIN_TAG:
			if( len == 9 ){
				memcpy( (char*)&dbmax, data+1, 4);
				memcpy( (char*)&dbmin, data+5, 4);
				MAX_ALARM_VOICE_VALUE = dbmax;
				MIN_ALARM_VOICE_VALUE = dbmin;
				_LOG("set db max = %d\n", dbmax);
				sprintf((char*)buffer,"db=[%d,%d]",dbmin, dbmax);
				userStation_log( (char*) buffer );
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
	for( i=0 ; i<len; i++ ){
		if( 1 == protocol_parse( &decoder, buffer[i] ) ){
			userStation_handleMsg( decoder.data, decoder.len );
		}
	}
	
}














Uart_t *victor8165Uart;

#define VICTOR8165_BUFFSIZE 32
unsigned char victor8165_buf[VICTOR8165_BUFFSIZE];
unsigned char victor8165_inited = 0;
systick_time_t victor8165_cmd_timer;

int victor8165_cmd( char * cmd  )
{
	int i,count,len;
	
	while( systick_check_timer( &victor8165_cmd_timer ) == 0 ){
		systick_delay_us(500);
	}
	Uart_Clear_Rx( victor8165Uart );
	
	i = strlen(cmd);
	Uart_Put_Sync( victor8165Uart, (unsigned char *)cmd, i );
	//_LOG("cmd:%s",cmd);
	
	//get result , 1000ms timeout;
	count = 0;
	len = 0;
	for( i=0; i< 500 ; i++ ){
		count = Uart_Get( victor8165Uart, &victor8165_buf[len] , VICTOR8165_BUFFSIZE - len);
		len += count;
		if( len > 0 && victor8165_buf[len-1]==0x0A ){
			//systick_delay_us(50000);
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
	
	len = victor8165_cmd( (char*) cmd);
	if( len == strlen(result) && 0 == strncmp(result,(const char*) victor8165_buf, len ) ){
		return 1;
	}
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


int victor8165_getCurrent( float *A)
{
	int retry, len;
	
	
	if( victor8165_inited == 0 ||  0 == victor8165_check( "FUNC1?\n", "ADC\n") ){
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













#define CALI_DATA_COUNT 10
systick_time_t led_timer;
systick_time_t collect_1s_timer;
systick_time_t work_timer;
static int work_counter;
static int db_array[CALI_DATA_COUNT];
static float current_array[CALI_DATA_COUNT];
static unsigned char server_status; // 0: idel ,  1:cail
static unsigned char server_collect_over;
void server_init()
{
	server_status =0;
	work_counter = 0;
	server_collect_over = 0;
	victor8165_init();
}

void server_start()
{
	unsigned char tag;
	int delay_ms = 3000;
	
	//delay 10000ms for start read data
	systick_init_timer( &work_timer, delay_ms);
	//read current and db in 1s
	systick_init_timer( &collect_1s_timer, 7000+delay_ms );
	work_counter = 0;
	server_collect_over = 0;
	
	tag = USER_START_TAG;	
	userStation_send( &tag , 1 );
	
	//set led status
	led_on(LED_STATUS_ID);
	led_off( LED_ERROR_ID );
	led_off(LED_PASS_ID);
	led_off(LED_VOICE_ERROR_ID);
	led_off(LED_CURRENT_ERROR_ID);
}

void server_stop()
{
	int i,error;
	float current;
	int db;
	
	if( work_counter > 0 ){
		db = 0;
		for( i=0; i< work_counter; i++){
			current+=current_array[i];
			db+=db_array[i];
		}
		db = db/work_counter;
		current = current/work_counter;
		
	}
	
	
	//TODO set led status
	if( work_counter> 0 &&  current < MAX_ALARM_CURRENT_VALUE  && current > MIN_ALARM_CURRENT_VALUE && db < MAX_ALARM_VOICE_VALUE){
		led_on(LED_PASS_ID);
		led_off(LED_VOICE_ERROR_ID);
		led_off(LED_CURRENT_ERROR_ID);
		led_off(LED_ERROR_ID);
		userStation_report( db, current , work_counter, 0);
	}else {
		led_off(LED_PASS_ID);
		if( work_counter > 0 ){
			error = 0;
			if( current >= MAX_ALARM_CURRENT_VALUE || current <= MIN_ALARM_CURRENT_VALUE ){
				error |= USER_RES_CURRENT_FALSE_FLAG;
				led_on(LED_CURRENT_ERROR_ID);
			}
			if( db >= MAX_ALARM_VOICE_VALUE ){
				error |= USER_RES_VOICE_FALSE_FLAG;
				led_on(LED_VOICE_ERROR_ID);
			}
			userStation_report( db, current , work_counter, error);
		}else{
			led_on(LED_ERROR_ID);
			userStation_report( 0, 0 , work_counter, USER_RES_ERROR_FLAG);
		}
	}
	
	
	//update status
	led_off(LED_STATUS_ID);
}

void server_runtime()
{	
	int res;
	
	if( 0 == systick_check_timer( &work_timer ) )
		return ;
	
	if( server_collect_over == 1 ) 
		return ;
	
	//has collected enought records
	if( work_counter >= CALI_DATA_COUNT || 1 == systick_check_timer( &collect_1s_timer ) ){
		server_collect_over = 1;
		server_stop();
		//server_status = 0;
		return ;
	}
	
	
	res = 1;
	
	//get current by uart
	if( 0 == victor8165_getCurrent( &current_array[ work_counter ] ) ){
		_LOG("server_runtime: get current false\n");
		userStation_log("read current false");
		res = 0;
	}
	
	//get db
	db_array[ work_counter ] = get_voice_db();
	if( db_array[work_counter] < 0 ){
		_LOG("error: read adc \n");
		userStation_log("read adc false");
		res = 0;
	}
	if( db_array[work_counter] < MIN_ALARM_VOICE_VALUE ){
		_LOG("error: read Noise too low , check connection \n");
		userStation_log("error: Noise too low");
		res = 0;
	}


	//update counter
	if( res == 1 ){
		userStation_report( db_array[ work_counter ], current_array[ work_counter ], 0, 0 ); 
		work_counter++;
	}
	
	//will run this func after 100ms
	systick_init_timer( &work_timer, 600);
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
			//server_stop();
			server_status = 0;
		}
		
	}	
}

void test()
{
	unsigned char data;
	
	if( 0 < Uart_Get( &Uart2, &data,1 ) ){
		Uart_Put_Sync( &Uart2, &data, 1);
		Uart_Put_Sync( &Uart1, &data, 1);
	}
	
	if( 0 < Uart_Get( &Uart3, &data,1 ) ){
		Uart_Put_Sync( &Uart3, &data, 1);
		Uart_Put_Sync( &Uart1, &data, 1);
	}
}

void app_init()
{
	Uart_init();
	ADC_Configuration ();
	led_pin_config();
	
	//systick_delay_us(500000);
	
	switch_det_config();
	userStation_init();
	server_init();
}


void app_event()
{
	switch_det_event();
	server_event();
	userStation_listen_even();
}

