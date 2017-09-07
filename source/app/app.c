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

	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);



	Uart_config_console( CONSOLE_UART );

}




















#define LED_PWM_PERIOD 25500
typedef struct _led_color_t {
	unsigned int level;
	unsigned char r;
	unsigned char g;
	unsigned char b;
}led_color_t;
static led_color_t led1_color;
static led_color_t led2_color;

/*
*   config timer pwm , Note : need enable timer PeriphClock before call this 
*/
int pwm_init( TIM_TypeDef* timer , int channel, unsigned short period, int freq_hz , int high1_low0)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIMOCInitStructure;
	unsigned short prescaler ;
	unsigned int clk  ;//36M

	clk = SystemCoreClock/2;
	if( timer == TIM1 )
		clk = SystemCoreClock;

	// freq = [36M/(prescaler(16bit)+1)] / period 
	// preriod = 1000 , freq= 1000 Hz , prescaler = 36
	prescaler = clk/period/freq_hz;
	_LOG( "period=%d, freq=%d Hz, prescal=%d \n", period, freq_hz, prescaler);


	TIM_Cmd(timer, DISABLE);
	//TIM_DeInit(timer);
	TIM_InternalClockConfig(timer);
	
	//base config
	TIM_TimeBaseStructure.TIM_Period= period-1;
	TIM_TimeBaseStructure.TIM_Prescaler= prescaler-1 ;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

	//pwm
	TIMOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // pwm mode 1
	TIMOCInitStructure.TIM_Pulse = 0 ;//default 0%
	TIMOCInitStructure.TIM_OCPolarity = high1_low0==1? TIM_OCPolarity_High:TIM_OCPolarity_Low ;
	TIMOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	switch( channel ){
	case 1:
		TIM_OC1Init(timer, &TIMOCInitStructure);
		break;
	case 2:
		TIM_OC2Init(timer, &TIMOCInitStructure);
		break;
	case 3:
		TIM_OC3Init(timer, &TIMOCInitStructure);
		break;
	case 4:
		TIM_OC4Init(timer, &TIMOCInitStructure);
		break;
	default:
		return -1;
	}

	TIM_CtrlPWMOutputs(timer,ENABLE);
	TIM_Cmd(timer, ENABLE);
	return 0;
}


void led1_red_pwm_set( uint16_t pwm )
{//pwm5  C7(T3.2)
	TIM_SetCompare2( TIM3 , pwm );
}

void led1_blue_pwm_set( uint16_t pwm  )
{//pwm4 B9(T4.4)
	TIM_SetCompare4( TIM4 , pwm );
}

void led1_green_pwm_set( uint16_t pwm  )
{//pwm6 C6(T3.1)
	TIM_SetCompare1( TIM3 , pwm );
}

void led2_red_pwm_set( uint16_t pwm  )
{//pwm2 B0(T1.2N)
	TIM_SetCompare2( TIM1 , pwm );
}

void led2_green_pwm_set( uint16_t pwm  )
{//pwm3 B8(T4.3)
	TIM_SetCompare3( TIM4 , pwm );
}

void led2_blue_pwm_set( uint16_t pwm  )
{//pwm1 B1(T1.3N)
	TIM_SetCompare3( TIM1 , pwm );
}


void _led1_update( )
{
	led1_red_pwm_set( LED_PWM_PERIOD * led1_color.level * led1_color.r / 255 );
	led1_green_pwm_set( LED_PWM_PERIOD * led1_color.level * led1_color.g / 255 );
	led1_blue_pwm_set( LED_PWM_PERIOD * led1_color.level * led1_color.b / 255 );
}
void led1_set_level( unsigned int level )
{
	if( level > 100 ) return;
	
	led1_color.level = level;
	_led1_update();
}
void led1_set_color( unsigned char red, unsigned char green, unsigned char blue )
{
	led1_color.r = red;
	led1_color.g = green;
	led1_color.b = blue;
	_led1_update();
}


void _led2_update( )
{
	led2_red_pwm_set( LED_PWM_PERIOD * led2_color.level * led2_color.r / 255 );
	led2_green_pwm_set( LED_PWM_PERIOD * led2_color.level * led2_color.g / 255 );
	led2_blue_pwm_set( LED_PWM_PERIOD * led2_color.level * led2_color.b / 255 );
}
void led2_set_level( unsigned int level )
{
	if( level > 100 ) return;
	
	led2_color.level = level;
	_led2_update();
}
void led2_set_color( unsigned char red, unsigned char green, unsigned char blue )
{
	led2_color.r = red;
	led2_color.g = green;
	led2_color.b = blue;
	_led2_update();
}



void testpwmled_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PC
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//remap
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); // T1
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); // T3
	
	//timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //APB1 36M clk
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //APB2 72M clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //APB1 36M clk
	
	pwm_init( TIM1, 2, LED_PWM_PERIOD , 100 , 0 );
	pwm_init( TIM1, 3, LED_PWM_PERIOD , 100 , 0 );
	pwm_init( TIM4, 3, LED_PWM_PERIOD , 100 , 1 );
	pwm_init( TIM4, 4, LED_PWM_PERIOD , 100 , 1 );
	pwm_init( TIM3, 1, LED_PWM_PERIOD , 100 , 1 );
	pwm_init( TIM3, 2, LED_PWM_PERIOD , 100 , 1 );
	
	led1_set_color( 255, 255, 255 );
	led1_set_level( 20 );

	led2_set_color( 255, 255, 255 );
	led2_set_level( 20 );
}













// packget  head tag 
#define USER_DATA_TAG	1
#define USER_LOG_TAG	2
#define USER_START_TAG	3
#define USER_CMD_CHMOD_TAG 4
#define USER_CMD_CURRENT_MAXMIN_TAG 5
#define USER_CMD_VOICE_MAXMIN_TAG 6
#define USER_CMD_LED 7

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
		
		case USER_CMD_LED:
			if( len == 6 ){
				if( data[1] == 1 ){
					led1_set_level( data[2] );
					led1_set_color( data[3], data[4] , data[5]);
				}
				if( data[1] == 2 ){
					led2_set_level( data[2] );
					led2_set_color( data[3], data[4] , data[5]);
				}				
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
	userStation_init();
	systick_delay_us( 1000000 );
	testpwmled_init();
}


void app_event()
{

	userStation_listen_even();
}

