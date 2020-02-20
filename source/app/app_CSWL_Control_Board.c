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
#include "Pmsg.h"
#include "switcher.h"
#include "miniMeter.h"

#if BOARD_CSWL_CONTROL_BOARD



// choise one version for build
#define CSWL_CONTROL_BOARD_V1 0
#define CSWL_CONTROL_BOARD_V2 1  



volatile int debug_en = 1;
#if 0
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)
#else
#define _LOGW(X...) {}
#define _LOGE(X...) {}
#endif

	
	
#define PC_UART &Uart1
#define LED_UART &Uart4
#define VMETER_UART &Uart2
#define AMETER_UART &Uart3
#define CONSOLE_UART &Uart5


#define USING_CMD_EVEN  0
#define USING_TONGHUI_TH963_VMETER 1



PMSG_t PC_pmsg;
PMSG_t LED_pmsg;
PMSG_t VMETER_pmsg;
PMSG_t AMETER_pmsg;

miniMeter_t Ameter;



#define PC_TAG_LOGE  (PMSG_TAG_LOG|0x1)
#define PC_TAG_LOGI  (PMSG_TAG_LOG|0x2)

char logbuffer[ PROTOCOL_MAX_PACKET_SIZE ];
#define PC_LOG(T, X...) { snprintf( logbuffer, sizeof(logbuffer)-1, X); logbuffer[strlen(logbuffer)]=0; PMSG_send_msg( &PC_pmsg , T , (unsigned char*)logbuffer, strlen(logbuffer) );}
#define PC_LOGI(X...) {PC_LOG( PC_TAG_LOGI , X);_LOGW(X);}
#define PC_LOGE(X...) {PC_LOG( PC_TAG_LOGE , X);_LOGE(X);}



//PC to LED
#define PC_TAG_CMD_CAPTURE_EN  (PMSG_TAG_CMD | 0x01 ) // data[0] = 1 auto start, data[0] = 0 stop   ; 4101  4100
#define PC_TAG_CMD_LED_SELECT  (PMSG_TAG_CMD | 0x02 )  // data[0] = led id [1~12], data[1] = status  ; 420138  42010B 
#define PC_TAG_CMD_CAPTURE_INTERVAL  (PMSG_TAG_CMD | 0x03 )  //data[0] = ms_L, data[1] = ms_H    ;  430A00 (10ms)
#define PC_TAG_CMD_TEST (PMSG_TAG_CMD | 0x04 ) 
#define PC_TAG_CMD_SWITCHES_TESTMODE (PMSG_TAG_CMD | 0x05 ) // data[0] = 2,3,4,5 , FF mean off all ; 4502  4503  4504  45FF 
#define PC_TAG_CMD_SWITCHES_CHANNEL (PMSG_TAG_CMD | 0x06 ) // data[0] = 0~15 , FF mean off all    ; 4603 (Vled) 460C (VDD) 
// PC->Control
#define PC_TAG_CMD_SWITCH  (PMSG_TAG_CMD | 0x07 ) // data[0] = SWITCH status   4701 (shave) 4702 (RESISTOR) 4704 (OUTPUT_METER)  4708 (VDD_METER) 4710 (POWER) 4720 (SIGNAL air )
#define PC_TAG_CMD_UART_TEST  (PMSG_TAG_CMD | 0x08 ) // data[0] = Uart ID , 0 = stop
#define PC_TAG_CMD_AMETER_READ  (PMSG_TAG_CMD | 0x09 ) //data[0] = read n times and send the mean data , 0 stop 
#define PC_TAG_CMD_VMETER_READ  (PMSG_TAG_CMD | 0x0A ) //data[0] = read n times and send the mean data , 0 stop ;  data[1] = vmeter_type

//LED to PC
#define PC_TAG_DATA_LED_BRIGHTNESS (PMSG_TAG_DATA|0x1)
//Control to PC
#define PC_TAG_DATA_VMETER (PMSG_TAG_DATA|0x2)
#define PC_TAG_DATA_AMETER (PMSG_TAG_DATA|0x3)
#define PC_TAG_DATA_BUTTON (PMSG_TAG_DATA|0x4) // startbutton data[0] = 1 pressed 
#define PC_TAG_DATA_QRCODE (PMSG_TAG_DATA|0x5)

//PC to boards
#define PC_TAG_FORWARD_LEDBOARD  ( PMSG_TAG_FORWARD | 0x04)  // [PMSG_TAG_FORWARD] [ tag for next board] ... [tag for final board][data...]

Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;
Uart_t Uart4;
Uart_t Uart5;
Uart_t* UartList[5] = { &Uart1,&Uart2,&Uart3,&Uart4,&Uart5};




void Uart_USB_SWJ_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1								  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
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
	
#if 1
	//USART2				
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart2, USART2, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
#endif

#if 1
	//USART3		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart3, USART3, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
#if 1
	//UART4					
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart4, UART4, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif


#if 1
	//UART5				
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart5, UART5, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	console_init( CONSOLE_UART_TYPE ,CONSOLE_UART , NULL, 0);
	//console_init( CONSOLE_USB_TYPE ,NULL );
	
}




#if CSWL_CONTROL_BOARD_V2

#define RELAY_OUTPUT_TO_SHAVE(X) GPIO_WriteBit( GPIOC, GPIO_Pin_6 ,0); GPIO_WriteBit( GPIOC, GPIO_Pin_5 ,X)
#define RELAY_OUTPUT_TO_METER(X) GPIO_WriteBit( GPIOA, GPIO_Pin_7 ,0); GPIO_WriteBit( GPIOA, GPIO_Pin_6 ,X)
#define RELAY_OUTPUT_TO_RESISTOR(X) GPIO_WriteBit( GPIOA, GPIO_Pin_5 ,0); GPIO_WriteBit( GPIOA, GPIO_Pin_4 ,X)
#define RELAY_VDD_TO_METER(X) if( X==1 ){GPIO_WriteBit( GPIOA, GPIO_Pin_7 ,0); GPIO_WriteBit( GPIOA, GPIO_Pin_6 ,0);}
#define RELAY_POWER_TO_CSWL(X) GPIO_WriteBit( GPIOB, GPIO_Pin_2 ,0); GPIO_WriteBit( GPIOB, GPIO_Pin_1 ,X)


#define SIGNAL_TO_AIRPRESS(X) GPIO_WriteBit( GPIOC, GPIO_Pin_8 ,0); GPIO_WriteBit( GPIOC, GPIO_Pin_4 ,X)

#define INDEX_SHAVE  0x1
#define INDEX_RESISTOR 0x2
#define INDEX_OUTPUT_METER 0x8
#define INDEX_VDD_METER 0x4
#define INDEX_POWER  0x10
#define INDEX_SIGNAL 0x20

volatile unsigned char relay_status;

void relay_update_status( unsigned char status )
{

	SIGNAL_TO_AIRPRESS( ((INDEX_SIGNAL & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_SHAVE( ((INDEX_SHAVE & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_METER( ((INDEX_OUTPUT_METER & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_RESISTOR( ((INDEX_RESISTOR & status)==0 ? 0: 1) );
	RELAY_VDD_TO_METER( ((INDEX_VDD_METER & status)==0 ? 0: 1) );
	RELAY_POWER_TO_CSWL( ((INDEX_POWER & status)==0 ? 0: 1) );
	relay_status = status;
}

void relay_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	relay_update_status(0);


}

#endif




#if CSWL_CONTROL_BOARD_V1

#define RELAY_ON 0
#define RELAY_OFF 1

#define RELAY_OUTPUT_TO_SHAVE(X) GPIO_WriteBit( GPIOC, GPIO_Pin_5 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_7 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_RESISTOR(X) GPIO_WriteBit( GPIOC, GPIO_Pin_6 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_VDD_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_4 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_POWER_TO_CSWL(X) GPIO_WriteBit( GPIOC, GPIO_Pin_8 , X==1? RELAY_ON:RELAY_OFF)

#define SIGNAL_ON 1
#define SIGNAL_OFF 0
#define SIGNAL_TO_AIRPRESS(X) GPIO_WriteBit( GPIOA, GPIO_Pin_1 , X==1? SIGNAL_ON:SIGNAL_OFF)

#define INDEX_SHAVE  0x1
#define INDEX_RESISTOR 0x2
#define INDEX_OUTPUT_METER 0x8
#define INDEX_VDD_METER 0x4
#define INDEX_POWER  0x10
#define INDEX_SIGNAL 0x20

volatile unsigned char relay_status;

void relay_update_status( unsigned char status )
{

	SIGNAL_TO_AIRPRESS( ((INDEX_SIGNAL & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_SHAVE( ((INDEX_SHAVE & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_METER( ((INDEX_OUTPUT_METER & status)==0 ? 0: 1) );
	RELAY_OUTPUT_TO_RESISTOR( ((INDEX_RESISTOR & status)==0 ? 0: 1) );
	RELAY_VDD_TO_METER( ((INDEX_VDD_METER & status)==0 ? 0: 1) );
	RELAY_POWER_TO_CSWL( ((INDEX_POWER & status)==0 ? 0: 1) );
	relay_status = status;
}

void relay_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	relay_update_status(0);


}

#endif






#define SW_INTERVAL_COUNT  40  // SW_INTERVAL_MS 检查一次GPIO口， SW_INTERVAL_COUNT次后发出按键事件, 用作过滤
#define SW_INTERVAL_TOLERANCE 10 // 25%
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
struct switcher startButton;
//volatile unsigned char startButton_triggered =0 ;

void switch_key_press_handler()
{
	unsigned char data = 1;
	PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_BUTTON , (unsigned char*)&data, 1 );
}

void switch_init()
{
	
#if CSWL_CONTROL_BOARD_V2
	switcher_init( &startButton, SW_INTERVAL_COUNT ,SW_INTERVAL_TOLERANCE , 1 , 0 , GPIOA, GPIO_Pin_0, GPIO_Mode_IN_FLOATING, switch_key_press_handler , NULL);
#endif
#if CSWL_CONTROL_BOARD_V1
	switcher_init( &startButton, SW_INTERVAL_COUNT, SW_INTERVAL_TOLERANCE, 1 , 0 , GPIOB, GPIO_Pin_5, GPIO_Mode_IN_FLOATING, switch_key_press_handler, NULL);
#endif
	
	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &startButton );

}





Uart_t *uart_pressure_dev = NULL;
volatile int uart_pressure_test_en = 0;
volatile int uart_pressure_round = 0; // 1000,000 times send result
void uart_pressure_test_loop()
{
	volatile int i,ms;
	unsigned char buffer[64];
	unsigned char data[4] = {0x5A,0x55,0xAA,0xA5};
	unsigned char val;
	
	if( uart_pressure_dev == NULL ) return;
	
	for( i=0; i< sizeof(buffer); i++ ) buffer[i] = data[i%4];
	
	Uart_Clear_Rx(uart_pressure_dev);
	
	Uart_Put(uart_pressure_dev,buffer,sizeof(buffer));
	
	ms=0;
	for( i=0; i< sizeof(buffer); i++ )
	{
		while(1)
		{
			if( 1 == Uart_Get(uart_pressure_dev, &val,1) )
			{
				ms = 0;
				if( val != buffer[i] ){
					_LOGE("Uart test Failed\n");
					systick_delay_ms(10);
					uart_pressure_test_en =0;
					return;
				}
				break;
			}else{
				if( ms > 50 ){
					_LOGE("Time Out\n");
					systick_delay_ms(10);
					uart_pressure_test_en =0;
					return;
				}else{
					systick_delay_ms(1);
					ms++;
				}
			}
		}
	}

	uart_pressure_round += sizeof(buffer);
	if( uart_pressure_round >= 1000000 ) {
		uart_pressure_round = 0;
		_LOGW("Test 1000000 bytes OK\n");
	}
}


volatile int Ameter_en_cnt;
volatile float Ameter_val;
volatile int  Ameter_val_count;

void Ameter_init()
{
	miniMeter_init( &Ameter ,AMETER_UART , 0x00);
	Ameter_en_cnt = 0;
	Ameter_val_count = 0;
}

int Ameter_start( int cnt )
{
	Ameter_en_cnt = cnt;
	
	if( Ameter_en_cnt == 0 ){
		miniMeter_stop(&Ameter);
		return 1;
	}else{
		if( 1 == miniMeter_start(&Ameter) ){
			Ameter_val_count = 0;
			Ameter_val = 0;
			return 1;
		}else{
			Ameter_en_cnt = 0;
			return 0;
		}
	}
}

void Ameter_even()
{
	float value;
	unsigned char data[4];
	
	if( Ameter_en_cnt == 0 ) return;
	
	if( 1== miniMeter_check( &Ameter, &value) ){
		Ameter_val_count++;
		Ameter_val += value;
		if( Ameter_val_count >= Ameter_en_cnt ){
			Ameter_val /= Ameter_val_count;
			memcpy( data, (unsigned char*) &Ameter_val , 4 );
			if( 0 == PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_AMETER, data , sizeof(data) ) ){
				PC_LOGE("Ameter_even: send data to PC error");
			}
			Ameter_val_count = 0;
			Ameter_val = 0;
		}
	}
}


/*
volatile int Vmeter_en_cnt;
volatile float Vmeter_val;
volatile int  Vmeter_val_count;

void Vmeter_init()
{
	miniMeter_init( &Vmeter ,VMETER_UART , 0x00);
	Vmeter_en_cnt = 0;
	Vmeter_val_count = 0;
}

int Vmeter_start( int cnt )
{
	Vmeter_en_cnt = cnt;
	
	if( Vmeter_en_cnt == 0 ){
		miniMeter_stop(&Vmeter);
		return 1;
	}else{
		if( 1 == miniMeter_start(&Vmeter) ){
			Vmeter_val_count = 0;
			Vmeter_val = 0;
			return 1;
		}else{
			Vmeter_en_cnt = 0;
			return 0;
		}
	}
}

void Vmeter_even()
{
	float value;
	unsigned char data[4];
	
	if( Vmeter_en_cnt == 0 ) return;
	
	if( 1== miniMeter_check( &Vmeter, &value) ){
		Vmeter_val_count++;
		Vmeter_val += value;
		if( Vmeter_val_count >= Vmeter_en_cnt ){
			Vmeter_val /= Vmeter_val_count;
			memcpy( data, (unsigned char*) &Vmeter_val , 4 );
			if( 0 == PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_VMETER, data , sizeof(data) ) ){
				PC_LOGE("Vmeter_even: send data to PC error");
			}
			Vmeter_val_count = 0;
			Vmeter_val = 0;
		}
	}
}
*/




#if USING_TONGHUI_TH963_VMETER

// cmd 'MEAS:VOLT:DC? 100\n' -> '2.45693330E-04\r\n'  or  '-2.45693330E-04\r\n' 

//or 
	
// cmd 'conf:volt:dc 100;:trig:sour bus;:trig:coun 1;:samp:coun 3\n' ,  'read?;*trg' or 'init;fetc?;*trg\n'
// cmd 'abort'

//*CLS 清除错误信息
//:SYSTem:ERROr?  读错误信息  -> 0,"No error";\n    或  -113,"Undefined header"\n  （ 8165 支持， th963 不支持,因它直接反回cmd错误信息）
//*RST 回复到开机状态 
//:LOCal ( 8165 no support , th963 support )

#define TH963_BUFFER_SIZE 32

typedef struct TH963_Meter {
	systick_time_t timer;
	Uart_t *uart;
	volatile float measureVal;
	volatile unsigned char updated;
	volatile unsigned char started;
	char cmdRes[TH963_BUFFER_SIZE];
	volatile int cmdStep;
	volatile int initStep;
	volatile int cmdbufferIndex;
}TH963_Meter_t;


void TH963_cmd_reset(TH963_Meter_t * th963)
{
	th963->cmdbufferIndex = 0;
	th963->cmdRes[0] =0;
	th963->cmdStep = 0;
	Uart_Clear_Rx( th963->uart );
}

// get a string : return 1 , result store in th963->cmdRes
// otherwise : return 0
int TH963_cmd_read_string( TH963_Meter_t * th963 )
{
	volatile unsigned char data;
	
	while(1)
	{
		if( 1 == Uart_Get( th963->uart, &data ,1 ))
		{
			if( th963->cmdbufferIndex >= (sizeof( th963->cmdRes ) -2) )
			{
				th963->cmdbufferIndex = 0;
				PC_LOGE("TH963_cmd_read_string: overflow!");
			}
			th963->cmdRes[ th963->cmdbufferIndex++ ] = data;
			if( data == '\n' )
			{
				th963->cmdRes[ th963->cmdbufferIndex++ ] = 0;
				th963->cmdbufferIndex = 0;
				return 1;
			}
		}else{
			return 0;
		}
	}
}	


//0 : waiting for cmd runing
//1 : finish , result in th963->cmdRes 
//-1 : error / failed , result in th963->cmdRes 
int TH963_cmd_even(TH963_Meter_t * th963, char * cmd , int timeoutMs , int waitResult , int waitTimeout)
{
	int rest;
	volatile unsigned char data;
	
	rest = 0;
	
	switch( th963->cmdStep )
	{		
		case 0:  // send cmd
			TH963_cmd_reset( th963 );
			Uart_Put(th963->uart, (unsigned char*) cmd, strlen(cmd));
			th963->cmdStep++;
			systick_init_timer( &th963->timer, timeoutMs);
		break;
		
		case 1: // verify cmd 
			// if has result , no  wait for cmd error msg
			if( waitResult == 1 ){
				th963->cmdStep++;
				break;
			}
			
			// if any string send by meter , it should be the error msg
			// error string store in th963->cmdRes
			if( 1 == TH963_cmd_read_string( th963 ) ){
				rest = -1;
				break;
			}
			
			//if not require reply string , timeout meant ok;
			if( 1 == systick_check_timer( &th963->timer ) ){
				rest = 1;
				break;
			}
		break;
			
		case 2: //get result
			if( 1 == TH963_cmd_read_string( th963 ) )
			{
				#if 0 // th963 will reply the Error imediaterily
				if( strncmp(th963->cmdRes, "Error 5", 7) == 0){
					PC_LOGE("TH963_cmd_even: Unknow cmd  !");
					rest = -1;
					break;
				}
				#endif
				
				if( waitTimeout != 0 && 0 == systick_check_timer( &th963->timer ) ) {
					th963->cmdStep++;
					break;
				}else{
					rest = 1;
					break;
				}
			}
			
			if( 1 == systick_check_timer( &th963->timer ) ){
				PC_LOGE("TH963_cmd_even: cmd timeout !");
				rest = -1;
				break;
			}
		break;
			
			
		case 3: // some cmd need time to delay , this step just like for interval cmd loop
			if( 1 == systick_check_timer( &th963->timer ) ){
				rest = 1;
			}
		break;
		
		default:
			PC_LOGE("TH963_cmd_even: error step!");
		break;
	}
	
	if( rest != 0 )
		th963->cmdStep = 0;
	
	return rest;
}

int TH963_cmd_sync(TH963_Meter_t * th963, char * cmd , int delayMs, int has_result)
{
	int val = 0;
	
	while( 1 ){
		val = TH963_cmd_even( th963, cmd , delayMs, has_result , 0) ;
		if( val  != 0 ) break;
	}
	
	return val;
}



TH963_Meter_t Vmeter_th963;


void TH963_clear(TH963_Meter_t * th963)
{
	th963->started = 0;
	th963->updated = 0;
	th963->initStep = 0;
	TH963_cmd_reset( th963 );
}

int TH963_init(TH963_Meter_t * th963, Uart_t *uart_p )
{
	th963->uart = uart_p;
	TH963_clear(th963);
	return 0;
}

int TH963_start( TH963_Meter_t * th963 , int en )
{
	if( th963->started == 0 && en ==1 ){
		
		TH963_clear( th963 );
		
		// flush the unknow bytes in meter rs232 pipe
		TH963_cmd_sync( th963, "\n", 200, 0) ;
		
		if( 1 != TH963_cmd_sync( th963, "*RST\n", 500, 0) ){
			PC_LOGE("TH963 RST error : %s\n",th963->cmdRes);
		}
	}
	
	th963->started = en;
	
	if( th963->started == 0 ){
		//if( 1 != TH963_cmd_sync( th963, ":LOCal\n", 100, 0, 1 ) ){
		//	PC_LOGE("TH963 local error\n");
		//};
		PC_LOGI("TH963 Vmeter stop\n");
	}
	if( th963->started == 1 ) {
		PC_LOGI("TH963 Vmeter start\n");
	}
	
	return 1;
}


void TH963_even(TH963_Meter_t * th963)
{
	int value, i;
	unsigned char data[4];
	
	if( th963->started != 1 ) return;
	
	value = TH963_cmd_even( th963, ":MEASure:VOLTage:DC? 100\n", 1000, 1 , 0); // can't be less than 800ms
	
	if( value == 1 ){
		//victor 8165 : "%e;\n"
		//TH963 :  "%e\r\n"
		i = strlen((const char*)th963->cmdRes);
		if( i > 2 ) {
			th963->cmdRes[i-1] = 0;
			th963->cmdRes[i-2] = 0;
		}
		if( 1 == sscanf((const char*)th963->cmdRes , "%e",  &th963->measureVal) ){
			//PC_LOGI("th963>%f",th963->measureVal);
			th963->updated = 1;
			memcpy( data, (unsigned char*) &th963->measureVal , 4 );
			if( 0 == PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_VMETER, data , sizeof(data) ) ){
				PC_LOGE("TH963_even: send data to PC error");
			}
		}else{
			PC_LOGE("TH963_even: cmd error>%s",th963->cmdRes);
		}
	}
	if( value == -1 ){
		PC_LOGI("TH963 cmd failed. Reset...\n");
		if( 1 != TH963_cmd_sync( th963, "*RST\n", 1000, 0) ){
			PC_LOGE("TH963 RST error\n");
		};
	}
}




#endif















void cmd_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = Uart_Get(CONSOLE_UART,buffer,1);// USB_RxRead( buffer, 8 );
	
	if( len == 0 ) return;

	if( buffer[0] == 'd' ){
		debug_en = debug_en ? 0:1;
	}else if( buffer[0] == '1' ){
		uart_pressure_dev = &Uart1;
		uart_pressure_test_en = 1;
		uart_pressure_round = 0;
		_LOGW("Uart Test Uart1\n");
	}else if( buffer[0] == '2' ){
		uart_pressure_dev = &Uart2;
		uart_pressure_test_en = 1;
		uart_pressure_round = 0;
		_LOGW("Uart Test Uart2\n");
	}else if( buffer[0] == '3' ){
		uart_pressure_dev = &Uart3;
		uart_pressure_test_en = 1;
		uart_pressure_round = 0;
		_LOGW("Uart Test Uart3\n");
	}else if( buffer[0] == '4' ){
		uart_pressure_dev =  &Uart4;
		uart_pressure_test_en = 1;
		uart_pressure_round = 0;
		_LOGW("Uart Test Uart4\n");
	}else if( buffer[0] == '0' ){
		uart_pressure_dev = NULL;
		uart_pressure_test_en = 0;
		_LOGW("Uart Test STOP\n");
	}
}


volatile int relayTestIndex = 0;
volatile int relayTestCounter = 0;
void relay_test_even()
{
	static int on_off = 0;
	
	if( relayTestIndex == 0 ) return;
	
	systick_delay_ms(500);
	if( on_off ){
		relay_update_status(relayTestIndex);
		on_off = 0;
		relayTestCounter++;
	}else{
		relay_update_status(0);
		on_off = 1;
	}
}


void PC_msg_handler(PMSG_msg_t msg)
{
	int value,res;
	
	switch( msg.tag )
	{
		case PC_TAG_CMD_SWITCH:
			if( msg.data_len == 1 ){
				relay_update_status( msg.data[0] );
				PC_LOGI("PC_TAG_CMD_SWITCH: status = %02x", relay_status);
			}else{
				PC_LOGE("PC_TAG_CMD_SWITCH: unknow data");
			}
		break;
			
		case PC_TAG_CMD_UART_TEST:
			if( msg.data_len != 1 ){
				if( msg.data[0] == 0 ){
					uart_pressure_dev = NULL;
					uart_pressure_test_en = 0;
					_LOGW("Uart Test STOP\n");
				}else if( msg.data[0] >= 1 && msg.data[0] <= 5){
					uart_pressure_test_en = 1;
					uart_pressure_round = 0;
					uart_pressure_dev = UartList[msg.data[0]-1];
					PC_LOGE("PC_TAG_CMD_UART_TEST: Uart%d start test\n",msg.data[0]-1);
				}else{
					PC_LOGE("PC_TAG_CMD_UART_TEST: data out of range");
				}
			}else{
				PC_LOGE("PC_TAG_CMD_UART_TEST: unknow data");
			}
		break;
			
		case PC_TAG_CMD_CAPTURE_EN:
		case PC_TAG_CMD_LED_SELECT:
		case PC_TAG_CMD_CAPTURE_INTERVAL:
		case PC_TAG_CMD_TEST:
		case PC_TAG_CMD_SWITCHES_TESTMODE:
		case PC_TAG_CMD_SWITCHES_CHANNEL:
			
			if( 0 == PMSG_send_msg_no_Tag( &LED_pmsg , msg.alldata, msg.alldata_len) ){
				PC_LOGE("PC_msg_handler: Forward bytes to LED Failed");
			}
		break;
			
		case PMSG_TAG_IAP_START_ID:
			if( msg.data_len == 1 ){
				iap_jump();
			}else{
				PC_LOGE("PMSG_TAG_IAP_START_ID: data error");
			}
		break;
			
		case PC_TAG_CMD_AMETER_READ:
			if( msg.data_len == 1 ){
				if( 0 == Ameter_start( msg.data[0] ) ){
					PC_LOGE("PC_TAG_CMD_AMETER_READ: start meter failed");
				}
			}else{
				PC_LOGE("PC_TAG_CMD_AMETER_READ: data error");
			}
		break;
			
		case PC_TAG_CMD_VMETER_READ:
			if( msg.data_len == 1 ){
				//msg.data[0] == 1 start vmeter  victor 8164 
				//msg.data[0] == 2 start vmeter  th963
				//msg.data[0] == 0 stop
				#if USING_TONGHUI_TH963_VMETER
				value = msg.data[0]==0 ? 0:1 ;
				if( 0 == TH963_start( &Vmeter_th963 , value ) ){
					PC_LOGE("PC_TAG_CMD_VMETER_READ: start meter failed");
				}
				#endif
			}else{
				PC_LOGE("PC_TAG_CMD_VMETER_READ: data error");
			}
		break;
			
		case PC_TAG_FORWARD_LEDBOARD:
			if( msg.data_len >= 1 ){
				if( 0 == PMSG_send_msg_no_Tag( &LED_pmsg , msg.data, msg.data_len) ){
					PC_LOGE("PC_TAG_FORWARD_LEDBOARD: send Failed");
				}
			}else{
				PC_LOGE("PC_TAG_FORWARD_LEDBOARD: data error");
			}
		break;
			
		case 0xff:
			PC_LOGI("Last Relay Test: 0x%02x, On-Off count:%d",relayTestIndex, relayTestCounter);
			relayTestIndex = msg.data[0];
			relayTestCounter = 0;
			PC_LOGI("Start Relay Test: 0x%02x",relayTestIndex);
		break;
			
		default:
			if( (msg.tag & 0xf0) != PMSG_TAG_FORWARD )
			{
				PC_LOGE("PC_PMSG:  unknow data(0x%02x)",msg.tag);
				break;
			}
			
			if( (msg.tag&0x0f) == 0x2 ){
				Uart_Put(&Uart2, msg.data,msg.data_len);
			}else if( (msg.tag&0xf) == 0x3 ){
				Uart_Put(&Uart3, msg.data,msg.data_len);
			}else if( (msg.tag&0xf) == 0x4 ){
				Uart_Put(&Uart4, msg.data,msg.data_len);
			}else if( (msg.tag&0xf) == 0x5 ){
				Uart_Put(&Uart5, msg.data,msg.data_len);
			}else{
				PC_LOGE("PMSG_FORWARD:  unknow way (0x%02x)",msg.tag);
			}
			break;
	}
	
}


//just forward to PC 
void LED_msg_handler(PMSG_msg_t msg)
{
	if( 0 == PMSG_send_msg_no_Tag( &PC_pmsg , msg.alldata, msg.alldata_len) ){
		PC_LOGE("LED_msg_handler: Forward to PC error");
	}
}








#if BOARD_IAP 
void app_init()
{
	relay_init();
	Uart_USB_SWJ_init();
#if IAP_APP_PORT_UART
	iap_app_init(PC_UART,PORT_UART_TYPE );
#endif
}
void app_event()
{
	iap_app_event();
}
#else



void app_init()
{
	relay_init();
	Uart_USB_SWJ_init();
	PMSG_init_uart( &PC_pmsg, PC_UART, PC_msg_handler );
	PMSG_init_uart( &LED_pmsg, LED_UART, LED_msg_handler );
	//PMSG_init_uart_irq( &LED_pmsg, LED_UART, LED_msg_handler );
	switch_init();
	Ameter_init();
	

#if USING_TONGHUI_TH963_VMETER
	TH963_init( &Vmeter_th963 , VMETER_UART );
#endif
	

	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(1000);
		while(1);
	}
	
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
	
	PC_LOGI("Init OK");
}


void app_event()
{

	switch_even();
	
#if USING_CMD_EVEN
	cmd_even();
#endif
	
	//relay_test_even();
	
	if( uart_pressure_test_en == 1 )
	{ 
		uart_pressure_test_loop();
	}else{
		PMSG_even( &PC_pmsg );
		PMSG_even( &LED_pmsg );
	}

#if USING_TONGHUI_TH963_VMETER
		TH963_even(&Vmeter_th963);
#endif
	
}


#endif // BORAD_IAP






#endif //BOARD_CSWL_LED_MONITOR
