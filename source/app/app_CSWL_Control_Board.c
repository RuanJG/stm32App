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



volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)





PMSG_t PC_pmsg;
PMSG_t LED_pmsg;
PMSG_t VMETER_pmsg;
PMSG_t AMETER_pmsg;

miniMeter_t Ameter;

#define PC_TAG_LOGE  (PMSG_TAG_LOG|0x1)
#define PC_TAG_LOGI  (PMSG_TAG_LOG|0x2)

char logbuffer[ PROTOCOL_MAX_PACKET_SIZE ];
#define PC_LOG(T, X...) { snprintf( logbuffer, sizeof( logbuffer ), X);  PMSG_send_msg( &PC_pmsg , T , (unsigned char*)logbuffer, strlen(logbuffer) );}
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


//LED to PC
#define PC_TAG_DATA_LED_BRIGHTNESS (PMSG_TAG_DATA|0x1)
//Control to PC
#define PC_TAG_DATA_VMETER (PMSG_TAG_DATA|0x2)
#define PC_TAG_DATA_AMETER (PMSG_TAG_DATA|0x3)
#define PC_TAG_DATA_BUTTON (PMSG_TAG_DATA|0x4) // startbutton data[0] = 1 pressed 



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;
Uart_t Uart4;
Uart_t Uart5;
Uart_t* UartList[5] = { &Uart1,&Uart2,&Uart3,&Uart4,&Uart5};

#define PC_UART &Uart1
#define LED_UART &Uart4
#define VMETER_UART &Uart2
#define AMETER_UART &Uart5
#define CONSOLE_UART &Uart3




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
	
	Uart_Configuration (&Uart2, USART2, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
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
	
	Uart_Configuration (&Uart5, UART5, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
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






#define RELAY_ON 0
#define RELAY_OFF 1

#define RELAY_OUTPUT_TO_SHAVE(X) GPIO_WriteBit( GPIOC, GPIO_Pin_5 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_4 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_RESISTOR(X) GPIO_WriteBit( GPIOC, GPIO_Pin_6 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_VDD_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_7 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_POWER_TO_CSWL(X) GPIO_WriteBit( GPIOC, GPIO_Pin_8 , X==1? RELAY_ON:RELAY_OFF)

#define SIGNAL_ON 1
#define SIGNAL_OFF 0
#define SIGNAL_TO_AIRPRESS(X) GPIO_WriteBit( GPIOA, GPIO_Pin_1 , X==1? SIGNAL_ON:SIGNAL_OFF)

#define INDEX_SHAVE  0x1
#define INDEX_RESISTOR 0x2
#define INDEX_OUTPUT_METER 0x4
#define INDEX_VDD_METER 0x8
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





#define SW_INTERVAL_COUNT  10  // SW_INTERVAL_MS 检查一次GPIO口， SW_INTERVAL_COUNT次后发出按键事件, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher startButton;
//volatile unsigned char startButton_triggered =0 ;

void switch_key_release_handler()
{
	unsigned char data = 1;
	PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_BUTTON , (unsigned char*)&data, 1 );
}

void switch_init()
{
	switcher_init( &startButton, SW_INTERVAL_COUNT, 1 , 0 , GPIOB, GPIO_Pin_5, GPIO_Mode_IN_FLOATING, NULL , switch_key_release_handler);

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
			if( msg.data_len == 2 ){
				iap_jump();
			}else{
				PC_LOGE("PMSG_TAG_IAP_START_ID: data error");
			}
		break;
			
		default:
			PC_LOGE("PC_PMSG:  unknow data");
			break;
	}
	
}


//just forward to PC 
void LED_msg_handler(PMSG_msg_t msg)
{
	if( 0 == PMSG_send_msg_no_Tag( &PC_pmsg , msg.alldata, msg.alldata_len) ){
		PC_LOGE("LED_msg_handler: Forward bytes to PC error");
	}
}





void app_init()
{
	relay_init();
	Uart_USB_SWJ_init();
	PMSG_init_uart( &PC_pmsg, PC_UART, PC_msg_handler );
	PMSG_init_uart( &LED_pmsg, LED_UART, LED_msg_handler );
	switch_init();
	miniMeter_init( &Ameter ,AMETER_UART , 0x00);

	
	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(100);
		while(1);
	}
	
	PC_LOGI("Init OK");
}


void app_event()
{

	switch_even();
	cmd_even();
	
	if( uart_pressure_test_en == 1 )
	{ 
		uart_pressure_test_loop();
	}else{
		PMSG_even( &PC_pmsg );
		PMSG_even( &LED_pmsg );
	}
}

#endif //BOARD_CSWL_LED_MONITOR
