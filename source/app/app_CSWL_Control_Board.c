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
#define PC_TAG_CMD_AMETER_READ  (PMSG_TAG_CMD | 0x09 ) //data[0] = read n times and send the mean data , 0 stop 
#define PC_TAG_CMD_VMETER_READ  (PMSG_TAG_CMD | 0x0A ) //data[0] = read n times and send the mean data , 0 stop 


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
	
	

	
}






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





#define SW_INTERVAL_COUNT  40  // SW_INTERVAL_MS 检查一次GPIO口， SW_INTERVAL_COUNT次后发出按键事件, 用作过滤
#define SW_INTERVAL_TOLERANCE 10 // 30%
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
	switcher_init( &startButton, SW_INTERVAL_COUNT, SW_INTERVAL_TOLERANCE, 1 , 0 , GPIOB, GPIO_Pin_5, GPIO_Mode_IN_FLOATING, switch_key_release_handler, NULL);

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




#define VICTOR8165_BUFFSIZE 31

Uart_t *victor8165_uart;
volatile int victor8165_step;
volatile int victor8165_received_result;
volatile int victor8165_buffer_index;
volatile int victor8165_updated;
volatile float victor8165_value;
volatile int victor8165_en;
char victor8165_buffer[VICTOR8165_BUFFSIZE+1];
systick_time_t victor8165_timer;



void victor8165_send_cmd( char* cmd)
{
	Uart_Put( victor8165_uart, (unsigned char *)cmd,  strlen(cmd) );
}

int victor8165_check( float *value)
{
	if( victor8165_updated == 1 ){
		*value = victor8165_value;
		victor8165_updated = 0;
		return 1;
	}
	return 0;
}

void victor8165_start()
{
	victor8165_step = 0;
	victor8165_buffer_index = 0;
	victor8165_received_result = 0;
	victor8165_updated= 0;
	victor8165_en =1;
}

void victor8165_stop()
{
	victor8165_updated= 0;
	victor8165_en =0;
}

void victor8165_init( Uart_t *uart )
{
	victor8165_uart = uart;
	victor8165_step = 0;
	victor8165_buffer_index = 0;
	victor8165_received_result = 0;
	victor8165_updated= 0;
	victor8165_en = 0;
}

void victor8165_even()
{
	int i,len;
	unsigned char data[4];
	
	for( i=0; i< VICTOR8165_BUFFSIZE; i++ ){
		if( 1== Uart_Get( victor8165_uart, (victor8165_buffer + victor8165_buffer_index) , 1) ){
			if( victor8165_buffer[victor8165_buffer_index] == 0x0A ){
				victor8165_received_result = victor8165_buffer_index+1;
				victor8165_buffer[victor8165_received_result] = 0; // set string end tag
				victor8165_buffer_index = 0;
				break;
			}else{
				victor8165_buffer_index ++;
				if( victor8165_buffer_index >= VICTOR8165_BUFFSIZE ) victor8165_buffer_index = 0;
			}
		}else{
			break;
		}
	}
	
	if( victor8165_en == 0 ) return; 
	
	switch (victor8165_step)
	{
		case 0: victor8165_send_cmd("VDC\n"); victor8165_step++; systick_init_timer(&victor8165_timer, 200); break;
		case 1: if(systick_check_timer(&victor8165_timer)){victor8165_send_cmd("RATE M\n");victor8165_step++;}break;
		case 2: if(systick_check_timer(&victor8165_timer)){victor8165_send_cmd("RANGE 3\n");victor8165_step++;}break;
		case 3: if(systick_check_timer(&victor8165_timer)){victor8165_send_cmd("FUNC1?\n");victor8165_step++;systick_init_timer(&victor8165_timer, 300);}break;
		case 4: 
				if( victor8165_received_result > 0 )
				{
						if( victor8165_received_result == strlen("VDC\n") && 0 == strncmp("VDC\n",(char*)victor8165_buffer , victor8165_received_result) ){
							victor8165_step++; 
						}else{
							victor8165_step = 0;
						}
						victor8165_received_result=0;
				}else{ 
				     if(systick_check_timer(&victor8165_timer)){
							 victor8165_step = 0;
							 PC_LOGE("victor8165_even: Init timeout");
						 }  
				}; 
	  break;	
		case 5: systick_init_timer(&victor8165_timer, 200);victor8165_send_cmd("MEAS1?\n");victor8165_step++;break;
		case 6:
			if( victor8165_received_result > 2 ){
				sscanf((const char*)victor8165_buffer , "%e;\n",  &victor8165_value);
				victor8165_updated = 1;
				memcpy( data, (unsigned char*) &victor8165_value , 4 );
				if( 0 == PMSG_send_msg( &PC_pmsg , PC_TAG_DATA_VMETER, data , sizeof(data) ) ){
					PC_LOGE("victor8165_even: send data to PC error");
				}
				victor8165_received_result = 0;
				//no delay
				//victor8165_send_cmd("MEAS1?\n");
			}else{
				if(systick_check_timer(&victor8165_timer)){
					if( victor8165_updated == 0) PC_LOGE("victor8165_even: MEAS1 cmd timeout");
					//delay 
					victor8165_updated = 0;
					victor8165_send_cmd("MEAS1?\n");
				}
			}
		break;
			
		default:
			PC_LOGE("victor8165_even: Something error!");
		break;
		
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
				if( msg.data[0] == 0 ){
					victor8165_stop();
				}else{
					victor8165_start();
				}
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
		PC_LOGE("LED_msg_handler: Forward bytes to PC error");
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
	//console_init( CONSOLE_USB_TYPE ,NULL );
	
	PMSG_init_uart( &PC_pmsg, PC_UART, PC_msg_handler );
	PMSG_init_uart( &LED_pmsg, LED_UART, LED_msg_handler );
	switch_init();
	Ameter_init();
	victor8165_init( VMETER_UART );
	

	
	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(1000);
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
	
	victor8165_even();
	Ameter_even();
}
#endif //iap

#endif //BOARD_CSWL_LED_MONITOR
