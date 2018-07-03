#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "switcher.h"

#if BOARD_MULTIKILL_PCBA




#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

//#define CONSOLE_UART &Uart3
//#define PC_UART &Uart3
//#define IAP_UART &Uart3
#define METER_UART &Uart1
//#define CURRENT_UART &Uart1

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

#if 0
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





#define FALSE_LED_ON() GPIOB->BSRR = GPIO_Pin_12;
#define FALSE_LED_OFF() GPIOB->BRR = GPIO_Pin_12;
#define PASS_LED_ON() 	GPIOB->BSRR = GPIO_Pin_13;
#define PASS_LED_OFF()	GPIOB->BRR = GPIO_Pin_13;
#define RUNING_LED_ON()	GPIOB->BSRR = GPIO_Pin_11;
#define RUNING_LED_OFF()	GPIOB->BRR = GPIO_Pin_11;

void led_pin_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	FALSE_LED_OFF();
	PASS_LED_OFF();
	RUNING_LED_OFF();
}







#define SW_INTERVAL_COUNT  10  // 10ms���һ��GPIO�ڣ� 10�ξ���100ms, ��������
#define SW_INTERVAL_MS 20
systick_time_t sw_timer;
volatile struct switcher start_key,dual_key, connect_key;


void switch_init()
{
	switcher_init( &start_key, SW_INTERVAL_COUNT, 1 , 0 , GPIOB, GPIO_Pin_0, NULL , NULL);
	switcher_init( &dual_key, SW_INTERVAL_COUNT, 1, 0 , GPIOB, GPIO_Pin_1 , NULL, NULL);
	switcher_init( &connect_key, SW_INTERVAL_COUNT, 1, 0 , GPIOA, GPIO_Pin_5 , NULL, NULL);
	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &start_key );
	switcher_interval_check( &dual_key );
	switcher_interval_check( &connect_key );
}










/*
����ʱ�Ǳ�������9600,������������
ÿ̨�豸����ʱͨѸ��ַ����0x00

A:���ζ�ȡ�豸����(�豸���յ��������,����������1�����ݺ�ص��ȴ�״̬)
0x88+0xAE+�豸��ַ+0x11  ʵ��88AE0011

B:������ȡ�豸����(�豸���յ��������,����ÿ��6����ٶ�������������������)
0x88+0xAE+�豸��ַ+0x21

C:ֹͣ�豸�������,������ܵ�ǰ����������һ̨�豸,����ֹͣ���
0x88+0xAE+�����ֽ�+0x01

D:�豸���ظ�����(PC)���ݸ�ʽ
0xFA+0xFB+�豸��ַ+�����ֽ�0+�����ֽ�1+�����ֽ�2+�����ֽ�3+У���ֽ�
������4���ֽ����(ע��:������ǵ��ֽں�ϰ�����Ƿ���),���պ���Ҫת���ɸ��������ܶ�Ӧ�豸��ʾ����.У���ֽ���4�������ֽ���ӵĽ��(��λ��ȥ)
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
	return 0;
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

int miniMeter_getCurrent( float *A)
{
	unsigned char packget[8];
	int i,count,res;
	
	res = 0;

	count = Uart_Get( METER_UART , packget, sizeof( packget ) );
	if( count == 0 ) return res;
	
	for( i=0; i< count ; i++){
		if( 1 == miniMeterCoder_prase( &currentMeter , packget[i] ) ){
			//sprintf(buffer,"%3.5fV",currentMeter.value);
			//userStation_log(buffer);
			//_LOG( "service_getCurrent: %.5fV" , currentMeter.value);
			res = 1;
			*A = currentMeter.value;
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
		if( 1 == miniMeter_getCurrent( &A) ){
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
	static unsigned char tag = 0;
	char buffer[16];
	if( systick_check_timer( &heart_led_timer ) ){
		GPIO_WriteBit( GPIOC, GPIO_Pin_13 , led);
		led = led==Bit_RESET ? Bit_SET: Bit_RESET ;
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
			//PASS_LED_ON();
			if( buffer[1] == '1'){ 
				PASS_LED_ON();
			}else PASS_LED_OFF();
			USB_TxWrite(buffer,1);
		}else if( buffer[0] == '2' ){
			//sw_on_tag = 0;
			if( buffer[1] == '1'){ 
				RUNING_LED_ON();
			}else RUNING_LED_OFF();
			USB_TxWrite(buffer,1);
		}else if( buffer[0] == '3' ){
			//sw_on_tag = 0;
			//FALSE_LED_ON();
			if( buffer[1] == '1'){ 
				FALSE_LED_ON();
			}else FALSE_LED_OFF();
			USB_TxWrite(buffer,1);
		}
}







volatile int server_step = 0;
volatile int server_testerror = 0;
volatile int server_A_over_count = 0;

void server_init()
{
	miniMeter_init();
	miniMeter_start();
	
}


void server_even()
{
	float A;
	
	//check the current in realtime
	if( 1 == miniMeter_getCurrent( &A ) )
	{
		_LOG("Current:%f\n",A);
		if( A >= 0.80 ){
			server_A_over_count ++;
			if( server_A_over_count > 6 ){
				FALSE_LED_ON();
				PASS_LED_OFF();
				server_testerror = 1;
			}
		}
	}
	
	
	//check the testing loop 
	if( start_key.state != start_key.press_level )
	{
		server_step = 0;
		server_testerror = 0;
		server_A_over_count = 0;
		RUNING_LED_OFF();
		FALSE_LED_OFF();
		PASS_LED_OFF();
		return;
	}
	RUNING_LED_ON();
	
	switch( server_step )
	{
		case 0:
		{
			//detect motor single drive back 
			
		}break;
		
		case 1:
		{
			//detect motor dual  drive back
			
		}break;
		
		case 2:
		{
			//detect all motor drive forward and back ; and finish
			
		}break;
		
		default:
		{
			_LOG("error: unknow step\n");
		}break;
		
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
	led_pin_config();
	switch_init();
	heart_led_init();
	server_init();

}





void app_event()
{

	heart_led_even();
	cmd_even();
	switch_even();
	server_even();

}

#endif