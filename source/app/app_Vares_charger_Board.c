#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "systick.h"
#include "switcher.h"
#include "i2c.h"

#if BOARD_VARES_CHARGER_BOARD

volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)
#define _LOGI(X...) if( debug_en ) { printf("Log: "); printf(X);}




Uart_t Uart1;

#define CONSOLE_UART &Uart1
#define CONSOLE_CMD_MAX_LEN 32
unsigned char cmd_buffer[CONSOLE_CMD_MAX_LEN];



void Uart_USB_SWJ_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	//Uart_Configuration (&Uart1, USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_Cmd(USART1, DISABLE);
	
	USART_StructInit (&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;  
	USART_Init(USART1, &USART_InitStructure); 
	
	USART_Cmd(USART1, ENABLE);	 
	
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	
}

int fputc(int ch, FILE *f)
{
	//TODO lock ?
	
	while( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
	USART_SendData(USART1, ch);
	
	return (ch);
}






#define LED_RED(X) GPIO_WriteBit( GPIOB, GPIO_Pin_12 , X )
#define LED_GREEN(X) GPIO_WriteBit( GPIOB, GPIO_Pin_13 , X )
#define CHARGER_EN(X) GPIO_WriteBit( GPIOA, GPIO_Pin_1 , (X==0 ? 1:0) )

systick_time_t led_timer;
volatile int led_indicate;
volatile int led_flash_id = 0;

void misc_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);							  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	LED_RED(0);
	LED_GREEN(0);
	CHARGER_EN(0);
	
	systick_init_timer( &led_timer, 200 );
	led_indicate = 0;
	led_flash_id = 0;
	//led_flash_id = 1;
}

void led_flash_even()
{
	if( led_indicate == 2 ) return ;
	
	if( systick_check_timer(&led_timer ) )
	{
		if( 0 != (led_flash_id & 0x01) )LED_GREEN(led_indicate);
		if( 0 != (led_flash_id & 0x02) )LED_RED(led_indicate);
		led_indicate = (led_indicate==0 ? 1:0 ) ;
	}
}
void led_flash_en(int en)
{
	if( en == 1 ) led_indicate = 0;
	else led_indicate = 2;
}







systick_time_t sw_timer;
volatile struct switcher tpButton;
volatile struct switcher irButton;
volatile int charger_en =0 ;
volatile int ircount=0;

void switch_key_release_handler()
{
	if( charger_en == 0 ){
		charger_en = 1;
		_LOGI("Button Pressed , Start charging \n");
	}else{
		charger_en = 0;
		_LOGI("Button Pressed , Stop charging \n");
	}
	
	LED_RED(charger_en);
	CHARGER_EN(charger_en);
}

void ir_key_release_handler()
{
	if( charger_en == 0 ){
		charger_en = 1;
		_LOGI("IR Triggered , Start charging \n");
	}else{
		charger_en = 0;
		_LOGI("IR Triggered , Stop charging \n");
	}
	
	LED_RED(charger_en);
	CHARGER_EN(charger_en);
}

void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line10);
		ircount++;
		//USART_SendData(USART1, '0');
		//LED_GREEN(1);
	}
}

void switch_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	
	
#if 0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	switcher_init( &tpButton, 20, 1 , 0 , GPIOA, GPIO_Pin_0, GPIO_Mode_IN_FLOATING, NULL , switch_key_release_handler);
	switcher_init( &irButton, 5, 1 , 0 , GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, NULL , ir_key_release_handler);
	systick_init_timer( &sw_timer, 10 );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	switcher_interval_check( &tpButton );
	switcher_interval_check( &irButton );
	
#if 0
	if( ircount > 6 ){
		ircount = 0;
		ir_key_release_handler();
	}else{
		ircount = 0;
	}
#endif
	
}




void p9242_status_led_flash(unsigned char status1, unsigned char status2)
{
	int i=0;
	
	
	if( status1 == 9 )//charging
	{
		led_flash_id |= 1;
		led_flash_en(1);
	}else{
		LED_GREEN(0);
		led_flash_id &= 0xE;
		if( status2 == 2 ) //full
		{
			LED_GREEN(1);
		}else{
			LED_GREEN(0);
		}
	}
			
	if( status2 ==0 ) return;
	
	LED_RED(0);
	systick_delay_ms(200);
	
	for( i=0; i< status2; i++ )
	{
		LED_RED(1);
		systick_delay_ms(50);
		LED_RED(0);
		systick_delay_ms(50);
	}
	
	systick_delay_ms(200);
	LED_RED(1);
}



#define USE_GPIO_I2C 0

int p9242_read_reg( unsigned short reg , unsigned char *data , int size )
{
	unsigned short regp = reg;
	
#if USE_GPIO_I2C
	return gpio_i2c_read_regs( 0x61, (unsigned char *) &regp, 2 ,data , size );
#else
	return stm32f10x_i2c_master_sync_read( I2C1 , 0x61 , (unsigned char *)&regp ,2, data, size );
#endif
	
}


systick_time_t i2c_timer;
systick_time_t p9242_timer;
int p9242_check_id  = 0;

void i2c_init()
{
#if USE_GPIO_I2C
	gpio_i2c_init(GPIOB , GPIO_Pin_7, GPIOB , GPIO_Pin_6 );
#else
	stm32f10x_i2c_master_sync_init(I2C1, GPIOB , GPIO_Pin_7, GPIOB , GPIO_Pin_6, 50000);
#endif
	systick_init_timer( &i2c_timer, 800 );
	systick_init_timer( &p9242_timer, 10);
}

void i2c_even()
{
	unsigned short reg;
	unsigned char data[8];
	int res;
	
	if( charger_en == 0 ) return ;
	
	if( systick_check_timer( &p9242_timer ) ){
		reg = 0x06E0;
		res = p9242_read_reg( reg , data, 2 );
		if( 2 == res )
		{
			p9242_status_led_flash(data[0],data[1]);
		}
	}
	if( systick_check_timer( &i2c_timer ) ){
		
		if( p9242_check_id != 1 )
		{
			reg = 0x0104;
			res = p9242_read_reg( reg , data , 2 );
			if( 2 == res )
			{
				if( data[0] == 0x42 && data[1] == 0x92 ){
					p9242_check_id = 1;
					_LOGI("I2C: read id 0x%02x successfully \n", (data[0] | (data[1]<<8)) );
				}else{
					_LOGI("I2C: read id = 0x%02x , failed\n", (data[0] | (data[1]<<8)) );
				}
			}else{
				_LOGI("I2C: read id error , %d \n",res);
			}
		}
		
		//status
		reg = 0x06E0;
		res = p9242_read_reg( reg , data, 6 );
		if( 6 == res )
		{
			_LOGI("I2C: read status = %d,%d ; Icoil = %d mA ; Vcoil = %d mV \n", data[0],data[1], (data[2]|(data[3]<<8)) ,  (data[4]|(data[5]<<8)));

		}else{
			_LOGI("I2C: read status error, %d \n",res);
		}
		
		//tempture
		reg = 0x06E8;
		res = p9242_read_reg( reg , data, 2 );
		if( 2 == res  )
		{
			_LOGI("I2C: read tempture = %d \n", (data[0]|(data[1]<<8)) );
		}else{
			_LOGI("I2C: read tempture error , %d \n",res);
		}
		
	}
}





void app_init()
{
	misc_gpio_init();
	Uart_USB_SWJ_init();
	switch_init();
	i2c_init();

	if( SystemCoreClock != 24000000 ){
		_LOGE("CLK init error , clk = %d",SystemCoreClock);
		systick_delay_ms(1000);
		while(1);
	}
	
	for ( int i= 0 ; i< 3; i++ ){
		LED_RED(1);
		systick_delay_ms(100);
		LED_RED(0);
		systick_delay_ms(200);	
	}
	
	_LOGI("Init OK");
	
	switch_key_release_handler();
	systick_delay_ms(1000);
}


void app_event()
{

	switch_even();
	i2c_even();
	led_flash_even();

}

#endif //BOARD_CSWL_LED_MONITOR
