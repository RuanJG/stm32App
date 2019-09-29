#include "main_config.h"
#include "stm32f10x_conf.h"
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
#include "i2c.h"


#if BOARD_STUSB4500_PROGRAM_BOARD

#include "stusb4500_nvm.h"

volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)
#define _LOGI(X...) if( debug_en ) { printf("Log: "); printf(X);}




Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;


#define CONSOLE_UART &Uart1
#define CONSOLE_CMD_MAX_LEN 32
unsigned char cmd_buffer[CONSOLE_CMD_MAX_LEN];



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
	
#if 0
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

#if 0
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
	
#if 0
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


#if 0
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
	console_cmd_config( cmd_buffer, CONSOLE_CMD_MAX_LEN);
}



#define I2C_GPIO_USING 0


#define NVM_PROGRAM_LED(X) GPIO_WriteBit( GPIOC, GPIO_Pin_13 , (X==1?0:1) )
#define NVM_RESET(X) GPIO_WriteBit( GPIOA, GPIO_Pin_5 , X )

void nvm_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	NVM_RESET(0);
	NVM_PROGRAM_LED(0);
}


HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataR ,uint16_t Length)
{
  //return  HAL_I2C_Mem_Read(hi2c[Port],(I2cDeviceID_7bit<<1), Address ,AddressSize, DataR, Length ,2000); //STUSBxx_DEVICEID_7BIT , 1 byte addr
	int reg = Address;
	int result;
	
	
#if I2C_GPIO_USING
	result = gpio_i2c_read_regs( STUSBxx_DEVICEID_7BIT , (unsigned char *) &reg, 1 , DataR , Length );
#else
	result = stm32f10x_i2c_master_sync_read(I2C2 , STUSBxx_DEVICEID_7BIT, (unsigned char *) &reg, 1 , DataR , Length) ;
#endif
	if( result == Length){
		#if 0
		printf("Read reg 0x%02x :", reg);
		for( int i=0; i< Length; i++){
			printf("0x%02x,",DataR[i]);
		}
		printf("\r\n");
		#endif
		return HAL_OK;
	}else{
		printf("bsp I2C read error %d\n", result);
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataW ,uint16_t Length)
{
  //return  HAL_I2C_Mem_Write(hi2c[Port],(I2cDeviceID_7bit<<1), Address ,AddressSize, DataW, Length, 2000 ); // unmask all alarm status
	int reg = Address;
	int result;

#if I2C_GPIO_USING
	//if( stm32f10x_i2c_master_sync_write(I2C2 , STUSBxx_DEVICEID_7BIT, (unsigned char *) &reg, 1 , DataW , Length)  == Length){
	result = gpio_i2c_write_regs( STUSBxx_DEVICEID_7BIT , (unsigned char *) &reg, 1 , DataW , Length );
#else
	result = stm32f10x_i2c_master_sync_write(I2C2 , STUSBxx_DEVICEID_7BIT, (unsigned char *) &reg, 1 , DataW , Length);
#endif
	if( result == Length){
		return HAL_OK;
	}else{
		printf("bsp I2C write error %d\n", result);
		return HAL_ERROR;
	}
}

void board_Reset_Stusb4500()
{
	NVM_RESET(1);
	systick_delay_ms(1);
	NVM_RESET(0);
	systick_delay_ms(200);
}














systick_time_t sw_timer;
volatile struct switcher tpButton;

void switch2_key_release_handler()
{
	NVM_PROGRAM_LED(1);

	if( 0 != stusb4500_program_NVM() ) {
		_LOGI( "STUSB4500 Program NVM Failed \n");
	}
	//stusb4500_check_id();

	NVM_PROGRAM_LED(0);
}

void switch_init()
{
	switcher_init( &tpButton, 5, 1 , 0 , GPIOB, GPIO_Pin_14, GPIO_Mode_IN_FLOATING, NULL , switch2_key_release_handler);
	systick_init_timer( &sw_timer, 10 );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	switcher_interval_check( &tpButton );
}


void cmd_even()
{
	unsigned char i, len;
	
	len = console_cmd_check();
	
	if( len == 0 ) return;
	
	if( len == 1 && cmd_buffer[1] == 'r' )
	{
		if( 0 != stusb4500_read_NVM() ) {
			_LOGI("stusb4500_read_NVM failed\n");
		}else{
			_LOGI("stusb4500_read_NVM OK\n");
		}
	}
	if( len == 1 && cmd_buffer[1] == 'p' )
	{
		if( 0 != stusb4500_program_NVM() ){
			_LOGI("stusb4500_program_NVM failed\n");
		}else{
			_LOGI("stusb4500_program_NVM OK\n");
		}
	}
	if( len == 1 && cmd_buffer[1] == 'i' )
	{
		if( 0 != stusb4500_check_id() ){
			_LOGI("stusb4500_check_id failed\n");
		}else{
			_LOGI("stusb4500_check_id OK\n");
		}
	}

	//gpio_i2c_read_regs

	//stm32f10x_i2c_master_sync_read(I2C2 , 0x28, unsigned char regAddr, unsigned char *data, int size );

}



void app_init()
{
	Uart_USB_SWJ_init();
	switch_init();
	nvm_init();
	
#if I2C_GPIO_USING
	gpio_i2c_init(GPIOB , GPIO_Pin_11, GPIOB , GPIO_Pin_10 );
#else
	stm32f10x_i2c_master_sync_init(I2C2, GPIOB , GPIO_Pin_11, GPIOB , GPIO_Pin_10, 10000);
#endif

	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(1000);
		while(1);
	}
	
	_LOGI("Init OK");
}


void app_event()
{
	cmd_even();
	switch_even();
}

#endif //BOARD_CSWL_LED_MONITOR
