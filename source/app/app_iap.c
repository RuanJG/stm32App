#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "iap.h"
#include "hw_config.h"
#include "flashRW.h"
#include "Pmsg.h"


#if BOARD_IAP



//program step tag
#define PROGRAM_STEP_NONE 0
#define PROGRAM_STEP_GET_START 1
#define PROGRAM_STEP_GET_DATA 2
#define PROGRAM_STEP_END 3
volatile unsigned char program_step = PROGRAM_STEP_NONE;


//buffer for store file data
#define PROGRAM_BUFF_SIZE FLASH_PAGE_SIZE
unsigned char program_buff[FLASH_PAGE_SIZE];
volatile unsigned int program_buff_index = 0;


//data packget seq 
unsigned char program_data_frame_seq = 0;

//use for program flash
static volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

// coders
protocol_t decoder;
protocol_t encoder;

Uart_t *iapUart1;

int iap_app_port_type;

// if flash has programed , iap_has_flashed_data will set 1
volatile int iap_has_flashed_data = 0;

int IPS_US ;

// uart rx buffer
#define UART_RX_BUFFER_SIZE 8
unsigned char uartRxBuffer[UART_RX_BUFFER_SIZE];


void _memcpy(void *dst, const void *src, unsigned int n)
{
	const unsigned char *p = src;
	unsigned char *q = dst;

	while (n--) {
		*q++ = *p++;
	}
}
void _memset(void *dst, unsigned char data, unsigned int n)
{
	unsigned char *q = dst;
	while (n--) {
		*q++ = data;
	}
}

void iap_app_delayus(int us)
{
	bsp_loopDelay_us(us);
	
}

void iap_jump_to_app()
{
	iapFunction Jump_To_Application;
	uint32_t JumpAddress;
	
	//return;
	
	bsp_deinit();
	
	// if flash has programed , reset 
	if( iap_has_flashed_data == 1 )
		iap_reset();
	
	/* If Program has been written */
	if ( is_app_flashed())
	{	
		// Set system control register SCR->VTOR , and the app need to set vector too 
		//NVIC_SetVectorTable(NVIC_VectTab_FLASH, (IAP_APP_ADDRESS-IAP_FIRMWARE_ADRESS));
		//jump
		JumpAddress = *(__IO uint32_t*) (IAP_APP_ADDRESS + 4);
		Jump_To_Application = (iapFunction) JumpAddress;
		__set_MSP(*(__IO uint32_t*) IAP_APP_ADDRESS);
		Jump_To_Application(); 
	}
	
}


void iap_jump_to_app_or_deamon()
{
	iapFunction Jump_To_Application;
	uint32_t JumpAddress;

	if ( is_app_flashed())
	{	
		JumpAddress = *(__IO uint32_t*) (IAP_APP_ADDRESS + 4);
		Jump_To_Application = (iapFunction) JumpAddress;
		__set_MSP(*(__IO uint32_t*) IAP_APP_ADDRESS);
		Jump_To_Application(); 
	}else{
		while(1);
	}
	
}

static int iap_receive_data(unsigned char * data, int size)
{
	int count;
	
#if IAP_APP_PORT_UART
	count = Uart_Get( iapUart1 , data, size );
#endif
	
#if IAP_APP_PORT_CAN1 
	count = Can1_get(data, size );
#endif
	 
#if IAP_APP_PORT_USB 
	count = USB_RxRead( data, size );
#endif
	
	return count;
}


static void iap_send_packget( unsigned char *data , unsigned int size)
{
	protocol_encode(&encoder,data,size);
	
#if IAP_APP_PORT_UART
	Uart_Put_Sync(iapUart1, encoder.data,encoder.len);
#endif
#if  IAP_APP_PORT_CAN1
	Can1_Send(0x10,iap_encoder.data,iap_encoder.len);
#endif
	
#if IAP_APP_PORT_USB 
	USB_TxWrite_Sync(encoder.data, encoder.len);
#endif
	
}



static void answer_ack_ok(unsigned char id)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_OK;
	data[2] = id;
	iap_send_packget(data , 3);
}
static void answer_ack_false(char error)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_FALSE;
	data[2] = error;
	iap_send_packget(data , 3);
}

int verify_page(uint32_t page_addr , unsigned char *data, int len)
{
	int i ,index,res;
	uint32_t word, addr;
	__IO int* pageAddr ;
	
	pageAddr = (__IO int*) page_addr;
	res = 1;
	
	for( index = 3, i=0 ; index < len && index < FLASH_PAGE_SIZE ; index+=4,i++)
	{
		word = (data[index]<<24) | (data[index-1]<<16) | (data[index-2]<<8) | data[index-3];
		if( *(pageAddr+i) != word ) 
		{
			res = 0;
			break;
		}
	}
	return res;
}

int program_page_to_flash(uint32_t page_addr , unsigned char *data, int len)
{
	// ok return 1 ; len < page return 0 ; erase error return -1 ;program error return  -2
	int timeout ,index;
	uint32_t word, addr;
	
	if( len < FLASH_PAGE_SIZE ) return 0;

	// Erase Page
	timeout = 0;
	do{
		timeout++;
		if(timeout > 100) return -1;
		FLASHStatus = FLASH_ErasePage(page_addr);
	}while( FLASHStatus != FLASH_COMPLETE );
	
	// Program data
	addr = page_addr;
	for( index = 3 ; index < len && index < FLASH_PAGE_SIZE ; index+=4){
		timeout = 0;
		do{
			timeout++;
			if(timeout > 100) return -2;
			word = (data[index]<<24) | (data[index-1]<<16) | (data[index-2]<<8) | data[index-3];
			FLASHStatus = FLASH_ProgramWord(addr,word);
		}while( FLASHStatus != FLASH_COMPLETE );
		addr+=4;
	}
	
	if( 1 == verify_page( page_addr, data, len ) ){
		iap_has_flashed_data = 1;
		return 1;
	}
	
	return -2;
}




volatile uint32_t program_addr;

void flash_process_init()
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	program_addr = IAP_APP_ADDRESS;
	program_data_frame_seq = 0;
}

void flash_process_end()
{
	FLASH_Lock();
}

void init_program_buff()
{
	//初始化写flash app的操作
	_memset(program_buff,0,PROGRAM_BUFF_SIZE);
	program_buff_index = 0;
	
}
int  flush_program_buff()
{
	//将还没有写到flash的数据写入
	// return 0 ok;
	// return PACKGET_ACK_FALSE_SEQ_FALSE seq error 
	//return PACKGET_ACK_FALSE_ERASE_ERROR / PACKGET_ACK_FALSE_Program_ERROR program error
	int res;
	if( program_buff_index > 0 )
	{
		res = program_page_to_flash(program_addr, program_buff, FLASH_PAGE_SIZE);
		// ok return 1 ; len < page return 0 ; erase error return -1 ;program error return  -2
		if( res == -1){
			return PACKGET_ACK_FALSE_ERASE_ERROR;
		}else if( res == -2){
			return PACKGET_ACK_FALSE_PROGRAM_ERROR;
		}else{
			//ok
			init_program_buff();
			program_addr+= FLASH_PAGE_SIZE;
		}
	}
	return 0;
}

int  handle_data_packget_data(unsigned char *data, int len)
{
//	储存数据，达到1page时，写入, 如果数据序列不符合，要求重发
	// return 0 ok;
	// return PACKGET_ACK_FALSE_SEQ_FALSE seq error 
	//return PACKGET_ACK_FALSE_ERASE_ERROR / PACKGET_ACK_FALSE_Program_ERROR program error
	int i,res;
	unsigned char new_seq;
	
	//check seq
	new_seq = (program_data_frame_seq+1)% PACKGET_MAX_DATA_SEQ;
	if( new_seq != data[0] )
	{
		//error
		answer_ack_false( PACKGET_ACK_FALSE_SEQ_FALSE );
		return -1;
	}
	program_data_frame_seq = new_seq;
	
	// prgram data
	res = 0;
	for( i=1; i< len; i++){
		program_buff[ program_buff_index++ ] =  data[i];
		if( program_buff_index >= FLASH_PAGE_SIZE ){
			res = program_page_to_flash(program_addr, program_buff, FLASH_PAGE_SIZE);
			init_program_buff();
		}
	}
	
	// ok return 1 ; erase error return -1 ;program error return  -2
	if( res == -1){
		answer_ack_false(PACKGET_ACK_FALSE_ERASE_ERROR);
	}else if( res == -2){
		answer_ack_false( PACKGET_ACK_FALSE_PROGRAM_ERROR);
	}else if ( res > 0){
		//ok
		program_addr+= FLASH_PAGE_SIZE;
		answer_ack_ok(PACKGET_DATA_ID);
	}else{
		answer_ack_ok(PACKGET_DATA_ID);
	}
	return 0;
}

void handle_end_packget_data(unsigned char *data, int len)
{
	int res;

	if( program_step == PROGRAM_STEP_GET_DATA ) {
		res = flush_program_buff();
		if( 0 ==  res){
			answer_ack_ok(PACKGET_END_ID);
			if(data[0] == PACKGET_END_JUMP ){
				iap_app_delayus(600000);
				iap_jump_to_app();
			}
		}else {
			answer_ack_false(res);// remote will resend this packget
		}
		flash_process_end();
		program_step = PROGRAM_STEP_NONE;
	}else{
		// if no in program flash step , jump 
		answer_ack_ok(PACKGET_END_ID);
		if(data[0] == PACKGET_END_JUMP )
		{
			iap_app_delayus(50000);
			iap_jump_to_app();
		}
	}
}




void handle_packget(unsigned char *pkg, unsigned int len)
{
	unsigned char id = pkg[0];
	unsigned char *data = &pkg[1];
	
	switch ( id ){
		case PACKGET_START_ID:
			//if( program_step != PROGRAM_STEP_GET_DATA )
			{
				program_step = PROGRAM_STEP_GET_DATA ;
				flash_process_init();
				init_program_buff();
				answer_ack_ok(PACKGET_START_ID);
			}
			break;
			
		case PACKGET_DATA_ID:
			handle_data_packget_data(data,len-1);
			break;
		
		case PACKGET_END_ID:
			if ( len == 2 )
				handle_end_packget_data(data,len-1);
			break;
	}
	
}



//this function call by uart or can1  receiver
__STATIC_INLINE void iap_parase(unsigned char c)
{
	//iap_lost_ms = 0;
	if(1 == protocol_parse( &decoder,c) )
	{
		handle_packget(decoder.data,decoder.len);
	}
}


/*
void iap_app_uart_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IAP_APP_UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IAP_APP_UART_TX_GPIO, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IAP_APP_UART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(IAP_APP_UART_RX_GPIO, &GPIO_InitStructure); 
	
	IAP_APP_UART_PIN_REMAP_FUNC() ;

	Uart_Configuration (&iapUart1, IAP_APP_UARTDEV, IAP_APP_UART_BAUDRATE, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
}
*/





void iap_app_init(void *pridata, int port_type)
{
	if( 0 == is_iap_tag_set() ){
		iap_jump_to_app();
	}else{	
		clean_iap_tag();
	}
	
	protocol_init(&encoder);
	protocol_init(&decoder);
	
//#if IAP_APP_PORT_UART
//	iap_app_uart_init();
//#endif
#if IAP_APP_PORT_UART
	iapUart1 = (Uart_t*)pridata;
#endif
	
	iap_app_port_type = port_type;
	
	IPS_US = 1;//SystemCoreClock /1000000;
}



void iap_app_event()
{
	volatile int i, count;
	
	count = iap_receive_data(uartRxBuffer,UART_RX_BUFFER_SIZE);
	for(i=0; i< count ; i++)
		iap_parase(uartRxBuffer[i]);
}













#endif //BOARD_IAP
