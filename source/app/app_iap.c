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
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;


// coders
protocol_t decoder;
protocol_t encoder;


// iap_lost_ms if no data receive in iap_lost_ms , jump to app
unsigned int _iap_lost_ms_max = 0;
volatile unsigned int iap_lost_ms;

// if flash has programed , iap_has_flashed_data will set 1
volatile int iap_has_flashed_data = 0;

// inited flag
volatile char iap_inited_flag = 0;



// timer , check timeout
systick_time_t timeout_timer;



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


int is_app_flashed()
{
	#if 1
	if (((*(__IO uint32_t*)IAP_APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
		return 1;
	else
		return 0;
	
	#else
	return 1;
	#endif
}

int get_iap_tag()
{
	return *(__IO int*) IAP_TAG_ADDRESS;
}

int set_iap_tag(int tag)
{//1 ok, 0 flase
	int timeout = 10;
	char res, ntag;
	
	
	FLASHStatus = FLASH_COMPLETE;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	res = 0;
	timeout = 10;
	while( timeout-- > 0)
	{
		FLASHStatus = FLASH_ErasePage(IAP_TAG_ADDRESS);
		if( FLASHStatus == FLASH_COMPLETE ){
			res = 1;
			break;
		}
	}
	
	if( res == 1 )
	{
		res = 0;
		timeout = 10;
		while( timeout-- > 0)
		{
			FLASHStatus = FLASH_ProgramWord(IAP_TAG_ADDRESS,tag) ;//FLASH_ProgramOptionByteData(IAP_TAG_ADDRESS,tag);
			if( FLASHStatus == FLASH_COMPLETE ){
				res = 1;
				break;
			}
		}
	}

	FLASH_Lock();
	
	if( res == 1)
	{
		ntag = get_iap_tag();
		if( ntag != tag )
			res = 0;
	}
	
	return res;
}

int is_iap_tag_set()
{
	// force into iap
	int tag;
	tag = get_iap_tag();
	if( tag == IAP_TAG_UPDATE_VALUE)
		return 1;
	else
		return 0;
}
int clean_iap_tag()
{
	return set_iap_tag(0);
}


void iap_reset()
{
		//关中断
	__set_PRIMASK(1);
	__set_FAULTMASK(1);
	
	// vectreset reset cm3 but other extern hardword
	//*((uint32_t*)0xE000ED0C) = 0x05FA0001;
	// sysresetReq reset all ic hardword system
	*((uint32_t*)0xE000ED0C) = 0x05FA0004;
	while(1);
}


typedef void(*iapFunction)(void);
void iap_jump_to_app()
{
	iapFunction Jump_To_Application;
	uint32_t JumpAddress;
	
	//return;
	if( is_iap_tag_set() )
		clean_iap_tag();
	
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

int iap_receive_data(unsigned char * data, int size)
{
	int count;
	unsigned char *pointer;
	
	count = 0;
	pointer = data;
	
#if IAP_PORT_UART
	count += Uart_get(pointer+count,size-count);
	if ( count >= size ) return count;
#endif
	
#if IAP_PORT_CAN1 
	count += Can1_get(pointer+count,size-count);
	if ( count >= size ) return count;
#endif
	
#if IAP_PORT_USB 
	count += USB_RxRead(pointer+count,size-count);
	if ( count >= size ) return count;
#endif
	
	return count;
}


void iap_send_packget( unsigned char *data , unsigned int size)
{
	protocol_encode(&encoder,data,size);
	
#if IAP_PORT_UART
		Uart_send(encoder.data,encoder.len);
#endif
#if  IAP_PORT_CAN1
		Can1_Send(encoder.data, encoder.len);
#endif
	
#if IAP_PORT_USB 
	USB_TxWrite(encoder.data, encoder.len);
#endif
	
}



void answer_ack_ok(unsigned char id)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_OK;
	data[2] = id;
	iap_send_packget(data , 3);
}
void answer_ack_false(char error)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_FALSE;
	data[2] = error;
	iap_send_packget(data , 3);
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
	
	iap_has_flashed_data = 1;
	
	return 1;
	
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
				systick_delay_us(600000);
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
			systick_delay_us(50000);
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
	if( iap_inited_flag ==1 && 1 == protocol_parse( &decoder,c) )
	{
		iap_lost_ms = 0;
		handle_packget(decoder.data,decoder.len);
	}
	
}



/*
****************************************   irq call back   *************

void uart_receive_event(unsigned char c)
{
	if( 1==IAP_PORT_UART)
		iap_parase(c);
}



void can1_receive_event(CanRxMsg *msg)
{

	int i;
	
	if( 1 == IAP_PORT_CAN1){
		for( i=0 ; i< msg->DLC; i++)
		{
			iap_parase(msg->Data[i]);
		}
	}

}

**************************************************************************
*/





void iap_init()
{
	
	protocol_init(&encoder);
	protocol_init(&decoder);
	
	systick_init_timer( &timeout_timer, 10);
	
	iap_inited_flag = 1;
	
	_iap_lost_ms_max = 60*1000 ;
	
}



void iap_loop()
{
	int i, count;
	//check iap_lost_ms, it will be fill 0 in can1 or uart receive event
	
	count = iap_receive_data(uartRxBuffer,UART_RX_BUFFER_SIZE);
	for(i=0; i< count ; i++)
		iap_parase(uartRxBuffer[i]);
	
	//if( count > 0 ) USB_TxWrite(uartRxBuffer, count);
	
	if( systick_check_timer( &timeout_timer ) == 1)
	{
		if( iap_lost_ms  >= _iap_lost_ms_max )
		{
			iap_jump_to_app();
		}else{
			iap_lost_ms += 10;
		}
	}	
}













#endif //BOARD_IAP
