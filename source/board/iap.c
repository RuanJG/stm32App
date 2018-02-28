#include "stm32f10x_conf.h"
#include "protocol.h"
#include "iap.h"
#include "uart.h"
#include "can1.h"
#include "hw_config.h"
#include "systick.h"


//program step tag
#define PROGRAM_STEP_NONE 0
#define PROGRAM_STEP_GET_START 1
#define PROGRAM_STEP_GET_DATA 2
#define PROGRAM_STEP_END 3


//use for program flash
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;



// coders
protocol_t iap_decoder;
protocol_t iap_encoder;


unsigned char iap_port_type=0;
Uart_t *iapUart;


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




void iap_config_vect_table()
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, IAP_APP_ADDRESS-IAP_FIRMWARE_ADRESS);
}







void iap_send_packget( unsigned char *data , unsigned int size)
{
	int i;
	protocol_encode(&iap_encoder,data,size);
	//TODO
	if( iap_port_type ==1 )
	{//use uart
		Uart_Put(iapUart, iap_encoder.data, iap_encoder.len);
	}
	
	if( iap_port_type == 2 )
	{//use can1
		Can1_Send(0x10,iap_encoder.data,iap_encoder.len);
	}
	
	if( iap_port_type == 3 )
	{
		USB_TxWrite_Sync(iap_encoder.data, iap_encoder.len);
	}
	
}



void answer_ack_false(char error)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_FALSE;
	data[2] = error;
	iap_send_packget(data , 3);
}

void answer_ack_restart(char error)
{
	unsigned char data[4];
	data[0] = PACKGET_ACK_ID;
	data[1] = PACKGET_ACK_RESTART;
	data[2] = error;
	iap_send_packget(data , 3);
}



//__weak void main_deinit();
void jump_by_reset()
{
	//main_deinit();
	
		//¹ØÖÐ¶Ï
	__set_PRIMASK(1);
	__set_FAULTMASK(1);
	
	// vectreset reset cm3 but other extern hardword
	//*((uint32_t*)0xE000ED0C) = 0x05FA0001;
	// sysresetReq reset all ic hardword system
	*((uint32_t*)0xE000ED0C) = 0x05FA0004;
	
	
	while(1);
}

void iap_jump()
{
		if(1== set_iap_tag(IAP_TAG_UPDATE_VALUE) ){
				#if IAP_PORT_USB
				answer_ack_restart(0); // ask for iapApplication reOpen uart 
				systick_delay_us(5000);
				USB_Deinit();
				#endif
				jump_by_reset();
		}else{
				answer_ack_false(PACKGET_ACK_FALSE_PROGRAM_ERROR);
		}
}



static void handle_packget(unsigned char *pkg, unsigned int len)
{
	unsigned char id = pkg[0];
	//unsigned char *data = &pkg[1];
	
	if( len != 2 ) return;
	switch ( id ){
		case PACKGET_START_ID:
			iap_jump();
			break;
			
		case PROGRAM_STEP_GET_DATA:
			break;
		
		case PACKGET_END_ID:
			break;
	}
	
}



//this function call by uart or can1  receiver
__STATIC_INLINE void iap_parase(unsigned char c)
{
	if( 1 == protocol_parse( &iap_decoder,c) )
	{
		handle_packget(iap_decoder.data,iap_decoder.len);
	}
	
}


void iap_uart_receive_handler(unsigned char c)
{
	iap_parase(c);
}


void iap_init_in_uart(Uart_t *uart)
{
	protocol_init( &iap_decoder );
	protocol_init( &iap_encoder );
	iapUart = uart;
	iapUart->read_cb = iap_uart_receive_handler;
	iap_port_type = 1;
}



//************************************************  usb interface
void iap_usb_receive_handler(unsigned char c)
{
	iap_parase(c);
}


void iap_init_in_usb()
{
	protocol_init( &iap_decoder );
	protocol_init( &iap_encoder );
	
	iap_port_type = 3;
}







void iap_can_receive_handler(unsigned char c )
{
	iap_parase(c);
}


void iap_init_in_can1()
{
	protocol_init( &iap_decoder );
	protocol_init( &iap_encoder );
	iap_port_type = 2;
}

