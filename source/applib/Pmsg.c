#include "Pmsg.h"


Uart_t *pUart;
PMSG_transmit_handler_type _PMSG_sendbytes_handler;
PMSG_transmit_handler_type _PMSG_readbytes_handler;
PMSG_msg_handler_type _PMSG_msg_handler;
protocol_t PMSG_decoder;
protocol_t PMSG_encoder;
PMSG_msg_t PMSG_msg;

//the lowest send bytes function
void _PMSG_send_bytes( unsigned char *data, int len)
{
	if( pUart != NULL ){
		Uart_Put(pUart, data, len);
	}
	if( _PMSG_sendbytes_handler != NULL ){
		_PMSG_sendbytes_handler(data, len);
	}
}

// check one byte for msg, if get one , fill in msg and return 1, else 0
int PMSG_parse_byte( unsigned char data , PMSG_msg_t *msg )
{
	if( 1 == protocol_parse( &PMSG_decoder , data ) )
	{
		msg->tag = PMSG_decoder.data[0];
		if( PMSG_decoder.len > 1 ){
			msg->data_len = PMSG_decoder.len-1;
			memcpy( msg->data, PMSG_decoder.data+1, msg->data_len);
		}else{
			msg->data_len = 0;
		}
		
		return 1;
	}
	return 0;
}



// read data  and if get one , fill in msg and return 1, else 0
int PMSG_receive_msg( PMSG_msg_t *msg )
{
	unsigned char val;
	int len, i;
	
	if( _PMSG_readbytes_handler != NULL )
	{
		for( i=0 ;i < PROTOCOL_MAX_PACKET_SIZE ; i++)
		{
			if( 1 == _PMSG_readbytes_handler( &val,1 ) ){
				if( 1 == PMSG_parse_byte( val, msg ) )
				{
					return 1;
				}
			}else{
				break;
			}
			
		}
	}
	
	if( pUart != NULL  )
	{
		for( i=0 ;i < PROTOCOL_MAX_PACKET_SIZE ; i++)
		{
			if( 1 == Uart_Get(pUart,&val,1 ) ){
				if( 1 == PMSG_parse_byte( val, msg ) )
				{
					return 1;
				}
			}else{
				break;
			}
			
		}
	}
	return 0;
}

int PMSG_send_msg( unsigned char tag, unsigned char *data, int len)
{
	unsigned char packget[PROTOCOL_MAX_PACKET_SIZE];
	
	if( len > (PROTOCOL_MAX_PACKET_SIZE - 1) ) return 0;
	
	packget[0] = tag;
	memcpy( &packget[1] , data, len);
	if( 0 == protocol_encode( &PMSG_encoder, packget, len+1) ) return 0;
	_PMSG_send_bytes( PMSG_encoder.data, PMSG_encoder.len );
	return 1;
}


// init , can use uart or customed send and receive interface , if no avaliable , set NULL 
void PMSG_init( Uart_t *uart , PMSG_msg_handler_type msg_func, PMSG_transmit_handler_type sendbytes_func , PMSG_transmit_handler_type  readbytes_func)
{
	pUart = uart;
	_PMSG_sendbytes_handler = sendbytes_func;
	_PMSG_readbytes_handler = readbytes_func;
	_PMSG_msg_handler = msg_func;
	protocol_init( &PMSG_decoder );
	protocol_init( &PMSG_encoder );
}


// user can ignore this even() , you can use PMSG_receive_msg() or PMSG_parse_byte() to get the msg and then deal with them yourselve
void PMSG_even()
{
	
	if( _PMSG_msg_handler == NULL ) return;
	
	if( 1 == PMSG_receive_msg( &PMSG_msg ) )
	{
		_PMSG_msg_handler( PMSG_msg );
	}
}



