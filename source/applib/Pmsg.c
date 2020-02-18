#include "Pmsg.h"





//the lowest send bytes function
void PMSG_send_bytes( PMSG_t *pmsg, unsigned char *data, int len)
{
	if( pmsg->sendbytes_handler != NULL ){
		pmsg->sendbytes_handler(data, len);
	}else if(  pmsg->pUart != NULL  ){
		Uart_Put(pmsg->pUart, data, len);
	}else{
		return ;
	}
}
//the lowest receive bytes function
int PMSG_receive_bytes( PMSG_t *pmsg, unsigned char *data, int len )
{
	if( pmsg->readbytes_handler != NULL ){
		return pmsg->readbytes_handler( data, len );
	}else if( pmsg->pUart != NULL  ){
		return Uart_Get(pmsg->pUart, data ,len );
	}else{
		return 0;
	}
}

// check one byte for msg, if get one , fill in msg and return 1, else 0
int PMSG_parse_byte( PMSG_t *pmsg, unsigned char data )
{
	if( 1 == protocol_parse( & pmsg->decoder , data ) )
	{
		memcpy( pmsg->msg.alldata, pmsg->decoder.data, pmsg->decoder.len);
		pmsg->msg.alldata_len = pmsg->decoder.len;
		pmsg->msg.tag = pmsg->msg.alldata[0];
		pmsg->msg.data = &pmsg->msg.alldata[1];
		pmsg->msg.data_len = pmsg->decoder.len-1;
		/*
		pmsg->msg.tag = pmsg->decoder.data[0];
		if( pmsg->decoder.len > 1 ){
			pmsg->msg.data_len = pmsg->decoder.len-1;
			memcpy( pmsg->msg.data, pmsg->decoder.data+1, pmsg->msg.data_len);	
		}else{
			pmsg->msg.data_len = 0;
		}
		*/
		return 1;
	}
	return 0;
}



// read data  and if get one , fill in msg and return 1, else 0
int PMSG_receive_msg( PMSG_t *pmsg )
{
	unsigned char val;
	int len, i;
	
	for( i=0 ;i < PROTOCOL_MAX_PACKET_SIZE ; i++)
	{
		if( 1 == PMSG_receive_bytes( pmsg , &val, 1 ) )
		{
			if( 1 == PMSG_parse_byte( pmsg , val ) )
			{
				return 1;
			}
		}else{
			break;
		}
	}
	return 0;
}

int PMSG_send_msg_no_Tag( PMSG_t *pmsg, unsigned char *data, int len)
{
	if( 0 == protocol_encode( &pmsg->encoder, data, len) ) return 0;
	PMSG_send_bytes( pmsg, pmsg->encoder.data, pmsg->encoder.len );
	return 1;
}


int PMSG_send_msg( PMSG_t *pmsg, unsigned char tag, unsigned char *data, int len)
{
	unsigned char packget[PROTOCOL_MAX_PACKET_SIZE];
	
	if( len > (PROTOCOL_MAX_PACKET_SIZE - 1) ) return 0;
	
	packget[0] = tag;
	memcpy( &packget[1] , data, len);

	return PMSG_send_msg_no_Tag( pmsg, packget, len+1 );
}


void PMSG_irq_callBack_handler(void *pridata, unsigned char data)
{
	PMSG_t *pmsg;
	pmsg = ( PMSG_t *) pridata;
	
	if( pmsg == NULL ) return;
	
	if( 1 == PMSG_parse_byte( pmsg , data ) )
	{
		pmsg->msg_handler( pmsg->msg );
	}
	
}


// init , can use uart or customed send and receive interface , if no avaliable , set NULL 
void PMSG_init_handler( PMSG_t *pmsg , PMSG_msg_handler_type msg_func, PMSG_transmit_handler_type sendbytes_func , PMSG_transmit_handler_type  readbytes_func)
{
	pmsg->pUart = NULL;
	pmsg->sendbytes_handler = sendbytes_func;
	pmsg->readbytes_handler = readbytes_func;
	pmsg->msg_handler = msg_func;
	protocol_init( &pmsg->decoder );
	protocol_init( &pmsg->encoder );
}
// init , normal uart module
void PMSG_init_uart( PMSG_t *pmsg , Uart_t *uart , PMSG_msg_handler_type msg_func )
{
	pmsg->pUart = uart;
	pmsg->sendbytes_handler = NULL;
	pmsg->readbytes_handler = NULL;
	pmsg->msg_handler = msg_func;
	protocol_init( &pmsg->decoder );
	protocol_init( &pmsg->encoder );
}
//init , irq even
void PMSG_init_uart_irq( PMSG_t *pmsg , Uart_t *uart , PMSG_msg_handler_type msg_func )
{
	pmsg->pUart = uart;
	pmsg->sendbytes_handler = NULL;
	pmsg->readbytes_handler = NULL;
	pmsg->msg_handler = msg_func;
	pmsg->pUart->read_cb = PMSG_irq_callBack_handler;
	pmsg->pUart->pridata = pmsg;
	protocol_init( &pmsg->decoder );
	protocol_init( &pmsg->encoder );
}

// user can ignore this even() , you can use PMSG_receive_msg() or PMSG_parse_byte() to get the msg and then deal with them yourselve
void PMSG_even( PMSG_t *pmsg )
{
	
	if( pmsg->msg_handler == NULL ) return;
	
	if( 1 == PMSG_receive_msg( pmsg ) )
	{
		pmsg->msg_handler( pmsg->msg );
	}
}



