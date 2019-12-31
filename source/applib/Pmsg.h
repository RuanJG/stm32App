#ifndef _PMSG_H_
#define _PMSG_H_


#include "stdio.h"
#include "protocol.h"
#include "stm32f10x.h"
#include "uart.h"



//protocol msg packget  [tag][data]
// tag: high 4 bits is fixed for packget type
//      low 4 bits is for custom
#define PMSG_TAG_IAP		0x00
#define PMSG_TAG_ACK		0x10
#define PMSG_TAG_LOG		0x20
#define PMSG_TAG_DATA		0x30
#define PMSG_TAG_CMD		0x40
#define IS_PMSG_TAG(T,V) (( V & 0xf0 ) == T)


//IAP ID 
#define PMSG_TAG_IAP_START_ID 	(PMSG_TAG_IAP|0x1) //[id-start]data{[0]}
#define PMSG_TAG_IAP_ACK_ID 		(PMSG_TAG_IAP|0x2) //[id-ack]data{[ACK_OK/ACK_FALSE/ACK/RESTART][error code]}
#define PMSG_TAG_IAP_DATA_ID 	(PMSG_TAG_IAP|0x3) //[id-data] data{[seq][]...[]}
#define PMSG_TAG_IAP_END_ID 		(PMSG_TAG_IAP|0x4) //[id-end] data{[stop/jump]}
//data
#define PMSG_IAP_ACK_OK 1
#define PMSG_IAP_ACK_FALSE 2
#define PMSG_IAP_ACK_RESTART 3
#define PMSG_IAP_END_JUMP 1
#define PMSG_IAP_END_STOP 2
#define PMSG_IAP_MAX_DATA_SEQ 200  // new_seq = (last_seq+1)% PACKGET_MAX_DATA_SEQ
// ack error code
#define PMSG_IAP_ACK_FALSE_PROGRAM_ERROR 21 // the send data process stop
#define PMSG_IAP_ACK_FALSE_ERASE_ERROR 22 // the send data process stop
#define PMSG_IAP_ACK_FALSE_SEQ_FALSE 23 // the send data process will restart
// if has ack false , the flash process shuld be restart



typedef struct _PMSG_msg_t{
	unsigned char tag;
	unsigned char* data;
	int data_len;
	unsigned char alldata[PROTOCOL_MAX_PACKET_SIZE];
	int alldata_len;
}PMSG_msg_t;


//transmit func : for sending , return the number sent; for reading , return the number read which data in *data, data len in len
typedef int (*PMSG_transmit_handler_type)( unsigned char *data, int len );
typedef void (*PMSG_msg_handler_type)( PMSG_msg_t msg);


typedef struct _PMSG_t{
	Uart_t *pUart;
	PMSG_transmit_handler_type sendbytes_handler;
	PMSG_transmit_handler_type readbytes_handler;
	PMSG_msg_handler_type msg_handler;
	protocol_t decoder;
	protocol_t encoder;
	PMSG_msg_t msg;
}PMSG_t;


//the lowest send bytes function
void PMSG_send_bytes( PMSG_t *pmsg, unsigned char *data, int len);
//the lowest receive bytes function
int PMSG_receive_bytes( PMSG_t *pmsg, unsigned char *data, int len );
	
// check one byte for msg, if get one , fill in pmsg.msg and return 1, else 0
int PMSG_parse_byte( PMSG_t *pmsg, unsigned char data );

// read data  and if get one , fill in pmsg.msg and return 1, else 0
int PMSG_receive_msg( PMSG_t *pmsg );

int PMSG_send_msg( PMSG_t *pmsg, unsigned char tag, unsigned char *data, int len);
int PMSG_send_msg_no_Tag( PMSG_t *pmsg, unsigned char *data, int len);

// init , can use uart or customed send and receive interface , if no avaliable , set NULL 
void PMSG_init_uart( PMSG_t *pmsg , Uart_t *uart , PMSG_msg_handler_type msg_func );
void PMSG_init_handler(PMSG_t *pmsg, PMSG_msg_handler_type msg_func, PMSG_transmit_handler_type sendbytes_func , PMSG_transmit_handler_type  readbytes_func);
void PMSG_init_uart_irq( PMSG_t *pmsg , Uart_t *uart , PMSG_msg_handler_type msg_func );

// call  PMSG_receive_msg() , if 1 , call msg_func();
void PMSG_even( PMSG_t *pmsg );


#endif





/* ********  usage

//for example 

#define PC_TAG_LOGE  (PMSG_TAG_LOG|0x1)
#define PC_TAG_LOGI  (PMSG_TAG_LOG|0x2)

unsigned char logbuffer[ PROTOCOL_MAX_PACKET_SIZE ];
#define PC_LOG(T, X...) { snprintf( logbuffer, sizeof( logbuffer ), X);  PMSG_send_msg( T , logbuffer, strlen(logbuffer) ); }
#define PC_LOGI(X...) PC_LOG( PC_TAG_LOGI , X)
#define PC_LOGE(X...) PC_LOG( PC_TAG_LOGE , X)


#define PC_TAG_CMD_CAPTURE_EN  (PMSG_TAG_CMD | 0x01 ) // data[0] = 1 auto start, data[0] = 0 stop
#define PC_TAG_CMD_LED_SELECT  (PMSG_TAG_CMD | 0x02 )  // data[0] = led id [1~12], data[1] = status
#define PC_TAG_CMD_CAPTURE_INTERVAL  (PMSG_TAG_CMD | 0x03 )  //data[0] = ms_L, data[1] = ms_H
#define PC_TAG_CMD_TEST (PMSG_TAG_CMD | 0x04 ) 
#define PC_TAG_CMD_SWITCHES_TESTMODE (PMSG_TAG_CMD | 0x05 ) // data[0] = 2,3,4,5 , FF mean off all
#define PC_TAG_CMD_SWITCHES_CHANNEL (PMSG_TAG_CMD | 0x06 ) // data[0] = 0~15 , FF mean off all


#define PC_TAG_DATA_LED_BRIGHTNESS (PMSG_TAG_DATA|0x1)


PMSG_t PC_pmsg;

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
		default:
			PC_LOGE("PC_TAG_CMD_SWITCH:  unknow data");
			break;
	}
}

PMSG_init_uart( &PC_pmsg, PC_UART, PC_msg_handler );
PMSG_even( &PC_pmsg );


*/


