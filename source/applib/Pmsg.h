#ifndef _PMSG_H_
#define _PMSG_H_


#include "stdio.h"
#include "protocol.h"
#include "stm32f10x.h"
#include "uart.h"



//protocol msg packget  [tag][data]
// tag: high 4 bits is fixed for packget type
//      low 4 bits is for custom
#define PMSG_TAG_ACK		0x10
#define PMSG_TAG_LOG		0x20
#define PMSG_TAG_DATA		0x30
#define PMSG_TAG_CMD		0x40
#define IS_PMSG_TAG(T,V) (( V & 0xf0 ) == T)


typedef struct _PMSG_msg_t{
	unsigned char tag;
	unsigned char data[PROTOCOL_MAX_PACKET_SIZE];
	int data_len;
}PMSG_msg_t;

//transmit func : for sending , return the number sent; for reading , return the number read which data in *data, data len in len
typedef int (*PMSG_transmit_handler_type)( unsigned char *data, int len );
typedef void (*PMSG_msg_handler_type)( PMSG_msg_t msg);


//the lowest send bytes function
void _PMSG_send_bytes( unsigned char *data, int len);

// check one byte for msg, if get one , fill in msg and return 1, else 0
int PMSG_parse_byte( unsigned char data , PMSG_msg_t *msg );


// read data  and if get one , fill in msg and return 1, else 0
int PMSG_receive_msg( PMSG_msg_t *msg );

int PMSG_send_msg( unsigned char tag, unsigned char *data, int len);


// init , can use uart or customed send and receive interface , if no avaliable , set NULL 
void PMSG_init( Uart_t *uart , PMSG_msg_handler_type msg_func, PMSG_transmit_handler_type sendbytes_func , PMSG_transmit_handler_type  readbytes_func);

// user can ignore this even() , you can use PMSG_receive_msg() or PMSG_parse_byte() to get the msg and then deal with them yourselve
void PMSG_even();


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

*/


