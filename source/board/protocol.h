#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdint.h>

#define PACKET_START         0xAC
#define PACKET_END           0xAD
#define PACKET_ESCAPE        0xAE
#define PACKET_ESCAPE_MASK   0x80



/*********************** port-tag *********************************
| bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
|   0  | next |    port type       |    port number     |


next:        0 for current device, 1 for next device.
port type:   0 for UART, 1 for CAN ; 
port number: 0 for the first device, 1 for the second device ...


Frame: {
	[PACKET_START] 
	[port-tag] 
	( if(port-tag.port-type==CAN)[canid_HByte][canid_LByte] )
	data[N]{ 
		( if( data is for next device ) [port-tag]
		( if(data.prot-tag.port-type==CAN)   [canid_HByte][canid_LByte] )
		data[M]{...}
	} 
	[PACKET_END]
}
*********************************************************/
#define TRANSFER_DONE        0x00
#define TRANSFER_NEXT        0x01

#define PORT_USART           0x00
#define PORT_CAN             0x01

#define MAX_PACKET_SIZE      128



typedef struct _protocol_t {
	unsigned char data[MAX_PACKET_SIZE*2+3];
	unsigned char error_count;
	unsigned char ok_count;
	unsigned char decode_rate;
	unsigned int index;
	int len;
	unsigned char inited;
}protocol_t;

void protocol_init(protocol_t * coder);
/*
encoder data into coder, reutrn 1 ok 0 false;
*/
int protocol_encode(protocol_t * coder, unsigned char* data, int len);
/*
parse a byte for protocol coder
return 0 : no packget reciver
return 1 decoder packget ok , len is coder->len; data is coder->data
*/
int protocol_parse(protocol_t * coder, unsigned char c );

/*
* decode a packget in data[]
* success , return packget len
* false ,   return -1
*/
int protocol_decode(uint8_t *data, uint32_t size);








#endif
