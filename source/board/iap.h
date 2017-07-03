#ifndef _IAP_H
#define _IAP_H

#include "uart.h"

//*************************************************    program protocal

//id
#define PACKGET_START_ID 1 //[id-start]data{[0]}
#define PACKGET_ACK_ID 2 //[id-ack]data{[ACK_OK/ACK_FALSE/ACK/RESTART][error code]}
#define PACKGET_DATA_ID 3 //[id-data] data{[seq][]...[]}
#define PACKGET_END_ID 4 //[id-end] data{[stop/jump]}
//data
#define PACKGET_ACK_OK 1
#define PACKGET_ACK_FALSE 2
#define PACKGET_ACK_RESTART 3
#define PACKGET_END_JUMP 1
#define PACKGET_END_STOP 2
#define PACKGET_MAX_DATA_SEQ 200  // new_seq = (last_seq+1)% PACKGET_MAX_DATA_SEQ
// ack error code
#define PACKGET_ACK_FALSE_PROGRAM_ERROR 21 // the send data process stop
#define PACKGET_ACK_FALSE_ERASE_ERROR 22 // the send data process stop
#define PACKGET_ACK_FALSE_SEQ_FALSE 23 // the send data process will restart
// if has ack false , the flash process shuld be restart





//********************************    config


#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif


// Define Application Address Area */
// iap firmware adress: 0x8000000   			size = 0x1c00   7k 
// iap tag address : 	0x8000000 + 0x1c00	  	size=FLASH_PAGE_SIZE  			// 1K
// app firmware adress :0x8000000 + + 0x1c00+ FLASH_PAGE_SIZE =  0x8002000		size = (0x10000 - 0x2000) = 0xE000	//56k

// iap_v0 20170307 : IAP_FIRMWARE_SIZE = 0x1c00 -> flash in 0x8002000 0xE000
// iap_v1 20170309 : IAP_FIRMWARE_SIZE = 0x2000 -> flash in 0x8002400 0xdc00

#define IAP_FIRMWARE_ADRESS (0x8000000)
#define IAP_FIRMWARE_SIZE 0x2000    //8kB
//tag 
#define IAP_TAG_ADDRESS (IAP_FIRMWARE_ADRESS+IAP_FIRMWARE_SIZE)
#define IAP_TAG_UPDATE_VALUE 0xAB

#define IAP_APP_ADDRESS (IAP_TAG_ADDRESS + FLASH_PAGE_SIZE)
#define IAP_APP_SIZE (0x10000 - IAP_FIRMWARE_SIZE - FLASH_PAGE_SIZE)







void iap_config_vect_table(void);
void iap_can_receive_handler(unsigned char c );
void iap_init_in_can1(void);
void iap_init_in_uart(Uart_t *uart);






#endif //_IAP_H
