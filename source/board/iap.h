#ifndef _IAP_H
#define _IAP_H

#include "uart.h"
#include "main_config.h"

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






#define IAP_TAG_ADDRESS (IAP_FIRMWARE_ADRESS+IAP_FIRMWARE_SIZE)
#define IAP_TAG_UPDATE_VALUE 0xAB
#define IAP_APP_ADDRESS (IAP_TAG_ADDRESS + FLASH_PAGE_SIZE)
#define IAP_APP_SIZE (IAP_FLASH_SIZE - IAP_FIRMWARE_SIZE - FLASH_PAGE_SIZE)

#define PORT_UART_TYPE	1
#define PORT_USB_TYPE		2
#define PORT_CAN1_TYPE	4

typedef void(*iapFunction)(void);

void iap_config_vect_table(void);
void iap_can_receive_handler(unsigned char c);
void iap_init_in_can1(void);
void iap_init_in_uart(Uart_t *uart);
void iap_init_in_usb(void);
void iap_usb_receive_handler(unsigned char c);
void iap_jump(void);

int is_app_flashed();
int get_iap_tag();
int set_iap_tag(int tag);
void iap_reset();

void iap_init(void);
void iap_loop(void);
void iap_jump_to_app_or_deamon(void);
int is_iap_tag_set(void);
int clean_iap_tag();

void iap_app_event();
void iap_app_init(void *pridata, int port_type);

#endif //_IAP_H
