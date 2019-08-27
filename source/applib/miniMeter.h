#ifndef _MINIMETER_H_
#define _MINIMETER_H_


#include "stdio.h"
#include "protocol.h"
#include "stm32f10x.h"
#include "uart.h"

#define METER_STATE_IDEL  0
#define METER_STATE_STARTED 1


typedef struct _miniMeterCoder_t {
	unsigned char packet[8];
	unsigned char index;
	unsigned char addr;
	float 				value;
}miniMeterCoder_t;

typedef struct _miniMeter_t {
	Uart_t*				uartdev;
	miniMeterCoder_t decoder;
	int						state;
}miniMeter_t;



//void miniMeterCoder_init(miniMeterCoder_t * coder, unsigned char addr);
//int miniMeterCoder_prase( miniMeterCoder_t * coder, unsigned char data );


void miniMeter_clear(miniMeter_t *meter);
int miniMeter_start(miniMeter_t *meter);
void miniMeter_stop(miniMeter_t *meter);
void miniMeter_toggle(miniMeter_t *meter);
int miniMeter_check(miniMeter_t *meter, float *A);
void miniMeter_init(miniMeter_t *meter, Uart_t* uart, unsigned char meterAddr);


#endif //_MINIMETER_H_





/* ********  usage



*/


