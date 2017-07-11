#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "protocol.h"
#include "iap.h"

protocol_t uart_encoder;
protocol_t uart_decoder;
protocol_t can1_encoder;
protocol_t can1_decoder;






int main()
{
	bsp_init();
	iap_init();

	while(1){
		bsp_event();
		iap_loop();	
	}
}

