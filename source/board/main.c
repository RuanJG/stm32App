#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "app.h"


int main()
{
	
#if BOARD_PRIVATE_SETUP_CLK
	app_SystemInit();
#endif
	
	bsp_init();
	app_init();

	while(1){
		bsp_event();
		app_event();	
	}
}

