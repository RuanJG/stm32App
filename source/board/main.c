#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "app.h"


int main()
{
	bsp_init();
	app_init();

	while(1){
		bsp_event();
		app_event();	
	}
}

