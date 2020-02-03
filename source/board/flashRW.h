#ifndef _FLASHRW_H_
#define _FLASHRW_H_



#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif


int flash_read(unsigned int flash_addr, volatile unsigned int * data , int size4x);
int flash_write(unsigned int flash_addr, volatile unsigned int * data , int size4x);



/********************  example

struct config {
	int config_avaliable;
	float current_max;
	float current_min;
	int db_max;
	int db_min;
	int machine_no;
	float db_factor;
};
volatile struct config g_config;

void config_init()
{
	g_config.config_avaliable = 0;
	flash_read( (volatile unsigned int*)&g_config, sizeof(g_config) );
	if( g_config.config_avaliable != 1 ){
		g_config.config_avaliable = 1;
		g_config.current_max = 0.5 ; //A
		g_config.current_min = 0.35 ; //A
		g_config.db_max = 65;
		g_config.db_min = 30;
		g_config.machine_no = '1';
		g_config.db_factor = 0.080664;
		if( 0 != flash_write((volatile unsigned int*)&g_config, sizeof(g_config)) ){
			//error handler
		}
		if( 0 != flash_read( (volatile unsigned int*)&g_config, sizeof(g_config) )){
			// error handler
		}
	}
}

*/

#endif


