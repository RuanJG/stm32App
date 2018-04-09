#ifndef _SWITCHER_H_
#define _SWITCHER_H_

#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "systick.h"
#include "stdio.h"


typedef void (*SwitchHandler) (void);

struct switcher {
	unsigned char state ; // default level :0/1
	unsigned char press_level; // 0/1
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	SwitchHandler press_handler;
	SwitchHandler release_handler;

	unsigned short counter;
	unsigned short sum;
	unsigned short MAXCOUNT;
};




void switcher_init(struct switcher* sw,  int maxcount, int default_level, int press_level, GPIO_TypeDef* GPIOX , uint16_t GPIO_Pin_x , SwitchHandler press_handler , SwitchHandler release_handler   );
void switcher_interval_check(volatile  struct switcher *sw );


#endif






/* ********  usage

void switch_key_press_handler()
{
	current_point++;
	current_point %= 4;
	lcd_update_mouse();

}



#define SW_INTERVAL_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher round_add_key,round_reduce_key,start_key,switch_key;


void switch_init()
{
	switcher_init( &round_add_key, 1 , 0 , GPIOA, GPIO_Pin_5, round_add_key_toggle_handler , NULL);
	switcher_init( &round_reduce_key, 1, 0 , GPIOA, GPIO_Pin_4 , round_reduce_key_toggle_handler, NULL);
	switcher_init( &start_key , 1, 0 , GPIOB, GPIO_Pin_0, start_key_press_handler ,NULL);
	switcher_init( &switch_key, 1, 0 , GPIOB, GPIO_Pin_1, switch_key_press_handler ,NULL);
	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &round_add_key );
	switcher_interval_check( &round_reduce_key );
	switcher_interval_check( &start_key );
	switcher_interval_check( &switch_key );
}


*/