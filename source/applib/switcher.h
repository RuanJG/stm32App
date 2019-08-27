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




void switcher_init(struct switcher* sw, int maxcount, int default_level, int press_level, GPIO_TypeDef* GPIOX , uint16_t GPIO_Pin_x , GPIOMode_TypeDef inputmode, SwitchHandler press_handler , SwitchHandler release_handler   );
void switcher_interval_check(volatile  struct switcher *sw );


#endif






/* ********  usage


#define SW_INTERVAL_COUNT  10  // SW_INTERVAL_MS 检查一次GPIO口， SW_INTERVAL_COUNT次后发出按键事件, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher startButton;
volatile int startButton_triggered =0 ;


int is_startButton_triggered()
{
	if( startButton_triggered == 1 ){
		startButton_triggered = 0;
		return 1;
	}else{
		return 0;
	}
}

void switch_key_press_handler()
{

}

void switch_key_release_handler()
{
	startButton_triggered = 1;
}

void switch_init()
{
	switcher_init( &startButton, SW_INTERVAL_COUNT, 1 , 0 , GPIOB, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, NULL , switch_key_release_handler);

	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &startButton );

}



*/
