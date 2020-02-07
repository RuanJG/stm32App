#include "switcher.h"

void switcher_init(struct switcher* sw, int maxcount, int tolerance,  int default_level, int press_level, GPIO_TypeDef* GPIOX , uint16_t GPIO_Pin_x , GPIOMode_TypeDef inputmode, SwitchHandler press_handler , SwitchHandler release_handler   )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int i;
	
	if( sw->GPIOx == GPIOA )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if ( sw->GPIOx == GPIOB )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if ( sw->GPIOx == GPIOC )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if ( sw->GPIOx == GPIOD )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if ( sw->GPIOx == GPIOE )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = sw->GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = inputmode ;
	GPIO_Init(sw->GPIOx, &GPIO_InitStructure);
	
	
	sw->GPIOx = GPIOX;
	sw->GPIO_Pin = GPIO_Pin_x;
	sw->state = default_level; // GPIO_ReadInputDataBit( sw->GPIOx, sw->GPIO_Pin );
	sw->press_level = press_level ; // gpio=0, when press 
	sw->press_handler = press_handler;
	sw->release_handler = release_handler;
	
	sw->counter = 0;
	sw->sum = 0;
	sw->MAXCOUNT = maxcount;
	
	sw->tolerance = tolerance;
	
	for( i=0; i< SW_SAMPLES_COUNT; i++ ){
		sw->samples[i] = default_level;
	}
	sw->samples_index = 0;
}


void switcher_interval_check(volatile  struct switcher *sw )
{
	char level ;
	uint8_t pin = 0;
	
	pin = GPIO_ReadInputDataBit( sw->GPIOx, sw->GPIO_Pin );
	sw->sum += pin;
	sw->counter++;
	
	if( sw->counter  < sw->MAXCOUNT ) return;
	
	if( sw->sum <= sw->tolerance ){
		// totally level 0
		level = 0;
	}else if( sw->sum >= (sw->counter - sw->tolerance) ){
		// totally level 1
		level = 1;
	}else{
		level = 2;
	}
	
	sw->counter = 0;
	sw->sum = 0;
	if( level == 2 ) return ;

	if( level == sw->press_level ){
		if( sw->state != level && sw->press_handler != NULL ) sw->press_handler();
		sw->state = level;
		
	}else{
		if( sw->state != level && sw->release_handler != NULL ) sw->release_handler();
		sw->state = level;
	}
		
}

void switcher_interval_check2(volatile  struct switcher *sw )
{
	char level ;
	int i,sum;
	uint8_t pin = 0;
	
	pin = GPIO_ReadInputDataBit( sw->GPIOx, sw->GPIO_Pin );
	
	sw->samples[sw->samples_index++] = pin;
	if( sw->samples_index >= SW_SAMPLES_COUNT ) sw->samples_index = 0;
	
	for( i=0,sum=0; i< SW_SAMPLES_COUNT; i++){
		sum += sw->samples[i];
	}
	
	if( sum <= sw->tolerance ){
		// totally level 0
		level = 0;
	}else if( sum >= (sw->counter - sw->tolerance) ){
		// totally level 1
		level = 1;
	}else{
		level = 2;
	}
	
	if( level == 2 ) return ;

	if( level == sw->press_level ){
		if( sw->state != level && sw->press_handler != NULL ) sw->press_handler();
		sw->state = level;
		
	}else{
		if( sw->state != level && sw->release_handler != NULL ) sw->release_handler();
		sw->state = level;
	}
		
}
