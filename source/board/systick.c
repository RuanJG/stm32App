#include "misc.h"
#include "core_cm3.h"
#include "systick.h"
#include "main_config.h"

#define SysTick_Reloadvalue  8 //9-1


#define SYSTICK_MS_MAX 0x7fffffff
#define SYSTICK_OVERYFLOW_MAX 0x3fffffff


volatile u32 TimingDelay = 0;
volatile u32 systick_ms=0;
volatile u32 systick_us=0;
volatile char systick_ms_overflow = 0;

void systick_init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);		//×î¸ß9MHz

	SysTick->LOAD=SysTick_Reloadvalue;
	NVIC_SetPriority(SysTick_IRQn, CUSTOM_SYSTICK_IRQ_PRIORITY);
	SysTick->CTRL|=0x02;
	SysTick->CTRL|=0x01;
}

void systick_deinit(void)
{
	SysTick->CTRL &= ~0x02;
	SysTick->CTRL &= ~0x01;
}

void SysTick_Handler(void)
{
	systick_us++;
	
	/*
	if( systick_us >= 1000 ){
		systick_ms ++;
		systick_us =0;
	}
	*/
	if(TimingDelay > 0)
	{
		TimingDelay--;
	}
}

void systick_event()
{
	//this event for check the systick_us overflow 
	if( systick_us >= 1000 ){
		systick_ms += (systick_us/1000);
		systick_us = systick_us%1000;
	}
	if( systick_ms >= SYSTICK_MS_MAX ){
		systick_ms_overflow ++;
		systick_ms_overflow %= SYSTICK_OVERYFLOW_MAX;
		systick_ms = 0;//0.0;
	}
}

void systick_delay_us(u32 us)
{
	TimingDelay = us;
	while(TimingDelay != 0);
}

void systick_delay_ms(u32 ms)
{
	/*
	volatile u32 end_ms;
	end_ms = ms+systick_ms;
	while(1){
		if( systick_ms >= end_ms ) break;
	}*/
	TimingDelay = ms*1000;
	while(1){
		if( TimingDelay <= 0) break;
	}		
}

int systick_init_timer(systick_time_t *time_t, int ms)
{
	time_t->interval_ms = ms;
	time_t->systick_ms = (systick_ms+ time_t->interval_ms +  (systick_us+200)/1000 );// ((float)systick_us/1000.0+0.5) ) + ;
	time_t->systick_ms_overflow = systick_ms_overflow;
	return 0;
}

int systick_check_timer(systick_time_t *time_t)
{
	char force_update = 0;
	unsigned int now_ms =systick_ms+ (systick_us+200)/1000;// ((float)systick_us/1000.0+0.5) ;
	
	if( systick_ms_overflow != time_t->systick_ms_overflow ){
		time_t->systick_ms_overflow = systick_ms_overflow;
		force_update = 1;
	}		
	if( force_update || now_ms >= time_t->systick_ms ){
		time_t->systick_ms = now_ms + time_t->interval_ms ;
		return 1;
	}
	return 0;
}

unsigned int systick_get_ms()
{
	systick_event();
	return systick_ms;
}

//End of File
