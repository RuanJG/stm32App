#ifndef _SYSTICK_H_
#define _SYSTICK_H_


#define SYSTICK_MS_MAX 0x0fffffff
#define SYSTICK_OVERYFLOW_MAX 0x3fffffff

#define u32 unsigned int

// base funtion after run SysTick_Configuration()
void systick_init(void);
void systick_deinit(void);
void systick_delay_us(unsigned int us);

//if run systick_event in hight freq loop , can use these functions
typedef struct _systick_time_t {
	unsigned int systick_ms;
	unsigned int systick_ms_overflow;
	unsigned int interval_ms;
}systick_time_t;

void systick_event(void);
unsigned int systick_get_ms(void);
int systick_check_timer(systick_time_t *time_t);
int systick_init_timer(systick_time_t *time_t, int ms);
void systick_delay_ms(unsigned int ms);

#endif
