#ifndef _SYSTICK_H_
#define _SYSTICK_H_


#define SYSTICK_MS_MAX 0x7fffffff
#define SYSTICK_OVERYFLOW_MAX 0x3fffffff

// base funtion after run SysTick_Configuration()
void systick_init(unsigned int freq);
void systick_deinit(void);
void systick_delay_us(u32 us);
void systick_delay_ms(u32 ms);

//if run systick_event in hight freq loop , can use these functions
typedef struct _systick_time_t {
	u32 systick_ms;
	u32 systick_ms_overflow;
	u32 interval_ms;
}systick_time_t;

void systick_event(void);
unsigned int systick_get_ms(void);
int systick_check_timer(systick_time_t *time_t);
int systick_init_timer(systick_time_t *time_t, int ms);

#endif
