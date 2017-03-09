#ifndef _BSP_CONFIG_H
#define _BSP_CONFIG_H

#include "stm32f10x_conf.h"

void bsp_init(void);
void bsp_event(void);
void bsp_deinit(void);

void Can1_Send(uint8_t id, uint8_t *data, int len);
u8 Can1_Configuration_mask(u8 FilterNumber, u16 ID, uint32_t id_type,  u16 ID_Mask , uint8_t sjw ,uint8_t bs1, uint8_t bs2, uint8_t prescale );
int Can1_get( unsigned char *data, int size);
int Can1_getChar(unsigned char * data);


void Uart_Configuration (void);
void Uart_send(unsigned char *data, int len);
int Uart_get(unsigned char * data, int size);
int Uart_getChar(unsigned char * data); 
void uart_receive_event(unsigned char c);
void can1_receive_event(CanRxMsg *msg);

/*
************* systick 
*/
// base funtion after run SysTick_Configuration()
void systick_init(void);
void systick_deinit(void);
void systick_delay_us(u32 us);

//if run systick_event in hight freq loop , can use these functions
void systick_event(void);
typedef struct _systick_time_t {
	u32 systick_ms;
	u32 systick_ms_overflow;
	u32 interval_ms;
}systick_time_t;
u32 systick_get_ms(void);
int systick_check_timer(systick_time_t *time_t);
int systick_init_timer(systick_time_t *time_t, int ms);






#define CUSTOM_SYSTICK_IRQ_PRIORITY 	NVIC_PriorityGroup_2

#endif
