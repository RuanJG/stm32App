#ifndef _BSP_CONFIG_H
#define _BSP_CONFIG_H

#define CONSOLE_NONE_TYPE 0
#define CONSOLE_UART_TYPE 1
#define CONSOLE_USB_TYPE 2

void bsp_init(void);
void bsp_event(void);
void bsp_deinit(void);
void console_init(int type, void * pridata );

#endif
