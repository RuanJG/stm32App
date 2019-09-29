#ifndef I2C_H_
#define I2C_H_

#include "fifo.h"
#include  <ctype.h>
#include  <string.h>
#include  <stdio.h>	
#include "stm32f10x.h"


// gpio i2c write and read
#define GPIO_I2C_ACK 0
#define GPIO_I2C_NACK 1
#define GPIO_I2C_ERROR_START -1
#define GPIO_I2C_ERROR_ACK -2
#define GPIO_I2C_ERROR_SENDBIT -3
#define GPIO_I2C_ERROR_BUSBUSY -4
int gpio_i2c_read_regs( unsigned char slaverAddr, unsigned char * startReg, int regSize , unsigned char *data , int size );
int gpio_i2c_write_regs( unsigned char slaverAddr, unsigned char * startReg, int regSize , unsigned char *data , int size );
void gpio_i2c_init(GPIO_TypeDef *GPIO_SDA , uint16_t GPIO_PIN_SDA, GPIO_TypeDef *GPIO_CLK , uint16_t GPIO_PIN_CLK );
int gpio_i2c_check_bus();




// sync stm32 i2c write and read
int stm32f10x_i2c_master_sync_read(I2C_TypeDef *i2cx , unsigned char slaveAddr, unsigned char * regAddress, int regByteSize, unsigned char *data , int size);
int stm32f10x_i2c_master_sync_write(I2C_TypeDef *i2cx , unsigned char slaveAddr, unsigned char * regAddress, int regByteSize, unsigned char *data , int size);
void stm32f10x_i2c_master_sync_init(I2C_TypeDef *i2cx, GPIO_TypeDef *GPIO_SDA , uint16_t GPIO_PIN_SDA, GPIO_TypeDef *GPIO_CLK , uint16_t GPIO_PIN_CLK , int freq );






#endif
