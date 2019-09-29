/******************************************************************************
 * @file:    SetUART.c
 * @purpose: functions related to UART
 * @version: V1.00
 * @date:    11. Jul 2011
 *----------------------------------------------------------------------------
 ******************************************************************************/

#include "stm32f10x.h"
#include "fifo.h"
#include  <ctype.h>
#include  <string.h>
#include  <stdio.h>	
#include "main_config.h"
#include "systick.h"
#include "i2c.h"


#define _IICDELAY_ 1500

void stm32f10x_i2c_master_sync_init(I2C_TypeDef *i2cx, GPIO_TypeDef *GPIO_SDA , uint16_t GPIO_PIN_SDA, GPIO_TypeDef *GPIO_CLK , uint16_t GPIO_PIN_CLK , int freq )
{
//�ȿ���ʱ��ʲô��
	I2C_InitTypeDef I;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if( i2cx == I2C1 ){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}else{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	I2C_Cmd(i2cx, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //GPIO_Speed_10MHz
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_SDA, GPIO_PIN_SDA, Bit_SET);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CLK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //GPIO_Speed_10MHz
	GPIO_Init(GPIO_CLK, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_CLK, GPIO_PIN_CLK, Bit_SET);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIO_CLK, &GPIO_InitStructure);//����GPIO���ã��Ȱ�GPIO��Ϊ��������������ߵ�ƽ��ת��Ϊ���ÿ�©���������������Ч���I2C�豸����������


	I2C_DeInit(i2cx);
	
	I.I2C_Mode = I2C_Mode_I2C;
	I.I2C_DutyCycle = I2C_DutyCycle_2;//I2C_DutyCycle_16_9; //I2C_DutyCycle_2
	I.I2C_Ack = I2C_Ack_Enable;
	I.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I.I2C_ClockSpeed = freq;
	I.I2C_OwnAddress1 = 0;
	
	I2C_StretchClockCmd(i2cx, DISABLE);
	I2C_SoftwareResetCmd(i2cx, ENABLE);
	I2C_SoftwareResetCmd(i2cx, DISABLE);//I2C��λ�����GPIOǿ���������¶�ȡ�����ݰ��豸�����ٲ���Ҳ�ܿ���ס���ߣ���λ���񲻻��Լ����㣬Ҫ��ENABLE��DISABLE
	
	I2C_Init(i2cx, &I);
	
	I2C_Cmd(i2cx, ENABLE);
}

int stm32f10x_i2c_master_sync_read(I2C_TypeDef *i2cx , unsigned char slaveAddr, unsigned char * regAddress, int regByteSize, unsigned char *data , int size)
{
	int i ;
	volatile int Timer;
	
	//I2C_Cmd(i2cx,ENABLE);
	
	I2C_AcknowledgeConfig(i2cx,ENABLE); //NACK

	Timer = _IICDELAY_;
	while (--Timer&&(I2C_GetFlagStatus(i2cx, I2C_FLAG_BUSY) == ENABLE));//�ȴ����߿���
	if (!Timer&&Timer > _IICDELAY_) return -1;

	//1
	I2C_GenerateSTART(i2cx, ENABLE);//������ʼλ
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT) == SUCCESS));
	if (!Timer&&Timer > _IICDELAY_) return -2; 

	//2
	I2C_Send7bitAddress(i2cx, (slaveAddr << 1 ), I2C_Direction_Transmitter);//�����豸Ӳ����ַ
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == SUCCESS));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -3; 
	
	I2C_Cmd(i2cx, ENABLE); //Clear EV6 by setting again the PE bit
	
	//3
	for( i=1; i<= regByteSize; i++){
		I2C_SendData(i2cx, regAddress[regByteSize-i] );//�����豸��ַ
		Timer = _IICDELAY_;
		while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == SUCCESS));//�ȴ��¼�����
		if (!Timer&&Timer > _IICDELAY_) return -4; 
	}
	
#if 1
	I2C_GenerateSTOP(i2cx, ENABLE);//ֹͣ����
	Timer = _IICDELAY_;
	while (--Timer && !((i2cx->CR1 & I2C_CR1_STOP) == 0));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -5; 
#endif 

	//4
	I2C_GenerateSTART(i2cx, ENABLE);
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT) == SUCCESS));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -6; 

	//5
	I2C_Send7bitAddress(i2cx, (slaveAddr << 1 ), I2C_Direction_Receiver);//�����豸Ӳ����ַ
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == SUCCESS));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -7; 

	
	
#if 1
	__disable_irq();
#endif
	
	//6
	for( i=0; i< size; i++ ){
#if 1 // now slaver has send data , so stop , and then read the data in i2c reg.
		if( i == (size-1) ){
			I2C_AcknowledgeConfig(i2cx,DISABLE); //NACK
			
			I2C_GenerateSTOP(i2cx, ENABLE);//ֹͣ����
			Timer = _IICDELAY_;
			while (--Timer && !((i2cx->CR1 & I2C_CR1_STOP) == 0));//�ȴ��¼�����
			if (!Timer&&Timer > _IICDELAY_) return -9; 
		}
#endif 
		Timer = _IICDELAY_;
		while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))//�ȴ��յ�һ���ֽ�
		if (!Timer&&Timer > _IICDELAY_) return -8; 
		data[i] = I2C_ReceiveData(i2cx);//��������
	}
	
#if 1
	 __enable_irq();
#endif
	
	//I2C_Cmd(i2cx, DISABLE);

	return size;
	
}



int stm32f10x_i2c_master_sync_write(I2C_TypeDef *i2cx , unsigned char slaveAddr, unsigned char * regAddress, int regByteSize, unsigned char *data , int size)
{
	int i ;
	volatile int Timer;
	
	//I2C_Cmd(i2cx,ENABLE);
	
	I2C_AcknowledgeConfig(i2cx,ENABLE); //NACK

	Timer = _IICDELAY_;
	while (--Timer&&(I2C_GetFlagStatus(i2cx, I2C_FLAG_BUSY) == ENABLE));//�ȴ����߿���
	if (!Timer&&Timer > _IICDELAY_) return -1;

	//1
	I2C_GenerateSTART(i2cx, ENABLE);//������ʼλ
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT) == SUCCESS));
	if (!Timer&&Timer > _IICDELAY_) return -2; 

	//2
	I2C_Send7bitAddress(i2cx, (slaveAddr << 1 ), I2C_Direction_Transmitter);//�����豸Ӳ����ַ
	Timer = _IICDELAY_;
	while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == SUCCESS));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -3; 
	
	//I2C_Cmd(i2cx, ENABLE); //Clear EV6 by setting again the PE bit
	
	//3
	for( i=1; i<= regByteSize; i++){
		I2C_SendData(i2cx, regAddress[regByteSize-i] );//�����豸��ַ
		Timer = _IICDELAY_;
		while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == SUCCESS));//�ȴ��¼�����
		if (!Timer&&Timer > _IICDELAY_) return -4; 
	}
	
	for( i=0; i< size; i++ )
	{
		I2C_SendData(i2cx, data[i]);//�����豸��ַ
		Timer = _IICDELAY_;
		while (--Timer && !(I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == SUCCESS));//�ȴ��¼�����
		if (!Timer&&Timer > _IICDELAY_) return -5; 
	}
	
	I2C_GenerateSTOP(i2cx, ENABLE);//ֹͣ����
	Timer = _IICDELAY_;
	while (--Timer && !((i2cx->CR1 & I2C_CR1_STOP) == 0));//�ȴ��¼�����
	if (!Timer&&Timer > _IICDELAY_) return -6; 
	
	//I2C_Cmd(i2cx, DISABLE);

	return size;

}








GPIO_TypeDef *sda_gpio ;
uint16_t sda_pin;
GPIO_TypeDef *clk_gpio ;
uint16_t clk_pin;


//time for each __NOP() 
int Time_tick = 13 ; // 1000 000 000/system_clk 

//time for clk keep high after sda fall
int Time_clk_start_hold = 1000; // > 600ns 
//time for clk keep low
int Time_clk_hold_low = 2000 ; // > 1300ns
//time for keep clk high at the begin of stop operate 
int Time_clk_stop_setup = 1000 ; // >600ns ,  time for  clk low ->high ~  data low->high
//time for keep clk sda high at the end of stop operate
int Time_start_stop = 2000; // > 1300ns , between start stop operate 

//time for clk/data rise or fall 
int Time_riseOrfall = 100; // 20-300 ns 
//time for data update when clk low : clk=0 -> wait(Time_data_hold) -> SDA=x -> wait(Time_data_setup)-> start clk=1
int Time_data_hold = 130 ; // 40~900 ns
int Time_data_setup = 150000 ; // > 100000ns
//time for clk keep high 
int Time_clk_hold = 1000; // > 600ns 


//time for wait for slave ready after receive/send ack   !! not standar
int Time_ack_delay = 500000 ; 


#define SCL_H()         clk_gpio->BSRR = clk_pin 	 //��PIN1����
#define SCL_L()         clk_gpio->BRR  = clk_pin   //��PIN1����
    
#define SDA_H()         sda_gpio->BSRR = sda_pin 	 //��PIN2����
#define SDA_L()         sda_gpio->BRR  = sda_pin 	 //��PIN2����

#define SCL_read()      (((clk_gpio->IDR  & clk_pin) != 0 )? 1:0) 	 //��ȡPIN1����
#define SDA_read()      (((sda_gpio->IDR  & sda_pin) != 0 )? 1:0) 	 //��ȡPIN2����


static void gpio_i2c_delay(int ticks) 
{ 
   volatile int i= 200;//ticks; 
   while(i--) __NOP();  
} 


static int gpio_i2c_start(void) 
{ 

	SDA_H(); 			  //release SDA��
	SCL_H(); 			  //����SCL��
	gpio_i2c_delay(Time_start_stop); 
	
	if(!SDA_read())return GPIO_I2C_ERROR_BUSBUSY; //SDA��Ϊ�͵�ƽ������æ,�˳� 
	
	SDA_L(); 			  //����SDA�ߣ�����start signal
	gpio_i2c_delay(Time_riseOrfall); 
	if(SDA_read()) return -2; //SDA��Ϊ�ߵ�ƽ�����߳���,�˳� 
	
	gpio_i2c_delay(Time_clk_start_hold); 
	SCL_L();
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold_low); 
	
	return 0; 
} 

static void gpio_i2c_stop(void) 
{ 
	SCL_L(); 			  
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	SDA_L(); 
	gpio_i2c_delay(Time_data_setup); 
	
	SCL_H(); 			
	gpio_i2c_delay(Time_clk_stop_setup); 
	SDA_H();
	gpio_i2c_delay(Time_riseOrfall); 
	gpio_i2c_delay(Time_start_stop); 
} 


static int gpio_i2c_sendBit(int bit)
{
	if( bit == 0 ){
		SDA_L();
	}else{
		SDA_H();
	}
	gpio_i2c_delay(Time_data_setup); 
	
	if(bit != SDA_read()) return GPIO_I2C_ERROR_SENDBIT;
	
	SCL_H(); 
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
	SCL_L(); 		
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	
	return 0;
}

static int gpio_i2c_readBit(void)
{
	int bit;
	
	SDA_H(); 				//Master release SDA�ߣ�����SLAVE�ƿ�
	gpio_i2c_delay(Time_data_setup); 
	
	SCL_H(); 
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
	
	bit = SDA_read(); 		

	SCL_L(); 				//����SCL�ߴ��ڵ͵�ƽ
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold);
	
	return bit;
}



static int gpio_i2c_send_byte(unsigned char data)
{
	int i;
	for( i=0; i<8; i++ )
	{ 
		if( 0 == (data&0x80)){ 
			if(0 != gpio_i2c_sendBit(0) ) return GPIO_I2C_ERROR_SENDBIT;
		}else{
			if(0 != gpio_i2c_sendBit(1) ) return GPIO_I2C_ERROR_SENDBIT;		
		}
		data<<=1; 			 //�ƶ�һ��bit,׼��������һ��bit
	} 
	
	return 0;
}

static unsigned char gpio_i2c_read_byte(void)
{
	int i;
	unsigned char data;
	
	data = 0;
	for( i=0; i<8; i++ )
	{ 
		data <<= 1;
		data = (data | gpio_i2c_readBit());
	} 
	
	return data;
}




/********************
read count bytes start from slaverAddr's startReg[2][1][0]... , store in dataBuffer, return read bytes count , other return GPIO_I2C_ERROR_* error 
**********************/
int gpio_i2c_read_regs( unsigned char slaverAddr, unsigned char * startReg, int regByteSize , unsigned char *data , int size )
{
	int i, res;

//Phase 1
	//start
	res = gpio_i2c_start(); if( res != 0){ return res;}
	//send addr+W
	res = gpio_i2c_send_byte( (slaverAddr<<7) ); if( res != 0){ gpio_i2c_stop();  return res;}
	//wait ACK
	if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
	gpio_i2c_delay(Time_ack_delay);
	//send reg
	for( i=1; i<= regByteSize ; i++){
		//send reg
		res = gpio_i2c_send_byte( startReg[regByteSize-i] ); if( res != 0){ gpio_i2c_stop();  return res;}
		//wait ACK
		if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
		gpio_i2c_delay(Time_ack_delay);
	}
	
#if 1
	//stop
	gpio_i2c_stop();
#endif
	
//Phase 2	
	//start
	res = gpio_i2c_start(); if( res != 0){ return res;}
	//send addr+R
	res = gpio_i2c_send_byte( ((slaverAddr<<7)|0x01) ); if( res != 0){ gpio_i2c_stop();  return res;}
	//wait ACK
	if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
	gpio_i2c_delay(Time_ack_delay);
	//Read data
	for( i=0; i< size; i++ ){
	  //read byte
		data[i] = gpio_i2c_read_byte() ;
		// send ack/nack
		if( i < (size -1) ){
			res = gpio_i2c_sendBit( GPIO_I2C_ACK );
		}else{
			res = gpio_i2c_sendBit( GPIO_I2C_NACK );
		}
		if( 0 != res ){ gpio_i2c_stop(); return res; }
		gpio_i2c_delay(Time_ack_delay);
	}
	//stop
	gpio_i2c_stop();
	
	return size;	
}


int gpio_i2c_write_regs( unsigned char slaverAddr, unsigned char *startReg, int regByteSize , unsigned char *data , int size )
{
	int i,res;
	

//Phase 1
	//start
	res = gpio_i2c_start(); if( res != 0){ return res;}
	//send addr+W
	res = gpio_i2c_send_byte( (slaverAddr<<7) ); if( res != 0){ gpio_i2c_stop();  return res;}
	//wait ACK
	if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
	gpio_i2c_delay(Time_ack_delay);
	
	for( i=1; i<= regByteSize ; i++){
		//send reg
		res = gpio_i2c_send_byte( startReg[regByteSize-i] ); if( res != 0){ gpio_i2c_stop();  return res;}
		//wait ACK
		if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
		gpio_i2c_delay(Time_ack_delay);
	}
	
	
	//send data
	for( i=0; i< size; i++ ){
		// send byte
		res = gpio_i2c_send_byte( data[i] ) ;  if( 0 != res ){ gpio_i2c_stop(); return res; }
		//wait ACK
		if( GPIO_I2C_ACK != gpio_i2c_readBit() ) { gpio_i2c_stop();return GPIO_I2C_ERROR_ACK; }
		gpio_i2c_delay(Time_ack_delay);
	}
	//stop
	gpio_i2c_stop();
	
	return size;
}

void gpio_i2c_init(GPIO_TypeDef *GPIO_SDA , uint16_t GPIO_PIN_SDA, GPIO_TypeDef *GPIO_CLK , uint16_t GPIO_PIN_CLK )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //GPIO_Speed_10MHz
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_SDA, GPIO_PIN_SDA, Bit_SET);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CLK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //GPIO_Speed_10MHz
	GPIO_Init(GPIO_CLK, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_CLK, GPIO_PIN_CLK, Bit_SET);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIO_CLK, &GPIO_InitStructure);//����GPIO���ã��Ȱ�GPIO��Ϊ��������������ߵ�ƽ��ת��Ϊ���ÿ�©���������������Ч���I2C�豸����������
	
	sda_gpio = GPIO_SDA;
	sda_pin = GPIO_PIN_SDA;
	clk_gpio = GPIO_CLK;
	clk_pin = GPIO_PIN_CLK;
	
	Time_tick = 1000000000 / SystemCoreClock;
	
	Time_riseOrfall /= Time_tick;
	Time_clk_start_hold /= Time_tick;
	Time_clk_hold_low /= Time_tick ;
	Time_data_hold /= Time_tick ;
	Time_data_setup /= Time_tick ;
	Time_clk_hold /= Time_tick;
	Time_clk_stop_setup /= Time_tick ;
	Time_start_stop /= Time_tick;
	Time_ack_delay /= Time_tick;

	gpio_i2c_stop();
}








/******************************************
�������ƣ�I2C_Ack()
��    �ܣ�֪ͨSLAVE��I2C���յ�����
˵    ����I2CЭ����Ҫ��masterÿ�յ�һ��byte��Ҫ��slave����ACK�źţ�
ACK�źž��ǽ�SDA������
*******************************************/
static int gpio_i2c_sendAck(void) 
{ 
	u16 i;
	//SCL_L(); 
	//gpio_i2c_delay(); 
	SDA_L(); 			  
	gpio_i2c_delay(Time_data_setup); 
	if(SDA_read()) return -1;
	SCL_H(); 
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
	SCL_L(); 			
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	
	return 0;
}
/*********************************************
�������ƣ�I2C_NoAck()
��    �ܣ�֪ͨSLAVE�����ݽ��ս����������ٷ�������
˵    ����I2CЭ����Ҫ��master���յ����һ��byteʱ��slave����NOACK�źţ�
��ʾ�����Ѿ�ȫ���յ��������ٷ����ݡ�NOACK�źž��ǽ�SDA������
*******************************************/
static int gpio_i2c_sendNAck(void) 
{ 
	//SCL_L(); 
	//gpio_i2c_delay(); 
	SDA_H(); 			   //��SCL�ߵ�ʱ������SDA�ߣ�����NOACK�ź�
	gpio_i2c_delay(Time_data_setup); 
	if(!SDA_read()) return -1;
	SCL_H(); 
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
	SCL_L(); 			   //����SCL�ߴ��ڵ͵�ƽ
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	
	return 0;
} 

/******************************************
�������ƣ�I2C_WaitAck()
��    �ܣ��ȴ�SLAVE�յ����ݺ��ȷ���ź�
˵    ����I2CЭ����Ҫ��SLAVEÿ�յ�һ��byte�����ݣ���Ҫ֪ͨMASTER.
�źž��Ƿ��͸�MASTERһ��low���źţ���ʾ�յ�����
*******************************************/
static int gpio_i2c_waitAck(void)   
{ 
	u16 i;
	//SCL_L(); 
	//gpio_i2c_delay(); 
	SDA_H(); 				//Master release SDA�ߣ�����SLAVE�ƿ�
	gpio_i2c_delay(Time_data_setup); 
	SCL_H(); 
	gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
	if(SDA_read()) 		//���SDA��ΪHIGH,��ʾSLAVEû��ACK
	{ 
		SCL_L(); 
		gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
		return -1; 
	} 
	SCL_L(); 				//����SCL�ߴ��ڵ͵�ƽ
	gpio_i2c_delay(Time_riseOrfall+Time_data_hold);

	return 0; 
}

/******************************************
�������ƣ�I2C_SendByte()
��    �ܣ�Master��Slave����һ��byte������
˵    ���������ǴӸ�λ����λbit by bit����
*******************************************/
int gpio_i2c_sendByte(u8 SendByte) 
{ 
	u8 i=8; 
	while(i--) 
	{ 
		//SCL_L(); 					 //����SCL�ߣ��ɸı�SDA������
		//gpio_i2c_delay(); 
		if(SendByte&0x80)		 //����λbit�ǲ���HIGH
		{ 
			SDA_H();				 //��SDA�����ߣ���ʾ����ΪHIGH
			gpio_i2c_delay(Time_data_setup); 
			if(!SDA_read()) return -1;
		}   
		else  
		{
			SDA_L();				 //��SDA�����ͣ���ʾ����ΪLOW
			gpio_i2c_delay(Time_data_setup);
			if(SDA_read()) return -1;			
		}
		
		SCL_H(); 					 //��SCL���ߣ�����һ��cycle
		gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
		SendByte<<=1; 			 //�ƶ�һ��bit,׼��������һ��bit
		
		SCL_L(); 
		gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	} 
} 


/******************************************
�������ƣ�I2C_ReceiveByte()
��    �ܣ�Master��Slave����һ��byte������
˵    ���������ǴӸ�λ����λbit by bit����
*******************************************/
static u8 gpio_i2c_receiveByte(void)  
{  
	u8 i=8; 
	u8 ReceiveByte=0; 

	SDA_H(); 						  //Master release SDA�ߣ�����SLAVE����
	while(i--) 
	{ 
		ReceiveByte<<=1;     
		gpio_i2c_delay(Time_data_setup);		
		SCL_H(); 					  
		gpio_i2c_delay(Time_riseOrfall+Time_clk_hold); 
		if(SDA_read()) 			  
		{ 
			ReceiveByte|=0x01; 	  
		} 
		SCL_L(); 					  
		gpio_i2c_delay(Time_riseOrfall+Time_data_hold); 
	} 
	
	return ReceiveByte; 
} 


//End of File
