
#include "LCD1602.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "systick.h"


#if 0
void Delay_us(uint32 CountLing)
{
	volatile char i;
	while(CountLing--){
		i = 10;
		while(i--);
	}
}

#else

#define Delay_us(U) systick_delay_us(U)

#endif




#define LCD1602_RS0()		GPIOB->BRR  = GPIO_Pin_7
#define LCD1602_RS1()		GPIOB->BSRR  = GPIO_Pin_7
#define LCD1602_RW0()		GPIOB->BRR  = GPIO_Pin_6
#define LCD1602_RW1()		GPIOB->BSRR  = GPIO_Pin_6
#define LCD1602_EN0()		GPIOB->BRR  = GPIO_Pin_5
#define LCD1602_EN1()		GPIOB->BSRR  = GPIO_Pin_5
#define LCD1602_DATA(D) do{	GPIOB->BSRR  = D<<8; 	GPIOB->BRR  = (~D)<<8; }while(0)


void LCD1602_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}


void LCD1602_Cmd(unsigned char cmd)
{

	LCD1602_RS0();

	LCD1602_RW0();

	LCD1602_EN0();

	LCD1602_DATA(cmd);

	Delay_us(340);

	LCD1602_EN1();

	Delay_us(340);

	LCD1602_EN0();

}


void LCD1602_Data(unsigned char data)
{
	LCD1602_RS1();

	LCD1602_RW0();

	LCD1602_EN0();

	LCD1602_DATA(data);

	Delay_us(340);

	LCD1602_EN1();

	Delay_us(340);

	LCD1602_EN0();
}

void LCD1602_Write(int x,int y,char ch)
{
	if( y < 0 || y > 0xf || x< 0 || x >1 )
		return ;
	
	if(x==0)
	{
		LCD1602_Cmd(0x80+y);
		LCD1602_Data(ch);
	}
	else
	{
		LCD1602_Cmd(0xc0+y);
		LCD1602_Data(ch);
	}
}

void LCD1602_Init(void)
{
	LCD1602_GPIO_Init();
	
	LCD1602_Cmd(0x38);

	Delay_us(6);

	LCD1602_Cmd(0x0C);

	Delay_us(6);

	LCD1602_Cmd(0x06);

	Delay_us(6);

	LCD1602_Cmd(0x01);

	Delay_us(6);
}
