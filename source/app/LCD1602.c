
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
void  LCD1602_DATA( unsigned char data) 
{
	GPIOB->BSRR  = (data<<8) & 0xFF00 ; 
	
	GPIOB->BRR  = ((~data)<<8) & 0xFF00;

}

void LCD1602_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	LCD1602_EN0();
	Delay_us(400);
}


#define _LOG(X...) if( 1 ) printf(X);
int LCD1602_busy()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t data;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	LCD1602_RS0();
	LCD1602_RW1();
	LCD1602_EN1();
	
	Delay_us(100);
	data = GPIO_ReadInputData( GPIOB );
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//_LOG( "state= 0x%x \n",data);
	if( (data & 0x8000)  != 0 ) return 1;
	else return 0;
	
}

void LCD1602_Cmd(unsigned char cmd)
{

	#if 0
	LCD1602_RS0();

	LCD1602_RW0();

	LCD1602_EN0();

	LCD1602_DATA(cmd);

	Delay_us(340);

	LCD1602_EN1();

	Delay_us(340);

	LCD1602_EN0();
	
	#else
	
	LCD1602_RS0(); //cmd addr	

	LCD1602_RW0(); // write 

	LCD1602_DATA(cmd);
	
	Delay_us(100); //50

	LCD1602_EN1(); // en low pulse
	
	Delay_us(400);

	LCD1602_EN0();

	Delay_us(400);

	#endif

}


void LCD1602_Data(unsigned char data)
{
	#if 0
	LCD1602_RS1();

	LCD1602_RW0();

	LCD1602_EN0();

	LCD1602_DATA(data);

	Delay_us(340);

	LCD1602_EN1();

	Delay_us(340);

	LCD1602_EN0();
	
	#else
	
	LCD1602_RS1(); //data addr	

	LCD1602_RW0(); // write 

	LCD1602_DATA(data);

	Delay_us(100); //50

	LCD1602_EN1(); // en low pulse
	
	Delay_us(400);

	LCD1602_EN0();

	Delay_us(400);
	
	#endif
}


void LCD1602_Cmd_NB( unsigned char cmd )
{
	while( LCD1602_busy() ); 
	LCD1602_Cmd( cmd );
}

void LCD1602_Data_NB(unsigned char data)
{
	while( LCD1602_busy() ); 
	LCD1602_Data( data);
}

void LCD1602_Write_NB(int x,int y,char ch)
{
	if( y < 0 || y > 0xf || x< 0 || x >1 )
		return ;

	if(x==0)
	{
		LCD1602_Cmd_NB(0x80+y);
		LCD1602_Data_NB(ch);
	}
	else
	{
		LCD1602_Cmd_NB(0xc0+y);
		LCD1602_Data_NB(ch);
	}
}


void LCD1602_Write(int x,int y,char ch)
{
	
	//LCD1602_Write_NB(x,y,ch);
	//return;
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

void LCD1602_SetMouse(int x,int y)
{
	if( y < 0 || y > 0xf || x< 0 || x >1 )
		return ;
	
	if(x==0)
	{
		LCD1602_Cmd(0x80+y);
		//LCD1602_Data(ch);
	}
	else
	{
		LCD1602_Cmd(0xc0+y);
		//LCD1602_Data(ch);
	}
}


void LCD1602_Init(void)
{
	int y;
	
	LCD1602_GPIO_Init();
	
	#if 0
	LCD1602_Cmd(0x1);
	
	LCD1602_Cmd(0x38);
	
	//LCD1602_Cmd(0x0B); // 1011

	LCD1602_Cmd(0x0F); // 0c

	LCD1602_Cmd(0x06);
	
	#else
	
	systick_delay_us(15000);
	LCD1602_Cmd( 0x38); 
	systick_delay_us(5000);
	LCD1602_Cmd( 0x38);
	systick_delay_us(5000);
	LCD1602_Cmd( 0x38);
	
	LCD1602_Cmd_NB( 0x38);
	LCD1602_Cmd_NB( 0x08);
	LCD1602_Cmd_NB( 0x01);
	LCD1602_Cmd_NB( 0x06);
	LCD1602_Cmd_NB( 0x0D);//0x0c
	
	
	#endif
	


}
