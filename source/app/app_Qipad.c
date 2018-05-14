#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "switcher.h"
#if BOARD_QIPAD



#define _LOG(X...) if( 1 ) printf(X);


/*
Uart1 A Meter  ( PA9 PA10)
Uart3 V Meter  ( PB10 PB11 )

ADC ( PA0-red PA4-white1 PA5-white2 )

switch (PB0 PB1)

valve ( PA7 , PA6, PA15, PA2 , PA3 ,[PA1] ) 



*/



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;




void Uart_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	Uart_Configuration (&Uart1, USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#if 0
	//USART2								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart2, USART2, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
#endif

#if 1
	//USART3								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart3, USART3, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif

#if 0
	//CAN
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
									
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	Can1_Configuration_mask(0, CAN1_ID, CAN_ID_STD, 0x1ff , CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_5tq, 4);
#endif
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
}










#define ADC_SAMPLE_COUNT 34  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
#define ADC_SAMPLE_CHANNEL_COUNT 3
static unsigned short escADCConvert[ADC_SAMPLE_COUNT][ADC_SAMPLE_CHANNEL_COUNT];
volatile int adc_updated;

void ADC_Configuration ()
{										 
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC_SAMPLE_CHANNEL_COUNT;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// 239.5 sampleTime => (239.5+12.5)/12M = 21us; 21*ADC_SAMPLE_COUNT * 3 = ADC_SAMPLE_COUNT * 63us = 1.890ms each group
	// 71.5 smapleTime => 7us; 7us*ADC_SAMPLE_COUNT*3 = ADC_SAMPLE_COUNT* 21us = 0.630 ms 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);//ADC_SampleTime_71Cycles5	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_239Cycles5);
 
	// set ADC channel for temperature sensor
	//ADC_TempSensorVrefintCmd(ENABLE);
	
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);
 
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_CHANNEL_COUNT*ADC_SAMPLE_COUNT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&escADCConvert;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);


  /* Enable DMA channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_DAM1_IRQ_PREPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_DAM1_IRQ_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	adc_updated = 0;

}

void DMA1_Channel1_IRQHandler(void)
{
 if(DMA_GetITStatus(DMA1_IT_TC1))
 {
	 adc_updated = 1;
	 DMA_ClearITPendingBit(DMA1_IT_GL1);
 }
}


unsigned short Cali_Adc_Value(int id)
{
	int i;
	u32 max=0,min=0,tmp;
	u32 sensor = 0; 
	
	max = escADCConvert[0][id];	
	min = escADCConvert[0][id];
	for(i=0;i<ADC_SAMPLE_COUNT;i++)
	{
		tmp = escADCConvert[i][id];
		if( tmp > max ) max = tmp;
		if( tmp < min ) min = tmp;
		sensor += tmp;
	}
	sensor -= (max+min);
	sensor = sensor/(ADC_SAMPLE_COUNT-2);
	//sensor = sensor>>5;
	return sensor;
}



#define RED_LED_ADC_ID 0
#define WHITE1_LED_ADC_ID 1
#define WHITE2_LED_ADC_ID 2

#define RED_LED_HIGHT_LEVEL  800
#define WHITE1_LED_HIGHT_LEVEL 1000
#define WHITE2_LED_HIGHT_LEVEL 1000

volatile int red_led_adc , white1_led_adc, white2_led_adc;
void LED_Detection_init()
{
	ADC_Configuration ();
	red_led_adc = 0;
	white1_led_adc = 0;
	white2_led_adc = 0;
}


int is_red_led_on()
{
	if( adc_updated != 0 ){
		red_led_adc = Cali_Adc_Value(RED_LED_ADC_ID);
	}
	
	red_led_adc = 4096 - red_led_adc;
	//_LOG("red adc=%d\n",red_led_adc);
	
	if( red_led_adc >= RED_LED_HIGHT_LEVEL ){
		//_LOG("red adc=%d\n",red_led_adc);
		return 1;
	}else{ 
		return 0;
	}
}

int is_white1_led_on()
{
	if( adc_updated != 0 ){
		white1_led_adc = Cali_Adc_Value(WHITE1_LED_ADC_ID);
	}
	
	white1_led_adc = 4096 - white1_led_adc;
	//_LOG("white1 adc=%d\n",white1_led_adc);
	
	if( white1_led_adc >= WHITE1_LED_HIGHT_LEVEL ){
		return 1;
	}else{ 
		return 0;
	}
}

int is_white2_led_on()
{
	if( adc_updated != 0 ){
		white2_led_adc = Cali_Adc_Value(WHITE2_LED_ADC_ID);
	}
	
	white2_led_adc = 4096 - white2_led_adc;
	//_LOG("white2 adc=%d\n",white2_led_adc);
	
	if( white2_led_adc >= WHITE2_LED_HIGHT_LEVEL ){
		return 1;
	}else{ 
		return 0;
	}
}


volatile int check_led_toggle_server_tag;
volatile int check_led_toggle_counter;
volatile int check_led_toggle_level;

void check_led_toggle_server_init()
{
	check_led_toggle_server_tag = 1;
	check_led_toggle_counter = 0;
	check_led_toggle_level = 0;
}

void check_led_toggle_server_event()
{
	int level;
	
	if( check_led_toggle_server_tag == 0) return;
	
	level = is_red_led_on() + is_white1_led_on() + is_white2_led_on();
	if( level == 3 ){
		if( check_led_toggle_level == 0 ){
			check_led_toggle_counter++;
		}
		check_led_toggle_level = level;
	}else{
		check_led_toggle_level = 0;
	}
	
}

void check_led_toggle_server_stop()
{
	check_led_toggle_server_tag = 0;
}

int is_check_led_toggle_server_start()
{
	return check_led_toggle_server_tag;
}







#define FALSE_LED_ON() GPIOB->BSRR = GPIO_Pin_8;
#define FALSE_LED_OFF() GPIOB->BRR = GPIO_Pin_8;
#define PASS_LED_ON() 	GPIOB->BSRR = GPIO_Pin_5;
#define PASS_LED_OFF()	GPIOB->BRR = GPIO_Pin_5;
#define RUNING_LEN_ON()	GPIOB->BSRR = GPIO_Pin_6;
#define RUNING_LEN_OFF()	GPIOB->BRR = GPIO_Pin_6;

void led_pin_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	FALSE_LED_OFF();
	PASS_LED_OFF();
	RUNING_LEN_ON();
}








/*
出厂时仪表波特率是9600,可以自行设置
每台设备出厂时通迅地址都是0x00

A:单次读取设备数据(设备接收到此命令后,向主机发送1次数据后回到等待状态)
0x88+0xAE+设备地址+0x11  实例88AE0011

B:连续读取设备数据(设备接收到此命令后,将以每秒6组的速度向主机连续发送数据)
0x88+0xAE+设备地址+0x21

C:停止设备输出数据,此命令不管当前工作的是哪一台设备,都将停止输出
0x88+0xAE+任意字节+0x01

D:设备返回给主机(PC)数据格式
0xFA+0xFB+设备地址+数据字节0+数据字节1+数据字节2+数据字节3+校验字节
数据由4个字节组成(注意:靠左的是低字节和习惯上是反的),接收后需要转换成浮点数才能对应设备显示数据.校验字节是4个数据字节相加的结果(进位舍去)
*/
typedef struct miniMeterCoder {
	unsigned char packet[8];
	unsigned char index;
	unsigned char addr;
	unsigned char started;
	float 				value;
	Uart_t				*uart;
	
}miniMeterCoder_t;

void miniMeterCoder_init(miniMeterCoder_t * coder, Uart_t* uart)
{
	int i;
	for( i=0; i< sizeof(coder->packet); i++)
		coder->packet[i]=0;
	coder->index = 0;
	coder->addr = 0;
  coder->uart = uart;
	coder->started = 0;
}

int miniMeterCoder_prase( miniMeterCoder_t * coder, unsigned char data )
{
	unsigned char crc;
	unsigned char *value ;
	
	if( coder->index == 0  && data != 0xFA ) return 0;
	if( coder->index == 1 && data != 0xFB ) { coder->index = 0; return 0; }
	if( coder->index == 2  && data != coder->addr ) { coder->index = 0; return 0; }
	
	coder->packet[ coder->index++ ] = data;
	
	if( coder->index < sizeof(coder->packet) ) return 0;
	
	//check crc
	coder->index = 0;
	crc = coder->packet[3]+coder->packet[4]+coder->packet[5]+coder->packet[6];
	if( crc == coder->packet[7] ) {
		//coder->value = *((float*) &coder->packet[3] );
		value = (unsigned char *)&coder->value;
		value[0] = coder->packet[6];
		value[1] = coder->packet[5];
		value[2] = coder->packet[4];
		value[3] = coder->packet[3];
		return 1;
	}
	return 0;
}

int miniMeterCoder_read(miniMeterCoder_t * coder, float *value)
{
	unsigned char packget[8];
	int i,count;
	char res = 0;

	count = Uart_Get( coder->uart	, packget, sizeof( packget ) );
	if( count == 0 ) return 0;
	
	for( i=0; i< count ; i++){
		if( 1 == miniMeterCoder_prase( coder , packget[i] ) ){
			//_LOG( "%.5f" , coder->value);
			*value = coder->value;
			res = 1;
		}
	}
	
	return res;
}

void miniMeterCoder_clear( miniMeterCoder_t * coder )
{
	unsigned char tmp[8];
	
	coder->index = 0;
	if( coder->uart != NULL )
		while(  Uart_Get( coder->uart	, tmp, sizeof(tmp) ) >= sizeof(tmp) );
		//Uart_Clear_Rx( coder->uart );
}


#define A_METER_UART &Uart1
#define V_METER_UART &Uart3
miniMeterCoder_t currentMeter;
miniMeterCoder_t voltageMeter;

void miniMeter_init()
{
	miniMeterCoder_init( &currentMeter , A_METER_UART);
	miniMeterCoder_init( &voltageMeter , V_METER_UART);
}

void miniMeter_start(miniMeterCoder_t * coder)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x21};
	
	if( coder->started == 0 )
		Uart_Put(coder->uart, packget, sizeof( packget) );
	coder->started = 1;
}

void miniMeter_stop(miniMeterCoder_t * coder)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x01};

	if( coder->started == 1 )
		Uart_Put(coder->uart, packget, sizeof( packget) );
	
	coder->started = 0;
}

void miniMeter_toggle(miniMeterCoder_t * coder)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x11};
	
	Uart_Put(coder->uart, packget, sizeof( packget) );
}



volatile char currentCali_server_tag = 0;
volatile float current_sum;
volatile float	current_counter;
void currentCali_server_start()
{
	miniMeterCoder_clear(&currentMeter );
	miniMeter_start( &currentMeter );
	currentCali_server_tag = 1;
	current_sum=0;
	current_counter=0;
}

void currentCali_server_even()
{
	float value;
	
	if( currentCali_server_tag == 0 ) return;
	
	if( 1 == miniMeterCoder_read(&currentMeter, &value) ){
		//ignore 0 ?
		//if( value <= 0.0 ) return ;
		// sum up
		current_sum += value;
		current_counter +=1 ;
		_LOG("currentCali C=%f, count=%f \n",value,current_counter);
	}
}
void currentCali_server_stop()
{
	//miniMeter_stop( &currentMeter );
	currentCali_server_tag = 0;
}
int is_currentCali_server_start()
{
	return currentCali_server_tag;
}



volatile char voltageCali_server_tag = 0;
volatile float voltage_sum;
volatile float	voltage_counter;
void voltageCali_server_start()
{
	miniMeterCoder_clear(&voltageMeter );
	miniMeter_start( &voltageMeter );
	voltageCali_server_tag = 1;
	voltage_sum=0;
	voltage_counter=0;
}

void voltageCali_server_even()
{
	float value;
	
	if( voltageCali_server_tag == 0 ) return;
	
	if( 1 == miniMeterCoder_read(&voltageMeter, &value) ){
		//ignore 0 ?
		//if( value <= 0.0 ) return ;
		// sum up
		voltage_sum += value;
		voltage_counter+=1;
		_LOG("VolCali V=%f, count=%f \n",value,voltage_counter);
	}
}
void voltageCali_server_stop()
{
	voltageCali_server_tag = 0;
	//miniMeter_stop( &voltageMeter );
}
int is_voltageCali_server_start()
{
	return voltageCali_server_tag;
}

















#define VALVE_S1_ON()				GPIOA->BSRR = GPIO_Pin_7
#define VALVE_S1_OFF()			GPIOA->BRR = GPIO_Pin_7
#define VALVE_S2_ON()			GPIOA->BSRR = GPIO_Pin_6
#define VALVE_S2_OFF()			GPIOA->BRR = GPIO_Pin_6
#define VALVE_5V_ON()			GPIOA->BSRR = GPIO_Pin_15
#define VALVE_5V_OFF()			GPIOA->BRR = GPIO_Pin_15
#define VALVE_COIL_PANEL_ON()			GPIOA->BSRR = GPIO_Pin_1
#define VALVE_COIL_PANEL_OFF()			GPIOA->BRR = GPIO_Pin_1
#define VALVE_RECEIVE_PANEL_ON()			GPIOA->BSRR = GPIO_Pin_3
#define VALVE_RECEIVE_PANEL_OFF()			GPIOA->BRR = GPIO_Pin_3
#define VALVE_TEST_MODE_ON()			GPIOA->BSRR = GPIO_Pin_2
#define VALVE_TEST_MODE_OFF()			GPIOA->BRR = GPIO_Pin_2

void valve_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1 | GPIO_Pin_15 | GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	VALVE_S1_OFF();
	VALVE_S2_OFF();
	VALVE_5V_OFF();
	VALVE_COIL_PANEL_OFF();
	VALVE_RECEIVE_PANEL_OFF();
	VALVE_TEST_MODE_OFF();
}








systick_time_t heart_led_timer;
BitAction heart_led;

void heart_led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	systick_init_timer(&heart_led_timer, 500);
	heart_led = Bit_RESET;
	GPIO_WriteBit( GPIOC, GPIO_Pin_13 , heart_led);
	
}

void heart_led_even()
{
	
	
	if( systick_check_timer( &heart_led_timer ) ){
		heart_led = heart_led==Bit_RESET ? Bit_SET: Bit_RESET ;
		GPIO_WriteBit( GPIOC, GPIO_Pin_13 , heart_led);
	}
}


/*
volatile int launch_switch;
volatile int pump_switch;

void Switch_irq_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	launch_switch = 0;
	pump_switch = 0;
	
}


void EXTI0_IRQHandler()
{
	launch_switch = launch_switch==0 ?1:0;
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler()
{
	pump_switch = 1;
	EXTI_ClearITPendingBit(EXTI_Line1);
}
*/





#define SW_INTERVAL_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
#define SW_INTERVAL_MS 20
systick_time_t sw_timer;
volatile struct switcher left_key,right_key;


void switch_init()
{
	switcher_init( &left_key, SW_INTERVAL_COUNT, 1 , 0 , GPIOB, GPIO_Pin_0, NULL , NULL);
	switcher_init( &right_key, SW_INTERVAL_COUNT, 1, 0 , GPIOB, GPIO_Pin_1 , NULL, NULL);
	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &left_key );
	switcher_interval_check( &right_key );
}











#define MS_COIL_PANEL_MOVE   2000
#define MS_RECEIVE_PANEL_MOVE 2000

enum SERVER_STEP{
	STEP_IDLE =0,
	STEP_PULL_DOWM_COIL_PANEL, 
	STEP_POWER_5V_UP,		
	STEP_CAIL_CURRENT,	
	STEP_CHECK_LED_TOGGLE,	
	STEP_PULL_DOWN_RECEIVE_PANEL, 
	STEP_CHECK_LED_ON,
	STEP_CHECK_VOL_CURRENT,
	STEP_S1_ON,
	STEP_S2_ON,
	STEP_CHECK_FINAL_CURRENT,
	STEP_FINISH
};

volatile enum SERVER_STEP test_step ;
volatile int server_start_tag;
systick_time_t delay_timer;

volatile int hold_step = STEP_FINISH;//STEP_FINISH;


void service_break()
{	
	VALVE_5V_OFF();
	systick_delay_ms(500);
	VALVE_RECEIVE_PANEL_OFF();
	//systick_delay_ms( 100 );
	VALVE_COIL_PANEL_OFF();
	//systick_delay_us( MS_COIL_PANEL_MOVE *1000 );
	VALVE_S1_OFF();
	VALVE_S2_OFF();
	VALVE_TEST_MODE_OFF();
	
	test_step = STEP_IDLE;
	server_start_tag = 0;
	systick_init_timer( &delay_timer, 10 );
	
	currentCali_server_stop();
	voltageCali_server_stop();
	check_led_toggle_server_stop();
}


void service_init()
{
	VALVE_5V_OFF();
	systick_delay_ms( 1000  );
	VALVE_RECEIVE_PANEL_OFF();
	//systick_delay_ms( MS_RECEIVE_PANEL_MOVE );
	VALVE_COIL_PANEL_OFF();
	//systick_delay_ms( MS_COIL_PANEL_MOVE );
	VALVE_S1_OFF();
	VALVE_S2_OFF();
	VALVE_TEST_MODE_OFF();
	
	test_step = STEP_IDLE;
	server_start_tag = 0;
	systick_init_timer( &delay_timer, 10 );
	
	currentCali_server_stop();
	voltageCali_server_stop();
	check_led_toggle_server_stop();
}


void service_even()
{
	float value;
	
	if( left_key.state == left_key.press_level || right_key.state == right_key.press_level ){
		if( server_start_tag == 2 ) {
			//stop current operation
			_LOG("switch stop\n");
			service_break();
			
		}else if( left_key.state == left_key.press_level && right_key.state == right_key.press_level ){
			//prepare start as two switch is on
			server_start_tag = 1;
			//_LOG("switch toggle\n");
		}
	}else{
		if( server_start_tag == 1 ){
			server_start_tag = 2;//start as tow switch is toggled
			_LOG("switch start\n");
		}
	}
	
	if( 0 == systick_check_timer( &delay_timer ) ){
		return;
	}
	
	if( hold_step < test_step ) return;
	
	switch( test_step ){
		case STEP_IDLE:{
			if( server_start_tag == 2 ){
				//start test
				test_step++;
			}
			break;
		}
		case STEP_PULL_DOWM_COIL_PANEL:{
			PASS_LED_OFF();
			FALSE_LED_OFF();
			VALVE_COIL_PANEL_ON();
			systick_init_timer( &delay_timer, MS_COIL_PANEL_MOVE );
			test_step++;
			_LOG("******** STEP_PULL_DOWM_COIL_PANEL : Start\n");
			break;
		}
		case STEP_POWER_5V_UP:{
			VALVE_5V_ON();
			systick_init_timer( &delay_timer, 1000 );
			test_step++;
			_LOG("******** STEP_POWER_5V_UP : Start\n");
			break;
		}

		case STEP_CAIL_CURRENT:{
			if( !is_currentCali_server_start() ){
				//start
				currentCali_server_start();
				systick_init_timer( &delay_timer, 4000 );
				_LOG("******** STEP_CAIL_CURRENT Start...\n");
			}else{
				//check result
				currentCali_server_stop();
				if( current_counter > 5 ){
					value = current_sum/current_counter;
					if( value > 0.0000 && value <= 0.001 ){
						test_step++;
						systick_init_timer( &delay_timer, 1);
						_LOG("STEP_CAIL_CURRENT OK\n");
						break;
					}
				}else{
					_LOG("STEP_CAIL_CURRENT : no current data\n");
				}
				_LOG("STEP_CAIL_CURRENT : false c=%f\n",value);
				FALSE_LED_ON();
				test_step = STEP_FINISH;
				systick_init_timer( &delay_timer, 100 );
			}
			break;
		}
		case STEP_CHECK_LED_TOGGLE:{
			if( !is_check_led_toggle_server_start() ){
				check_led_toggle_server_init();
				systick_init_timer( &delay_timer, 3000 );
				_LOG("******** STEP_CHECK_LED_TOGGLE : Start\n");
			}else{
				check_led_toggle_server_stop();
				if( check_led_toggle_counter == 3 ){
					test_step++;
					systick_init_timer( &delay_timer, 1);
					_LOG("STEP_CHECK_LED_TOGGLE : OK\n");
				}else{
					_LOG("STEP_CHECK_LED_TOGGLE : false\n");
					FALSE_LED_ON();
					test_step = STEP_FINISH;
					systick_init_timer( &delay_timer, 100 );
				}
			}
			break;
		}
		case STEP_PULL_DOWN_RECEIVE_PANEL:{ // place the receive 
			VALVE_RECEIVE_PANEL_ON();
			systick_init_timer( &delay_timer, MS_RECEIVE_PANEL_MOVE + 500); // 500 for led light
			test_step++;
			_LOG("******** STEP_PULL_DOWN_RECEIVE_PANEL : Start\n");
			break;
		}
		
		case STEP_CHECK_LED_ON:{ // 2 white led is on , wait at least 3s before next step 
			if( 1 == is_white1_led_on() && 1 == is_white2_led_on() && 0==is_red_led_on() ){
				test_step++;
				systick_init_timer( &delay_timer, 3000);
				_LOG("******** STEP_CHECK_LED_ON : OK\n");
			}else{
				_LOG("STEP_CHECK_LED_ON : false\n");
				FALSE_LED_ON();
				test_step = STEP_FINISH;
				systick_init_timer( &delay_timer, 100 );
			}
			break;
		}
		case STEP_CHECK_VOL_CURRENT:{  // V=[5,5.5] A= [1.4,1.7]
			if( !is_currentCali_server_start() && !is_voltageCali_server_start() ){
				currentCali_server_start();
				voltageCali_server_start();
				systick_init_timer( &delay_timer, 2000 );
				_LOG("******** STEP_CHECK_VOL_CURRENT : Start\n");
			}else{
				voltageCali_server_stop();
				currentCali_server_stop();
				if( current_counter > 5 && voltage_counter > 5 ){
					value = current_sum/current_counter;
					if( value > 1.7 || value < 1.4 ){
						_LOG("STEP_CHECK_VOL_CURRENT : current[1.4,1.7] false (%fA)\n",value);
					}else{
						value = voltage_sum/voltage_counter;
						if( value > 5.5 || value < 5.0 ){
							_LOG("STEP_CHECK_VOL_CURRENT : voltage[5.0,5.5] false (%fV)\n",value);
						}else{
							test_step++;
							systick_init_timer( &delay_timer, 100 );
							_LOG("STEP_CHECK_VOL_CURRENT : OK\n");
							break;
						}
					}
				}else{
					_LOG("STEP_CHECK_VOL_CURRENT : leak data  current(%d),voltage(%d)\n",current_counter,voltage_counter);
				}
				_LOG("STEP_CHECK_VOL_CURRENT : false\n");
				FALSE_LED_ON();
				test_step = STEP_FINISH;
				systick_init_timer( &delay_timer, 100 );
			}
			break;
		}
		case STEP_S1_ON:{ // red led is on 
			if( !is_check_led_toggle_server_start() ){
				VALVE_S1_ON();
				VALVE_S2_OFF();
				check_led_toggle_server_init();
				systick_init_timer( &delay_timer, 500 );
				_LOG("******** STEP_S1_ON : Start\n");
			}else{
				check_led_toggle_server_stop();
				if( is_red_led_on() ){
					test_step++;
					_LOG("STEP_S1_ON : OK\n");
				}else{
					_LOG("STEP_S1_ON : false\n");
					FALSE_LED_ON();
					test_step = STEP_FINISH;
				}
				VALVE_S1_OFF();
				VALVE_S2_OFF();
				systick_init_timer( &delay_timer, 100 );
			}
			break;
		}	
		case STEP_S2_ON:{ // all leds are off, current= [0.1mA,1mA]
			if( !is_currentCali_server_start() ){
				//start
				VALVE_S1_OFF();
				VALVE_S2_ON();
				systick_delay_ms(1000);
				currentCali_server_start();
				systick_init_timer( &delay_timer, 2000 );
				_LOG("******** STEP_S2_ON : Start\n");
			}else{
				//check result
				currentCali_server_stop();
				if( is_red_led_on() || is_white1_led_on() || is_white2_led_on() ){
					_LOG("STEP_S2_ON : false,  led is light\n");
				}else{
					if( current_counter <=  0 ){
						_LOG("STEP_S2_ON : no current data\n");
					}else{
						value = current_sum/current_counter;
						if( value >= 0.0000 && value <= 0.001 ){
							test_step++;
							systick_init_timer( &delay_timer, 500); // 500ms for switch origan status for next test
							VALVE_S2_OFF();
							_LOG("STEP_S2_ON : OK\n");
							break;
						}else{
							_LOG("STEP_S2_ON :current[0.0001,0.001] false  %fA\n",value);
						}
						
					}
				}
				_LOG("STEP_S2_ON : false\n");
				FALSE_LED_ON();
				test_step = STEP_FINISH;
				systick_init_timer( &delay_timer, 100 );
				VALVE_S2_OFF();
			}
			break;
		}
		case STEP_CHECK_FINAL_CURRENT:{  //all leds are off(3s内), current= [0.1mA,1mA]
			if( !is_currentCali_server_start() ){
				currentCali_server_start();
				systick_init_timer( &delay_timer, 2000 );
				_LOG("******** STEP_CHECK_FINAL_CURRENT: Start\n");
			}else{
				//check result
				currentCali_server_stop();
				if( is_red_led_on() || is_white1_led_on() || is_white2_led_on() ){
					_LOG("STEP_CHECK_FINAL_CURRENT :  led is on \n");
				}else{
					if( current_counter > 0 ){
						value = current_sum/current_counter;
						if( value > 0.0000 && value <= 0.001 ){
							test_step++;
							systick_init_timer( &delay_timer, 1);
							PASS_LED_ON();
							_LOG("STEP_CHECK_FINAL_CURRENT: OK\n");
							break;
						}else{
							_LOG("STEP_CHECK_FINAL_CURRENT : current[0.0001,0.001] false  %fA\n",value);
						}
					}else{
						_LOG("STEP_CHECK_FINAL_CURRENT : no current data\n");
					}
				}
				_LOG("STEP_CHECK_FINAL_CURRENT : false\n");
				FALSE_LED_ON();
				test_step = STEP_FINISH;
				systick_init_timer( &delay_timer, 100 );
			}
			break;
		}		
		case STEP_FINISH:{
			_LOG("STEP_FINISH: finish\n");
			service_init();
			break;
		}
		default:{
			service_init();
			_LOG("Server: ERROR ! get a unknow setp\n");
			break;
		}
	}
}









void cmd_even()
{
	static char buffer[4]={0};
	static char index = 0;
	
	//if( Uart_Get( CONSOLE_UART , (unsigned char*) &buffer[index], 1 ) ){
	if( USB_RxRead((unsigned char*) &buffer[index], 1 ) ){
		if( buffer[index] == ';' ){
			if( index == 2 ){
				switch( buffer[index -2 ]){
					case '1':
						if( buffer[index-1]== '1' ){
							VALVE_S1_ON();
							_LOG("1-S1 ON \n");
						}else{
							VALVE_S1_OFF();
							_LOG("1-S1 OFF \n");
						}
						break;
					case '2':
						if( buffer[index-1]== '1' ){
							VALVE_S2_ON();
							_LOG("2-S2 ON \n");
						}else{
							VALVE_S2_OFF();
							_LOG("2-S2 OFF \n");
						}
						break;								
					case '3':
						if( buffer[index-1]== '1' ){
							VALVE_5V_ON();
							_LOG("3-5V ON \n");
						}else{
							VALVE_5V_OFF();
							_LOG("3-5V OFF \n");
						}
						break;		

						
					case '4':
						if( buffer[index-1]== '1' ){
							VALVE_COIL_PANEL_ON();
							_LOG("4-VALVE_COIL_PANEL ON \n");
						}else{
							VALVE_COIL_PANEL_OFF();
							_LOG("4-VALVE_COIL_PANEL OFF \n");
						}
						break;						
					case '5':
						if( buffer[index-1]== '1' ){
							VALVE_RECEIVE_PANEL_ON();
							_LOG("5-VALVE_RECEIVE_PANEL ON \n");
						}else{
							VALVE_RECEIVE_PANEL_OFF();
							_LOG("5-VALVE_RECEIVE_PANEL OFF \n");
						}
						break;
					case '6':
						if( buffer[index-1]== '1' ){
							VALVE_TEST_MODE_ON();
							_LOG("VALVE_TEST_MODE_ON ON \n");
						}else{
							VALVE_TEST_MODE_OFF();
							_LOG("VALVE_TEST_MODE_ON OFF \n");
						}
						//	voltageCali_server_start();
						//	currentCali_server_start();
					break;
					case '7':
							if( buffer[index-1] == 'r') {RUNING_LEN_ON();}
							else if(buffer[index-1] == 'p') {PASS_LED_ON();}
							else if(buffer[index-1] == 'f') {FALSE_LED_ON();}
							break;
					case '8':
							if( buffer[index-1] == 'r') {RUNING_LEN_OFF();}
							else if(buffer[index-1] == 'p') {PASS_LED_OFF();}
							else if(buffer[index-1] == 'f') {FALSE_LED_OFF();}
					break;
							
				  case 's':
							if( buffer[index -1 ] == '+' ) hold_step++;
							else if ( buffer[index -1 ] == '-' ) hold_step--;
							else if( buffer[index -1 ] > '0' && buffer[index -1 ] <= '9' ){
								hold_step = buffer[index -1 ] - '0';
							}
					break;
				}
			}
			index = 0;
			return;
		}
		index++;
		index = index % 4;
	}
}




void app_init()
{
	valve_init();
	
	Uart_init();

#if BOARD_HAS_IAP
	#if IAP_PORT_USB 
	iap_init_in_usb();
	#endif
	#if IAP_PORT_CAN1
	iap_init_in_can1();
	#endif
	#if  IAP_PORT_UART
	iap_init_in_uart( IAP_UART );
	#endif
#endif
	
	//console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	

	heart_led_init();
	LED_Detection_init();
	led_pin_config();
	miniMeter_init();
	//Switch_irq_init();
	service_init();

	switch_init();

}


void app_event()
{
	switch_even();
	check_led_toggle_server_event();
	currentCali_server_even();
	voltageCali_server_even();
	heart_led_even();
	service_even();
	
	cmd_even();
	/*
	if( voltage_counter >= 6 ){
		voltageCali_server_stop();
		_LOG("Vmeter=%f\n",voltage_sum/voltage_counter);
		voltageCali_server_start();
	}
	if( current_counter >= 6 ){
		currentCali_server_stop();
		_LOG("Ameter=%f\n",current_sum/current_counter);
		currentCali_server_start();
	}
	*/
}










#endif
