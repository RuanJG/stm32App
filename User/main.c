#include "stm32f0xx.h" 
#include "stdio.h"
#include "systick.h"
#include "switcher.h"


void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTBE;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_LATENCY;                         // Flash 1 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE_DIV1;                         // PCLK = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  //  PLL configuration:  = HSI/2 * 12 = 48 MHz
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);
#if defined(STM32F042x6) || defined(STM32F048xx)  || defined(STM32F070x6) \
 || defined(STM32F078xx) || defined(STM32F071xB)  || defined(STM32F072xB) \
 || defined(STM32F070xB) || defined(STM32F091xC) || defined(STM32F098xx)  || defined(STM32F030xC)
  /* HSI used as PLL clock source : SystemCoreClock = HSI/PREDIV * PLLMUL */
  RCC->CFGR2 =  (RCC_CFGR2_PREDIV_DIV2);
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI_PREDIV  | RCC_CFGR_PLLMUL12);
#else
  /* HSI used as PLL clock source : SystemCoreClock = HSI/2 * PLLMUL */
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);
#endif


  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}





//usart 

void Usart_Init(uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

    /*
    PA9-TX-推挽复用
    PA10-RX-浮空输入/上拉输入
    */
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);  

    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOA,&GPIO_InitStruct);

    /*USART基本配置*/
    USART_InitStruct.USART_BaudRate=BaudRate;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART2,&USART_InitStruct);

    /*使能接收中断*/
    //NVIC_Config(USART2_IRQn);
    //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART2,ENABLE);
}


uint8_t USART2_ReciverBuf(void)
{
     /*判断接收缓冲区是否为非空*/
    while(!USART_GetFlagStatus(USART2,USART_FLAG_RXNE));
    return USART_ReceiveData(USART2);
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART2,(uint8_t)ch);
    while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    return (ch);
}

void USART2_SendBuf(uint8_t *pBuf, uint32_t u32Len)
{
    while(u32Len--)
    {
        /*判断发送缓冲区是否为空*/
        while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
        USART_SendData(USART2,*pBuf++);
    }
}






/// led

#define LED_PASS_ON() GPIO_SetBits(GPIOC,GPIO_Pin_7)
#define LED_PASS_OFF() GPIO_ResetBits(GPIOC,GPIO_Pin_7)
#define LED_FAILUE_ON() GPIO_SetBits(GPIOC,GPIO_Pin_6)
#define LED_FAILUE_OFF() GPIO_ResetBits(GPIOC,GPIO_Pin_6)

systick_time_t led_timer;
volatile int led_flash;

void led_init()
{
	GPIO_InitTypeDef GPIO_Initialize;
	systick_init_timer( &led_timer, 300);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	
	GPIO_Initialize.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initialize.GPIO_OType = GPIO_OType_PP;
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_7;
	GPIO_Initialize.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Initialize.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOC,&GPIO_Initialize);
	
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC,&GPIO_Initialize);
	GPIO_SetBits(GPIOC,GPIO_Pin_6);
	
	led_flash = 0;
	LED_PASS_ON();
	LED_FAILUE_OFF();
	
}
	
void led_event()
{
	if( led_flash == 1  && systick_check_timer(&led_timer) ){
		if( GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6) == Bit_RESET){
			LED_FAILUE_ON();
			LED_PASS_ON();
		}else {
			LED_FAILUE_OFF();
			LED_PASS_OFF();
		}
	}
	
}
	
	

	




//////////////////////////  PWM

#define MOTOR_ON_PWM 9
#define MOTOR_OFF_PWM 0


// PA6 TIM3.1 PA7 TIM3.2

void PWM_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 50kHZ = 48M/960
	uint16_t Period = 10, duty = 0, Pcs = 96;



	//GPIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);       //F0还需要设置AF为功能1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);       //F0还需要设置AF为功能1

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);       //F0还需要设置AF为功能1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);       //F0还需要设置AF为功能1
	
	
	//base timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = Pcs-1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = Period -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//OC config
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = duty;
	
	// OC change config
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	//enable
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
}


void Motor1_Back ()
{
	// B
	TIM_SetCompare2(TIM3,0);
	// A
	TIM_SetCompare1(TIM3,MOTOR_ON_PWM);
}

void Motor1_Forward()
{
	// A
	TIM_SetCompare1(TIM3,0);
	// B
	TIM_SetCompare2(TIM3,MOTOR_ON_PWM);

}

void Motor1_Stop()
{
	// A
	TIM_SetCompare1(TIM3,0);
	// B
	TIM_SetCompare2(TIM3,0);
}

void Motor2_Back()
{
	// B
	TIM_SetCompare4(TIM3,0);
	// A
	TIM_SetCompare3(TIM3,MOTOR_ON_PWM);
}

void Motor2_Forward ()
{
	// A
	TIM_SetCompare3(TIM3,0);
	// B
	TIM_SetCompare4(TIM3,MOTOR_ON_PWM);

}

void Motor2_Stop()
{
	// A
	TIM_SetCompare3(TIM3,0);
	// B
	TIM_SetCompare4(TIM3,0);
}






#define ADC_SAMPLE_COUNT 32  // 2^5 = 32 , 32+2( min max )= 34  ; (sum-min-max)>>5 == (sum-min-max)/32
#define ADC_SAMPLE_CHANNEL_COUNT 1
volatile static unsigned short escADCConvert[ADC_SAMPLE_COUNT]={0};
volatile int adc_updated;

void ADC_Configuration ()
{										 
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	

	ADC_DeInit(ADC1);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_1); 
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution  = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode  = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign  = ADC_DataAlign_Right; //ADC_DataAlign_Left
	ADC_InitStructure.ADC_ScanDirection  = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);  

	ADC_ChannelConfig(ADC1, ADC_Channel_15, ADC_SampleTime_55_5Cycles);        
       

	ADC_GetCalibrationFactor(ADC1); 
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);     

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
	ADC_StartOfConversion(ADC1);



	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_CHANNEL_COUNT*ADC_SAMPLE_COUNT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)escADCConvert;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);


  /* Enable DMA channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
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


unsigned short Cali_Adc_Value()
{
	int i;
	unsigned int sensor = 0; 	
	for(i=0;i<ADC_SAMPLE_COUNT;i++)
	{
		sensor += escADCConvert[i];
	}
	sensor = sensor/ADC_SAMPLE_COUNT;
	//printf("adc=%d\n",sensor);
	//printf("c=%d\n",(int)((float)sensor*0.39894));
	//printf("adc=%d,v=%d\n",sensor,(int)(sensor*3300/4095/10.1));
	return sensor;
}

__inline unsigned short get_adc_loop()
{
	//printf("try get adc\n");
	while( adc_updated == 0 );
	adc_updated = 0;
	return Cali_Adc_Value();
}

#define ADC_MAX_VAL 0x0FFF
#define ADC_REFERENCE_VOLTAGE 3.3f
#define BATTRY_VOLTAGE_SCALE_MV 2000.0f
#define MOTOR_CURENT_GAIN 10.1f
#define MOTOR_CURRENT_RESISTOR_VALUE 0.2f

#define MOTOR_MAX_CURRENT_MILLIAMPS_FWD 450
#define MOTOR_MAX_CURRENT_MILLIAMPS_REV 200

//0.39894
static const float motorMilliampsPerCount = ((1.0f / ((float) ADC_MAX_VAL)) * ADC_REFERENCE_VOLTAGE / MOTOR_CURENT_GAIN / MOTOR_CURRENT_RESISTOR_VALUE) * 1000.0f;
unsigned short  ADC_MAX_FWD , ADC_MAX_REV;

__inline  uint16_t current_to_adc(float c)
{
    return (uint16_t)(c / motorMilliampsPerCount);
}









#define EN_POWER_ADC() GPIO_SetBits(GPIOC,GPIO_Pin_4)
#define DIS_POWER_ADC() GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define is_endStoper_Press (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1) == Bit_RESET)
#define is_motor_started (motor_status>0)
#define is_motor_forward (motor_status ==1)
#define is_motor_back  (motor_status == 2)
#define CHASSIS_SHOOT_ADC 310

volatile char motor_status;
volatile systick_time_t motor_timer;
volatile unsigned int motor_ms;

volatile unsigned int adc_record ;
volatile int adc_record_count;
	

void motor_control_start()
{
	motor_status = 1;
	EN_POWER_ADC();
	systick_delay_ms(5);
	Motor2_Forward();
	//Motor1_Forward();
	motor_ms = systick_get_ms();
	
	led_flash = 1;
	adc_record = 0;
	adc_record_count = 0;
}


void motor_control_stop()
{
	motor_status = 0;
	DIS_POWER_ADC();
	Motor2_Stop();
	Motor1_Stop();
	
	led_flash = 0;	
}

void motor_control_init()
{
	GPIO_InitTypeDef GPIO_Initialize;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOF,ENABLE);
	
	//adc power control pin
	GPIO_Initialize.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initialize.GPIO_OType = GPIO_OType_PP;
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_4;
	GPIO_Initialize.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP
	GPIO_Initialize.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOC,&GPIO_Initialize);
	
	// end stop key pin
	GPIO_Initialize.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_1;
	GPIO_Initialize.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Initialize.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOC,&GPIO_Initialize);
	
	//switch power
	GPIO_Initialize.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initialize.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Initialize.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Initialize.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOF,&GPIO_Initialize);
	GPIO_SetBits(GPIOF,GPIO_Pin_0);
	
	motor_status = 0;
	
	PWM_Config();
	
	ADC_Configuration ();
	
	ADC_MAX_FWD = current_to_adc(MOTOR_MAX_CURRENT_MILLIAMPS_FWD);
	ADC_MAX_REV = current_to_adc(MOTOR_MAX_CURRENT_MILLIAMPS_REV); 
	
	printf("FWD_ADC:%d, BACK_ADC:%d\n",ADC_MAX_FWD,ADC_MAX_REV);
	
	motor_control_stop();
	
}

void motor_control_even()
{
	unsigned short adc;
	int next=0;
	unsigned int ms;
	
	if( !is_motor_started ) return;
	
	
	next = 0;
	if( is_motor_forward )
	{
		ms = systick_get_ms();

		//check
		if(  ms >=  (motor_ms+500)  )
		{
			adc = get_adc_loop();
			if( adc > CHASSIS_SHOOT_ADC ) 
			{
				adc_record += adc;
				adc_record_count ++;
			}
			if( adc >= ADC_MAX_FWD )
			{
				motor_control_stop();
				LED_FAILUE_ON();
				LED_PASS_OFF();
			}else{				
				if( ms >= ( motor_ms + 1500 ) )
				{
					if( adc_record_count > 1 ){
						//have shoot 
						next = 1;
					}else{
						// without shoot
						motor_control_stop();
						LED_FAILUE_ON();
						LED_PASS_OFF();
					}
				}
			}
			
		}
		
		//result
		if( next == 1)
		{
			//printf("fw end\n");
			Motor2_Stop();
			Motor1_Stop();
			motor_status = 2;
			systick_delay_ms(500); //wait 0.5s
			Motor2_Back();
			//Motor1_Back();
			motor_ms = systick_get_ms();
			//printf("back start\n");
		}
		
	}
	else if( is_motor_back )
	{
		ms = systick_get_ms();
		
		if( ms > (motor_ms+150) )
		{
			adc = get_adc_loop();
			if( adc >= ADC_MAX_REV )
				next = 1;

			if( ms >= ( motor_ms + 4000 ) )
				next = 1;
		}
		
		//result
		if( next == 1)
		{
			//printf("back end\n");
			motor_control_stop();
			if( is_endStoper_Press ) // back in a right position
			{
				LED_PASS_ON();
				LED_FAILUE_OFF();
			}else{
				LED_FAILUE_ON();
				LED_PASS_OFF();
			}

		}
				
	}
	else
	{
		printf("motor control : unknow status \n");
		motor_control_stop();
	}

	
}



#define SW_INTERVAL_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
#define SW_INTERVAL_MS 5
systick_time_t sw_timer;
volatile struct switcher startButton;

void switch_key_press_handler()
{

}

void switch_key_release_handler()
{
	//printf("key press\n");
	if( !is_motor_started )
		motor_control_start();
	else
		motor_control_stop();
}

void switch_init()
{
	GPIO_InitTypeDef GPIO_Initialize;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOF,ENABLE);
	//switch power
	GPIO_Initialize.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initialize.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initialize.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Initialize.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Initialize.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOF,&GPIO_Initialize);
	GPIO_SetBits(GPIOF,GPIO_Pin_0);
	
	switcher_init( &startButton, SW_INTERVAL_COUNT, 1 , 0 , GPIOC, GPIO_Pin_2, switch_key_press_handler , switch_key_release_handler);

	systick_init_timer( &sw_timer, SW_INTERVAL_MS );
}

__inline void switch_even()
{
	if( 0 == systick_check_timer( &sw_timer ) ) return;
	
	switcher_interval_check( &startButton );

}




void app_init()
{
	SystemCoreClockConfigure();
	systick_init();
	Usart_Init( 57600 );
	led_init();
	motor_control_init();
	switch_init();
}


__inline void app_event()
{
	systick_event();
	switch_even();
	motor_control_even();
	led_event();
}




















int main(void){

	app_init();
	
	while(1){
		app_event();
	}
}
