#ifndef PWM_H_
#define PWM_H_

int pwm_init( TIM_TypeDef* timer , int channel, unsigned short period, int freq_hz , int high1_low0, int enable_OCx_irq);
int pwm_set( TIM_TypeDef* timer , int pchannel, unsigned short pwm);
int pwm_restart(  TIM_TypeDef* timer , int pchannel, unsigned short pwm);
/*
*		this function just init the timer , you need init gpio and NVIC&irq_function
*   timer : TIM1 TIM2 ...
*   tim_channel : TIM_Channel_1 TIM_Channel_2 TIM_Channel_3 TIM_Channel_4
*/
int pwm_input( TIM_TypeDef* timer , unsigned short period, int freq_hz , int tim_channel);
void pwm_set_freq( TIM_TypeDef* timer , unsigned short freq);
#endif 










/* example





void pwm_output_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	pwm_init( TIM4, 3, 1000 , 50 , 1 );
	pwm_set( TIM4 , 3 , 1000/2 );
}


int pwm_capture_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if( 0 > pwm_input( TIM3 , 1000, 25 , TIM_Channel_2) ){
		_LOG("pwm input init error \n");
		return -1;
	}
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	return 0;
}

volatile uint16_t period,duty;
volatile int pwm_capture_availed = 0;

void TIM3_IRQHandler(void)
{
	if( SET == TIM_GetITStatus(TIM3, TIM_IT_CC2 ) )
	{
		period = TIM_GetCapture2(TIM3);
		if( period > 0 )
		{
			duty = TIM_GetCapture1(TIM3);
			pwm_capture_availed = 1;
		}else{
			//break here
			period = 0;
			duty = 0;
			pwm_capture_availed = 0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}else if( SET == TIM_GetITStatus(TIM3 , TIM_IT_Update) ){
		period = TIM_GetCapture2(TIM3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}else{
		//break here
		TIM_ClearITPendingBit(TIM3, 0xFF);
	}
}
 
 
void pwm_check_event()
{
	int diff;
	
	if( pwm_capture_availed ){
		pwm_capture_availed = 0;
		//_LOG("pwm: %d,%d\n", period,duty);
		diff = duty*2 - period;
		diff = diff > 0 ? diff:(-1*diff);
		if( diff < 5 ){
			GreenLed_On();
			_LOG("1");
		}else{
			GreenLed_Off();
			_LOG("0");
		}
	}else{
		GreenLed_Off();
	}
}
 








*/
