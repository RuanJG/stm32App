#include "stm32f10x.h"

/*
*   timer : TIM1 TIM2 ...
*   channel : 1 2 3 4 -1 -2 -3 -4
*   high1_low0 : positive or nagetive
*/
int pwm_init( TIM_TypeDef* timer , int channel, unsigned short period, int freq_hz , int high1_low0, int enable_OCx_irq)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIMOCInitStructure;
	unsigned short prescaler ;
	unsigned int clk  ;//36M

	clk = SystemCoreClock; // 72M
	// freq = [36M/(prescaler(16bit)+1)] / period 
	// preriod = 1000 , freq= 1000 Hz , prescaler = 36
	prescaler = clk/period/freq_hz;
	//printf( "clk=%d, period=%d, freq=%d Hz, prescal=%d \n", clk,period, freq_hz, prescaler);

	
	if( timer == TIM1 ){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //APB2 72M clk
	}else if( timer == TIM2 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //APB1 36M clk
	}else if( timer == TIM3 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //APB1 36M clk
	}else if( timer == TIM4 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //APB1 36M clk
	}
	

	TIM_Cmd(timer, DISABLE);
	//TIM_DeInit(timer);
	TIM_InternalClockConfig(timer);
	
	//base config
	TIM_TimeBaseStructure.TIM_Period= period-1;
	TIM_TimeBaseStructure.TIM_Prescaler= prescaler-1 ;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

	//pwm
	TIMOCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // pwm mode 1
	TIMOCInitStructure.TIM_Pulse = 0 ;//default 0%
	TIMOCInitStructure.TIM_OCPolarity = high1_low0==1? TIM_OCPolarity_High:TIM_OCPolarity_Low ;
	if( channel > 0 ){
		TIMOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		//TIMOCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	}else{
		TIMOCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		//TIMOCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		channel *= -1;
	}
	
	switch( channel ){
	case 1:
		TIM_OC1Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC1,ENABLE );
		break;
	case 2:
		TIM_OC2Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC2,ENABLE );
		break;
	case 3:
		TIM_OC3Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC3,ENABLE );
		break;
	case 4:
		TIM_OC4Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC4,ENABLE );
		break;
	default:
		return -1;
	}

	TIM_CtrlPWMOutputs(timer,ENABLE);
	TIM_Cmd(timer, ENABLE);
	return 0;
}



/*
*   timer : TIM1 TIM2 ...
*   channel : 1 2 3 4 -1 -2 -3 -4
*/
int pwm_set( TIM_TypeDef* timer , int pchannel, unsigned short pwm)
{
	int channel;
	channel = pchannel<0 ? -1*pchannel : pchannel;
	
	switch( channel ){
	case 1:
		TIM_SetCompare1( timer , pwm );
		break;
	case 2:
		TIM_SetCompare2( timer , pwm );
		break;
	case 3:
		TIM_SetCompare3( timer , pwm );
		break;
	case 4:
		TIM_SetCompare4( timer , pwm );
		break;
	default:
		return -1;
	}
	return 0; 
}


void pwm_set_freq( TIM_TypeDef* timer , unsigned short freq)
{
	unsigned short period;
	period = SystemCoreClock / (timer->PSC+1) / freq; 
	TIM_SetAutoreload( timer , period );
}

int pwm_restart(  TIM_TypeDef* timer , int pchannel, unsigned short pwm)
{
	
	TIM_Cmd(timer, DISABLE);
	//TIM_CtrlPWMOutputs( timer, DESABLE );
	
	pwm_set( timer, pchannel, pwm);
	
	TIM_Cmd(timer, ENABLE);
	//TIM_CtrlPWMOutputs( timer, ENABLE );
	
	return 0; 
}


/*
*		this function just init the timer , you need init gpio and NVIC&irq_function
*   timer : TIM1 TIM2 ...
*   tim_channel : TIM_Channel_1 TIM_Channel_2 TIM_Channel_3 TIM_Channel_4
*/

int pwm_input( TIM_TypeDef* timer , unsigned short period, int freq_hz , int tim_channel)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	unsigned short prescaler ;
	unsigned int clk  ;//36M

	clk = SystemCoreClock; // 72M
	// freq = [36M/(prescaler(16bit)+1)] / period 
	// preriod = 1000 , freq= 1000 Hz , prescaler = 36
	prescaler = clk/period/freq_hz;
	//printf( "clk=%d, period=%d, freq=%d Hz, prescal=%d \n", clk,period, freq_hz, prescaler);
	
	if( timer == TIM1 ){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //APB2 72M clk
	}else if( timer == TIM2 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //APB1 36M clk
	}else if( timer == TIM3 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //APB1 36M clk
	}else if( timer == TIM4 ) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //APB1 36M clk
	}else{
		return -1;
	}
	
	TIM_Cmd(timer, DISABLE);
	TIM_DeInit(timer);
	TIM_InternalClockConfig(timer);
	
	//base config
	TIM_TimeBaseStructure.TIM_Period= period-1;
	TIM_TimeBaseStructure.TIM_Prescaler= prescaler-1 ;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	
	// capture config
	if ( ! IS_TIM_PWMI_CHANNEL( tim_channel ) ){
		return -1;
	}
	TIM_ICInitStructure.TIM_Channel = tim_channel; //ͨ��ѡ�� , this channel( 1 or 2 ) is capture one period tick , and other channel( 2 or 1 ) capture the duty tick 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ش���, the other channel is Falling polarity
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�����������һ�β������������ź�Ƶ��û�б䣬��õ�����Ҳ����䡣����ѡ��4��Ƶ����ÿ�ĸ��������ڲ���һ�β��������������źű仯��Ƶ��������£����Լ�������������жϵĴ�����
	TIM_ICInitStructure.TIM_ICFilter = 0x0; //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_PWMIConfig(timer, &TIM_ICInitStructure); //���ݲ�������TIM������Ϣ

	if( tim_channel == TIM_Channel_2 ){
		TIM_SelectInputTrigger(timer, TIM_TS_TI2FP2); //ѡ��IC2Ϊʼ�մ���Դ
	}else{
		TIM_SelectInputTrigger(timer, TIM_TS_TI1FP1);
	}
	TIM_SelectSlaveMode(timer, TIM_SlaveMode_Reset);//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
	TIM_SelectMasterSlaveMode(timer, TIM_MasterSlaveMode_Enable); //������ʱ���ı�������

	TIM_Cmd(timer, ENABLE); //����TIM2 
	if( tim_channel == TIM_Channel_2 ){
		TIM_ITConfig(timer, TIM_IT_CC2, ENABLE); //���ж� 
	}else{
		TIM_ITConfig(timer, TIM_IT_CC1, ENABLE);
	}
	TIM_ITConfig(timer, TIM_IT_Update, ENABLE);

	return 0;
}

