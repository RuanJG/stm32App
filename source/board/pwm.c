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
		//if( enable_preload == 1 ) TIM_OC1PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 2:
		TIM_OC2Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC2,ENABLE );
		//if( enable_preload == 1 ) TIM_OC2PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 3:
		TIM_OC3Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC3,ENABLE );
		//if( enable_preload == 1 ) TIM_OC3PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 4:
		TIM_OC4Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC4,ENABLE );
		//if( enable_preload == 1 ) TIM_OC4PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	default:
		return -1;
	}
	
	//if( enable_preload == 1 ) TIM_ARRPreloadConfig( timer, ENABLE);

	TIM_CtrlPWMOutputs(timer,ENABLE);
	TIM_Cmd(timer, ENABLE);
	return 0;
}


int pwm_init2( TIM_TypeDef* timer , int channel, unsigned short period, int freq_hz , int high1_low0, int enable_OCx_irq, int enable_preload)
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
		if( enable_preload == 1 ) TIM_OC1PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 2:
		TIM_OC2Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC2,ENABLE );
		if( enable_preload == 1 ) TIM_OC2PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 3:
		TIM_OC3Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC3,ENABLE );
		if( enable_preload == 1 ) TIM_OC3PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	case 4:
		TIM_OC4Init(timer, &TIMOCInitStructure);
		if( enable_OCx_irq == 1 ) TIM_ITConfig(timer,TIM_IT_CC4,ENABLE );
		if( enable_preload == 1 ) TIM_OC4PreloadConfig( timer, TIM_OCPreload_Enable);
		break;
	default:
		return -1;
	}
	
	if( enable_preload == 1 ) TIM_ARRPreloadConfig( timer, ENABLE);

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


unsigned short pwm_get(TIM_TypeDef* timer, int pchannel)
{
	int channel;
	unsigned short pwm = 0;
	channel = pchannel<0 ? -1*pchannel : pchannel;
	
	switch( channel ){
	case 1:
		pwm = TIM_GetCapture1( timer );
		break;
	case 2:
		pwm = TIM_GetCapture2( timer );
		break;
	case 3:
		pwm = TIM_GetCapture3( timer );
		break;
	case 4:
		pwm = TIM_GetCapture4( timer );
		break;
	default:
		return -1;
	}
	return pwm; 
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
	TIM_ICInitStructure.TIM_Channel = tim_channel; //通道选择 , this channel( 1 or 2 ) is capture one period tick , and other channel( 2 or 1 ) capture the duty tick 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿触发, the other channel is Falling polarity
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //输入预分频。意思是控制在多少个输入周期做一次捕获，如果输入的信号频率没有变，测得的周期也不会变。比如选择4分频，则每四个输入周期才做一次捕获，这样在输入信号变化不频繁的情况下，可以减少软件被不断中断的次数。
	TIM_ICInitStructure.TIM_ICFilter = 0x0; //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_PWMIConfig(timer, &TIM_ICInitStructure); //根据参数配置TIM外设信息

	if( tim_channel == TIM_Channel_2 ){
		TIM_SelectInputTrigger(timer, TIM_TS_TI2FP2); //选择IC2为始终触发源
	}else{
		TIM_SelectInputTrigger(timer, TIM_TS_TI1FP1);
	}
	TIM_SelectSlaveMode(timer, TIM_SlaveMode_Reset);//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
	TIM_SelectMasterSlaveMode(timer, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发

	TIM_Cmd(timer, ENABLE); //启动TIM2 
	if( tim_channel == TIM_Channel_2 ){
		TIM_ITConfig(timer, TIM_IT_CC2, ENABLE); //打开中断 
	}else{
		TIM_ITConfig(timer, TIM_IT_CC1, ENABLE);
	}
	TIM_ITConfig(timer, TIM_IT_Update, ENABLE);

	return 0;
}





/**** Capture freq 

unsigned short mPreiod,mPrescal,mFreq;

void tc3200_freq_capture_init()
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  

	//timer2~7 72Mhz , 
	mPreiod = 64000; mPrescal = 5; mFreq = SystemCoreClock/mPreiod/(mPrescal);
	TIM_TimeBaseStructure.TIM_Period = mPreiod -1 ; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = mPrescal -1 ; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);


	// Enable the Interrupt Request 
	TIM_ITConfig(TIM2, TIM_IT_CC3 | TIM_IT_Update, ENABLE);
	
	// TIM enable counter
	TIM_Cmd(TIM2, ENABLE);

	// Enable the TIM2 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                       = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

volatile unsigned int periodCnt = 0;
volatile unsigned int periodStartCnt = 0;
volatile unsigned int periodUpdateCnt = 0;
volatile unsigned char periodTag = 0;


void TIM2_IRQHandler(void)
{ 

	unsigned short cnt;
	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

		cnt = TIM_GetCapture3(TIM2);
		periodCnt = periodUpdateCnt + cnt - periodStartCnt;
		periodTag = 1;
		//for next capture
		periodUpdateCnt = 0;
		periodStartCnt = cnt;

	}else if( TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		periodUpdateCnt += (mPreiod - periodStartCnt);
		periodStartCnt = 0; //TIM2->CNT;
	}
}

int tc3200_getFreq()
{
	if( periodTag == 1 ){
		periodTag = 0;
		return (SystemCoreClock/mPrescal/periodCnt);
	}else{
		return -1;
	}
	
}
***********/



