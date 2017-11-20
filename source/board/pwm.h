#ifndef PWM_H_
#define PWM_H_

int pwm_init( TIM_TypeDef* timer , int channel, unsigned short period, int freq_hz , int high1_low0);
int pwm_set( TIM_TypeDef* timer , int pchannel, unsigned short pwm);

/*
*		this function just init the timer , you need init gpio and NVIC&irq_function
*   timer : TIM1 TIM2 ...
*   tim_channel : TIM_Channel_1 TIM_Channel_2 TIM_Channel_3 TIM_Channel_4
*/
int pwm_input( TIM_TypeDef* timer , unsigned short period, int freq_hz , int tim_channel);

#endif 

