#include "swd_bsp.h"


#define DISABLE_IRQ_WHEN_PIN_CTRL 1

static int SWD_TURNAROUND = 1;  // 1-4
static int SWD_DATA_PHASE = 0;  // 1 or 0
static int SWD_IDLE_CYCLES = 0 ; //?

void PIN_DELAY_SLOW (uint32_t delay) {
  volatile uint32_t count = delay;
  while (--count);
}
void PIN_DELAY_FAST (void) {
	volatile int i= 1; // 0-3
	while(i--) __NOP();
}
__STATIC_INLINE void pin_out_init(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

__STATIC_INLINE void pin_out_od_init(GPIO_TypeDef* GPIOx, uint16_t pin)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
}

__STATIC_INLINE void pin_in_init(GPIO_TypeDef* GPIOx, uint16_t pin, GPIOMode_TypeDef mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}



#define PIN_SWCLK_SET() SWCLK_TCK_PIN_PORT->BSRR = SWCLK_TCK_PIN
#define PIN_SWCLK_CLR() SWCLK_TCK_PIN_PORT->BRR = SWCLK_TCK_PIN
#define PIN_DELAY() PIN_DELAY_SLOW( (((SystemCoreClock/2U) / DAP_DEFAULT_SWJ_CLOCK) - IO_PORT_WRITE_CYCLES) )
//#define PIN_DELAY() PIN_DELAY_FAST()

__STATIC_INLINE void PIN_SWDIO_OUT(uint32_t bit)
{
    if (bit & 1)
        SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
    else
        SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
}
__STATIC_INLINE uint32_t PIN_SWDIO_IN(void)
{
    return ((SWDIO_IN_PIN_PORT->IDR & SWDIO_IN_PIN) ? 1 : 0);
}
__STATIC_INLINE void PIN_SWDIO_OUT_ENABLE(void)
{
    pin_out_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN);
    SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
}


__STATIC_INLINE void PIN_SWDIO_OUT_DISABLE(void)
{
    pin_in_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN, 0);
    SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
}

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()


void SWJ_Sequence (uint32_t count, const uint8_t *data) {
  uint32_t val;
  uint32_t n;

  val = 0U;
  n = 0U;
	
#if DISABLE_IRQ_WHEN_PIN_CTRL
	__disable_irq();
#endif
	
  while (count--) {
    if (n == 0U) {
      val = *data++;
      n = 8U;
    }
		PIN_SWDIO_OUT(val);
		/*
    if (val & 1U) {
      SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
    } else {
      SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
    }*/
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
#if DISABLE_IRQ_WHEN_PIN_CTRL	
	__enable_irq();
#endif
}

void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
  uint32_t val;
  uint32_t bit;
  uint32_t n, k;

  n = info & SWD_SEQUENCE_CLK;
  if (n == 0U) {
    n = 64U;
  }
#if DISABLE_IRQ_WHEN_PIN_CTRL
	__disable_irq();
#endif	
  if (info & SWD_SEQUENCE_DIN) {
    while (n) {
      val = 0U;
      for (k = 8U; k && n; k--, n--) {
        SW_READ_BIT(bit);
        val >>= 1;
        val  |= bit << 7;
      }
      val >>= k;
      *swdi++ = (uint8_t)val;
    }
  } else {
    while (n) {
      val = *swdo++;
      for (k = 8U; k && n; k--, n--) {
        SW_WRITE_BIT(val);
        val >>= 1;
      }
    }
  }
#if DISABLE_IRQ_WHEN_PIN_CTRL	
	__enable_irq();
#endif	
}





// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]                           
uint8_t SWD_Transfer(uint32_t request, uint32_t *data) {         
  uint32_t ack;                                                                 
  uint32_t bit;                                                                 
  uint32_t val;                                                                 
  uint32_t parity;                                                              
                                                                                
  uint32_t n;    
	
#if DISABLE_IRQ_WHEN_PIN_CTRL
	__disable_irq();	
#endif               
	
  /* Packet Request */                                                          
  parity = 0U;                                                                  
  SW_WRITE_BIT(1U);                     /* Start Bit */                         
  bit = request >> 0;                                                           
  SW_WRITE_BIT(bit);                    /* APnDP Bit */                         
  parity += bit;                                                                
  bit = request >> 1;                                                           
  SW_WRITE_BIT(bit);                    /* RnW Bit */                           
  parity += bit;                                                                
  bit = request >> 2;                                                           
  SW_WRITE_BIT(bit);                    /* A2 Bit */                            
  parity += bit;                                                                
  bit = request >> 3;                                                           
  SW_WRITE_BIT(bit);                    /* A3 Bit */                            
  parity += bit;                                                                
  SW_WRITE_BIT(parity);                 /* Parity Bit */                        
  SW_WRITE_BIT(0U);                     /* Stop Bit */                          
  SW_WRITE_BIT(1U);                     /* Park Bit */                          
                                                                                
  /* Turnaround */                                                              
  PIN_SWDIO_OUT_DISABLE();                                                      
  for (n = SWD_TURNAROUND; n; n--) {                              
    SW_CLOCK_CYCLE();                                                           
  }                                                                             
                                                                                
  /* Acknowledge response */                                                    
  SW_READ_BIT(bit);                                                             
  ack  = bit << 0;                                                              
  SW_READ_BIT(bit);                                                            
  ack |= bit << 1;                                                              
  SW_READ_BIT(bit);                                                             
  ack |= bit << 2;                                                              
                                                                                
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       
    /* Data transfer */                                                         
    if (request & DAP_TRANSFER_RnW) {                                           
      /* Read data */                                                           
      val = 0U;                                                                 
      parity = 0U;                                                              
      for (n = 32U; n; n--) {                                                   
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  
        parity += bit;                                                          
        val >>= 1;                                                              
        val  |= bit << 31;                                                      
      }                                                                         
      SW_READ_BIT(bit);                 /* Read Parity */                       
      if ((parity ^ bit) & 1U) {                                                
        ack = DAP_TRANSFER_ERROR;                                               
      }                                                                         
      if (data) { *data = val; }                                                
      /* Turnaround */                                                          
      for (n = SWD_TURNAROUND; n; n--) {                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
      PIN_SWDIO_OUT_ENABLE();                                                   
    } else {                                                                    
      /* Turnaround */                                                          
      for (n = SWD_TURNAROUND; n; n--) {                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
      PIN_SWDIO_OUT_ENABLE();                                                   
      /* Write data */                                                          
      val = *data;                                                              
      parity = 0U;                                                              
      for (n = 32U; n; n--) {                                                   
        SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                 
        parity += val;                                                          
        val >>= 1;                                                              
      }                                                                         
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  
    }                                                                           
    /* Capture Timestamp */                                                     
    if (request & DAP_TRANSFER_TIMESTAMP) {        
			//JK: NO support now
      //DAP_Data.timestamp = TIMESTAMP_GET();                                      
    }                                                                           
    /* Idle cycles */                                                           
    n = SWD_IDLE_CYCLES;                                          
    if (n) {                                                                    
      PIN_SWDIO_OUT(0U);                                                        
      for (; n; n--) {                                                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
    }                                                                           
    PIN_SWDIO_OUT(1U); 
#if DISABLE_IRQ_WHEN_PIN_CTRL		
		__enable_irq();	
#endif		
    return ((uint8_t)ack);                                                      
  }                                                                             
                                                                                
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              
    /* WAIT or FAULT response */                                                
    if (SWD_DATA_PHASE && ((request & DAP_TRANSFER_RnW) != 0U)) { 
      for (n = 32U+1U; n; n--) {                                                
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   
      }                                                                         
    }                                                                           
    /* Turnaround */                                                            
    for (n = SWD_TURNAROUND; n; n--) {                            
      SW_CLOCK_CYCLE();                                                         
    }                                                                           
    PIN_SWDIO_OUT_ENABLE();                                                     
    if (SWD_DATA_PHASE && ((request & DAP_TRANSFER_RnW) == 0U)) { 
      PIN_SWDIO_OUT(0U);                                                        
      for (n = 32U+1U; n; n--) {                                                
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  
      }                                                                         
    }                                                                           
    PIN_SWDIO_OUT(1U); 
#if DISABLE_IRQ_WHEN_PIN_CTRL
		__enable_irq();	
#endif
    return ((uint8_t)ack);                                                      
  }                                                                             
                                                                                
  /* Protocol error */                                                          
  for (n = SWD_TURNAROUND + 32U + 1U; n; n--) {                   
    SW_CLOCK_CYCLE();                   /* Back off data phase */               
  }                                                                             
  PIN_SWDIO_OUT_ENABLE();                                                       
  PIN_SWDIO_OUT(1U);  
#if DISABLE_IRQ_WHEN_PIN_CTRL	
	__enable_irq();	
#endif
  return ((uint8_t)ack);                                                        
}



void SWD_PORT_SETUP(void)
{
    /* Enable port clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
    /* Configure I/O pin SWCLK */
    pin_out_init(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN);
    SWCLK_TCK_PIN_PORT->BSRR = SWCLK_TCK_PIN;

    pin_out_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN);
    SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;

    pin_in_init(SWDIO_IN_PIN_PORT, SWDIO_IN_PIN, GPIO_Mode_IN_FLOATING);

    pin_out_od_init(nRESET_PIN_PORT, nRESET_PIN);
    nRESET_PIN_PORT->BSRR = nRESET_PIN;
}

void SWD_PORT_OFF(void)
{
    pin_in_init(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN, GPIO_Mode_AIN);
    pin_in_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN, GPIO_Mode_AIN);
    pin_in_init(SWDIO_IN_PIN_PORT, SWDIO_IN_PIN, GPIO_Mode_AIN);
}

void SWD_target_reset( int val)
{
	if (val & 1)
			nRESET_PIN_PORT->BRR = nRESET_PIN;
	else
			nRESET_PIN_PORT->BSRR = nRESET_PIN;
}