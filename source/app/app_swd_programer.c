#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "swd_flash_manager.h"

#if BOARD_SWD_PROGRAMER


#define _LOG(X...) if( 1 ) printf(X);





Uart_t Uart1;

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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	Uart_Configuration (&Uart1, USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
}

#define CMDBUFFERSIZE 32
unsigned char usrCmdBuffer[CMDBUFFERSIZE];

#define BINFILESIZE 0x1000 //4*1024
unsigned char binfile[BINFILESIZE]={'3'};

static uint8_t swd_read_idcode(uint32_t *id);


/* Flash Access Control Register bits */
#define ACR_LATENCY_Mask         ((uint32_t)0x00000038)
#define ACR_HLFCYA_Mask          ((uint32_t)0xFFFFFFF7)
#define ACR_PRFTBE_Mask          ((uint32_t)0xFFFFFFEF)

/* Flash Access Control Register bits */
#define ACR_PRFTBS_Mask          ((uint32_t)0x00000020) 

/* Flash Control Register bits */
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE) 
#define CR_PER_Set               ((uint32_t)0x00000002)
#define CR_PER_Reset             ((uint32_t)0x00001FFD)
#define CR_MER_Set               ((uint32_t)0x00000004)
#define CR_MER_Reset             ((uint32_t)0x00001FFB)
#define CR_OPTPG_Set             ((uint32_t)0x00000010)
#define CR_OPTPG_Reset           ((uint32_t)0x00001FEF)
#define CR_OPTER_Set             ((uint32_t)0x00000020)
#define CR_OPTER_Reset           ((uint32_t)0x00001FDF)
#define CR_STRT_Set              ((uint32_t)0x00000040)
#define CR_LOCK_Set              ((uint32_t)0x00000080)

/* FLASH Mask */
#define RDPRT_Mask               ((uint32_t)0x00000002)
#define WRP0_Mask                ((uint32_t)0x000000FF)
#define WRP1_Mask                ((uint32_t)0x0000FF00)
#define WRP2_Mask                ((uint32_t)0x00FF0000)
#define WRP3_Mask                ((uint32_t)0xFF000000)
#define OB_USER_BFB2             ((uint16_t)0x0008)

/* FLASH Keys */
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

#define EraseTimeout          ((uint32_t)0x000B0000)
#define ProgramTimeout        ((uint32_t)0x00002000)

FLASH_Status FLASH_Get_Status(uint32_t sr_value)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((sr_value & FLASH_FLAG_BANK1_BSY) == FLASH_FLAG_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((sr_value & FLASH_FLAG_BANK1_PGERR) != 0)
    { 
      flashstatus = FLASH_ERROR_PG;
    }
    else 
    {
      if((sr_value & FLASH_FLAG_BANK1_WRPRTERR) != 0 )
      {
        flashstatus = FLASH_ERROR_WRP;
      }
      else
      {
        flashstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the Flash Status */
  return flashstatus;
}


int erase_page(uint32_t Page_Address)
{
	int i = 0,retry;
	FLASH_Status fstatus = FLASH_BUSY;
	uint32_t value,CR;
	
	
			if( 0 == swd_read_word(0x8010000, &value) ){
				_LOG(" read 0x8010000 false\n");
				return 0;
			}else{
				_LOG(" read 0x%x\n", value);
			}
	retry = 1;
	while( retry-- ){
		if( 0 == swd_read_word(FLASH->CR, &CR) ){
			_LOG(" read FLASH->CR false\n");
			if( retry <= 0 ) return 0;
		}else{
			break;
		}
	}
	
	/////////unlock flash
	//FLASH->KEYR = FLASH_KEY1;
  //FLASH->KEYR = FLASH_KEY2;
	if( 0 != swd_write_word( FLASH->KEYR , FLASH_KEY1) ){
		_LOG(" set FLASH->KEYR , FLASH_KEY1 failed\n");
		return 0;
	}
	if( 0 != swd_write_word( FLASH->KEYR , FLASH_KEY2) ){
		_LOG(" set FLASH->KEYR , FLASH_KEY1 failed\n");
		goto errout;
	}
	
	//FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	if( 0 != swd_write_word( FLASH->SR , FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR) ){
		_LOG(" FLASH_ClearFlag failed\n");
		goto errout;
	}
	

	//  FLASH->CR|= CR_PER_Set;
  //  FLASH->AR = Page_Address; 
  //  FLASH->CR|= CR_STRT_Set;
	
	CR |= CR_PER_Set;
	if( 0 != swd_write_word( FLASH->CR , CR_PER_Set ) ){
		_LOG(" FLASH->CR|= CR_PER_Set failed\n");
		goto errout;
	}
	if( 0 != swd_write_word( FLASH->AR , Page_Address) ){
		_LOG(" FLASH->AR = Page_Address failed\n");
		goto errout;
	}
	
	if( 0 != swd_write_word( FLASH->CR ,  CR_PER_Set|CR_STRT_Set ) ){
		_LOG(" FLASH->CR|= CR_STRT_Set failed\n");
		goto errout;
	}
	
	systick_delay_ms(500);
	
	//FLASH->CR &= CR_PER_Reset;
	CR &= CR_PER_Reset;
	if( 0 != swd_write_word( FLASH->CR , 0 ) ){
		_LOG(" FLASH->CR|= CR_PER_Reset failed\n");
	}
	
	
errout:
	CR |= CR_LOCK_Set;
	if( 0 != swd_write_word( FLASH->CR , CR_LOCK_Set) ){
		_LOG(" FLASH Lock failed\n");
	}
	
	systick_delay_ms(100);
	
	//status = FLASH_WaitForLastOperation(EraseTimeout);
	i = 4; // ms timeout
	while( (i--) > 0 ){
		if( 0 == swd_read_word(FLASH->SR, &value) ){
			_LOG(" read FLASH->SR retry...\n");
			systick_delay_ms(1);
			continue;
			//_LOG(" read FLASH->SR false\n");
			//return 0;
		}
		fstatus = FLASH_Get_Status( value );
		if( fstatus != FLASH_BUSY ){
			break;
		}
		systick_delay_ms(1);
	}
	
	if( fstatus == FLASH_BUSY ){
		_LOG(" FLASH_BUSY Timeout \n");
	}else if( fstatus ==  FLASH_ERROR_PG ){
		_LOG(" FLASH_ERROR_PG Error \n");
	}else if( fstatus ==  FLASH_ERROR_WRP ){
		_LOG(" FLASH_ERROR_WRP Error \n");
	}else if( fstatus ==  FLASH_COMPLETE){
		_LOG(" FLASH_COMPLETE  \n");
	}else{
		_LOG(" Unknow erase result \n");
	}
	
	
	if( fstatus ==  FLASH_COMPLETE){
		return 1;
	}else{
		return 0;
	}
}

void cmd_even()
{
	unsigned int len,i;
	unsigned int value;
	char data;
	uint8_t reg;
	
	len = console_cmd_check();
	if( len <= 0 ) return;
	usrCmdBuffer[len] = 0;
	
	switch(usrCmdBuffer[0])
	{
		case '1':
			sscanf((char*)usrCmdBuffer,"1value=%x",&value);
			_LOG("get value = 0x%x\n",value);
		break;
		case '2':
			//flash
			sscanf((char*)usrCmdBuffer,"2flash=%x,%x",&value,&data);
			_LOG("start flash at 0x%x, data=0x%x\n",value,data);
		
		  for( i=0; i< BINFILESIZE; i++){
				binfile[i]=data;
			}
			if( ERROR_SUCCESS != flash_manager_init( g_swd_flash_intf ) ){
				_LOG(" flash : init error , exit \n");
				if( ERROR_SUCCESS != flash_manager_uninit() ){
					_LOG(" flash : uinit error , exit \n");
				}
				break;
			}
			if( ERROR_SUCCESS != flash_manager_data( value , binfile, BINFILESIZE ) ){
				_LOG(" flash : program error , exit \n");
				if( ERROR_SUCCESS != flash_manager_uninit() ){
					_LOG(" flash : uinit error , exit \n");
				}
				break;
			}
			if( ERROR_SUCCESS != flash_manager_uninit() ){
				_LOG(" flash : uinit error , exit \n");
				break;
			}
		break;
			
		case '3':
			sscanf((char*)usrCmdBuffer,"3reset=%d",&value);
			_LOG(" reset : %d\n",value);
			SWD_target_reset( (value==1)? 1:0);
		break;
		
		case '4':
			sscanf((char*)usrCmdBuffer,"4reg=0x%x",&reg);
			_LOG(" read reg 0x%x...\n",reg);
		
			if( 0 == swd_set_target_state( HALT ) ){
				_LOG(" set target halt failed\n");
				break;
			}
			
		uint8_t tmp_in[1];
    uint8_t tmp_out[4];
    tmp_in[0] = 0x00;
    SWJ_Sequence(8, tmp_in);
    if (swd_read_dp(reg, (uint32_t *)tmp_out) != 0x01) {
        _LOG(" swd_read_dp error\n");
    }else{
			value = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];
			_LOG(" reg = 0x%02x\n",value);
		}
		
		if(  swd_set_target_state(RESET_RUN) ){
			_LOG(" reset OK\n");
		}else{
			_LOG(" reset failed\n");
		}
		break;	
		
		case '5':
			sscanf((char*)usrCmdBuffer,"5value=0x%x",&value);
			_LOG(" set value to 0x%x\n", value);
		
			if( 0 == swd_set_target_state( HALT ) ){
				_LOG(" set target halt failed\n");
				break;
			}
					
			if( 0 == swd_write_word(0x20000C00, value) ){
				_LOG(" write 0x20000C00 false\n");
				break;
			}else{
				_LOG(" write 0x20000C00  0x%x , successfully\n", value);
			}
			
			value = 0;
			if( 0 == swd_read_word(0x20000C00, &value) ){
				_LOG(" read 0x20000C00 false\n");
				break;
			}
			_LOG(" read 0x20000C00 =  0x%x\n", value);
			
		break;
			
		case '6':
			sscanf((char*)usrCmdBuffer,"6addr=0x%x",&value);
			_LOG(" addr = 0x%x\n", value);
		
			if( 0 == swd_set_target_state( HALT ) ){
				_LOG(" set target halt failed\n");
				break;
			}
			i = 0;
			if( 0 == swd_read_word(value, &i) ){
				_LOG(" read 0x%x false\n",value);
				break;
			}
			_LOG(" read 0x%x = 0x%x\n", value, i);
			
		break;
		
		case '7':
			sscanf((char*)usrCmdBuffer,"7addr=0x%x",&value);
			_LOG(" Erase addr = 0x%x\n", value);
		
			if( 0 == swd_set_target_state( HALT ) ){
				_LOG(" set target halt failed\n");
				break;
			}

			if( erase_page(value) ){
				_LOG(" Erase addr  0x%x  successfully\n", value);
			}else{
				_LOG(" Erase addr  0x%x  failed\n", value);
			}
			
			if(  swd_set_target_state(RESET_RUN) ){
				_LOG(" reset OK\n");
			}else{
				_LOG(" reset failed\n");
			}
		break;
			
		case '8':
			value = 0x2a2b2c2d;
			flash_write(0x800F800, &value,4);
		break;
		
		default:
			_LOG("unknow cmd\n");
		break;
	}
}


void app_init()
{
	Uart_init();
	console_init( CONSOLE_UART_TYPE ,&Uart1 ,usrCmdBuffer, CMDBUFFERSIZE );
	systick_init(1000000);
	_LOG("inited OK\n");
}

void app_event()
{
	cmd_even();
}

#endif