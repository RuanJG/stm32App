/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "fifo.h"
#include "systick.h"
#include "iap.h"


static uint8_t  m_UsbComRxBuf[USB_COM_RX_BUF_SIZE]   ;//   = {0};     
static uint8_t  m_UsbComTxBuf[USB_COM_TX_BUF_SIZE]   ;//   = {0};  

fifo_t usb_tx_fifo;
fifo_t usb_rx_fifo;

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  //GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
  //QUEUE_PacketCreate(&m_QueueUsbComRx, m_UsbComRxBuf, sizeof(m_UsbComRxBuf));
  //QUEUE_PacketCreate(&m_QueueUsbComTx, m_UsbComTxBuf, sizeof(m_UsbComTxBuf));
	
	fifo_init( &usb_tx_fifo, m_UsbComTxBuf, sizeof(uint8_t), sizeof(m_UsbComTxBuf));
	fifo_init( &usb_rx_fifo, m_UsbComRxBuf, sizeof(uint8_t), sizeof(m_UsbComRxBuf));
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/*
  // Configure USB pull-up pin
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
	*/
  
  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  if( SystemCoreClock == 72000000 ){
		RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	}else{
		RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
	}
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
//  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_CAN1_IRQ_PREPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_CAN1_IRQ_SUBPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_CAN1_IRQ_PREPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  if (NewState == DISABLE)
  {
    //GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_WriteBit(GPIOA,GPIO_Pin_12, Bit_RESET);
  }
  else
  {
    //GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

/*******************************************************************************
* Function Name : void USB_Config(void)
* Description   : USBϵͳ��ʼ��
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
void USB_Config(void)
{
    Set_System();

    Set_USBClock();

    USB_Interrupts_Config();

    USB_Init();
}



/*******************************************************************************
* Function Name : void USB_Deinit(void)
* Description   : USBϵͳ��ʼ��
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
void USB_Deinit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_CAN1_IRQ_PREPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_CAN1_IRQ_SUBPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_CAN1_IRQ_PREPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	PowerOff();
	systick_delay_us(5000);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
}




/*******************************************************************************
* Function Name : uint32_t USB_RxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : ��USB���ջ����ж�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_RxRead(uint8_t *buffter, uint32_t buffterSize)
{
    //return QUEUE_PacketOut(&m_QueueUsbComRx, buffter, buffterSize);
	int i;
	for( i=0; i< buffterSize; i++){
		if( 0 == fifo_get( &usb_rx_fifo , buffter+i ) ) break;
	}
	
	return i;
	
}
/*******************************************************************************
* Function Name : uint32_t USB_RxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : д���ݵ�USB���ջ�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_RxWrite(uint8_t *buffter, uint32_t writeLen)
{
    //return QUEUE_PacketIn(&m_QueueUsbComRx, buffter, writeLen);
	
	int i;
	for( i=0; i< writeLen; i++){
		fifo_put_force( &usb_rx_fifo, buffter[i] );
#if BOARD_HAS_IAP
#if IAP_PORT_USB
		iap_usb_receive_handler(buffter[i]);
#endif
#endif
	}
	return i;
	
}
/*******************************************************************************
* Function Name : uint32_t USB_TxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : ��USB���ͻ����ж�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_TxRead(uint8_t *buffter, uint32_t buffterSize)
{
    //return QUEUE_PacketOut(&m_QueueUsbComTx, buffter, buffterSize);;
	int i;
	for( i=0; i< buffterSize; i++){
		if( 0 == fifo_get( &usb_tx_fifo , buffter+i ) ) break;
	}
	
	return i;
}
/*******************************************************************************
* Function Name : uint32_t USB_TxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : д���ݵ�USB���ͻ�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_TxWrite(uint8_t *buffter, uint32_t writeLen)
{
    //return QUEUE_PacketIn(&m_QueueUsbComTx, buffter, writeLen);
	int i;
	for( i=0; i< writeLen; i++){
		if( 0 == fifo_put( &usb_tx_fifo, buffter[i] ) ) break;
	}
	return i;
}

uint32_t USB_TxWrite_Sync(uint8_t *buffter, uint32_t writeLen)
{
    //return QUEUE_PacketIn(&m_QueueUsbComTx, buffter, writeLen);
	int i;
	for( i=0; i< writeLen; i++){
		while( 0 == fifo_put( &usb_tx_fifo, buffter[i] ) ){
			systick_delay_us(500);
		}
	}
	while( fifo_valid(&usb_tx_fifo) > 0 ){
			systick_delay_us(500);
	}
	return i;
}



/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;  

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
