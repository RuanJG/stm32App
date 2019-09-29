#ifndef _STUSB4500_BSP_CONFIG_H
#define _STUSB4500_BSP_CONFIG_H

#include "stdio.h"
#include "stdint.h"
#include "stusb4500_nvm.h"


/* USER CODE BEGIN Prototypes */
//reset gpio
extern void board_Reset_Stusb4500();
//i2c read 
extern HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataR ,uint16_t Length);
//i2c write
extern HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataW ,uint16_t Length);
/* USER CODE END Prototypes */






#endif //_STUSB4500_BSP_CONFIG_H