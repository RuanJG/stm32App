#include "bsp_config.h"
#include "i2c.h"
#include "main_config.h"



#if LIB_STUSB4500_NVM 

#else

HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataR ,uint16_t Length)
{
  //return  HAL_I2C_Mem_Read(hi2c[Port],(I2cDeviceID_7bit<<1), Address ,AddressSize, DataR, Length ,2000); //STUSBxx_DEVICEID_7BIT , 1 byte addr
	return HAL_OK;
	
}

HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port, uint16_t Address ,uint8_t *DataW ,uint16_t Length)
{
  //return  HAL_I2C_Mem_Write(hi2c[Port],(I2cDeviceID_7bit<<1), Address ,AddressSize, DataW, Length, 2000 ); // unmask all alarm status
	
	return HAL_OK;
}

void board_Reset_Stusb4500()
{
	
}

#endif



