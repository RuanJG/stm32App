#ifndef _STUSB4500_NVM_H
#define _STUSB4500_NVM_H

typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


#define STUSBxx_DEVICEID_7BIT  0x28
#define STUSBxx_DEVICEID_8BIT  0x50
#define STUSB4500_I2C_DEVID_7BIT  0x28
#define STUSB4500_I2C_DEVID_8BIT  0x50

int stusb4500_program_NVM(void);
int stusb4500_read_NVM();
int stusb4500_flash_NVM();
int stusb4500_verify_NVM();
int stusb4500_check_id();



#endif //_STUSB4500_NVM_H
