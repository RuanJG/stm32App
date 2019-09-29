
/* Defines ------------------------------------------------------------------*/
#define READ_NVM 1   //0:false, 1:true
#define FLASH_NVM 1   //0:false, 1:true
#define VERIFY_NVM 1   //0:false, 1:true


/* Includes ------------------------------------------------------------------*/

#include <stdio.h> //for printf()
#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "bsp_config.h"
#include "USB_PD_defines.h"
#include "USBPD_CUST_NVM_API.h"
/* USER CODE END Includes */


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern unsigned int I2cDeviceID_7bit;
/* USER CODE END PV */


int stusb4500_check_id()
{
	unsigned char id;
	//ID=0x21 for STUSB4500
	if(I2C_Read_USB_PD(0,DEVICE_ID, &id, 1) != 0)  { printf("Error I2C \r\n"); return -1;}
	if( id != 0x21 ){
		printf("read id = 0x%02x, failed\n",id);
		return -1;
	}
	return 0;
}

int stusb4500_read_NVM()
{
	int status,i,j;
	unsigned char NVM_Sectors[5][8] = {0, 0};
	status = nvm_read( &NVM_Sectors[0][0], sizeof(NVM_Sectors) );
	if(status != 0) //Error
	{
			printf("Error NVM read\r\n");
			return -1;
	}else{
		printf("Read NVM: ");
		for( i=0; i< 5; i++){
			printf("\n\rSector%d: ",i);
			for( j=0;j<8;j++){
				printf("0x%02x,",NVM_Sectors[i][j]);
			}
		}
	}
	return 0;
}

int stusb4500_flash_NVM()
{
	int status;
	uint8_t Buffer;
	
	status = I2C_Read_USB_PD(0,DEVICE_ID, &Buffer, 1);     //ID=0x21 for STUSB4500
	if(status != 0)  { printf("Error I2C \r\n"); return -1;}
	if( Buffer != 0x21 ) { printf("STUSB4500 ID  wrong\r\n"); return -1;}
	
	if(Buffer & ID_Reg >= CUT)  
	{
			status = nvm_flash();
			if ( status != 0) 
			{
				printf("STUSB Flashing Failed \r\n"); // Port 0 mean I2C1 For I2C2 change to Port 1
				return -1;
			}else {
				printf("STUSB Flashing OK\r\n");
			}
	}else{
		printf("STUSB Flashing Not done : Bad version\r\n");
		return -1;
	}

	return 0;
}


int stusb4500_verify_NVM()
{
		uint8_t Verification_Sector[8];
		uint8_t Sector40[5][8];
		
		extern uint8_t Sector0[8];
		extern uint8_t Sector1[8];
		extern uint8_t Sector2[8];
		extern uint8_t Sector3[8];
		extern uint8_t Sector4[8];
		
		
		for(int i=0; i<8; i++) //copy sectors to 40byte array
		{
				Sector40[0][i] = Sector0[i];
				Sector40[1][i] = Sector1[i];
				Sector40[2][i] = Sector2[i];
				Sector40[3][i] = Sector3[i];
				Sector40[4][i] = Sector4[i];
		}
		
		if (CUST_EnterReadMode(0) == 0 ) //OK
		{
				for(int bk=0; bk<5; bk++)
				{
						if (CUST_ReadSector(0,bk, &Verification_Sector[0]) ==0)
						{
								if (  *(uint64_t *) Verification_Sector != *(uint64_t *) (Sector40[bk])   )
								{
										for ( int j = 0 ;j < 8 ; j++)
										{
												if (  Verification_Sector[j] != Sector40[bk][j])
														printf("NVM verification issue byte %i bank %i \r\n", j, bk);
										}
										return -1;
								}else {
									printf("NVM verification bank %i : OK \r\n", bk);
								}
						}else {
							printf("NVM read bank %i : failed \r\n", bk);
							return -1;
						}
				}
		}else{  
			printf("NVM read initialization: failed \n");
			return -1;
		}
		return 0;
}








int stusb4500_program_NVM(void)
{
    
    /* USER CODE BEGIN 2 */
    int status;
    
    
    printf("*** STUSB NVM programming ***\r\n");
    
		printf("1, reset stusb4500:\n");
		board_Reset_Stusb4500();
	
#if READ_NVM 
		printf("2, read stusb4500:\n");
		if( 0 != stusb4500_read_NVM() ){
			return -1;
		}
#endif
		
#if FLASH_NVM 
		printf("3, flash  stusb4500:\n");
		if( 0 != stusb4500_flash_NVM() ){
			return -2;
		}
#endif

#if VERIFY_NVM 
		printf("4, verify  stusb4500:\n");
		if( 0 != stusb4500_verify_NVM() ){
			return -3;
		}
#endif		
		
		return 0;
		
	
#if READ_NVM
    {
        unsigned char NVM_Sectors[5][8] = {0, 0};
        status = nvm_read( &NVM_Sectors[0][0], sizeof(NVM_Sectors) );
        if(status != 0) //Error
        {
            printf("Error NVM read\r\n");
            return -1;
        }
    }
#endif //READ_NVM
    
    
#if FLASH_NVM
    {
        uint8_t Buffer;
        status = I2C_Read_USB_PD(0,DEVICE_ID, &Buffer, 1);     //ID=0x21 for STUSB4500
        if(status != 0)  { printf("Error I2C \r\n"); return -1;}
        
        if(Buffer & ID_Reg >= CUT)  
        {
            status = nvm_flash();
            if ( status != 0) 
                printf("STUSB Flashing Failed \r\n"); // Port 0 mean I2C1 For I2C2 change to Port 1
            else 
                printf("STUSB Flashing OK\r\n");
        }
        else  printf("STUSB Flashing Not done : Bad version\r\n");
    }
#endif //FLASH_NVM
    
    
#if VERIFY_NVM
    {
        uint8_t Verification_Sector[8];
        uint8_t Sector40[5][8];
        
        extern uint8_t Sector0[8];
        extern uint8_t Sector1[8];
        extern uint8_t Sector2[8];
        extern uint8_t Sector3[8];
        extern uint8_t Sector4[8];
        
        
        for(int i=0; i<8; i++) //copy sectors to 40byte array
        {
            Sector40[0][i] = Sector0[i];
            Sector40[1][i] = Sector1[i];
            Sector40[2][i] = Sector2[i];
            Sector40[3][i] = Sector3[i];
            Sector40[4][i] = Sector4[i];
        }
        
        if (CUST_EnterReadMode(0) == 0 ) //OK
        {
            for(int bk=0; bk<5; bk++)
            {
                if (CUST_ReadSector(0,bk, &Verification_Sector[0]) ==0)
                {
                    if (  *(uint64_t *) Verification_Sector != *(uint64_t *) (Sector40[bk])   )
                    {
                        for ( int j = 0 ;j < 8 ; j++)
                        {
                            if (  Verification_Sector[j] != Sector40[bk][j])
                                printf("NVM verification issue byte %i bank %i \r\n", j, bk);
                        }
                    }  
                    else   printf("NVM verification bank %i : OK \r\n", bk);
                }
                else  printf("NVM read bank %i : failed \r\n", bk);
            }
        }
        else  printf("NVM read initialization: failed \r\n");
    }
#endif //VERIFY_NVM

    
    /* USER CODE END 2 */
    
    return 0;
}

