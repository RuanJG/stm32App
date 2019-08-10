#include "misc.h"
#include "core_cm3.h"
#include "systick.h"
#include "main_config.h"
#include "flashRW.h"






/*
		read data from flash[CONFIG_ADDRESS] 
    return 0 successfully , -1 error size
*/
int flash_read(volatile unsigned int * data , int size)
{
	volatile unsigned int * p;
	__IO unsigned int* addr;
	int i, int_count;
	
	addr = (__IO unsigned int*) CONFIG_ADDRESS;
	int_count = size/sizeof( unsigned int );	
	
	if( size%sizeof( unsigned int) != 0 ) return -1;
	
	p = (volatile unsigned int*)data;
	for( i=0; i< int_count  ; i++ ){
		*(p+i) = *addr;
		addr++;
	}
	
	return 0;
}

/*
		write data to flash[CONFIG_ADDRESS] 
    return 0 successfully, -1 failed , -2 data size error
*/
int flash_write(volatile unsigned int * data , int size)
{
	int timeout, i, res;
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	volatile unsigned int * pdata = (volatile unsigned int *) data;
	int int_count = size /sizeof( unsigned int );
	unsigned int addr = CONFIG_ADDRESS;
	
	if( size%sizeof(unsigned int) != 0 ) return -2;
	
	if( size >  CONFIG_SOTRE_SIZE ) return -2;
	
	
	FLASHStatus = FLASH_COMPLETE;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	res = 0;
	timeout = 10;
	while( timeout-- > 0)
	{
		FLASHStatus = FLASH_ErasePage(addr);
		if( FLASHStatus == FLASH_COMPLETE ){
			res = 1;
			break;
		}
	}
	
	if( res == 1 )
	{
		for ( i = 0; i< int_count ; i++ )
		{
			res = 0;
			timeout = 10;
			while( timeout-- > 0)
			{
				FLASHStatus = FLASH_ProgramWord( addr+4*i, *(pdata+i) ) ;//FLASH_ProgramOptionByteData(IAP_TAG_ADDRESS,tag);
				if( FLASHStatus == FLASH_COMPLETE ){
					res = 1;
					break;
				}
			}
			if( res == 0 ) break;
		}
	}

	FLASH_Lock();
	
	return res==1?0:-1;
}





//End of File
