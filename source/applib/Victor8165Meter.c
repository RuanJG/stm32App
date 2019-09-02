#include "Victor8165Meter.h"

#include "systick.h"


#if 0
Uart_t *victor8165Uart;

#define VICTOR8165_BUFFSIZE 32
volatile unsigned char victor8165_buf[VICTOR8165_BUFFSIZE];
volatile unsigned char victor8165_inited = 0;
systick_time_t victor8165_cmd_timer;

int victor8165_cmd( char * cmd  )
{
	int i,count,len;
	
	while( systick_check_timer( &victor8165_cmd_timer ) == 0 ){
		systick_delay_us(500);
	}
	Uart_Clear_Rx( victor8165Uart );
	//while( 1 == Uart_Get( victor8165Uart, &victor8165_buf[0] , 1) );
	
	i = strlen(cmd);
	Uart_Put_Sync( victor8165Uart, (unsigned char *)cmd, i );
	//_LOG("cmd:%s",cmd);
	
	//get result , 1000ms timeout;
	count = 0;
	len = 0;
	for( i=0; i< 500 ; i++ ){
		count = Uart_Get( victor8165Uart, &victor8165_buf[len] , VICTOR8165_BUFFSIZE - len -1);
		len += count;
		if( len > 0 && victor8165_buf[len-1]==0x0A ){
			//systick_delay_us(50000);
			victor8165_buf[len]= 0;
			systick_init_timer( &victor8165_cmd_timer , 10); // next cmd need to wait 50ms
			return len;
		}
		if( count > 0 ) i--;
		systick_delay_us( 1000 );
	}
	
	//Uart_Put( victor8165Uart, (unsigned char *)"\n", 1 );
	//systick_delay_us( 100000 );
	_LOG("cmd:%s  timeout\n",cmd);
	systick_init_timer( &victor8165_cmd_timer , 100); // next cmd need to wait 100ms
	
	return 0;
}

int victor8165_check(const char* cmd, const char *result)
{
	int len;
	int i;
	
	len = victor8165_cmd( (char*) cmd);
	if( len == strlen(result) && 0 == strncmp(result,(char*) victor8165_buf , len) ){
		return 1;
	}
	
	/*
	for( i=0; i< len; i++ ){
		
		if( result[i] != victor8165_buf[i] ){
			userStation_log( "x,");
			return 0;
		}else{
			userStation_log( "1,");
		}
	}
	
	return 1;
	*/
	
	userStation_log( "check :\n");
	userStation_log((char*)cmd);
	userStation_log((char*)victor8165_buf);
	return 0;
}


void victor8165_init()
{
	int retry,res;
	
	victor8165Uart = CURRENT_UART;
	systick_init_timer( &victor8165_cmd_timer , 10);
	
	//set DCI mode
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "ADC\n");
		if( 1 == victor8165_check( "FUNC1?\n", "ADC\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set DCI mode ok\n");
		userStation_log("init: DCI ok");
	}else{
		_LOG("victor8165_init: set DCI mode false\n");
		userStation_log("init: DCI false");
		return;
	}
	
	//set Rate
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "RATE M\n");
		if( 1 == victor8165_check( "RATE?\n", "M\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set Rate M mode ok\n");
		userStation_log("init: Rate ok");
	}else{
		_LOG("victor8165_init: set Rate M mode false\n");
		userStation_log("init: Rate false");
		return;
	}
	
	//set range AUTO
	res = 0;
	for( retry = 0; retry < 2; retry++ ){
		victor8165_cmd( "RANGE 5\n");
		if( 1 == victor8165_check( "RANGE1?\n", "6\n") ){
			res = 1;
			break;
		}
	}
	if( res ) {
		_LOG("victor8165_init: set AUTO mode ok\n");
		userStation_log("init: Auto ok");
	}else{
		_LOG("victor8165_init: set AUTO mode false\n");
		userStation_log("init: Auto false");
		return;
	}
	
	victor8165_inited = 1;
	
}


int service_getCurrent( float *A)
{
	int retry, len;
	
	//victor8165_check( "FUNC1?\n", "ADC\n");
	//if( victor8165_inited == 0 ||  0 == victor8165_check( "FUNC1?\n", "ADC\n") ){
	if( victor8165_inited == 0 ){
		_LOG("victor8165_getCurrent: need to init\n");
		victor8165_inited = 0;
		victor8165_init();
		return 0;
	}
	
	
	for( retry = 0; retry < 2; retry++ ){
		len = victor8165_cmd( "MEAS1?\n");
		if( len > 0 ) 
			break;
	}
	
	if( len == 0 ){
		return 0;
	}
	
	victor8165_buf[len] = 0; // set string end tag
	sscanf((const char*)victor8165_buf , "%e;\n",  A);
	//*mA = A*1000;
	
	return 1;
}




#endif








