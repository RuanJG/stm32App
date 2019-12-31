
#include "main_config.h"
#include "miniMeter.h"
#include "systick.h"

#ifdef USING_MINIMETER_MODULE

/*
出厂时仪表波特率是9600,可以自行设置
每台设备出厂时通迅地址都是0x00

A:单次读取设备数据(设备接收到此命令后,向主机发送1次数据后回到等待状态)
0x88+0xAE+设备地址+0x11  实例88AE0011

B:连续读取设备数据(设备接收到此命令后,将以每秒6组的速度向主机连续发送数据)
0x88+0xAE+设备地址+0x21

C:停止设备输出数据,此命令不管当前工作的是哪一台设备,都将停止输出
0x88+0xAE+任意字节+0x01

D:设备返回给主机(PC)数据格式
0xFA+0xFB+设备地址+数据字节0+数据字节1+数据字节2+数据字节3+校验字节
数据由4个字节组成(注意:靠左的是低字节和习惯上是反的),接收后需要转换成浮点数才能对应设备显示数据.校验字节是4个数据字节相加的结果(进位舍去)
*/




void miniMeterCoder_init(miniMeterCoder_t * coder, unsigned char addr)
{
	int i;
	for( i=0; i< sizeof(coder->packet); i++)
		coder->packet[i]=0;
	coder->index = 0;
	coder->addr = addr;
}

int miniMeterCoder_prase( miniMeterCoder_t * coder, unsigned char data )
{
	unsigned char crc;
	unsigned char *value ;
	
	if( coder->index == 0  && data != 0xFA ) return 0;
	if( coder->index == 1 && data != 0xFB ) { coder->index = 0; return 0; }
	if( coder->index == 2  && data != coder->addr ) { coder->index = 0; return 0; }
	
	coder->packet[ coder->index++ ] = data;
	
	if( coder->index < sizeof(coder->packet) ) return 0;
	
	//check crc
	coder->index = 0;
	crc = coder->packet[3]+coder->packet[4]+coder->packet[5]+coder->packet[6];
	if( crc == coder->packet[7] ) {
		//coder->value = *((float*) &coder->packet[3] );
		value = (unsigned char *)&coder->value;
		value[0] = coder->packet[6];
		value[1] = coder->packet[5];
		value[2] = coder->packet[4];
		value[3] = coder->packet[3];
		return 1;
	}
	
	return -1;
}








void miniMeter_clear(miniMeter_t *meter)
{
	Uart_Clear_Rx(meter->uartdev);
	miniMeterCoder_init(&meter->decoder, meter->decoder.addr);
}


void miniMeter_toggle(miniMeter_t *meter)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x11};
	
	packget[2] = meter->decoder.addr;
	Uart_Put(meter->uartdev, packget, sizeof( packget) );
}




int miniMeter_check(miniMeter_t *meter, float *A)
{
	unsigned char data;
	int i,count,res;
	
	res = 0;

	for( i=0,count=10; i< count ; i++)
 {
		if( 1 == Uart_Get( meter->uartdev , &data, 1 ) ) 
		{
			res = miniMeterCoder_prase( &meter->decoder , data );
			if( 1 == res )
		  {
				*A = meter->decoder.value;
				break;
			}else if( -1 == res ){
				//error
				break;
			}else{
				//continue
			}
		}else{
			// no uart data , break;
			res = 0;
			break;
		}
	}
	return res;
}



int miniMeter_start(miniMeter_t *meter)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x21};
	float A;
	int retry = 5;
	
	packget[2] = meter->decoder.addr;
	
	if ( meter->state == METER_STATE_STARTED ) return 1;
	miniMeter_clear(meter);
	
	while( 0 < retry-- ){
		Uart_Put(meter->uartdev, packget, sizeof( packget) );
		systick_delay_ms(300);
		if( 1 == miniMeter_check( meter, &A) ){
			meter->state = METER_STATE_STARTED;
			return 1;
		}
	}
	
	return 0;
}

void miniMeter_stop(miniMeter_t *meter)
{
	unsigned char packget[]={0x88,0xAE,0x00,0x01};

	packget[2] = meter->decoder.addr;
	
	if ( meter->state == METER_STATE_IDEL ) return;
	meter->state = METER_STATE_IDEL;
	Uart_Put(meter->uartdev, packget, sizeof( packget) );
}


void miniMeter_init(miniMeter_t *meter, Uart_t* uart, unsigned char meterAddr)
{
	miniMeterCoder_init( &meter->decoder, meterAddr);
	meter->uartdev = uart;
	meter->state = METER_STATE_IDEL;
	miniMeter_stop(meter);
}


#endif