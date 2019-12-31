
#include "main_config.h"
#include "miniMeter.h"
#include "systick.h"

#ifdef USING_MINIMETER_MODULE

/*
����ʱ�Ǳ�������9600,������������
ÿ̨�豸����ʱͨѸ��ַ����0x00

A:���ζ�ȡ�豸����(�豸���յ��������,����������1�����ݺ�ص��ȴ�״̬)
0x88+0xAE+�豸��ַ+0x11  ʵ��88AE0011

B:������ȡ�豸����(�豸���յ��������,����ÿ��6����ٶ�������������������)
0x88+0xAE+�豸��ַ+0x21

C:ֹͣ�豸�������,������ܵ�ǰ����������һ̨�豸,����ֹͣ���
0x88+0xAE+�����ֽ�+0x01

D:�豸���ظ�����(PC)���ݸ�ʽ
0xFA+0xFB+�豸��ַ+�����ֽ�0+�����ֽ�1+�����ֽ�2+�����ֽ�3+У���ֽ�
������4���ֽ����(ע��:������ǵ��ֽں�ϰ�����Ƿ���),���պ���Ҫת���ɸ��������ܶ�Ӧ�豸��ʾ����.У���ֽ���4�������ֽ���ӵĽ��(��λ��ȥ)
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