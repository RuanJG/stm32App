#include "fifo.h"

#define _LOG(X...) printf(X);


int fifo_trylock( fifo_t * fifo )
{
	if( fifo->lock != 0)
		return 0;

	fifo->lock ++;
	if( fifo->lock == 1 )
		return 1;

	// if lock++ run in other thread befor above check, we reduce our lock
	fifo->lock--;
	return 0;

}

void fifo_lock( fifo_t *fifo )
{
	while( 0 == fifo_trylock(fifo) ) ;	
}

void fifo_unlock( fifo_t *fifo )
{
	if ( fifo->lock > 0  ){
		fifo->lock--;
	}else if( fifo->lock < 0 ){
		_LOG("Error fifo lock\n");
		fifo->lock = 0;
	}
}

void fifo_clear( fifo_t *fifo ){
	fifo->head = 0;
	fifo->tail = 0;
	memset(fifo->byteArray_point,0,fifo->byteArray_len);
}

//return byte count in valid
unsigned int fifo_valid( fifo_t *fifo)
{
	int c = fifo->head - fifo->tail ;
	return  (c >= 0) ?  c : ( fifo->byteArray_len + c );
}

//return free count
unsigned int fifo_free( fifo_t *fifo)
{
	return (fifo->byteArray_len - 1 - fifo_valid(fifo) );
}

int fifo_put( fifo_t * fifo, unsigned char data)
{
	unsigned int next;

	next = (fifo->head+1)%fifo->byteArray_len;
	if( next == fifo->tail){
		return 0;
	}
	fifo->byteArray_point[ fifo->head ] = data;
	fifo->head = next;
	return 1;
}

int fifo_put_force( fifo_t * fifo, unsigned char data)
{
	while( 0 == fifo_put( fifo , data) ){
		fifo->tail = (fifo->tail+1) % fifo->byteArray_len;
	}
	return  1;
}

int fifo_get( fifo_t *fifo , unsigned char * data)
{
	if( fifo->tail == fifo->head){
		return 0;
	}
	*data = fifo->byteArray_point[ fifo->tail ];
	fifo->tail = (fifo->tail+1) % fifo->byteArray_len;
	return 1;
}

//return valid item count
int fifo_valid_item( fifo_t *fifo)
{
	unsigned int use =  fifo_valid(fifo);
	return use/fifo->size;
}

//return 0 :false;       1: write ok   
int fifo_put_item(fifo_t *fifo , void * item)
{
	unsigned int len ;
	unsigned char *itemP;
	int i;

	len = fifo->size;
	if( len >  fifo_free(fifo) )
		return 0;

	itemP = (unsigned char *) item;
	for( i=0 ; i<len ; i++ ){
		if( 0 == fifo_put( fifo, itemP[ i ]) ){
			_LOG("Error in put Item\n");
			fifo->head -= i;
			if( fifo->head < 0 )
				fifo->head += fifo->byteArray_len;
			return 0;
		}
	}

	return 1;
}


int fifo_put_item_force(fifo_t *fifo , void * item)
{
	unsigned int len ;
	int i;

	len = fifo->size;
	if( len >  fifo_free(fifo) ){
		fifo->tail = (fifo->tail+len) % fifo->byteArray_len;
	}
	return fifo_put_item( fifo, item );
}

int fifo_get_item(fifo_t *fifo, void * item)
{
	int i;
	unsigned char * itemP ;

	if( fifo_valid_item(fifo) == 0 ){
		return 0;
	}

	itemP = (unsigned char *)item;
	for( i=0; i< fifo->size ; i++){
		if( 0 == fifo_get( fifo, itemP+i) ){
			return 0;
		}
	}

	return 1;
}

void fifo_init(fifo_t *fifo, char *buf, int size, int count)
{
	fifo->lock = 0;
	fifo->byteArray_len = size*count;
	fifo->byteArray_point = buf;
	fifo->size = size;
  fifo->head = 0;
  fifo->tail = 0;
}


/*
int main()
{
	double i,j,res;
	int count = 5;
	FIFO_DEF(txfifo,sizeof(double),5);

	for( i=0 ; i< count*3; i++){
		fifo_lock( txfifo );
		fifo_clear(txfifo);

		_LOG("\nput: ");
		for( j=0; j< i; j++){
			//printf("0x%02x,",j);
			_LOG("%0.2f,",j);
			fifo_put_item_force(txfifo,&j);
		}
		_LOG("\nget: ");
		for( j=0; j<= i; j++){
			if( fifo_get_item (txfifo,&res) > 0 )
				//printf("0x%02x,",res);
				_LOG("%0.2f,",res);
		}
		fifo_unlock( txfifo );
		// printf("\narray: ");
		// for( j=0; j< txfifo->byteArray_len; j++){
		// 	printf("0x%02x,", txfifo->byteArray_point[j] );
		// }
	}


}
*/