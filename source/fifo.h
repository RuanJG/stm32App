#ifndef __FIFO_H
#define __FIFO_H

typedef struct _fifo_t {
	volatile int		lock;
	unsigned int		byteArray_len;
	unsigned char *	byteArray_point;
	size_t			size;
	volatile unsigned int		head;
	volatile unsigned int		tail;
}fifo_t;



#define FIFO_DEF(name,size,count)  \
unsigned char _fifo_array_##name [ size*count+1 ]={0}; \
fifo_t  _fifo_##name = { 0, size*count , _fifo_array_##name , size, 0 , 0}; \
fifo_t*  name = &_fifo_##name;



void fifo_unlock( fifo_t *fifo );
void fifo_lock( fifo_t *fifo );
int fifo_trylock( fifo_t * fifo );
void fifo_clear( fifo_t *fifo );
unsigned int fifo_valid( fifo_t *fifo);
unsigned int fifo_free( fifo_t *fifo);
int fifo_put( fifo_t * fifo, unsigned char data);
int fifo_put_force( fifo_t * fifo, unsigned char data);
int fifo_get( fifo_t *fifo , unsigned char * data);
int fifo_valid_item( fifo_t *fifo);
int fifo_put_item(fifo_t *fifo , void * item);
int fifo_put_item_force(fifo_t *fifo , void * item);
int fifo_get_item(fifo_t *fifo, void * item);
void fifo_init(fifo_t *fifo, char *buf, int size, int count);



#endif //__FIFO_H
