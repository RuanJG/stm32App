#ifndef __FIFO_H
#define __FIFO_H

typedef struct {
	int         	head;
	int         	tail;
	unsigned char	*buf;
	unsigned int 	len;
	unsigned int 	count;
	unsigned char	error_overflow;
} fifo_t;


int fifo_valid_count(fifo_t *fifo);
int fifo_free_count(fifo_t *fifo);
int fifo_overflow(fifo_t *fifo);
int fifo_put(fifo_t *fifo, unsigned char c);
int fifo_force_put(fifo_t *fifo, unsigned char c);
int fifo_get(fifo_t *fifo, unsigned char *pc);


// two way to init

void fifo_init(fifo_t *fifo, unsigned char *buf, int buf_len);

#define FIFO_DEF(name,size) \
	unsigned char fifoBuffer_##name[size]; \
	fifo_t name={0, 0, fifoBuffer_##name, size, 0, 0};



#endif //__FIFO_H
