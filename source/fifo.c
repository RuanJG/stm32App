#include "fifo.h"

void fifo_init(fifo_t *fifo, unsigned char *buf, int buf_len)
{
	fifo->head = 0;
	fifo->tail = 0;
	fifo->buf = buf;
	fifo->len = buf_len;
	fifo->count = 0;
	fifo->error_overflow = 0;
}

int fifo_valid_count(fifo_t *fifo)
{
  //return (fifo->len + fifo->head - fifo->tail) % fifo->len;
	return fifo->count;
}
int fifo_free_count(fifo_t *fifo)
{
  //return (fifo->len - 1 - fifo_avail(fifo));
	return fifo->len - 1 - fifo->count;
}

int fifo_overflow(fifo_t *fifo)
{
  return  fifo->error_overflow ;
}


int fifo_put(fifo_t *fifo, unsigned char c)
{
	int next;

	// check if FIFO has room
	next = (fifo->head + 1) % fifo->len;
	if (next == fifo->tail) {
		fifo->error_overflow = 1;
		return 0;
	}

	fifo->buf[fifo->head] = c;
	fifo->head = next;
	fifo->count++;

	return 1;
}

int fifo_force_put(fifo_t *fifo, unsigned char c)
{
	int next;

	// check if FIFO has room
	next = (fifo->head + 1) % fifo->len;
	if (next == fifo->tail) {
		fifo->error_overflow = 1;
		fifo->tail=(fifo->tail + 1) % fifo->len;
		fifo->count--;
	}

	fifo->buf[fifo->head] = c;
	fifo->head = next;
	fifo->count++;

	return 1;
}


int fifo_get(fifo_t *fifo, unsigned char *pc)
{
	int next;

	if (fifo->head == fifo->tail) {
		return 0;
	}

	next = (fifo->tail + 1) % fifo->len;
	*pc = fifo->buf[fifo->tail];
	fifo->tail = next;
	fifo->count--;

	return 1;
}
