#ifndef CIRCULARBUFFER_H_INCLUDED
#define CIRCULARBUFFER_H_INCLUDED

#include <stm32f10x.h>
#include <stdbool.h>
#include <malloc.h>


typedef struct {

    void    **data;
    uint32_t     write_pointer;
    uint32_t     read_pointer;
    uint32_t     buffer_size;

}   CircularBuffer;


bool    CircularBuffer_alloc(CircularBuffer *buffer, unsigned long size);
void    CircularBuffer_free(CircularBuffer *buffer, bool free_items);
void    CircularBuffer_put(CircularBuffer *buffer, void *data);
void*   CircularBuffer_get(CircularBuffer *buffer);
bool	CircularBuffer_is_empty(CircularBuffer *buffer);

#endif // CIRCULARBUFFER_H_INCLUDED
