#include <common/CircularBuffer.h>


bool CircularBuffer_alloc(CircularBuffer *buffer, uint32_t size)
{
    buffer->data = (void**) malloc(sizeof(void*)*size);
    if(buffer == NULL)
    {
        return false;
    }

    for(uint32_t i=0; i<size; i++)
    {
        buffer->data[i] = NULL;
    }

    buffer->write_pointer = 0;
    buffer->read_pointer = 0;
    buffer->buffer_size = size;

    return true;
}


void CircularBuffer_free(CircularBuffer *buffer, bool free_items)
{
    if(free_items)
    {
        for(uint32_t i=0; i<buffer->buffer_size; i++)
        {
            free((buffer->data)[i]);
        }
    }
    free(buffer->data);
}


void CircularBuffer_put(CircularBuffer *buffer, void *data)
{
    void **buffer_data = buffer->data;
    uint32_t write_pointer = buffer->write_pointer;
    buffer_data[write_pointer] = data;

    if(write_pointer < buffer->buffer_size-1)
    {
        buffer->write_pointer++;
    }
    else
    {
        buffer->write_pointer = 0;
    }
}


void* CircularBuffer_get(CircularBuffer *buffer)
{
    void *res = NULL;
    void **buffer_data = buffer->data;
    uint32_t read_pointer = buffer->read_pointer;
    if(buffer_data[read_pointer] != NULL)
    {
        res = buffer_data[read_pointer];
        buffer_data[read_pointer] = NULL;
        if(read_pointer < buffer->buffer_size-1)
        {
            buffer->read_pointer++;
        }
        else
        {
            buffer->read_pointer = 0;
        }
    }

    return res;
}


bool CircularBuffer_is_empty(CircularBuffer *buffer)
{
	void **data = buffer->data;
	uint32_t read_pointer = buffer->read_pointer;
	return (data[read_pointer] == NULL);
}
