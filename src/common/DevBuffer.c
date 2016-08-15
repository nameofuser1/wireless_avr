#include "DevBuffer.h"
#include <string.h>


static void enter_critical(DevBuffer *buf);
static void leave_critical(DevBuffer *buf);

/*
 *	Circular buffer for using in embedded interfaces like usart, spi, etc.
 *
 *	Overflow checking prevents overlapping of writing and reading bytes.
 *	enter_critical and leave_critical must provide thread safety operations.
 */


int8_t DevBuffer_alloc
(DevBuffer *buf, uint32_t size, DevBufferEnterCritical ec, DevBufferLeaveCritical lc)
{
    buf->size = size;
    buf->data = (uint8_t*)malloc(sizeof(uint8_t)*size);
    buf->available = 0;
    buf->enter_critical = ec;
    buf->leave_critical = lc;

    if(buf->data == NULL)
    {
    	return 0;
    }

    return 1;
}


int8_t DevBuffer_write(DevBuffer *buf, uint8_t *data, uint32_t items_num)
{
	// To prevent concurrent modification by reader
	enter_critical(buf);

	if(buf->available + items_num > buf->size)
	{
		leave_critical(buf);
		return 0;
	}

	leave_critical(buf);


    uint32_t read_offset = 0;

    if(buf->write_pointer + items_num > buf->size)
    {
    	uint32_t bytes_until_buf_end = buf->size - buf->write_pointer;
    	memcpy(&(buf->data[buf->write_pointer]), data, bytes_until_buf_end);

    	buf->write_pointer = 0;

    	// To prevent concurrent modification by reader
    	enter_critical(buf);
    	buf->available += bytes_until_buf_end;
    	leave_critical(buf);

    	read_offset = bytes_until_buf_end;
    	items_num -= bytes_until_buf_end;
    }

    memcpy(&(buf->data[buf->write_pointer]), &(data[read_offset]), items_num);

    buf->write_pointer += items_num;

    // To prevent concurrent modification by reader
    enter_critical(buf);
    buf->available += items_num;
    leave_critical(buf);

    return 1;
}


int8_t DevBuffer_read(DevBuffer *buf, uint8_t *ptr, uint32_t items_num)
{
	if(items_num > buf->size)
	{
		return 0;
	}

	// To prevent concurrent modification by writer
	enter_critical(buf);
	if(items_num > buf->available)
	{
		items_num = buf->available;
	}
	leave_critical(buf);

	uint32_t index = buf->read_pointer + items_num;
	uint32_t write_offset = 0;
	uint32_t items_until_buffer_end = 0;

	if(index > buf->size)
	{
		items_until_buffer_end = buf->size - buf->read_pointer;
		memcpy(ptr, &(buf->data[buf->read_pointer]), items_until_buffer_end*sizeof(uint8_t));

		buf->read_pointer = 0;

		// Here is possible concurrent modification by writing IRQ handler.
		enter_critical(buf);
		buf->available -= items_until_buffer_end;
		leave_critical(buf);

		write_offset += items_until_buffer_end;
		items_num -= items_until_buffer_end;
	}

	memcpy(&(ptr[write_offset]), &(buf->data[buf->read_pointer]), items_num);
	buf->read_pointer += items_num;

	//Here is possible concurrent modification by writing IRQ handler.
	enter_critical(buf);
	buf->available -= items_num;
	leave_critical(buf);

    return 1;
}


void DevBuffer_free(DevBuffer *buf)
{
	if(buf->data != NULL)
	{
		free(buf->data);
	}
}


static void enter_critical(DevBuffer *buf)
{
	if(buf->enter_critical)
	{
		buf->enter_critical();
	}
}


static void leave_critical(DevBuffer *buf)
{
	if(buf->leave_critical)
	{
		buf->leave_critical();
	}
}
