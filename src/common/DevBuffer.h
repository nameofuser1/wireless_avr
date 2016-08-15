#ifndef ARRAY_CIRCULAR_BUFFER_H_INCLUDED
#define ARRAY_CIRCULAR_BUFFER_H_INCLUDED

#include <stdlib.h>
#include <inttypes.h>


typedef void (*DevBufferEnterCritical)(void);
typedef void (*DevBufferLeaveCritical)(void);


typedef struct {

    uint32_t size;
    uint32_t write_pointer;
    uint32_t read_pointer;
    uint32_t available;

    uint8_t *data;
    void 	(*enter_critical)(void);
    void	(*leave_critical)(void);

} DevBuffer;


int8_t    	DevBuffer_alloc(DevBuffer *buf, uint32_t size,
							DevBufferEnterCritical ec, DevBufferLeaveCritical lc);

int8_t    	DevBuffer_write(DevBuffer *buf, uint8_t *data, uint32_t items_num);
int8_t	  	DevBuffer_read(DevBuffer *buf, uint8_t *ptr, uint32_t items_num);
void    	DevBuffer_free(DevBuffer *buf);

#endif // ARRAY_CIRCULAR_BUFFER_H_INCLUDED
