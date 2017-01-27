/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <Driver_USART.h>
#include <string.h>
#include <stdlib.h>

#include <stm32f10x.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#undef errno
extern int errno;
extern int  _end;

#define PRINTF_DRIVER Driver_USART1
extern ARM_DRIVER_USART PRINTF_DRIVER;

/*
 * !!!IMPORTANT!!!
 * We can't wait until log message is sent as it may take a long time
 * So we can free memory only on the next _write call
 * So be careful with sending big log messages
 */
static char *proc_data = NULL;
static int  *data_len = 0;


/*This function is used for handle heap option*/
__attribute__ ((used))
caddr_t _sbrk ( int incr )
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

    heap += incr;

    return (caddr_t) prev_heap;
}

__attribute__ ((used))
int link(char *old, char *new)
{
    return -1;
}

__attribute__ ((used))
int _close(int file)
{
    return -1;
}

__attribute__ ((used))
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

__attribute__ ((used))
int _isatty(int file)
{
    return 1;
}

__attribute__ ((used))
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/*Low layer read(input) function*/
__attribute__ ((used))
int _read(int file, char *ptr, int len)
{

#if 0
     //user code example
     int i;
     (void)file;

     for(i = 0; i < len; i++)
     {
        // UART_GetChar is user's basic input function
        *ptr++ = UART_GetChar();
     }

#endif

    return len;
}


/* Low layer write(output) function */
__attribute__ ((used))
int _write(int file, char *ptr, int len)
{
	(void)file;
#if 1

	while(!(USART1->SR & USART_SR_TXE));
	for(int i=0; i<len; i++)
	{
		USART1->DR = *ptr++;
		while(!(USART1->SR & USART_SR_TXE));
	}
/*
     while(PRINTF_DRIVER.GetStatus().tx_busy == 1);

     if(proc_data != NULL)
     {
    	 free(proc_data);
     }
     proc_data = (char*)malloc(sizeof(char)*len);
     memcpy(proc_data, ptr, len);

     PRINTF_DRIVER.Send((void*)proc_data, len);
*/
#endif

    return len;
}

__attribute__ ((used))
void abort(void)
{
    /* Abort called */
    while(1);
}


void _exit(int i)
{
    while(1);
}

/* --------------------------------- End Of File ------------------------------ */
