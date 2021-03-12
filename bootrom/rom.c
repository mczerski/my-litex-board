volatile char* UART_RXTX = (char *)0xf0001000;
volatile unsigned int * FLASH_BASE = (unsigned int *)0xd0100000;
volatile unsigned int * BAREBOX_START = (unsigned int *)0x41e00000;
const int FLASH_SZIE = 0x00100000;

inline void print(const char *string)
{
    for (; *string != 0; string++)
        *UART_RXTX = *string;
}

inline void print_n(const char *data, int len)
{
    for (; len!=0; len--, data++)
        *UART_RXTX = *data;
}

int _start()
{
    print("start\n\r");

    int length = FLASH_BASE[0];

    int words = length / 4;
    for (int i=0; i<words; i++)
        BAREBOX_START[i] = FLASH_BASE[i+2];

    int bytes = length % 4;
    volatile char * dst_tail = (volatile char *)&BAREBOX_START[words];
    volatile char * src_tail = (volatile char *)&FLASH_BASE[words+2];
    for (int i=0; i<bytes; i++)
        dst_tail[i] = src_tail[i];

    print("done\n\r");

    goto *BAREBOX_START;
}
