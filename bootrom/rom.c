#include "sdram_phy.h"

volatile char* UART_RXTX = (char *)0xf0001000;
volatile char* UART_TXFULL = (char *)0xf0001004;
volatile char* SPI_FLASH_BITBANG_EN = (char *)0xf0007808;
unsigned int * FLASH_START = (unsigned int *)0xd0100000;
unsigned int * BAREBOX_START = (unsigned int *)0x41e00000;

inline void print(const char *string)
{
    for (; *string != 0; string++) {
        while (*UART_TXFULL);
        *UART_RXTX = *string;
    }
}

#if 0
const char hex[] = "0123456789ABCDEF";

inline void print_n(const char *data, int len)
{

    for (; len!=0; len--, data++) {
        while (*UART_TXFULL);
        *UART_RXTX = hex[(*data) >> 4];
        while (*UART_TXFULL);
        *UART_RXTX = hex[(*data) && 0xF];
    }
}
#endif

int _start()
{
    print("\n\rinit sdram\n\r");
    init_sequence();
    print("init spiflash\n\r");
    *SPI_FLASH_BITBANG_EN = 0;
#if 0
    static unsigned int * RAM_START = (unsigned int *)0x40000000;
    static unsigned int RAM_WORDS = 0x2000000/4;
begin:
    print("\n\rram test\n\r");
    for (int i=0; i<RAM_WORDS; i++)
        RAM_START[i] = i;
    for (int i=0; i<RAM_WORDS; i++) {
        if (RAM_START[i] != i) {
            print("failed\n\r");
            for (int j=28; j>=0; j-=4) {
                while (*UART_TXFULL);
                *UART_RXTX = hex[(i >> j)&0xf];
            }
            print("\n\r");
            goto begin;
        }
    }
    print("done\n\r");
#endif
    print("loading\n\r");
    int bytes = FLASH_START[0];
    int words = bytes / 4;
    if (bytes % 4)
        bytes += 1;

    for (int i=0; i<words; i++)
        BAREBOX_START[i] = FLASH_START[i+2];

    print("done\n\r");

    goto *BAREBOX_START;
}
