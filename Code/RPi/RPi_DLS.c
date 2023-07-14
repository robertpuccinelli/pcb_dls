//RPi_DLS.c - Library for custom dynamic light scattering board - reads data and handles UART
/*
    All RPi pins used for the sensor can be read through GPLEV0, where bits 31-0 represent GPIO 31-0.
    In practice, all pins can be read simultaneously and then data can be read by filtering the 14 data bits.
    The address for GPLEV0 is 0x7E200034

    At reset all GPIO are set to input, so the function should not need to be changed
*/

#ifndef RPI_H
#define RPI_H

#include <stdint-gcc.h>

/* GENERAL DEFS */
#define PBASE 0x3F000000
extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );
extern void dummy ( unsigned int );

/* DATA DEFS */
#define GPLEV0          (PBASE+0x00200034)
#define D0              (12) //BCM GPIO 12
#define D1              (13)
#define D2              (16)
#define D3              (17)
#define D4              (18)
#define D5              (19)
#define D6              (20)
#define D7              (21)
#define D8              (22)
#define D9              (23)
#define D10             (24)
#define D11             (25)
#define D12             (26)
#define D13             (27)
#define DCLK            (11)

/* UART DEFS */
#define GPFSEL1         (PBASE+0x00200004)
#define GPSET0          (PBASE+0x0020001C)
#define GPCLR0          (PBASE+0x00200028)
#define GPPUD           (PBASE+0x00200094)
#define GPPUDCLK0       (PBASE+0x00200098)

#define AUX_ENABLES     (PBASE+0x00215004)
#define AUX_MU_IO_REG   (PBASE+0x00215040)
#define AUX_MU_IER_REG  (PBASE+0x00215044)
#define AUX_MU_IIR_REG  (PBASE+0x00215048)
#define AUX_MU_LCR_REG  (PBASE+0x0021504C)
#define AUX_MU_MCR_REG  (PBASE+0x00215050)
#define AUX_MU_LSR_REG  (PBASE+0x00215054)
#define AUX_MU_MSR_REG  (PBASE+0x00215058)
#define AUX_MU_SCRATCH  (PBASE+0x0021505C)
#define AUX_MU_CNTL_REG (PBASE+0x00215060)
#define AUX_MU_STAT_REG (PBASE+0x00215064)
#define AUX_MU_BAUD_REG (PBASE+0x00215068)

//GPIO14  TXD0 and TXD1
//GPIO15  RXD0 and RXD1

/* GPIO INPUT */
int gpioRead(void){ return GET32(GPLEV0); }

uint16_t dataRead(void)
{
    int ra = gpioRead();
    int d_lower = ra & (0b11 << D0);
    int d_upper = ra & (0b111111111111 << D2);
    uint16_t data = (d_upper >> (D2 - 1)) | (d_lower >> (D0 - 1));
    return data;
}

int dataClk(void)
{
    int clk_state = 0;
    int ra = gpioRead();
    if(ra & (0b1 << DCLK))
    {
        clk_state = 1;
    }
    return clk_state;
}


/* UART INIT */
void uartInit ( void )
{
    unsigned int ra;

    PUT32(AUX_ENABLES, 1);
    PUT32(AUX_MU_IER_REG, 0);
    PUT32(AUX_MU_CNTL_REG, 0);
    PUT32(AUX_MU_LCR_REG, 3);
    PUT32(AUX_MU_MCR_REG, 0);
    PUT32(AUX_MU_IER_REG, 0);
    PUT32(AUX_MU_IIR_REG, 0xC6);
    PUT32(AUX_MU_BAUD_REG, 270);
    ra = GET32(GPFSEL1);
    ra &= ~(7<<12); //gpio14
    ra |= 2<<12;    //alt5
    ra &= ~(7<<15); //gpio15
    ra |= 2<<15;    //alt5
    PUT32(GPFSEL1, ra);
    PUT32(GPPUD, 0);
    for(ra=0; ra<150; ra++) dummy(ra);
    PUT32(GPPUDCLK0, (1<<14)|(1<<15));
    for(ra=0; ra<150; ra++) dummy(ra);
    PUT32(GPPUDCLK0, 0);
    PUT32(AUX_MU_CNTL_REG, 3);
}

/* UART READ */
static unsigned int uartRecv ( void )
{
    while(1)
    {
        if(GET32(AUX_MU_LSR_REG)&0x01) break;
    }
    return(GET32(AUX_MU_IO_REG)&0xFF);
}

static unsigned int uartCheck ( void )
{
    if(GET32(AUX_MU_LSR_REG)&0x01) return(1);
    return(0);
}

void uartFlush ( void )
{
    while(1)
    {
        if((GET32(AUX_MU_LSR_REG)&0x100)==0) break;
    }
}

unsigned int uartRead(void)
{
    int c = 0x00;
    if(uartCheck())
    {
        c = uartRecv();
        uartFlush();
    }
    return c;
}

/* UART SEND */
void uartSend ( unsigned int c )
{
    while(1)
    {
        if(GET32(AUX_MU_LSR_REG)&0x20) break;
    }
    PUT32(AUX_MU_IO_REG,c);
}

#endif

// Contributions:
// A majority of the UART implementation was copied from David Welch's example: https://github.com/dwelch67/raspberrypi/blob/master/boards/pi3/aarch64/uart03/periph.c
//-------------------------------------------------------------------------
//
// Copyright (c) 2012 David Welch dwelch@dwelch.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------