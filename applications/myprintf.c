/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-07     RuijinVV       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "myprintf.h"

#ifdef MY_PRINTF
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// #define MY_PRINTF_USE_FLOAT  // 使能浮点数

#ifdef MY_PRINTF_USE_FIFO
#define MY_PRINTF_FIFO_SIZE     4096
static struct {
    volatile uint16_t wp;
    volatile uint16_t rp;
    char buf[MY_PRINTF_FIFO_SIZE];
} my_printf_fifo = {.wp=0, .rp=0};

// 使用FIFO时，在线程中周期性调用此函数打印fifo数据
void MyPrintfThreadPrint( void )
{
    uint16_t w = my_printf_fifo.wp;
    uint16_t r = my_printf_fifo.rp;
    int len;

    if ( r == w )
        return;

    len = ( r < w ) ? w-r : MY_PRINTF_FIFO_SIZE-r;
    if ( len > 200 )
        len = 200;

//    rt_kprintf("%*s", len, &my_printf_fifo.buf[r]);
#ifdef RT_USING_DEVICE
    rt_device_t dev = rt_console_get_device();
    if (dev == RT_NULL)
    {
        rt_hw_console_output(&my_printf_fifo.buf[r]);
    }
    else
    {
        len = rt_device_write(dev, 0, &my_printf_fifo.buf[r], len);
    }
#else
    rt_hw_console_output(rt_log_buf);
#endif /* RT_USING_DEVICE */

    r += len;
    my_printf_fifo.rp = ( r >= MY_PRINTF_FIFO_SIZE ) ? 0 : r;

#if 0
    if ( my_printf_fifo.rp < w )
    {
        rt_kprintf("%*s", w-my_printf_fifo.rp, &my_printf_fifo.buf[my_printf_fifo.rp]);
        my_printf_fifo.rp = w;
    }
    else {
        rt_kprintf("%*s", MY_PRINTF_FIFO_SIZE-my_printf_fifo.rp, &my_printf_fifo.buf[my_printf_fifo.rp]);
        my_printf_fifo.rp = 0;
    }
#endif
}

// 写入字节到fifo中
int MyPrintfPutFifo( char c )
{
    uint16_t w = my_printf_fifo.wp+1;
    uint16_t r = my_printf_fifo.rp;

    if ( w >= MY_PRINTF_FIFO_SIZE )
        w = 0;
    if ( w==r )
        return 0;

    my_printf_fifo.buf[my_printf_fifo.wp] = c;
    my_printf_fifo.wp = w;
    return 1;
}

#endif

static const char VAL2NUM[] = "0123456789ABCDEF";
uint8_t place_holder_flag = 0;  // 占位符
uint8_t limit_len = 0;          // 输出指定长度

void PrintBit(int Bit);

static void PrintChar(char ch)
{
#ifdef MY_PRINTF_USE_FIFO
    MyPrintfPutFifo(ch);
#else
    UartSendByte( &dbg_uart, ch );
#endif
}

//输出字符串
static void PrintPlaceHolders(char c, int n)
{
    while(n--)
        PrintChar(c);
}

//输出字符串
static void PrintString(char *str)
{
    while(*str)
        PrintChar(*str++);
}

#if 0
static int Ex(char num)
{
    int Temp=1;

    while(num--)
    {
        Temp*=10;
    }

    return Temp;
}
#endif

//输出正整数
static void PrintUint(unsigned int Num)
{
#if 1
    uint8_t len = 0;
    uint32_t v = Num;
    char buf[16];

    do {
        buf[len++] = VAL2NUM[v%10];
        v /= 10;
    } while( v );

    if (( limit_len > 0 )&&( len < limit_len ))    // 限制长度
    {
        PrintPlaceHolders( place_holder_flag!=0 ? '0' : ' ', limit_len-len );
    }

    while( len-- )
    {
        PrintChar(buf[len]);
    }

#else

    unsigned char i;
    unsigned char number;
    unsigned int Temp=Num;
    unsigned char Length=0;

    if(Num==0)
    {
        if ( limit_len > 0 )    // 限制长度
        {
            PrintPlaceHolders( place_holder_flag!=0 ? '0' : ' ', limit_len-1 );
        }
        PrintChar('0');

        return;
    }

    while(Temp)
    {
        Length++;

        Temp/=10;
    }

    if (( limit_len > 0 )&&( Length < limit_len ))    // 限制长度
    {
        PrintPlaceHolders( place_holder_flag!=0 ? '0' : ' ', limit_len-Length );
    }

    Temp=Num;

    for( i=0; i<Length; i++)
    {
        number=Temp/Ex(Length-bit-1);

        PrintChar(number+'0');

        Temp%=Ex(Length-bit-1);
    }
#endif
}

//输出整数
static void PrintInt_2(int Num)
{
#if 1
    if ( Num < 0 )
    {
        PrintChar('-');
        PrintUint(-Num);
    }
    else
    {
        PrintUint(Num);
    }
#else
    if(Num&(1<<31))
    {
        PrintChar('-');

      //  Num=Num&(1<<31);

        PrintInt(~(Num-1));
    }
    else
    {
        PrintInt(Num);
    }
#endif
}

// 输出16进制数
static void PrintHex(int h)
{
    uint32_t v = (uint32_t)h;
    int len = 0;
    char buf[16];

    do {
        buf[len++] = VAL2NUM[v&0xF];
        v >>= 4;
    }while ( v );

    if (( limit_len > 0 )&&( len < limit_len ))    // 限制长度
    {
        PrintPlaceHolders( place_holder_flag!=0 ? '0' : ' ', limit_len-len );
    }

    while( len-- )
    {
        PrintChar(buf[len]);
    }
}

//输出浮点数
static void PrintFloat(float Num)
{
#ifdef MY_PRINTF_USE_FLOAT
    int Temp=(int)Num;

    int fTemp=Ex(6)*(Num-(int)Num);

    PrintInt_2(Temp);

    PrintChar('.');
    if(fTemp&(1<<31))
        PrintInt_2(~(fTemp-1));
    else
    {
        PrintInt_2(fTemp);
    }
#endif
}

void MyPrintf(const char *fmt,...)
{
    va_list ap;

    va_start(ap, fmt);

    while(*fmt)
    {
        switch(*fmt)
        {
        case '%':
            {
                place_holder_flag = limit_len = 0;
                if ( *(fmt+1) == '0' )
                {
                    place_holder_flag = 1;
                    fmt++;
                }
                if (( *(fmt+1) >= '1' )&&( *(fmt+1) <= '9' ))
                {
                    fmt++;
                    limit_len = *fmt - '0';
                    while (( *(fmt+1) >= '0' )&&( *(fmt+1) <= '9' ))
                    {
                        fmt++;
                        limit_len = (limit_len*10) + (*fmt - '0');
                    }
                }
                switch(*(fmt+1))
                {
                case 'c':PrintChar(va_arg(ap,int));fmt++;break;

                case 's':PrintString(va_arg(ap,char*));fmt++;break;

                case 'u':PrintUint(va_arg(ap,unsigned int));fmt++;break;

                case 'd':PrintInt_2(va_arg(ap,int));fmt++;break;

                case 'X':
                case 'x':PrintHex(va_arg(ap,unsigned int));fmt++;break;

                case 'f':PrintFloat(va_arg(ap,double));fmt++;break;

                default:PrintChar('%');break;
                }
            }
            break;

        case '\r':PrintChar('\r');break;
        case '\n':PrintChar('\n');break;

        case '\t':PrintChar('\t');break;

        default :PrintChar(*fmt);

        }
        fmt++;
    }
    va_end(ap);
}

void MyMemDump( uint8_t* data, uint16_t len )
{
    while( len-- )
    {
        MyPrintf( " %02X", *data++);
    }
}
#endif
