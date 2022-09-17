/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-07     RuijinVV       the first version
 */
#ifndef _MYPRINTF_H_
#define _MYPRINTF_H_


#define MY_PRINTF
#define MY_PRINTF_USE_FIFO      // 使用FIFO缓存

#ifdef MY_PRINTF

void MyPrintf(const char *fmt,...);
void MyMemDump( uint8_t* data, uint16_t len );

#ifdef MY_PRINTF_USE_FIFO
void MyPrintfThreadPrint( void );
#else
#define MyPrintfThreadPrint()
#endif

#define DBG_PRINT(fmt...)       do { MyPrintf( fmt ); } while(0)
#define DBG_DUMPMEM(DATA, LEN)  do { MyMemDump( DATA, LEN ); } while(0)

#else

#define MyPrintfThreadPrint()
#define MyPrintf(fmt...)
#define DBG_PRINT(fmt...)
#define DBG_DUMPMEM(DATA, LEN)

#endif



#endif /* APPLICATIONS_MYPRINTF_H_ */
