/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-14     RT-Thread    first version
 */

#include <rtthread.h>
#include "gpio.h"
#include <board.h>
#include <rtdevice.h>
#include "myprintf.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED1    GET_PIN(A,12)
#define LED2    GET_PIN(A,11)

void tskLed( void* arg )
{
    rt_base_t pin = (rt_base_t)(arg);
    int v=0;

    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    while( 1 )
    {
        rt_pin_write(pin, v);
        rt_thread_mdelay(1000);
        v = 1-v;
    }
}

extern int clock_information(void);

int main(void)
{
    int count = 1;
    rt_thread_t tsk;

    MX_GPIO_Init();

    // Led任务1
    tsk=rt_thread_create("led1", tskLed, (void*)LED1, 1024, 9, 10);
    if ( tsk != NULL )
    {
        rt_thread_startup(tsk);
    }

    rt_thread_mdelay(500);
    // Led任务2
    tsk=rt_thread_create("led2", tskLed, (void*)LED2, 1024, 10, 10);
    if ( tsk != NULL )
    {
        rt_thread_startup(tsk);
    }


    LOG_D("Ver: %s %s", __DATE__, __TIME__);

#if 1
    // clock_information();
    LOG_D("System Clock information");
    LOG_D("SYSCLK_Frequency = %d", HAL_RCC_GetSysClockFreq());
    LOG_D("HCLK_Frequency   = %d", HAL_RCC_GetHCLKFreq());
    LOG_D("PCLK1_Frequency  = %d", HAL_RCC_GetPCLK1Freq());
    LOG_D("PCLK2_Frequency  = %d", HAL_RCC_GetPCLK2Freq());
#endif

    // loop
    while (1)
    {
        // LOG_D("Hello RT-Thread!");
        MyPrintfThreadPrint();  // 打印
        rt_thread_mdelay(100);
    }

    return RT_EOK;
}



