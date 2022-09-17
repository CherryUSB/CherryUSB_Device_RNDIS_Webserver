/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usb_osal.h"
#include "usb_errno.h"
#include <FreeRTOS.h>
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

usb_osal_thread_t usb_osal_thread_create(const char *name, uint32_t stack_size, uint32_t prio, usb_thread_entry_t entry, void *args)
{
    TaskHandle_t htask = NULL;
    stack_size /= sizeof(StackType_t);
    xTaskCreate(entry, name, stack_size, args, prio, &htask);
    return (usb_osal_thread_t)htask;
}

void usb_osal_thread_suspend(usb_osal_thread_t thread)
{
    vTaskSuspend(thread);
}

void usb_osal_thread_resume(usb_osal_thread_t thread)
{
    vTaskResume(thread);
}

usb_osal_sem_t usb_osal_sem_create(uint32_t initial_count)
{
    return (usb_osal_sem_t)xSemaphoreCreateCounting(1, initial_count);
}

void usb_osal_sem_delete(usb_osal_sem_t sem)
{
    vSemaphoreDelete((SemaphoreHandle_t)sem);
}

int usb_osal_sem_take(usb_osal_sem_t sem, uint32_t timeout)
{
    return (xSemaphoreTake((SemaphoreHandle_t)sem, pdMS_TO_TICKS(timeout)) == pdPASS) ? 0 : -ETIMEDOUT;
}

int usb_osal_sem_give(usb_osal_sem_t sem)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    ret = xSemaphoreGiveFromISR((SemaphoreHandle_t)sem, &xHigherPriorityTaskWoken);
    if (ret == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    return (ret == pdPASS) ? 0 : -EINVAL;
}

usb_osal_mutex_t usb_osal_mutex_create(void)
{
    return (usb_osal_mutex_t)xSemaphoreCreateMutex();
}

void usb_osal_mutex_delete(usb_osal_mutex_t mutex)
{
    vSemaphoreDelete((SemaphoreHandle_t)mutex);
}

int usb_osal_mutex_take(usb_osal_mutex_t mutex)
{
    return (xSemaphoreTake((SemaphoreHandle_t)mutex, portMAX_DELAY) == pdPASS) ? 0 : -ETIMEDOUT;
}

int usb_osal_mutex_give(usb_osal_mutex_t mutex)
{
    return (xSemaphoreGive((SemaphoreHandle_t)mutex) == pdPASS) ? 0 : -EINVAL;
}

usb_osal_event_t usb_osal_event_create(void)
{
    return (usb_osal_event_t)xEventGroupCreate();
}

void usb_osal_event_delete(usb_osal_event_t event)
{
    vEventGroupDelete((EventGroupHandle_t)event);
}

int usb_osal_event_recv(usb_osal_event_t event, uint32_t set, uint32_t *recved)
{
    *recved = xEventGroupWaitBits((EventGroupHandle_t)event, set, pdTRUE, pdFALSE, portMAX_DELAY);
    return 0;
}

int usb_osal_event_send(usb_osal_event_t event, uint32_t set)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int ret;

    ret = xEventGroupSetBitsFromISR((EventGroupHandle_t)event, set, &xHigherPriorityTaskWoken);
    if (ret == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return (ret == pdPASS) ? 0 : -EINVAL;
}

size_t usb_osal_enter_critical_section(void)
{
    taskDISABLE_INTERRUPTS();
    return 1;
}

void usb_osal_leave_critical_section(size_t flag)
{
    taskENABLE_INTERRUPTS();
}

void usb_osal_msleep(uint32_t delay)
{
    vTaskDelay(pdMS_TO_TICKS(delay));
}
