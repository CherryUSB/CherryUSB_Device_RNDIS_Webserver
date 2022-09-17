/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBD_CORE_H
#define USBD_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "usb_config.h"
#include "usb_util.h"
#include "usb_errno.h"
#include "usb_def.h"
#include "usb_list.h"
#include "usb_mem.h"
#include "usb_log.h"
#include "usb_dc.h"

enum usbd_event_type {
    /* USB DCD IRQ */
    USBD_EVENT_ERROR,        /** USB error reported by the controller */
    USBD_EVENT_RESET,        /** USB reset */
    USBD_EVENT_SOF,          /** Start of Frame received */
    USBD_EVENT_CONNECTED,    /** USB connected*/
    USBD_EVENT_DISCONNECTED, /** USB disconnected */
    USBD_EVENT_SUSPEND,      /** USB connection suspended by the HOST */
    USBD_EVENT_RESUME,       /** USB connection resumed by the HOST */

    /* USB DEVICE STATUS */
    USBD_EVENT_CONFIGURED,    /** USB configuration done */
    USBD_EVENT_SET_INTERFACE, /** USB interface selected */
    USBD_EVENT_UNKNOWN
};

typedef int (*usbd_request_handler)(struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
typedef void (*usbd_endpoint_callback)(uint8_t ep, uint32_t nbytes);
typedef void (*usbd_notify_handler)(uint8_t event, void *arg);

extern usb_slist_t usbd_intf_head;

struct usbd_endpoint {
    uint8_t ep_addr;
    usbd_endpoint_callback ep_cb;
};

struct usbd_interface {
    usb_slist_t list;
    /** Handler for USB Class specific commands*/
    usbd_request_handler class_handler;
    /** Handler for USB Vendor specific commands */
    usbd_request_handler vendor_handler;
    /** Handler for USB custom specific commands */
    usbd_request_handler custom_handler;
    /** Handler for USB event notify commands */
    usbd_notify_handler notify_handler;
    const uint8_t *hid_report_descriptor;
    uint32_t hid_report_descriptor_len;
    uint8_t intf_num;
};

void usbd_desc_register(const uint8_t *desc);
void usbd_msosv1_desc_register(struct usb_msosv1_descriptor *desc);
void usbd_msosv2_desc_register(struct usb_msosv2_descriptor *desc);
void usbd_bos_desc_register(struct usb_bos_descriptor *desc);

void usbd_add_interface(struct usbd_interface *intf);
void usbd_add_endpoint(struct usbd_endpoint *ep);

bool usb_device_is_configured(void);
void usbd_configure_done_callback(void);
int usbd_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CORE_H */
