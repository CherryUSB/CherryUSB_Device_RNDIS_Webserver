/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBH_CORE_H
#define USBH_CORE_H

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
#include "usb_hc.h"
#include "usb_osal.h"
#include "usbh_hub.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USBH_ROOT_HUB_INDEX       1 /* roothub index*/
#define USBH_EX_HUB_INDEX         2 /* external hub index */
#define USBH_HUB_PORT_START_INDEX 1 /* first hub port index */

#ifdef CONFIG_USBHOST_HUB
#define ROOTHUB(hport) ((hport)->parent == NULL)
#else
#define ROOTHUB(hport) true
#endif

#define USB_CLASS_MATCH_VENDOR        0x0001
#define USB_CLASS_MATCH_PRODUCT       0x0002
#define USB_CLASS_MATCH_INTF_CLASS    0x0004
#define USB_CLASS_MATCH_INTF_SUBCLASS 0x0008
#define USB_CLASS_MATCH_INTF_PROTOCOL 0x0010

#define CLASS_CONNECT(hport, i)    ((hport)->config.intf[i].class_driver->connect(hport, i))
#define CLASS_DISCONNECT(hport, i) ((hport)->config.intf[i].class_driver->disconnect(hport, i))

#ifdef __ARMCC_VERSION /* ARM C Compiler */
#define CLASS_INFO_DEFINE          __attribute__((section("usbh_class_info"))) __USED __ALIGNED(1)
#elif defined(__GNUC__)
#define CLASS_INFO_DEFINE          __attribute__((section(".usbh_class_info"))) __USED __ALIGNED(1)
#endif

enum usbh_event_type {
    USBH_EVENT_CONNECTED = (1 << 0),
    USBH_EVENT_DISCONNECTED = (1 << 1),
};

struct usbh_class_info {
    uint8_t match_flags; /* Used for product specific matches; range is inclusive */
    uint8_t class;       /* Base device class code */
    uint8_t subclass;    /* Sub-class, depends on base class. Eg. */
    uint8_t protocol;    /* Protocol, depends on base class. Eg. */
    uint16_t vid;        /* Vendor ID (for vendor/product specific devices) */
    uint16_t pid;        /* Product ID (for vendor/product specific devices) */
    const struct usbh_class_driver *class_driver;
};

struct usbh_hubport;
struct usbh_class_driver {
    const char *driver_name;
    int (*connect)(struct usbh_hubport *hport, uint8_t intf);
    int (*disconnect)(struct usbh_hubport *hport, uint8_t intf);
};

typedef struct usbh_endpoint {
    struct usb_endpoint_descriptor ep_desc;
} usbh_endpoint_t;

typedef struct usbh_interface {
    struct usb_interface_descriptor intf_desc;
    struct usbh_endpoint ep[CONFIG_USBHOST_EP_NUM];
    char devname[CONFIG_USBHOST_DEV_NAMELEN];
    struct usbh_class_driver *class_driver;
    void *priv;
} usbh_interface_t;

typedef struct usbh_configuration {
    struct usb_configuration_descriptor config_desc;
    struct usbh_interface intf[CONFIG_USBHOST_INTF_NUM];
} usbh_configuration_t;

typedef struct usbh_hubport {
    bool connected;    /* True: device connected; false: disconnected */
    bool port_change;  /* True: port changed; false: port do not change */
    uint8_t port;      /* Hub port index */
    uint8_t dev_addr;  /* device address */
    uint8_t speed;     /* device speed */
    usbh_epinfo_t ep0; /* control ep info */
    struct usb_device_descriptor device_desc;
    struct usbh_configuration config;
#if 0
    uint8_t* config_desc;
#endif
    struct usb_setup_packet *setup;
    struct usbh_hub *parent; /*if NULL, is roothub*/
} usbh_hubport_t;

typedef struct usbh_hub {
    usb_slist_t list;
    uint8_t index;    /* Hub index */
    uint8_t nports;   /* Hub port number */
    uint8_t dev_addr; /* Hub device address */
    usbh_epinfo_t intin;
    uint8_t *int_buffer;
    struct hub_port_status *port_status;
    struct usb_hub_descriptor hub_desc;
    struct usbh_hubport child[CONFIG_USBHOST_EHPORTS];
    struct usbh_hubport *parent; /* Parent hub port */
} usbh_hub_t;

int usbh_initialize(void);
int lsusb(int argc, char **argv);
struct usbh_hubport *usbh_find_hubport(uint8_t dev_addr);
void *usbh_find_class_instance(const char *devname);
void usbh_device_mount_done_callback(struct usbh_hubport *hport);
void usbh_device_unmount_done_callback(struct usbh_hubport *hport);

#ifdef __cplusplus
}
#endif

#endif /* USBH_CORE_H */
