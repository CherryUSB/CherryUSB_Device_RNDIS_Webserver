#include "usbd_core.h"
#include "usbd_cdc.h"
//#include "usbd_msc.h"
#include "usbd_rndis.h"
#include <rtthread.h>
#include <rtdevice.h>

#define ENABLE_CDC
#define ENABLE_RNDIS

#define USBD_VID           0xEFFF
#define USBD_PID           0xEFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#ifdef ENABLE_CDC
/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x82
#endif

#ifdef ENABLE_RNDIS
#define RNDIS_IN_EP  0x84
#define RNDIS_OUT_EP 0x04
#define RNDIS_INT_EP 0x83

#define RNDIS_DESCRIPTOR_LEN (8 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7)

#define RNDIS_DESCRIPTOR_INIT(bFirstInterface, int_ep, out_ep, in_ep, str_idx)               \
    /* Interface Associate */                                                                  \
0x08,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,             /* bDescriptorType */               \
    bFirstInterface,                                       /* bFirstInterface */               \
    0x02,                                                  /* bInterfaceCount */               \
    USB_DEVICE_CLASS_WIRELESS,                                  /* bFunctionClass */                \
    0x01, /*CDC_ABSTRACT_CONTROL_MODEL,*/                            /* bFunctionSubClass */             \
    0x03, /*CDC_COMMON_PROTOCOL_AT_COMMANDS,*/                       /* bFunctionProtocol */             \
    0x00,                                                  /* iFunction */                     \
0x09,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_INTERFACE,                         /* bDescriptorType */               \
    bFirstInterface,                                       /* bInterfaceNumber */              \
    0x00,                                                  /* bAlternateSetting */             \
    0x01,                                                  /* bNumEndpoints */                 \
    USB_DEVICE_CLASS_WIRELESS,                                  /* bInterfaceClass */               \
    0x01, /*CDC_ABSTRACT_CONTROL_MODEL,*/                            /* bInterfaceSubClass */            \
    0x03, /*CDC_COMMON_PROTOCOL_AT_COMMANDS,*/                       /* bInterfaceProtocol */            \
    str_idx,                                               /* iInterface */                    \
0x05,                                                  /* bLength */                       \
    CDC_CS_INTERFACE,                                      /* bDescriptorType */               \
    CDC_FUNC_DESC_HEADER,                                  /* bDescriptorSubtype = 0x00 */            \
    WBVAL(CDC_V1_10),                                      /* bcdCDC */                        \
0x05,                                                  /* bLength */                       \
    CDC_CS_INTERFACE,                                      /* bDescriptorType */               \
    CDC_FUNC_DESC_CALL_MANAGEMENT,                         /* bDescriptorSubtype = 0x01 Call Management */            \
    0x00,                                                  /* bmCapabilities */                \
    (uint8_t)(bFirstInterface + 1),                        /* bDataInterface */                \
0x04,                                                  /* bLength */                       \
    CDC_CS_INTERFACE,                                      /* bDescriptorType */               \
    CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT,             /* bDescriptorSubtype = 0x02 Abstract Control Management */            \
    0x00,                                                  /* bmCapabilities = Device supports the notification Network_Connection */                \
0x05,                                                  /* bLength */                       \
    CDC_CS_INTERFACE,                                      /* bDescriptorType */               \
    CDC_FUNC_DESC_UNION,                                   /* bDescriptorSubtype = 0x06 Union */            \
    bFirstInterface,                                       /* bControlInterface = "RNDIS Communications Control" */              \
    (uint8_t)(bFirstInterface + 1),                        /* bSubordinateInterface0 = "RNDIS Ethernet Data" */              \
0x07,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                          /* bDescriptorType */               \
    int_ep,                                                /* bEndpointAddress */              \
    0x03,                                                  /* bmAttributes */                  \
    0x40, 0x00,                                            /* wMaxPacketSize */                \
    0x0A,                                                  /* bInterval = 1 ms polling from host */                     \
0x09,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_INTERFACE,                         /* bDescriptorType */               \
    (uint8_t)(bFirstInterface + 1),                        /* bInterfaceNumber */              \
    0x00,                                                  /* bAlternateSetting */             \
    0x02,                                                  /* bNumEndpoints */                 \
    CDC_DATA_INTERFACE_CLASS,                              /* bInterfaceClass: CDC */               \
    0x00,                                                  /* bInterfaceSubClass */            \
    0x00,                                                  /* bInterfaceProtocol */            \
    0x00,                                                  /* iInterface */                    \
0x07,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                          /* bDescriptorType */               \
    out_ep,                                                /* bEndpointAddress = ENDPOINT [OUT] */              \
    0x02,                                                  /* bmAttributes = BULK */                  \
    0x40, 0x00,                                            /* wMaxPacketSize */                \
    0x00,                                                  /* bInterval */                     \
0x07,                                                  /* bLength */                       \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                          /* bDescriptorType */               \
    in_ep,                                                 /* bEndpointAddress = ENDPOINT [IN] */              \
    0x02,                                                  /* bmAttributes = BULK */                  \
    0x40, 0x00,                                            /* wMaxPacketSize */                \
    0x00                                                   /* bInterval */
#endif

/*!< config descriptor size */
#if defined(ENABLE_CDC) && defined(ENABLE_RNDIS)
#define USB_INTERFACE_COUNT 4
#define USB_INTERFACE_RNDIS_START 0
#define USB_INTERFACE_CDC_START 2
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN + RNDIS_DESCRIPTOR_LEN)
#elif defined(ENABLE_CDC)
#define USB_INTERFACE_COUNT 2
#define USB_INTERFACE_CDC_START 0
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)
#elif defined(ENABLE_RNDIS)
#define USB_INTERFACE_COUNT 2
#define USB_INTERFACE_RNDIS_START 0
#define USB_CONFIG_SIZE (9 + RNDIS_DESCRIPTOR_LEN)
#else
#error "no enable class"
#endif

/*!< global descriptor */
static const uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, USB_INTERFACE_COUNT, 0x01, USB_CONFIG_SELF_POWERED, USBD_MAX_POWER),
#ifdef ENABLE_RNDIS
    RNDIS_DESCRIPTOR_INIT(USB_INTERFACE_RNDIS_START, RNDIS_INT_EP, RNDIS_OUT_EP, RNDIS_IN_EP, 0x04),
#endif
#ifdef ENABLE_CDC
    CDC_ACM_DESCRIPTOR_INIT(USB_INTERFACE_CDC_START, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),
#endif
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x28,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'C', 0x00,                  /* wcChar10 */
    'D', 0x00,                  /* wcChar11 */
    'C', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'R', 0x00,                  /* wcChar14 */
    'N', 0x00,                  /* wcChar15 */
    'D', 0x00,                  /* wcChar16 */
    'I', 0x00,                  /* wcChar17 */
    'S', 0x00,                  /* wcChar18 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '2', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
    ///////////////////////////////////////
    /// string4 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'r', 0x00,                  /* wcChar0 */
    'n', 0x00,                  /* wcChar1 */
    'd', 0x00,                  /* wcChar2 */
    'i', 0x00,                  /* wcChar3 */
    's', 0x00,                  /* wcChar4 */
    ' ', 0x00,                  /* wcChar5 */
    'd', 0x00,                  /* wcChar6 */
    'e', 0x00,                  /* wcChar7 */
    'm', 0x00,                  /* wcChar8 */
    'o', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif


#ifdef ENABLE_CDC

uint8_t cdc_rx_buffer[2048];
uint8_t cdc_tx_buffer[2048];

volatile bool ep_tx_busy_flag = false;
volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
     for (int i = 0; i < nbytes; i++) {
         rt_kprintf("%02x ", cdc_rx_buffer[i]);
     }
     rt_kprintf("\r\n");
    /* setup next out ep read transfer */
    usbd_ep_start_read(CDC_OUT_EP, cdc_rx_buffer, 2048);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(CDC_IN_EP, NULL, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

void usbd_cdc_acm_set_dtr(uint8_t intf, bool dtr)
{
    if (dtr) {
        dtr_enable = 1;
    } else {
        dtr_enable = 0;
    }
}

void cdc_acm_data_send_with_dtr_test(void)
{
    if (dtr_enable) {
        memset(&cdc_tx_buffer[10], 'a', 20);
        ep_tx_busy_flag = true;
        usbd_ep_start_write(CDC_IN_EP, cdc_tx_buffer, 20);
        while (ep_tx_busy_flag) {
        }
    }
}

static void vcom_thread_entry(void *parameter)
{
    while (1)
    {
        cdc_acm_data_send_with_dtr_test();
        rt_thread_delay(3000);
    }

}

#endif

#ifdef ENABLE_RNDIS
// ----- RNDIS ------
#include <netif/ethernetif.h>
#include <lwipopts.h>
struct eth_device rndis_dev;

static uint8_t rndis_received[RNDIS_MTU + 20];
static volatile int rndis_recvSize = 0;

static void RndisPktHandle( uint8_t* data, uint32_t len );

volatile bool rndis_ep_tx_busy_flag = false;
uint8_t rndis_rx_buf[1536+100];
uint8_t rndis_tx_buf[1536+100];
#include "usbd_rndis.h"
#include "rndis_protocol.h"

int dump_count = 0;

void usbd_rndis_data_out(uint8_t ep, uint32_t nbytes)
{
    //USB_LOG_RAW("rndis out len:%d\r\n", nbytes);

    if ( nbytes>sizeof(rndis_data_packet_t) )
    {
        uint32_t rndis_MessageLength = ((rndis_data_packet_t *)rndis_rx_buf)->MessageLength;
        if ( ( nbytes == rndis_MessageLength )
            || ((nbytes==(rndis_MessageLength+1))&&((rndis_MessageLength&(CDC_MAX_MPS-1))==0)) )
        {
            RndisPktHandle( rndis_rx_buf, rndis_MessageLength );
        }
        else {
            DBG_PRINT("Err len %d %d\r\n", nbytes, rndis_MessageLength);
        }
    }

    /* setup next out ep read transfer */
    usbd_ep_start_read(RNDIS_OUT_EP, rndis_rx_buf, sizeof(rndis_rx_buf));
}

void usbd_rndis_data_in(uint8_t ep, uint32_t nbytes)
{
    //USB_LOG_RAW("rndis in len:%d\r\n", nbytes);

    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(RNDIS_IN_EP, NULL, 0);
    } else {
        rndis_ep_tx_busy_flag = false;
    }
}


/*!< endpoint call back */
struct usbd_endpoint rndis_out_ep = {
    .ep_addr = RNDIS_OUT_EP,
    .ep_cb = usbd_rndis_data_out
};

struct usbd_endpoint rndis_in_ep = {
    .ep_addr = RNDIS_IN_EP,
    .ep_cb = usbd_rndis_data_in
};

static void RndisPktHandle( uint8_t* data, uint32_t len )
{
    rndis_data_packet_t *p;

    if ( rndis_recvSize )
        return;

    p = (rndis_data_packet_t *)data;
    if (len < sizeof(rndis_data_packet_t))
        return;

    if (p->MessageType != REMOTE_NDIS_PACKET_MSG || p->MessageLength != len)
        return;

    if (p->DataOffset + sizeof(rndis_generic_msg_t) + p->DataLength != len)
    {
        // usb_eth_stat.rxbad++;
        return;
    }
    // usb_eth_stat.rxok++;
    //DBG_PRINT("saved %d\r\n", p->DataLength);
    memcpy(rndis_received, data + sizeof(rndis_generic_msg_t) + p->DataOffset, p->DataLength);
    rndis_recvSize = p->DataLength;
    eth_device_ready(&rndis_dev);
}

struct pbuf * rndis_dev_rx(rt_device_t dev)
{
    struct pbuf *p;

    if (rndis_recvSize == 0)
    {
        DBG_PRINT("#");
        return NULL;
    }

    p = pbuf_alloc(PBUF_RAW, rndis_recvSize, PBUF_POOL);
    if (p == NULL)
    {
        DBG_PRINT("eth rx nomem\r\n");
        return NULL;
    }

    memcpy(p->payload, rndis_received, rndis_recvSize);
    p->len = rndis_recvSize;
    rndis_recvSize = 0;

    DBG_PRINT("erx%d\r\n", p->len);
    if ( dump_count < 10 )
    {
        if ( ((uint8_t*)p->payload)[12] == 0x08 )
        {
            DBG_DUMPMEM( p->payload, (p->len > 64) ? 64 : p->len );
            dump_count++;
        }
        else
        {
            // DBG_DUMPMEM( p->payload, 24 );
        }
        DBG_PRINT("\r\n");
    }

    return p;
}

rt_err_t rndis_dev_tx(rt_device_t dev, struct pbuf *p)
{
    rt_err_t ret;
    struct pbuf *q;
    uint32_t byteslefttocopy, framelength;
    uint8_t* payload;

    for ( int wait=0; wait<200; wait++ )
    {
        if ( rndis_ep_tx_busy_flag == 0 )
            break;

        rt_thread_delay(1);
    }
    if ( rndis_ep_tx_busy_flag != 0 )
    {
        DBG_PRINT("ETX TOUT\r\n");
        return ERR_TIMEOUT;
    }

    rndis_data_packet_t *hdr = (rndis_data_packet_t *)rndis_tx_buf;
    payload = rndis_tx_buf + sizeof(rndis_data_packet_t);

    framelength = 0;
    /* 从 pbuf 复制数据到驱动中的缓冲区 */
    for (q = p; q != NULL; q = q->next)
    {
        /* 获取当前 LWIP 缓冲区中的字节 */
        byteslefttocopy = q->len;
        if ( (byteslefttocopy+framelength + sizeof(rndis_data_packet_t)) > sizeof(rndis_tx_buf) )
        {
            DBG_PRINT("ETX err LEN\r\n");
            return ERR_USE;
        }
        memcpy( payload, (uint8_t *)(q->payload), byteslefttocopy);
        payload = payload + byteslefttocopy;
        framelength = framelength + byteslefttocopy;
    }

    // rndis数据包头
    memset(hdr, 0, sizeof(rndis_data_packet_t));
    hdr->MessageType = REMOTE_NDIS_PACKET_MSG;
    hdr->MessageLength = sizeof(rndis_data_packet_t) + framelength;
    hdr->DataOffset = sizeof(rndis_data_packet_t) - sizeof(rndis_generic_msg_t);
    hdr->DataLength = framelength;
    DBG_PRINT("ETX%d\r\n", framelength);

    /* 发送数据帧 */
    rndis_ep_tx_busy_flag = true;
    usbd_ep_start_write(RNDIS_IN_EP, rndis_tx_buf, sizeof(rndis_data_packet_t) + framelength);
    ret = ERR_OK;

    return ret;
}

// 本地网卡的MAC(给Lwip的)
static const uint8_t rndis_dev_mac[6] = {0x20, 0x89, 0x84, 0x6A, 0x96, 0x0B };

rt_err_t rndis_dev_ioc(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        memcpy( args, rndis_dev_mac, 6 );
        return ERR_OK;

    default:
        DBG_PRINT("Unknown ioc %d\r\n", cmd);
    }
    return ERR_USE;
}

#include "dhserver.h"
#define NUM_DHCP_ENTRY 3

static dhcp_entry_t entries[NUM_DHCP_ENTRY] =
{
    /* mac    ip address        subnet mask        lease time */
    { {0}, {192, 168, 17, 2}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 17, 3}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 17, 4}, {255, 255, 255, 0}, 24 * 60 * 60 }
};

static dhcp_config_t dhcp_config =
{
    {192, 168, 17, 1}, 67, /* server address, port */
    {192, 168, 17, 1},     /* dns server */
    "stm",                /* dns suffix */
    NUM_DHCP_ENTRY,       /* num entry */
    entries               /* entries */
};


static void RndisNetInit(void)
{
    rndis_dev.parent.control = rndis_dev_ioc;
    rndis_dev.eth_rx = rndis_dev_rx;
    rndis_dev.eth_tx = rndis_dev_tx;
    eth_device_init(&rndis_dev, "rndis");
    eth_device_linkchange( &rndis_dev, RT_FALSE );
    eth_device_ready( &rndis_dev );

    if ( ERR_OK != dhserv_init(&dhcp_config) )
        DBG_PRINT("DHCP server start fail\r\n");

}
#endif

/* function ------------------------------------------------------------------*/
void usbd_configure_done_callback(void)
{
#ifdef ENABLE_RNDIS
    eth_device_linkchange( &rndis_dev, RT_TRUE );
    eth_device_ready( &rndis_dev );

    /* setup first out ep read transfer */
    usbd_ep_start_read(RNDIS_OUT_EP, rndis_rx_buf, sizeof(rndis_rx_buf));
#endif

#ifdef ENABLE_CDC
    usbd_ep_start_read(CDC_OUT_EP, cdc_rx_buffer, sizeof(cdc_rx_buffer));
#endif
}

void rndis_cdc_init(void)
{
    usbd_desc_register(cdc_descriptor);
#ifdef ENABLE_RNDIS
    usbd_add_interface(usbd_rndis_alloc_intf(RNDIS_INT_EP));
    usbd_add_interface(usbd_rndis_alloc_intf(RNDIS_INT_EP));
    usbd_add_endpoint(&rndis_out_ep);
    usbd_add_endpoint(&rndis_in_ep);
#endif

#ifdef ENABLE_CDC
    usbd_add_interface(usbd_cdc_acm_alloc_intf());
    usbd_add_interface(usbd_cdc_acm_alloc_intf());
    usbd_add_endpoint(&cdc_out_ep);
    usbd_add_endpoint(&cdc_in_ep);

#endif
    usbd_initialize();
}

int cherryusb_test(void)
{
#ifdef ENABLE_RNDIS
    RndisNetInit();
#endif

    rndis_cdc_init();

#ifdef ENABLE_CDC
    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("vcomtest", vcom_thread_entry, RT_NULL, 2048, 16, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }
#endif
    return RT_EOK;
}
INIT_APP_EXPORT(cherryusb_test);

//#include "usb_otg.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
void usb_dc_low_level_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN USB_OTG_HS_MspInit 0 */

    /* USER CODE END USB_OTG_HS_MspInit 0 */

      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**USB_OTG_HS GPIO Configuration
      PB14     ------> USB_OTG_HS_DM
      PB15     ------> USB_OTG_HS_DP
      */
      GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* USB_OTG_HS clock enable */
      __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

      /* USB_OTG_HS interrupt Init */
      HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    /* USER CODE BEGIN USB_OTG_HS_MspInit 1 */

    /* USER CODE END USB_OTG_HS_MspInit 1 */

}
