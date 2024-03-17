#include "usb_desc.h"
#include "ch32v20x_usbfs_device.h"

/* Standard Device Descriptor */
const uint8_t  MyDevDescr[] =
{
    0x12,                       // bLength
    USB_DESCR_TYP_DEVICE,       // bDescriptorType (Device)
    0x00, 0x02,                 // bcdUSB 2.0
    USB_DEV_CLASS_RESERVED,     // bDeviceClass: 0x00: Use class information in the Interface Descriptors 
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,         // bMaxPacketSize0 64
    (uint8_t)DEF_USB_VID, (uint8_t)(DEF_USB_VID >> 8),  // idVendor 0x1A86
    (uint8_t)DEF_USB_PID, (uint8_t)(DEF_USB_PID >> 8),  // idProduct 0x5537
    DEF_IC_PRG_VER, 0x00,       // bcdDevice 0.01 (Device Release Code)
    DEF_STRING_DESC_MANU,       // iManufacturer (String Index)
    DEF_STRING_DESC_PROD,       // iProduct (String Index)
    DEF_STRING_DESC_SERN,       // iSerialNumber (String Index)
    0x01,                       // bNumConfigurations 1
};

/* Standard Configuration Descriptor */
const uint8_t  MyCfgDescr[] =
{
    /* Configure descriptor */
    0x09,                       /* bLength */
    USB_DESCR_TYP_CONFIG,       /* bDescriptorType */
    0xB7, 0x00,                 /* wTotalLength */
    0x04,                       /* bNumInterfaces */
    0x01,                       /* bConfigurationValue */
    0x00,                       /* iConfiguration */
    0x80,                       /* bmAttributes: Bus Powered according to user configuration */
    DEF_USBD_MAX_POWER,         /* MaxPower (mA) */

    /// CDC VIRTUAL SERIAL PORT
    // https://cscott.net/usb_dev/data/devclass/usbcdc11.pdf

    /* IAD Descriptor(interface 0/1) */
    0x08,                       /*bLength*/
    USB_DESC_TYPE_IAD,          /*bDescriptorType*/ 
    0x00,                       /*bFirstInterface*/
    0x02,                       /*bInterfaceCount*/
    USB_DEV_CLASS_COMMUNIC,     /*bFunctionClass*/
    0x02,                       /*bFunctionSubClass: 0x02: Abstract Control Model (ACM) */
    0x00,                       /*bFunctionProtocol: 0x00: No class specific protocol required */
    0x00,                       /*iFunction: 0x00: No string descriptor */

    /* Interface 0 (CDC Class communication interface) descriptor */
    0x09,                       /* bLength */
    USB_DESCR_TYP_INTERF,       /* bDescriptorType */
    0x00,                       /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x01,                       /* bNumEndpoints */
    USB_DEV_CLASS_COMMUNIC,     /* bInterfaceClass */
    0x02,                       /* bInterfaceSubClass: 0x02: Abstract Control Model (ACM) */
    0x00,                       /* bInterfaceProtocol: 0x00: No class specific protocol required */
    0x00,                       /* iInterface: 0x00: No string descriptor */

    /* Header Functional Descriptor */
    0x05,                       /* bFunctionLength */
    USB_DESCR_TYP_CS_INTF,      /* bDescriptorType */
    0x00,                       /* bDescriptorSubtype: 0x00: Header Functional Descriptor */
    0x00, 0x02,                 /* bcdCDC */

    /* Call management descriptor (data class interface 1) */
    0x05,                       /* bFunctionLength */
    USB_DESCR_TYP_CS_INTF,      /* bDescriptorType */
    0x01,                       /* bDescriptorSubtype: 0x01: Call Management Functional Descriptor. */
    0x00,                       /* bmCapabilities */
    0x01,                       /* bDataInterface */

    /* Abstract Control Management Functional Descriptor */
    0x04,                       /* bFunctionLength */
    USB_DESCR_TYP_CS_INTF,      /* bDescriptorType */
    0x02,                       /* bDescriptorSubtype: 0x02: Abstract Control Management Functional Descriptor */
    0x00,                       /* bmCapabilities */ // TODO: kiszedni az implementációból a nem szükséges részek* kezelését. Eredetileg: 0x02 (*lásd pdf)

    /* Union Functional descriptor */
    0x05,                       /* bFunctionLength */
    USB_DESCR_TYP_CS_INTF,      /* bDescriptorType */
    0x06,                       /* bDescriptorSubtype: 0x06: Union Functional descriptor */
    0x00,                       /* bMasterInterface */
    0x01,                       /* bSlaveInterface0 */

    /* Interrupt upload endpoint descriptor */
    0x07,                           /* bLength */
    USB_DESCR_TYP_ENDP,             /* bDescriptorType */
    DEF_UEP_IN | DEF_UEP1,          /* bEndpointAddress */
    USB_EP_TYPE_INTR,               /* bmAttributes */
    (uint8_t)DEF_USBD_ENDP1_SIZE,   /* wMaxPacketSize */
    (uint8_t)( DEF_USBD_ENDP1_SIZE >> 8 ),
    0x01,                           /* bInterval */

    /* Interface 1 (CDC Class Data interface) descriptor */
    0x09,                       /* bLength */
    USB_DESCR_TYP_INTERF,       /* bDescriptorType */
    0x01,                       /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints */
    0x0A,                       /* bInterfaceClass: 0x0A: CDC Class Data Interface */
    0x00,                       /* bInterfaceSubClass: unused field, should be 0x00 */
    0x00,                       /* bInterfaceProtocol: 0x00: No class specific protocol required */
    0x00,                       /* iInterface: 0x00: No string descriptor */

    /* Endpoint descriptor */
    0x07,                           /* bLength */
    USB_DESCR_TYP_ENDP,             /* bDescriptorType */
    DEF_UEP_OUT | DEF_UEP2,         /* bEndpointAddress */
    USB_EP_TYPE_BULK,               /* bmAttributes */
    (uint8_t)DEF_USBD_ENDP2_SIZE,   /* wMaxPacketSize */
    (uint8_t)(DEF_USBD_ENDP2_SIZE >> 8),
    0x00,                           /* bInterval */

    /* Endpoint descriptor */
    0x07,                           /* bLength */
    USB_DESCR_TYP_ENDP,             /* bDescriptorType */
    DEF_UEP_IN | DEF_UEP2,          /* bEndpointAddress */
    USB_EP_TYPE_BULK,               /* bmAttributes */
    (uint8_t)DEF_USBD_ENDP2_SIZE,   /* wMaxPacketSize */
    (uint8_t)(DEF_USBD_ENDP2_SIZE >> 8),
    0x00,                           /* bInterval */

    /// AUDIO DEVICE
    // https://www.usb.org/sites/default/files/audio10.pdf

    /* IAD Descriptor(interface 2/3) */
    0x08,                       /*bLength*/
    USB_DESC_TYPE_IAD,          /*bDescriptorType*/ 
    0x02,                       /*bFirstInterface*/
    0x02,                       /*bInterfaceCount*/
    USB_DEV_CLASS_AUDIO,        /*bFunctionClass*/
    0x00,                       /*bFunctionSubClass: 0x00: Undefined */
    0x00,                       /*bFunctionProtocol: 0x00: Not used */
    0x00,                       /*iFunction: 0x00: No string descriptor */

    /* Interface 2, Alternate Setting 0 (USB audio Standard interface) descriptor */
    0x09,                       /* bLength */
    USB_DESCR_TYP_INTERF,       /* bDescriptorType */
    0x02,                       /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x00,                       /* bNumEndpoints */
    USB_DEV_CLASS_AUDIO,        /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOCONTROL,/* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,   /* bInterfaceProtocol */
    0x00,                       /* iInterface */

    /* USB audio Class-specific AC Interface Descriptor */
    (8+1),                      /* bLength */
    USB_DESCR_TYP_CS_INTF,      /* bDescriptorType */
    AUDIO_CONTROL_HEADER,       /* bDescriptorSubtype */
    0x00, 0x01,                 /* bcdADC:1.00 */
    (30+8+1), 0x00,             /* wTotalLength */
    0x01,                       /* bInCollection mic*/
    0x03,                       /* baInterfaceNr1 mic */

    /* USB mic Input Terminal Descriptor */
    0x0C,                               /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,       /* bDescriptorSubtype */
    0x04,                               /* bTerminalID: 4 */
    0x01, 0x01,                         /* wTerminalType 0x0101: USB Streaming*/
    0x00,                               /* bAssocTerminal */
    0x01,                               /* bNrChannels */
    0x00, 0x00,                         /* wChannelConfig 0x0000  Mono */
    0x00,                               /* iChannelNames */
    0x00,                               /* iTerminal */

    /* USB mic Audio Feature Unit Descriptor */
    0x09,                               /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
    AUDIO_CONTROL_FEATURE_UNIT,         /* bDescriptorSubtype */
    0x05,                               /* bUnitID :5*/
    0x04,                               /* bSourceID :4*/
    0x01,                               /* bControlSize */
    AUDIO_CONTROL_MUTE,                 /* bmaControls(0) */
    0x00,                               /* bmaControls(1) */
    0x00,                               /* iTerminal */

    /* USB mic Output Terminal Descriptor */
    0x09,                               /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,      /* bDescriptorSubtype */
    0x06,                               /* bTerminalID:6 */
    0x01, 0x01,                         /* wTerminalType: 0x0101: USB Streaming*/
    0x00,                               /* bAssocTerminal */
    0x05,                               /* bSourceID:5 */
    0x00,                               /* iTerminal */

    /* USB mic Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
    /* Interface 3, Alternate Setting 0                                              */
    0x09,                               /* bLength */
    USB_DESCR_TYP_INTERF,               /* bDescriptorType */
    0x03,                               /* bInterfaceNumber: 3 */
    0x00,                               /* bAlternateSetting */
    0x00,                               /* bNumEndpoints */
    USB_DEV_CLASS_AUDIO,                /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,           /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* USB mic Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface 3, Alternate Setting 1                                           */
    0x09,                               /* bLength */
    USB_DESCR_TYP_INTERF,               /* bDescriptorType */
    0x03,                               /* bInterfaceNumber: 3 */
    0x01,                               /* bAlternateSetting */
    0x01,                               /* bNumEndpoints */
    USB_DEV_CLASS_AUDIO,                /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,           /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* USB mic Audio Streaming Interface Descriptor */
    0x07,                               /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,            /* bDescriptorSubtype */
    0x06,                               /* bTerminalLink: 6 mic */
    0x01,                               /* bDelay */
    0x01, 0x00,                         /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */

    /* USB Audio Type I Format Interface Descriptor */
    0x0B,                               /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,        /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,                /* bFormatType */
    0x02,                               /* bNrChannels */
    0x02,                               /* bSubFrameSize: 2 Bytes per frame (16bits) */
    16,                                 /* bBitResolution (16-bits per sample) */
    0x01,                               /* bSamFreqType only one frequency supported */
    AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */

    /* Endpoint 3 - Standard Descriptor */
    0x09,                               /* bLength */
    USB_DESCR_TYP_ENDP,                 /* bDescriptorType */
    DEF_UEP_IN | DEF_UEP3,              /* bEndpointAddress 3 in endpoint */
    USB_EP_TYPE_ISOC,                   /* bmAttributes */
    AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),  /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01,                               /* bInterval */
    0x00,                               /* bRefresh */
    0x00,                               /* bSynchAddress */

    /* Endpoint 3 - Audio Streaming Descriptor */
    0x07,                               /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,             /* bDescriptor */
    0x00,                               /* bmAttributes */
    0x00,                               /* bLockDelayUnits */
    0x00,                               /* wLockDelay */
    0x00
};

/* Language Descriptor */
const uint8_t  MyLangDescr[] =
{
    0x04, USB_DESCR_TYP_STRING,
    0x09, 0x04
};

/* Manufactor Descriptor */
const uint8_t  MyManuInfo[] =
{
    /* HA3PB */
    0x0C, USB_DESCR_TYP_STRING,
    'H', 0, 'A', 0, '3', 0, 'P', 0, 'B', 0
};

/* Product Information */
const uint8_t  MyProdInfo[] =
{
    /* ZetaSDR */
    0x10, USB_DESCR_TYP_STRING,
    'Z', 0, 'e', 0, 't', 0, 'a', 0, 'S', 0, 'D', 0, 'R', 0
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[ ] =
{
    /* 0123456789 */
    22, USB_DESCR_TYP_STRING,
    '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0, '9', 0
};

