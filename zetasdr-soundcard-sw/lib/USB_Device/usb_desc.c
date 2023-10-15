#include "usb_desc.h"

/******************************************************************************/
/* Device Descriptor */
const uint8_t  MyDevDescrHD[] =
{
    0x12, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, DEF_USBD_UEP0_SIZE,
    0x86, 0x1A, 0xE1, 0xE6,
    0x00, 0x01, 0x01, 0x02, 0x00, 0x01,
};

/* USB����������(ȫ��) */
uint8_t  MyCfgDescr_FS[ ] =
{
#if 1
        /* Configuration 1 */
         0x09,                                 /* bLength */
         USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
         LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength */
         HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
         0x02,                                 /* bNumInterfaces  audio+mic AS*/
         0x01,                                 /* bConfigurationValue */
         0x00,                                 /* iConfiguration */
         0x80,                                 /* bmAttributes: Bus Powered according to user configuration */
         USBD_MAX_POWER,                       /* MaxPower (mA) */
         /* 09 byte*/
#endif
         /*19Byte audio����*/
#if 1
         /* USB audio Standard interface descriptor */
         AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
         USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
         0x00,                                 /* bInterfaceNumber */
         0x00,                                 /* bAlternateSetting */
         0x00,                                 /* bNumEndpoints */
         USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
         AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
         AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
         0x00,                                 /* iInterface */
         /* 09 byte*/

         /* USB audio Class-specific AC Interface Descriptor */
         (8+1),            /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
         0x00,          /* 1.00 */             /* bcdADC */
         0x01,
         (30+8+1),                          /* wTotalLength */
         0x00,
         0x01,                                 /* bInCollection mic*/
         0x01,                                 /* baInterfaceNr1 mic */
         /* 9 byte*/
#endif

         /*30 Byte mic�ն�*/
#if 1

         /* USB mic Input Terminal Descriptor */
         AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
         0x04,                                 /* bTerminalID : 4*/
         0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0201 ������mic*/
         0x02,
         0x00,                                 /* bAssocTerminal */
         0x01,                                 /* bNrChannels */
         0x00,                                 /* wChannelConfig 0x0000  Mono */
         0x00,
         0x00,                                 /* iChannelNames */
         0x00,                                 /* iTerminal */
         /* 12 byte*/

         /* USB mic Audio Feature Unit Descriptor */
         0x09,                                 /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
         0x05,                                 /* bUnitID :5*/
         0x04,                                 /* bSourceID :4*/
         0x01,                                 /* bControlSize */
         AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
         0,                                    /* bmaControls(1) */
         0x00,                                 /* iTerminal */
         /* 09 byte */

         /* USB mic Output Terminal Descriptor */
         0x09,      /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
         0x06,                                 /* bTerminalID:6 */
         0x01,                                 /* wTerminalType  0x0101  �������Ƶ��*/
         0x01,
         0x00,                                 /* bAssocTerminal */
         0x05,                                 /* bSourceID:5 */
         0x00,                                 /* iTerminal */
         /* 09 byte */
#endif

         /*52 Byte mic ��Ƶ��*/
#if 1
         /* USB mic Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
         /* Interface 1, Alternate Setting 0                                              */
         AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
         USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
         0x01,                                 /* bInterfaceNumber :2*/
         0x00,                                 /* bAlternateSetting */
         0x00,                                 /* bNumEndpoints */
         USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
         AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
         AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
         0x00,                                 /* iInterface */
         /* 09 byte*/

         /* USB mic Standard AS Interface Descriptor - Audio Streaming Operational */
         /* Interface 1, Alternate Setting 1                                           */
         AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
         USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
         0x01,                                 /* bInterfaceNumber :2*/
         0x01,                                 /* bAlternateSetting */
         0x01,                                 /* bNumEndpoints */
         USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
         AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
         AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
         0x00,                                 /* iInterface */
         /* 09 byte*/

         /* USB mic Audio Streaming Interface Descriptor */
         AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
         0x06,                                 /* bTerminalLink:6 mic����ն�*/
         0x01,                                 /* bDelay */
         0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */
         0x00,
         /* 07 byte*/

         /* USB Speaker Audio Type III Format Interface Descriptor */
         0x0B,                                 /* bLength */
         AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
         AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
         AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
         0x02,                                 /* bNrChannels */
         0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
         16,                                   /* bBitResolution (16-bits per sample) */
         0x01,                                 /* bSamFreqType only one frequency supported */
         AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */
         /* 11 byte*/

         /* Endpoint 1 - Standard Descriptor */
         AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
         USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
         0x83,                                 /* bEndpointAddress 1 in endpoint */
         USBD_EP_TYPE_ISOC,                    /* bmAttributes */
         AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
         AUDIO_FS_BINTERVAL,                   /* bInterval */
         0x00,                                 /* bRefresh */
         0x00,                                 /* bSynchAddress */
         /* 09 byte*/

         /* Endpoint - Audio Streaming Descriptor */
         AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
         AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
         AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
         0x00,                                 /* bmAttributes */
         0x00,                                 /* bLockDelayUnits */
         0x00,                                 /* wLockDelay */
         0x00,

#endif

};

#if 1
/* Language Descriptor */
const uint8_t  MyLangDescrHD[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufactor Descriptor */
const uint8_t  MyManuInfoHD[] =
{
    0x0C, 0x03, 'H', 0, 'A', 0, '3', 0, 'P', 0, 'B', 0
};

/* Product Information */
const uint8_t  MyProdInfoHD[] =
{
    0x10, 0x03, 'Z', 0, 'e', 0, 't', 0, 'a', 0, 'S', 0, 'D', 0, 'R', 0
};

/* USB���к��ַ��������� */
const uint8_t  MySerNumInfoHD[ ] =
{
    /* 0123456789 */
    22,03,48,0,49,0,50,0,51,0,52,0,53,0,54,0,55,0,56,0,57,0
};

/* USB�豸�޶������� */
const uint8_t MyUSBQUADescHD[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00
};
#endif