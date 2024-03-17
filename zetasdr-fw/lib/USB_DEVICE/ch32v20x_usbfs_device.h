#ifndef __CH32V20X_USBFS_DEVICE_H_
#define __CH32V20X_USBFS_DEVICE_H_

#include <ch32v20x.h>
#include <core_riscv.h>
#include "usb_desc.h"
#include "ch32v20x_usb.h"

/******************************************************************************/
/* Global Define */

/* end-point number */
#define DEF_UEP_IN                      0x80
#define DEF_UEP_OUT                     0x00
#define DEF_UEP0                        0x00
#define DEF_UEP1                        0x01
#define DEF_UEP2                        0x02
#define DEF_UEP3                        0x03
#define DEF_UEP_NUM                     4

/* Setup Request Packets */
#define pUSBFS_SetupReqPak              ((PUSB_SETUP_REQ)USBFS_EP0_Buf)

/*******************************************************************************/
/* Variable Definition */
/* Global */
extern const uint8_t *pUSBFS_Descr;

/* Setup Request */
extern volatile uint8_t  USBFS_SetupReqCode;
extern volatile uint8_t  USBFS_SetupReqType;
extern volatile uint16_t USBFS_SetupReqValue;
extern volatile uint16_t USBFS_SetupReqIndex;
extern volatile uint16_t USBFS_SetupReqLen;

/* USB Device Status */
extern volatile uint8_t  USBFS_DevConfig;
extern volatile uint8_t  USBFS_DevAddr;
extern volatile uint8_t  USBFS_DevSleepStatus;
extern volatile uint8_t  USBFS_DevEnumStatus;

/* Endpoint Buffer */
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP0_Buf[];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP1_Buf[];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP2_Buf[];
extern __attribute__ ((aligned(4))) uint8_t USBFS_EP3_Buf[];

#define USBFSD_UEP_DMA_BASE     0x50000010
#define USBFSD_UEP_BUF(n)       ((uint8_t *)(*((volatile uint32_t *)(USBFSD_UEP_DMA_BASE+n*0x04)))+0x20000000)

/* AUDIO related definitions and macros */
#define USBD_AUDIO_FREQ         192000U

#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq) \
  (uint8_t)(((frq * 2U * 2U) / 1000U) & 0xFFU), (uint8_t)((((frq * 2U * 2U) / 1000U) >> 8) & 0xFFU)

/* Audio samples */
extern __attribute__ ((aligned(4))) volatile uint8_t buf_a[DEF_USBD_ENDP3_SIZE];
extern __attribute__ ((aligned(4))) volatile uint8_t buf_b[DEF_USBD_ENDP3_SIZE];
extern volatile uint8_t *streaming_buffer, *recording_buffer;
extern volatile uint32_t sample_index;

/* USB IN Endpoint Busy Flag */
extern volatile uint8_t USBFS_Endp_Busy[];

/* UART RX buffer */
extern volatile int32_t UART_VFO_target;
extern volatile FlagStatus UARTPacketReceived;

/******************************************************************************/
/* external functions */
extern void USBFS_RCC_Init(void);
extern void USBFS_Device_Endp_Init(void);
extern void USBFS_Device_Init(FunctionalState sta);

extern void TransmitAUDIOPacket(void);
extern void HandleUARTpacket(uint32_t len);

#endif