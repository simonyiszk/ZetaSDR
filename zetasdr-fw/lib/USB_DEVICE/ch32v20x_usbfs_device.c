#include <string.h>
#include <stdlib.h>
#include <debug.h>
#include "usb_desc.h"
#include "ch32v20x_usbfs_device.h"

/*******************************************************************************/
/* Variable Definition */
/* Global */

const uint8_t       *pUSBFS_Descr;

/* Setup Request */
volatile uint8_t    USBFS_SetupReqCode;
volatile uint8_t    USBFS_SetupReqType;
volatile uint16_t   USBFS_SetupReqValue;
volatile uint16_t   USBFS_SetupReqIndex;
volatile uint16_t   USBFS_SetupReqLen;

/* USB Device Status */
volatile uint8_t    USBFS_DevConfig;
volatile uint8_t    USBFS_DevAddr;
volatile uint8_t    USBFS_DevSleepStatus;
volatile uint8_t    USBFS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBFS_EP0_Buf[DEF_USBD_UEP0_SIZE];
__attribute__ ((aligned(4))) uint8_t USBFS_EP1_Buf[DEF_USBD_ENDP1_SIZE];
__attribute__ ((aligned(4))) uint8_t USBFS_EP2_Buf[DEF_USBD_ENDP2_SIZE];
__attribute__ ((aligned(4))) uint8_t USBFS_EP3_Buf[DEF_USBD_ENDP3_SIZE];

/* USB IN Endpoint Busy Flag */
volatile uint8_t    USBFS_Endp_Busy[DEF_UEP_NUM];

/* UART RX buffer */
volatile int32_t UART_VFO_target = 0;
volatile FlagStatus UARTPacketReceived = RESET;

/* Audio samples */
__attribute__ ((aligned(4))) volatile uint8_t buf_a[DEF_USBD_ENDP3_SIZE];
__attribute__ ((aligned(4))) volatile uint8_t buf_b[DEF_USBD_ENDP3_SIZE];
volatile uint8_t *streaming_buffer = buf_a, *recording_buffer = buf_b;

/* Interrupt Service Routine Declaration*/
void USBHD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USBFS_RCC_Init
 * @brief   Initializes the usbotg clock configuration.
 * @return  none
 */
void USBFS_RCC_Init(void)
{
    if(SystemCoreClock == 144000000){
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    }else if(SystemCoreClock == 96000000){
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
    }else if(SystemCoreClock == 48000000){
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
    }
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE);
}

/*********************************************************************
 * @fn      USBFS_Device_Endp_Init
 * @brief   Initializes USB device endpoints.
 * @return  none
 */
void USBFS_Device_Endp_Init(void)
{
    USBOTG_FS->UEP4_1_MOD = USBFS_UEP1_TX_EN;
    USBOTG_FS->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN | USBFS_UEP3_TX_EN | USBFS_UEP3_BUF_MOD,

    USBOTG_FS->UEP0_DMA = (uint32_t)USBFS_EP0_Buf;
    USBOTG_FS->UEP1_DMA = (uint32_t)USBFS_EP1_Buf;
    USBOTG_FS->UEP2_DMA = (uint32_t)USBFS_EP2_Buf;
    USBOTG_FS->UEP3_DMA = (uint32_t)USBFS_EP3_Buf;

    USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;

    USBOTG_FS->UEP1_TX_LEN = 0;
    USBOTG_FS->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;

    USBOTG_FS->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;
    
    USBOTG_FS->UEP2_TX_LEN = DEF_USBD_ENDP2_SIZE;
    USBOTG_FS->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;

    USBOTG_FS->UEP3_TX_LEN = DEF_USBD_ENDP3_SIZE;
    USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NONE;

    /* Clear End-points Busy Status */
    for(uint8_t i=0; i<DEF_UEP_NUM; i++){
        USBFS_Endp_Busy[i] = 0;
    }

    for(uint16_t i=0; i<DEF_USBD_ENDP3_SIZE; i++){
        buf_a[i] = 0x5A;
        buf_b[i] = 0xA5;
    }
}

inline void TransmitAUDIOPacket(void){

    USBFS_Endp_Busy[DEF_UEP3] = 1;
    static int b = Bit_RESET;

    if(b==Bit_SET){
        b=Bit_RESET;
    }else{
        b=Bit_SET;
    }

    sample_index = 0;

    if(recording_buffer == buf_a){
        recording_buffer = buf_b;
        streaming_buffer = buf_a;
    }else{
        recording_buffer = buf_a;
        streaming_buffer = buf_b;
    }

    USBOTG_FS->UEP3_DMA = (uint32_t)streaming_buffer;

    USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NONE;
    USBOTG_FS->UEP3_TX_LEN  = DEF_USBD_ENDP3_SIZE;

    TIM_SetCounter(TIM1, 200);
    TIM_Cmd(TIM1, ENABLE);
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, b);

    /*for(uint32_t i=0; i<DEF_USBD_ENDP3_SIZE; i++){
        recording_buffer[i] = 0xA5;
    }*/

    USBFS_Endp_Busy[DEF_UEP3] = 0;
}

inline void HandleUARTpacket(uint32_t len){
    USBFS_Endp_Busy[DEF_UEP2] = 1;

    UART_VFO_target = 0;

    for(uint8_t i=0; i<DEF_USBD_ENDP2_SIZE; i++){
        if(i<len){
            UART_VFO_target = UART_VFO_target * 10 + (USBFS_EP2_Buf[i] - '0');
            //(USBFSD_UEP_BUF(DEF_UEP2) + 64)[i] = USBFS_EP2_Buf[i];
            (USBFSD_UEP_BUF(DEF_UEP2) + 64)[i] = 0;
        }else{
            (USBFSD_UEP_BUF(DEF_UEP2) + 64)[i] = 0;
        }
    }

    itoa(UART_VFO_target, (char*)(USBFSD_UEP_BUF(DEF_UEP2) + 64), 10);

    USBOTG_FS->UEP2_DMA = (uint32_t)USBFS_EP2_Buf; 
    USBOTG_FS->UEP2_TX_LEN = len;
    USBOTG_FS->UEP2_TX_CTRL &= (~USBFS_UEP_T_RES_MASK);
    USBOTG_FS->UEP2_TX_CTRL |= USBFS_UEP_T_RES_ACK;

    UARTPacketReceived = SET;

    USBFS_Endp_Busy[DEF_UEP2] = 0;
}

/*********************************************************************
 * @fn      USBFS_Device_Init
 * @brief   Initializes USB device.
 * @return  none
 */
void USBFS_Device_Init(FunctionalState sta)
{
    Delay_Init();
    if(sta){
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us(10);
        USBOTG_H_FS->BASE_CTRL = 0x00;
        USBOTG_FS->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
        USBOTG_FS->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBFS_Device_Endp_Init();
        USBOTG_FS->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
        EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
        NVIC_EnableIRQ(USBHD_IRQn);
    }else{
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us(10);
        USBOTG_FS->BASE_CTRL = 0x00;
        NVIC_DisableIRQ(USBHD_IRQn);
    }
}

/*********************************************************************
 * @fn      USBHD_IRQHandler
 * @brief   This function handles HD-FS exception.
 * @return  none
 */
void USBHD_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBOTG_FS->INT_FG;
    intst   = USBOTG_FS->INT_ST;

    if(intflag & USBFS_UIF_TRANSFER){
        switch(intst & USBFS_UIS_TOKEN_MASK)
        {
            /* data-in stage processing */
            case USBFS_UIS_TOKEN_IN:
                switch(intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK))
                {
                    /* end-point 0 data in interrupt */
                    case(USBFS_UIS_TOKEN_IN | DEF_UEP0):
                        if(USBFS_SetupReqLen == 0){
                            USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                        }
                        if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD){
                            /* Non-standard request endpoint 0 Data upload */
                        }else{
                            /* Standard request endpoint 0 Data upload */
                            switch(USBFS_SetupReqCode){
                                case USB_GET_DESCRIPTOR:
                                        len = USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                                        memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                                        USBFS_SetupReqLen -= len;
                                        pUSBFS_Descr += len;
                                        USBOTG_FS->UEP0_TX_LEN   = len;
                                        USBOTG_FS->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
                                        break;
                                case USB_SET_ADDRESS:
                                        USBOTG_FS->DEV_ADDR = (USBOTG_FS->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;
                                        USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                        USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
                                        break;
                                default:
                                        USBOTG_FS->UEP0_TX_LEN = 0;
                                        USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                        USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
                                        break;
                            }
                        }
                        break;
                    
                    /* end-point 1 data in interrupt */
                    case(USBFS_UIS_TOKEN_IN | DEF_UEP1):
                        USBOTG_FS->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
                        USBOTG_FS->UEP1_TX_CTRL = (USBOTG_FS->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        USBFS_Endp_Busy[DEF_UEP1] = 0;
                        break;

                    /* end-point 2 data in interrupt */
                    case(USBFS_UIS_TOKEN_IN | DEF_UEP2):
                        USBOTG_FS->UEP2_TX_CTRL ^= USBFS_UEP_T_TOG;
                        USBOTG_FS->UEP2_TX_CTRL = (USBOTG_FS->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        USBFS_Endp_Busy[DEF_UEP2] = 0;
                        break;

                    /* end-point 3 data in interrupt */
                    case(USBFS_UIS_TOKEN_IN | DEF_UEP3):

                        TransmitAUDIOPacket();
                        break;

                    default:
                        break;
                }
                break;

            /* data-out stage processing */
            case USBFS_UIS_TOKEN_OUT:
                switch(intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK))
                {
                    /* end-point 0 data out interrupt */
                    case(USBFS_UIS_TOKEN_OUT | DEF_UEP0):
                        len = USBOTG_FS->RX_LEN;
                         if(intst & USBFS_UIS_TOG_OK){
                            if((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD){
                                 /* Non-standard request end-point 0 Data download */
                                USBFS_SetupReqLen = 0;
                                if(USBFS_SetupReqCode == CDC_SET_LINE_CODING){
                                    // not implemented, beacuse no uart hardware in use
                                }
                            }else{
                                    /* Standard request end-point 0 Data download */
                                    /* Add your code here */
                            }
                            if(USBFS_SetupReqLen == 0){
                                USBOTG_FS->UEP0_TX_LEN  = 0;
                                USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                            }
                        }
                        break;
                    
                    /* end-point 2 data out interrupt */
                    case(USBFS_UIS_TOKEN_OUT | DEF_UEP2):

                        HandleUARTpacket(USBOTG_FS->RX_LEN);
                        break;

                    default:
                        break;
                }
                break;
            
            /* Setup stage processing */
            case USBFS_UIS_TOKEN_SETUP:
                USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK;
                USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK;
                /* Store All Setup Values */
                USBFS_SetupReqType  = pUSBFS_SetupReqPak->bRequestType;
                USBFS_SetupReqCode  = pUSBFS_SetupReqPak->bRequest;
                USBFS_SetupReqLen   = pUSBFS_SetupReqPak->wLength;
                USBFS_SetupReqValue = pUSBFS_SetupReqPak->wValue;
                USBFS_SetupReqIndex = pUSBFS_SetupReqPak->wIndex;
                len = 0;
                errflag = 0;

                if((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD){
                    /* usb non-standard request processing */
                    if(USBFS_SetupReqType & USB_REQ_TYP_CLASS){
                        /* Class requests */
                        switch(USBFS_SetupReqCode){
                            case CDC_GET_LINE_CODING:
                                break;
                            case CDC_SET_LINE_CODING:
                                break;
                            case CDC_SET_LINE_CTLSTE:
                                break;
                            case CDC_SEND_BREAK:
                                break;
                            case AUDIO_REQ_GET_CUR:
                                /* Feature Unit */
                                if((USBFS_SetupReqIndex == 0x200) || (USBFS_SetupReqIndex == 0x500))
                                {
                                    if(USBFS_SetupReqValue==0x100){ //mute
                                        uint8_t class_request[1] = {0x00};
                                        pUSBFS_Descr = class_request;
                                        len = 1;
                                    }
                                    if(USBFS_SetupReqValue==0x200){ //mute
                                        uint8_t class_request[2] = {0x00, 0xF9};
                                        pUSBFS_Descr = class_request;
                                        len = 2;
                                    }
                                }
                                break;
                            case AUDIO_REQ_GET_MIN:
                                break;
                            case AUDIO_REQ_GET_MAX:
                                break;
                            case AUDIO_REQ_GET_RES:
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    }else if(USBFS_SetupReqType & USB_REQ_TYP_VENDOR){
                        /* Manufacturer request */
                    }else{
                        errflag = 0xFF;
                    }
                    /* Copy Descriptors to Endp0 DMA buffer */
                    len = (USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                    memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                    pUSBFS_Descr += len;
                }else{
                    /* usb standard request processing */
                    switch(USBFS_SetupReqCode){
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            switch((uint8_t)(USBFS_SetupReqValue >> 8))
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBFS_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBFS_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch((uint8_t)(USBFS_SetupReqValue & 0xFF))
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBFS_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBFS_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBFS_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBFS_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                default :
                                    errflag = 0xFF;
                                    break;
                            }
                            /* Copy Descriptors to Endp0 DMA buffer */
                            if(USBFS_SetupReqLen>len){
                                USBFS_SetupReqLen = len;
                            }
                            len = (USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                            memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                            pUSBFS_Descr += len;
                            break;
                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            USBFS_DevAddr = (uint8_t)(USBFS_SetupReqValue & 0xFF); // audio-sban: SetupReqLen = (pSetupReqPakHD->wValue)&0xff;
                            break;
                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            USBFS_EP0_Buf[0] = USBFS_DevConfig;
                            if(USBFS_SetupReqLen > 1){
                                USBFS_SetupReqLen = 1;
                            }
                            break;
                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            USBFS_DevConfig = (uint8_t)(USBFS_SetupReqValue & 0xFF);
                            USBFS_DevEnumStatus = 0x01;
                            break;
                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE){
                                /* Clear one device feature */
                                if((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP){
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBFS_DevSleepStatus &= ~0x01;
                                }
                            }else if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP){
                                /* Clear End-point Feature */
                                if((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT){
                                    switch((uint8_t)(USBFS_SetupReqIndex & 0xFF))
                                    {
                                        case(DEF_UEP_IN | DEF_UEP1):
                                            /* Set End-point 1 IN NAK */
                                            USBOTG_FS->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                            break;

                                        case(DEF_UEP_OUT | DEF_UEP2):
                                            /* Set End-point 2 OUT ACK */
                                            USBOTG_FS->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;
                                            break;

                                        case(DEF_UEP_IN | DEF_UEP2):
                                            /* Set End-point 2 IN NAK */
                                            USBOTG_FS->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                            break;

                                        case(DEF_UEP_IN | DEF_UEP3):
                                            /* Set End-point 3 IN NAK */
                                            USBOTG_FS->UEP3_TX_CTRL = (USBOTG_FS->UEP3_TX_CTRL & ~( USBFS_UEP_T_TOG|USBFS_UEP_T_RES_MASK )) | USBFS_UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }else{
                                    errflag = 0xFF;
                                }
                            }else{
                                errflag = 0xFF;
                            }
                            break;
                        /* Set or enable one usb feature */
                        case USB_SET_FEATURE:
                            if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE){
                                /* Set Device Feature */
                                if((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP){
                                    if(MyCfgDescr[7] & 0x20){
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBFS_DevSleepStatus |= 0x01;
                                    }else{
                                        errflag = 0xFF;
                                    }
                                }else{
                                    errflag = 0xFF;
                                }
                            }else if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP){
                                /* Set End-point Feature */
                                if((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT){
                                    /* Set end-points status stall */
                                    switch((uint8_t)(USBFS_SetupReqIndex & 0xFF))
                                    {
                                        case(DEF_UEP_IN | DEF_UEP1):
                                            /* Set End-point 1 IN STALL */
                                            USBOTG_FS->UEP1_TX_CTRL = (USBOTG_FS->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        case(DEF_UEP_OUT | DEF_UEP2):
                                            /* Set End-point 2 OUT STALL */
                                            USBOTG_FS->UEP2_RX_CTRL = (USBOTG_FS->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL;
                                            break;

                                        case(DEF_UEP_IN | DEF_UEP2):
                                            /* Set End-point 2 IN STALL */
                                            USBOTG_FS->UEP2_TX_CTRL = (USBOTG_FS->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        case(DEF_UEP_IN | DEF_UEP3):
                                            /* Set End-point 3 IN STALL */
                                            USBOTG_FS->UEP3_TX_CTRL = (USBOTG_FS->UEP3_TX_CTRL &= ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }else{
                                    errflag = 0xFF;
                                }
                            }else{
                                errflag = 0xFF;
                            }
                            break;
                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            USBFS_EP0_Buf[0] = 0x00;
                            if(USBFS_SetupReqLen > 1){
                                USBFS_SetupReqLen = 1;
                            }
                            break;
                        case USB_SET_INTERFACE:
                            if(USBFS_SetupReqIndex==0x02){
                                if(USBFS_SetupReqValue==1){
                                    USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NONE;
                                    USBOTG_FS->UEP3_TX_LEN  = 0;
                                }else{
                                    USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                }
                            }
                            break;
                        /* Host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            USBFS_EP0_Buf[0] = 0x00;
                            USBFS_EP0_Buf[1] = 0x00;
                            if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE){
                                if(USBFS_DevSleepStatus & 0x01){
                                    USBFS_EP0_Buf[0] = 0x02;
                                }
                            }else if((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP){
                                switch((uint8_t)(USBFS_SetupReqIndex & 0xFF))
                                {
                                    case(DEF_UEP_IN | DEF_UEP1):
                                        if(((USBOTG_FS->UEP1_TX_CTRL) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL){
                                            USBFS_EP0_Buf[0] = 0x01;
                                        }
                                        break;

                                    case(DEF_UEP_OUT | DEF_UEP2):
                                        if(((USBOTG_FS->UEP2_RX_CTRL) & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL){
                                            USBFS_EP0_Buf[0] = 0x01;
                                        }
                                        break;

                                    case(DEF_UEP_IN | DEF_UEP2):
                                        if(((USBOTG_FS->UEP2_TX_CTRL) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL){
                                            USBFS_EP0_Buf[0] = 0x01;
                                        }
                                        break;

                                    case(DEF_UEP_IN | DEF_UEP3):
                                        if((USBOTG_FS->UEP3_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL){
                                            USBFS_EP0_Buf[0] = 0x01;
                                        }
                                        break;

                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }else{
                                errflag = 0xFF;
                            }

                            if(USBFS_SetupReqLen > 2){
                                USBFS_SetupReqLen = 2;
                            }

                            break;
                        default:
                            errflag = 0xFF;
                            break;
                    }
                }
                if(errflag == 0xFF){
                    /* if one request not support, return stall */
                    USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_STALL;
                    USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_STALL;
                }else{
                    /* end-point 0 data Tx/Rx */
                    if(USBFS_SetupReqType & DEF_UEP_IN){
                        /* tx */
                        len = (USBFS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                        USBFS_SetupReqLen -= len;
                        USBOTG_FS->UEP0_TX_LEN  = len;
                        USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_ACK;
                    }else{
                        /* rx */
                        if(USBFS_SetupReqLen == 0){
                            USBOTG_FS->UEP0_TX_LEN  = 0;
                            USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_ACK;
                        }else{
                            USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK;
                        }
                    }
                }
                break;
            /* Sof pack processing */
            case USBFS_UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        USBOTG_FS->INT_FG = USBFS_UIF_TRANSFER;

    }else if(intflag & USBFS_UIF_BUS_RST){
        /* usb reset interrupt processing */
        USBOTG_FS->DEV_ADDR = 0;
        USBFS_Device_Endp_Init();
        USBOTG_FS->INT_FG = USBFS_UIF_BUS_RST;
    }else if(intflag & USBFS_UIF_SUSPEND){
        /* usb suspend interrupt processing */
        if(USBOTG_FS->MIS_ST & USBFS_UMS_SUSPEND){
            USBFS_DevSleepStatus |= 0x02;
            if(USBFS_DevSleepStatus == 0x03){
                /* Handling usb sleep here */
            }
        }else{
            USBFS_DevSleepStatus &= ~0x02;
        }
        USBOTG_FS->INT_FG = USBFS_UIF_SUSPEND;
    }else{
        /* other interrupts */
        USBOTG_FS->INT_FG = intflag;
    }
}