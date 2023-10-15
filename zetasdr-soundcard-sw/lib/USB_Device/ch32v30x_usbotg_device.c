/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbotg_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the USBOTG firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x_usbotg_device.h"
#include "usb_desc.h"

/******************************************************************************/
/* global variable */
/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t EP0_DatabufHD[64]; //ep0(64)
__attribute__ ((aligned(4))) uint8_t EP3_DatabufHD[PACK_SIZE];  //ep3_out(PACK_SIZE)

uint8_t*  pEP0_RAM_Addr;                       //ep0(64)
uint8_t*  pEP3_RAM_Addr;                       //ep3_in(PACK_SIZE)

const uint8_t *pDescr;
volatile uint16_t USBHD_Dev_SetupReqLen = 0x00;                                   /* USBOTG�豸Setup������ */
volatile uint8_t  USBHD_Dev_SleepStatus = 0x00;                                  /* USBOTG�豸�ٶ� */

volatile uint8_t   DevConfig;
volatile uint8_t   SetupReqCode;
volatile uint16_t  SetupReqLen;

void USBHD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USBOTG_FS_DeviceInit
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 */
void USBDeviceInit( void )
{
    USBOTG_FS->BASE_CTRL = 0x00;

    USBOTG_FS->UEP2_3_MOD = USBHD_UEP3_TX_EN|USBHD_UEP3_BUF_MOD;//128+128

    USBOTG_FS->UEP0_DMA = (uint32_t)pEP0_RAM_Addr;
    USBOTG_FS->UEP3_DMA = (uint32_t)pEP3_RAM_Addr;

    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
    
    USBOTG_FS->UEP3_TX_LEN = DEF_USBD_SYC_PACK_SIZE;
    USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_TOUT|DATA(0);

    USBOTG_FS->INT_FG   = 0xFF;
    USBOTG_FS->INT_EN   = USBHD_UIE_SUSPEND | USBHD_UIE_BUS_RST | USBHD_UIE_TRANSFER;
    USBOTG_FS->DEV_ADDR = 0x00;

    USBOTG_FS->BASE_CTRL = USBHD_UC_DEV_PU_EN | USBHD_UC_INT_BUSY | USBHD_UC_DMA_EN;
    USBOTG_FS->UDEV_CTRL = USBHD_UD_PD_DIS|USBHD_UD_PORT_EN;
}

/*********************************************************************
 * @fn      USBOTG_RCC_Init
 *
 * @brief   Initializes the usbotg clock configuration.
 *
 * @return  none
 */
void USBOTG_RCC_Init( void )
{
    if( SystemCoreClock == 144000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div3 );
    }
    else if( SystemCoreClock == 96000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div2 );
    }
    else if( SystemCoreClock == 48000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div1 );
    }
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_OTG_FS, ENABLE );
}

/*********************************************************************
 * @fn      USBOTG_Init
 *
 * @brief   Initializes the USBOTG full speed device.
 *
 * @return  none
 */
void USBOTG_Init( void )
{
    /* Endpoint buffer initialization */
    pEP0_RAM_Addr = EP0_DatabufHD;
    pEP3_RAM_Addr = EP3_DatabufHD;

    /* ʹ��usbʱ�� */
    USBOTG_RCC_Init( );
    Delay_Us(100);
    /* usb�豸��ʼ�� */
    USBDeviceInit( );
    EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
    /* ʹ��usb�ж� */
    NVIC_EnableIRQ( USBHD_IRQn );
}

/*********************************************************************
 * @fn      OTG_FS_IRQHandler
 *
 * @brief   This function handles OTG_FS exception.
 *
 * @return  none
 */
uint8_t data[PACK_SIZE];

void USBHD_IRQHandler( void )
{
    uint16_t len, chtype;
    uint8_t  intflag, errflag = 0;
    intflag = USBOTG_FS->INT_FG;

    if( intflag & USBHD_UIF_TRANSFER )
    {
        switch ( USBOTG_FS->INT_ST & USBHD_UIS_TOKEN_MASK )
        {
            /* SETUP������ */
            case USBHD_UIS_TOKEN_SETUP:

                USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_NAK;
                USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_ACK;
                SetupReqLen  = pSetupReqPakHD->wLength;
                SetupReqCode = pSetupReqPakHD->bRequest;
                chtype = pSetupReqPakHD->bRequestType;
                len = 0;
                errflag = 0;
                /* �жϵ�ǰ�Ǳ�׼�������������� */
                if ( ( pSetupReqPakHD->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* ��������,��������,��������� */
                    if( pSetupReqPakHD->bRequestType & 0x40 )
                    {
                        /* �������� */
                        switch( pSetupReqPakHD->bRequest )
                        {
                            default:
                                errflag = 0xFF;/* ����ʧ�� */
                                break;
                        }
                    }
                    else if(pSetupReqPakHD->bRequestType == 0xa1)
                    {
                        switch( pSetupReqPakHD->bRequest )
                        {
                        case 0x81://GET CUR
                            if((pSetupReqPakHD->wIndex==0x200)||(pSetupReqPakHD->wIndex==0x500))//Feature Unit
                            {
                                if(pSetupReqPakHD->wValue==0x100)//mute
                                {
                                    u8 class_request[1]={0x00};
                                    pDescr=class_request;
                                    len=1;
                                }
                                if(pSetupReqPakHD->wValue==0x200)//mute
                                {
                                    u8 class_request[2]={0x00,0xf9};
                                    pDescr=class_request;
                                    len=2;
                                }
                            }
                            break;
                        case 0x82://GET MIN
                            break;
                        case 0x83://GET MAX
                            break;
                        case 0x84://GET RES
                            break;
                        default:
                            errflag = 0xFF;
                            break;
                        }
                    }
                    else if( pSetupReqPakHD->bRequestType & 0x20 )
                    {
                        /* HID������ */
                        switch( pSetupReqPakHD->bRequest )
                        {
                        case 0x01: //GetReport
                            break;
                        case 0x02: //GetIdle
                            break;
                        case 0x03: //GetProtocol
                            break;
                        case 0x09: //SetReport
                            break;
                        case 0x0A: //SetIdle
                            break;
                        case 0x0B: //SetProtocol
                            break;

                        default:
                            errflag = 0xFF;
                            break;
                        }
                    }

                    /* �ж��Ƿ������������ */
                    if( errflag != 0xFF )
                    {
                        if( SetupReqLen>len )   SetupReqLen = len;
                        len = (SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : SetupReqLen;
                        memcpy( pEP0_DataBuf, pDescr, len );
                        pDescr += len;

                    }
                }
                else
                {
                    /* ������׼USB����� */
                    switch( SetupReqCode )
                    {
                        case USB_GET_DESCRIPTOR:
                        {
                            switch( ((pSetupReqPakHD->wValue)>>8) )
                            {
                                case USB_DESCR_TYP_DEVICE:
                                    /* ��ȡ�豸������ */
                                    pDescr = MyDevDescrHD;
                                    len = MyDevDescrHD[0];
                                    break;

                                case USB_DESCR_TYP_CONFIG:
                                    /* ��ȡ���������� */
                                    pDescr = MyCfgDescr_FS;
                                    len = ((uint16_t)MyCfgDescr_FS[2] + (uint16_t)(MyCfgDescr_FS[3] << 8));
                                    break;

                                case USB_DESCR_TYP_STRING:
                                    /* ��ȡ�ַ��������� */
                                    switch( (pSetupReqPakHD->wValue)&0xff )
                                    {
                                        case 0:
                                            /* �����ַ��������� */
                                            pDescr = MyLangDescrHD;
                                            len = MyLangDescrHD[0];
                                            break;

                                        case 1:
                                            /* USB�����ַ��������� */
                                            pDescr = MyManuInfoHD;
                                            len = MyManuInfoHD[0];
                                            break;

                                        case 2:
                                            /* USB��Ʒ�ַ��������� */
                                            pDescr = MyProdInfoHD;
                                            len = MyProdInfoHD[0];
                                            break;

                                        case 3:
                                            /* USB���к��ַ��������� */
                                            pDescr = MySerNumInfoHD;
                                            len = ((uint16_t)MySerNumInfoHD[0]);
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                case USB_DESCR_TYP_REPORT:
                                    /* USB�豸���������� */
                                    if( (uint8_t)((pSetupReqPakHD->wIndex)&0xFF) == 0 )
                                    {

                                    }
                                    else if( (uint8_t)((pSetupReqPakHD->wIndex)&0xFF) == 1 )
                                    {

                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                case USB_DESCR_TYP_QUALIF:
                                    /* �豸�޶������� */
                                    pDescr = ( uint8_t* )&MyUSBQUADescHD[ 0 ];
                                    len = ((uint16_t) MyUSBQUADescHD[0] );
                                    break;

                                case USB_DESCR_TYP_SPEED:
                                    /* �����ٶ����������� */
                                    errflag = 0xFF;
                                    break;

                                case USB_DESCR_TYP_BOS:
                                    /* BOS������ */
                                    /* USB2.0�豸��֧��BOS������ */
                                    errflag = 0xFF;
                                    break;

                                default :
                                    errflag = 0xff;
                                    break;

                            }

                            if( SetupReqLen>len )   SetupReqLen = len;
                            len = (SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : SetupReqLen;
                            memcpy( pEP0_DataBuf, pDescr, len );
                            pDescr += len;
                        }
                            break;

                        case USB_SET_ADDRESS:
                            /* ���õ�ַ */
                            SetupReqLen = (pSetupReqPakHD->wValue)&0xff;
                            break;

                        case USB_GET_CONFIGURATION:
                            /* ��ȡ����ֵ */
                            pEP0_DataBuf[0] = DevConfig;
                            if ( SetupReqLen > 1 ) SetupReqLen = 1;
                            break;

                        case USB_SET_CONFIGURATION:
                            /* ��������ֵ */
                            DevConfig = (pSetupReqPakHD->wValue)&0xff;
                            break;

                        case USB_CLEAR_FEATURE:
                            /* ������� */
                            if ( ( pSetupReqPakHD->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* ����˵� */
                                switch( (pSetupReqPakHD->wIndex)&0xff )
                                {
                                case 0x83:
                                    USBOTG_FS->UEP3_TX_CTRL = (USBOTG_FS->UEP3_TX_CTRL & ~( USBHD_UEP_T_TOG|USBHD_UEP_T_RES_MASK )) | USBHD_UEP_T_RES_NAK;
                                    break;
                                default:
                                    errflag = 0xFF;
                                    break;

                                }
                            }
                            else    errflag = 0xFF;
                            break;

                        case USB_SET_FEATURE:
                            /* �������� */
                            if( ( pMySetupReqPakHD->bRequestType & 0x1F ) == 0x00 )
                            {
                                /* �����豸 */
                                if( pMySetupReqPakHD->wValue == 0x01 )
                                {
                                    if( MyCfgDescr_FS[ 7 ] & 0x20 )
                                    {
                                        /* ���û���ʹ�ܱ�־ */
                                        USBHD_Dev_SleepStatus = 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( pMySetupReqPakHD->bRequestType & 0x1F ) == 0x02 )
                            {
                                /* ���ö˵� */
                                if( pMySetupReqPakHD->wValue == 0x00 )
                                {
                                    /* ����ָ���˵�STALL */
                                    switch( ( pMySetupReqPakHD->wIndex ) & 0xff )
                                    {
                                        case 0x83:
                                            /* ���ö˵�3 IN STALL */
                                            USBOTG_FS->UEP3_TX_CTRL = ( USBOTG_FS->UEP3_TX_CTRL &= ~USBHD_UEP_T_RES_MASK ) | USBHD_UEP_T_RES_STALL;
                                            break;
                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        case USB_GET_INTERFACE:
                            /* ��ȡ�ӿ� */
                            pEP0_DataBuf[0] = 0x00;
                            if ( SetupReqLen > 1 ) SetupReqLen = 1;
                            break;

                        case USB_SET_INTERFACE:
                            if(pSetupReqPakHD->wIndex==0x02)//mic
                            {
                                if(pSetupReqPakHD->wValue==1)
                                {
                                    USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_TOUT|DATA(0);
                                    USBOTG_FS->UEP3_TX_LEN  = 0;
                                }
                                else
                                {
                                    USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_NAK;
                                }
                            }
                            break;

                        case USB_GET_STATUS:
                            /* ���ݵ�ǰ�˵�ʵ��״̬����Ӧ�� */
                            EP0_DatabufHD[ 0 ] = 0x00;
                            EP0_DatabufHD[ 1 ] = 0x00;
                            if( pMySetupReqPakHD->wIndex == 0x83 )
                            {
                                if( ( USBOTG_FS->UEP3_TX_CTRL & USBHD_UEP_T_RES_MASK ) == USBHD_UEP_T_RES_STALL )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x01;
                                }
                            }
                            if( USBHD_Dev_SetupReqLen > 2 )
                            {
                                USBHD_Dev_SetupReqLen = 2;
                            }
                            break;

                        default:
                            errflag = 0xff;
                            break;
                    }
                }
                if( errflag == 0xff)
                {
                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_STALL;
                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_STALL;
                }
                else
                {
                    if( chtype & 0x80 )
                    {
                        len = (SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : SetupReqLen;
                        SetupReqLen -= len;
                    }
                    else  len = 0;

                    USBOTG_FS->UEP0_TX_LEN  = len;
                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_ACK;
                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_ACK;
                }
                break;

            case USBHD_UIS_TOKEN_IN:
                switch ( USBOTG_FS->INT_ST & ( USBHD_UIS_TOKEN_MASK | USBHD_UIS_ENDP_MASK ) )
                {
                    case USBHD_UIS_TOKEN_IN:
                        switch( SetupReqCode )
                        {
                            case USB_GET_DESCRIPTOR:
                                    len = SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : SetupReqLen;
                                    memcpy( pEP0_DataBuf, pDescr, len );
                                    SetupReqLen -= len;
                                    pDescr += len;
                                    USBOTG_FS->UEP0_TX_LEN   = len;
                                    USBOTG_FS->UEP0_TX_CTRL ^= USBHD_UEP_T_TOG;
                                    break;

                            case USB_SET_ADDRESS:
                                    USBOTG_FS->DEV_ADDR = (USBOTG_FS->DEV_ADDR&USBHD_UDA_GP_BIT) | SetupReqLen;
                                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
                                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
                                    break;

                            default:
                                    USBOTG_FS->UEP0_TX_LEN = 0;
                                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
                                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
                                    break;

                        }
                        break;

                    case USBHD_UIS_TOKEN_IN | 3:

                        //������û�䣬��data���뻺����
                        memcpy( pEP3_IN_DataBuf, data, PACK_SIZE );

                        USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_TOUT|DATA(0);
                        USBOTG_FS->UEP3_TX_LEN  = PACK_SIZE;

                        TIM_Cmd(TIM1, ENABLE);
                        break;
                    default :
                        break;

                }
                break;

            case USBHD_UIS_TOKEN_OUT:
                switch ( USBOTG_FS->INT_ST & ( USBHD_UIS_TOKEN_MASK | USBHD_UIS_ENDP_MASK ) )
                {
                    case USBHD_UIS_TOKEN_OUT:
                            break;

                    case USBHD_UIS_TOKEN_OUT | 3:
                        // OUT ENDPOINT 3
                        break;
                    default:
                        break;
                }

                break;

            case USBHD_UIS_TOKEN_SOF:

                break;

            default :
                break;

        }

        USBOTG_FS->INT_FG = USBHD_UIF_TRANSFER;
    }
    else if( intflag & USBHD_UIF_BUS_RST )
    {
        USBOTG_FS->DEV_ADDR = 0;

        USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;

        USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_TOUT|DATA(0);

        USBOTG_FS->INT_FG |= USBHD_UIF_BUS_RST;
    }
    else if( intflag & USBHD_UIF_SUSPEND )
    {
        if ( USBOTG_FS->MIS_ST & USBHD_UMS_SUSPEND ) {;}
        else{;}
        USBOTG_FS->INT_FG = USBHD_UIF_SUSPEND;
    }
    else
    {
        USBOTG_FS->INT_FG = intflag;
    }
}