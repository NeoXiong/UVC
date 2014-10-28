/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: usb_host_video_inf.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*   This file Contains the implementation of class driver for Video devices.
*
*END************************************************************************/
#include "usb_host_config.h"
#if USBCFG_HOST_VIDEO
#include "usb.h"
#include "usb_host_stack_interface.h"
#include "usb_host_video.h"
#include "usb_host_common.h"
#include "usb_host_ch9.h"
#include "usb_host_dev_mng.h"

static void* usb_class_video_find_descriptor_index(void *descriptor, uint32_t size, uint8_t type, uint8_t sub_type)
{
    video_descriptor_union_t des_ptr;
    uint32_t index = 0;

    des_ptr.pntr = descriptor;

    if (NULL == descriptor)
    {
        return NULL;
    }

    while(index < size)
    {
        if ((des_ptr.video_common->bDescriptorType == type) && (des_ptr.video_common->bDescriptorSubtype == sub_type))
        {
            return des_ptr.pntr;
        }
        index += des_ptr.video_common->bLength;
        des_ptr.word += des_ptr.video_common->bLength;
    }
    return NULL;
}

static void* usb_class_video_find_frame_descriptor(void *descriptor, uint32_t count, uint8_t type, uint8_t sub_type, uint8_t index)
{
    video_descriptor_union_t des_ptr;
    uint32_t i = 0;

    des_ptr.pntr = descriptor;

    if (NULL == descriptor)
    {
        return NULL;
    }

    while (i < count)
    {
        if ((des_ptr.video_frame_common->bDescriptorType == type) 
            && (des_ptr.video_frame_common->bDescriptorSubtype == sub_type)
            )
        {
            if (des_ptr.video_frame_common->bFrameIndex == index)
            {
                return des_ptr.pntr;                
            }
            else
            {
                i++;
            }
        }
        des_ptr.word += des_ptr.video_common->bLength;
    }
    return NULL;
}

static uint32_t usb_class_video_find_descriptor_count(void *descriptor, uint32_t size, uint8_t type, uint8_t sub_type)
{
    video_descriptor_union_t des_ptr;
    uint32_t index = 0;
    uint32_t count = 0;

    des_ptr.pntr = descriptor;

    if (NULL == descriptor)
    {
        return 0;
    }

    while(index < size)
    {
        if ((des_ptr.video_common->bDescriptorType == type) && (des_ptr.video_common->bDescriptorSubtype == sub_type))
        {
            count++;
        }
        index += des_ptr.video_common->bLength;
        des_ptr.word += des_ptr.video_common->bLength;
    }
    return count;
}

static usb_status usb_class_video_fill_endpoint_descriptor(usb_video_stream_struct_t* video_stream_ptr,uint8_t if_alternate)
{
    usb_device_interface_struct_t*     pDeviceIntf = NULL;    
    descriptor_union_t                 ptr1;
    descriptor_union_t                 ptr2;
    usb_device_ep_struct_t *           pEndpoint = NULL;
    usb_status                         ret = USB_OK;
    uint8_t                            ep_num = 0;
    /*--------------------------------------------------------**
    ** If descriptor type is Configuration, the pointer is    **
    **    found in ptr1.  Other types of descriptor need      **
    **    to be found by stepping through the config one      **
    **    descriptor at a time.                               **
    ** To prevent parsing past the config buffer, ptr2 is     **
    **    set to the starting address plus its total size.    **
    **--------------------------------------------------------*/
    pDeviceIntf = video_stream_ptr->stream_interface_ptr;
    ptr1.pntr = pDeviceIntf->interfaceEx;
    ptr2.word = ptr1.word + pDeviceIntf->interfaceExlength;
    while (ptr1.word < ptr2.word)
    {
        /* the first interface descriptor found */
        if (ptr1.common->bDescriptorType == USB_DESC_TYPE_IF)
        {
            if (ptr1.intf->bAlternateSetting == if_alternate)
            {
                if (ptr1.intf->bNumEndpoints > USBCFG_HOST_MAX_EP_PER_INTERFACE)
                {
                    USB_PRINTF("too many endpoints in one interface, please increase the USBCFG_HOST_MAX_EP_PER_INTERFACE value\n");
                    ret = USBERR_ERROR;
                    break;
                }

                if (ptr1.intf->bNumEndpoints > 0)
                {
                    video_stream_ptr->altrnate_if_ep_num = ptr1.intf->bNumEndpoints;
                }
                else
                {
                    return USB_OK;
                }

                if(video_stream_ptr->altrnate_if_ep_descriptor_list)
                {
                    OS_Mem_free(video_stream_ptr->altrnate_if_ep_descriptor_list);
                }
                video_stream_ptr->altrnate_if_ep_descriptor_list = (usb_device_ep_struct_t*)OS_Mem_alloc_zero(video_stream_ptr->altrnate_if_ep_num * sizeof(usb_device_ep_struct_t));

                ptr1.word += ptr1.common->bLength;
                while (ptr1.word < ptr2.word)
                {
                    if (ptr1.common->bDescriptorType != USB_DESC_TYPE_EP)
                    {
                        ptr1.word += ptr1.common->bLength;
                    }
                    else
                    {
                        break;
                    }
                }

                /* now the ptr1 should point to endpoint descriptor */
                if (ptr1.common->bDescriptorType != USB_DESC_TYPE_EP)
                {
                    USB_PRINTF("interface descriptor error\n");
                    ret = USBERR_ERROR;
                    break;
                }

                for (ep_num = 0; ep_num < video_stream_ptr->altrnate_if_ep_num; ep_num++)
                {
                    if ((ptr1.ndpt->bDescriptorType != USB_DESC_TYPE_EP) ||
                        (ptr1.word >= ptr2.word))
                    {
                        USB_PRINTF("endpoint descriptor error\n");
                        ret = USBERR_ERROR;
                        break;
                    }
                    pEndpoint = &video_stream_ptr->altrnate_if_ep_descriptor_list[ep_num];
                    pEndpoint->lpEndpointDesc = ptr1.ndpt;
                    ptr1.word += ptr1.common->bLength;

                    while (ptr1.word < ptr2.word)
                    {
                        if (ptr1.common->bDescriptorType != USB_DESC_TYPE_EP)
                        {
                            ptr1.word += ptr1.common->bLength;
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                if (ret != USB_OK)
                {
                    break;
                }
            }
            else
            {
                ptr1.word += ptr1.common->bLength;
            }
        }
        else
        {
            ptr1.word += ptr1.common->bLength;
        }
    }

    return ret;
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_control_init
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_control_init
(
    /* [IN]  the device handle related to the class driver */
    usb_device_instance_handle      dev_handle,
    /* [IN]  the interface handle related to the class driver */
    usb_interface_descriptor_handle intf_handle,
    /* [OUT] printer call struct pointer */
    class_handle*                    class_handle_ptr
)
{ /* Body */
    usb_video_control_struct_t*      video_control_ptr = NULL;
    usb_device_interface_struct_t*  pDeviceIntf = NULL;
    //interface_descriptor_t*      intf = NULL;
    endpoint_descriptor_t*       ep_desc = NULL;
    uint8_t                       ep_num;
    usb_status                    status = USB_OK;
    pipe_init_struct_t              pipe_init;

    video_control_ptr = (usb_video_control_struct_t*)OS_Mem_alloc_zero(sizeof(usb_video_control_struct_t));
    if (video_control_ptr == NULL)
    {
#ifdef _DEBUG
        USB_PRINTF("usb_class_video_control_init fail on memory allocation\n");
#endif
        return USBERR_ERROR;
    }
    
    video_control_ptr->dev_handle  = dev_handle;
    video_control_ptr->intf_handle = intf_handle;
    video_control_ptr->host_handle = (usb_host_handle)usb_host_dev_mng_get_host(video_control_ptr->dev_handle);
    
    pDeviceIntf = (usb_device_interface_struct_t*)intf_handle;

    video_control_ptr->control_interface_ptr = pDeviceIntf;
    video_control_ptr->vc_interface_header_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_HEADER);
    video_control_ptr->vc_input_terminal_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_INPUT_TERMINAL);
    video_control_ptr->vc_output_terminal_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_OUTPUT_TERMINAL);
    video_control_ptr->vc_processing_unit_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_PROCESSING_UNIT);
    video_control_ptr->vc_extension_unit_list_ptr.extension_descriptor_list_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_EXTENSION_UNIT);
    video_control_ptr->vc_extension_unit_list_ptr.extension_num = usb_class_video_find_descriptor_count(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VC_EXTENSION_UNIT);
    //intf = pDeviceIntf->lpinterfaceDesc;

    for (ep_num = 0; ep_num < pDeviceIntf->ep_count; ep_num++)
    {
        ep_desc = pDeviceIntf->ep[ep_num].lpEndpointDesc;
        if ((ep_desc->bEndpointAddress & IN_ENDPOINT) && ((ep_desc->bmAttributes & EP_TYPE_MASK) == IRRPT_ENDPOINT))
        {
            pipe_init.endpoint_number  = (ep_desc->bEndpointAddress & ENDPOINT_MASK);
            pipe_init.direction        = USB_RECV;
            pipe_init.pipetype         = USB_INTERRUPT_PIPE;
            pipe_init.max_packet_size  = (uint16_t)(USB_SHORT_UNALIGNED_LE_TO_HOST(ep_desc->wMaxPacketSize) & PACKET_SIZE_MASK);
            pipe_init.interval         = ep_desc->iInterval;
            pipe_init.flags            = 0;
            pipe_init.dev_instance     = video_control_ptr->dev_handle;
            pipe_init.nak_count        = USBCFG_HOST_DEFAULT_MAX_NAK_COUNT;
            status = usb_host_open_pipe(video_control_ptr->host_handle, &video_control_ptr->control_interrupt_in_pipe, &pipe_init);
            if (status != USB_OK)
            {
#ifdef _DEBUG
                USB_PRINTF("usb_class_video_init fail to open in pipe\n");
#endif
                *class_handle_ptr = (class_handle)video_control_ptr;
                return USBERR_ERROR;
            }
        }
    }
    
    video_control_ptr->in_setup = FALSE;
    video_control_ptr->ctrl_callback = NULL;
    video_control_ptr->ctrl_param = NULL;
    video_control_ptr->recv_callback = NULL;
    video_control_ptr->recv_param = NULL;
    
    *class_handle_ptr = (class_handle)video_control_ptr;

    //USB_PRINTF("Video class driver initialized\n");
    
    return USB_OK;
   
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_control_deinit
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_control_deinit
(
    /* [IN]  the class driver handle */
    class_handle      handle
)
{
    usb_video_control_struct_t*      video_control_ptr = (usb_video_control_struct_t*)handle;
    usb_status                    status = USB_OK;
    if (video_control_ptr == NULL)
    {
#ifdef _DEBUG
        USB_PRINTF("usb_class_video_control_deinit fail\n");
#endif
        return USBERR_ERROR;
    }

    if (video_control_ptr->control_interrupt_in_pipe != NULL)
    {
        status = usb_host_close_pipe(video_control_ptr->host_handle, video_control_ptr->control_interrupt_in_pipe);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_deinit to close pipe\n");
#endif
        }
    }
    
    OS_Mem_free(handle);
    //USB_PRINTF("Video class driver de-initialized\n");
    return USB_OK;
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_control_pre_deinit
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_control_pre_deinit
    (
        /* [IN]  the class driver handle */
        class_handle      handle
     )
{
    usb_video_control_struct_t*      video_control_ptr = (usb_video_control_struct_t*)handle;
    usb_status                   status = USB_OK;

    if (video_control_ptr == NULL)
    {
#ifdef _DEBUG    
        USB_PRINTF("usb_class_video_control_pre_deinit fail\n");
#endif
        return USBERR_ERROR;
    }

    if (video_control_ptr->control_interrupt_in_pipe != NULL)
    {
        status = usb_host_cancel(video_control_ptr->host_handle, video_control_ptr->control_interrupt_in_pipe, NULL);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_pre_deinit to close pipe\n");
#endif
        }
    }

    //USB_PRINTF("Video class driver pre_deinit\n");
    return USB_OK;
} /* Endbody */


/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_stream_init
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_stream_init
(
    /* [IN]  the device handle related to the class driver */
    usb_device_instance_handle      dev_handle,
    /* [IN]  the interface handle related to the class driver */
    usb_interface_descriptor_handle intf_handle,
    /* [OUT] printer call struct pointer */
    class_handle*                    class_handle_ptr
)
{ /* Body */
    usb_video_stream_struct_t*      video_stream_ptr = NULL;
    usb_device_interface_struct_t*  pDeviceIntf = NULL;
    //interface_descriptor_t*      intf = NULL;
    endpoint_descriptor_t*       ep_desc = NULL;
    uint8_t                       ep_num;
    usb_status                    status = USB_OK;
    pipe_init_struct_t              pipe_init;

    video_stream_ptr = (usb_video_stream_struct_t*)OS_Mem_alloc_zero(sizeof(usb_video_stream_struct_t));
    if (video_stream_ptr == NULL)
    {
#ifdef _DEBUG
        USB_PRINTF("usb_class_video_stream_init fail on memory allocation\n");
#endif
        return USBERR_ERROR;
    }
    
    video_stream_ptr->dev_handle  = dev_handle;
    video_stream_ptr->intf_handle = intf_handle;
    video_stream_ptr->host_handle = (usb_host_handle)usb_host_dev_mng_get_host(video_stream_ptr->dev_handle);

    video_stream_ptr->altrnate_if_ep_num = 0;
    video_stream_ptr->altrnate_if_ep_descriptor_list = NULL;
    
    pDeviceIntf = (usb_device_interface_struct_t*)intf_handle;

    video_stream_ptr->stream_interface_ptr = pDeviceIntf;
    video_stream_ptr->current_stream_interface_ptr = pDeviceIntf->lpinterfaceDesc;
    video_stream_ptr->vs_input_header_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VS_INPUT_HEADER);
    video_stream_ptr->vs_mjpeg_format_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VS_FORMAT_MJPEG);
    video_stream_ptr->vs_mjpeg_frame_list.frame_descriptor_list_ptr = usb_class_video_find_descriptor_index(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VS_FRAME_MJPEG);
    video_stream_ptr->vs_mjpeg_frame_list.frame_num = usb_class_video_find_descriptor_count(pDeviceIntf->interfaceEx, pDeviceIntf->interfaceExlength, CS_INTERFACE, VS_FRAME_MJPEG);
    //intf = pDeviceIntf->lpinterfaceDesc;

    for (ep_num = 0; ep_num < pDeviceIntf->ep_count; ep_num++)
    {
        ep_desc = pDeviceIntf->ep[ep_num].lpEndpointDesc;
        if ((ep_desc->bEndpointAddress & IN_ENDPOINT) && ((ep_desc->bmAttributes & EP_TYPE_MASK) == ISOCH_ENDPOINT))
        {
            pipe_init.endpoint_number  = (ep_desc->bEndpointAddress & ENDPOINT_MASK);
            pipe_init.direction        = USB_RECV;
            pipe_init.pipetype         = USB_ISOCHRONOUS_PIPE;
            pipe_init.max_packet_size  = (uint16_t)(USB_SHORT_UNALIGNED_LE_TO_HOST(ep_desc->wMaxPacketSize) & PACKET_SIZE_MASK);
            pipe_init.interval         = ep_desc->iInterval;
            pipe_init.flags            = 0;
            pipe_init.dev_instance     = video_stream_ptr->dev_handle;
            pipe_init.nak_count        = USBCFG_HOST_DEFAULT_MAX_NAK_COUNT;
            status = usb_host_open_pipe(video_stream_ptr->host_handle, &video_stream_ptr->stream_iso_in_pipe, &pipe_init);
            if (status != USB_OK)
            {
#ifdef _DEBUG
                USB_PRINTF("usb_class_video_init fail to open in pipe\n");
#endif
                *class_handle_ptr = (class_handle)video_stream_ptr;
                return USBERR_ERROR;
            }
        }
        else if ((ep_desc->bEndpointAddress & OUT_ENDPOINT) && ((ep_desc->bmAttributes & EP_TYPE_MASK) == ISOCH_ENDPOINT))
        {
            pipe_init.endpoint_number  = (ep_desc->bEndpointAddress & ENDPOINT_MASK);
            pipe_init.direction        = USB_SEND;
            pipe_init.pipetype         = USB_ISOCHRONOUS_PIPE;
            pipe_init.max_packet_size  = (uint16_t)(USB_SHORT_UNALIGNED_LE_TO_HOST(ep_desc->wMaxPacketSize) & PACKET_SIZE_MASK);
            pipe_init.interval         = ep_desc->iInterval;
            pipe_init.flags            = 0;
            pipe_init.dev_instance     = video_stream_ptr->dev_handle;
            pipe_init.nak_count        = USBCFG_HOST_DEFAULT_MAX_NAK_COUNT;
            status = usb_host_open_pipe(video_stream_ptr->host_handle, &video_stream_ptr->stream_iso_out_pipe, &pipe_init);
            if (status != USB_OK)
            {
#ifdef _DEBUG
                USB_PRINTF("usb_class_video_init fail to open in pipe\n");
#endif
                *class_handle_ptr = (class_handle)video_stream_ptr;
                return USBERR_ERROR;
            }
        }

    }
    
    video_stream_ptr->recv_callback = NULL;
    video_stream_ptr->recv_param = NULL;
    
    *class_handle_ptr = (class_handle)video_stream_ptr;

    //USB_PRINTF("Video class driver initialized\n");
    
    return USB_OK;
   
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_stream_deinit
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_stream_deinit
(
    /* [IN]  the class driver handle */
    class_handle      handle
)
{
    usb_video_stream_struct_t*      video_stream_ptr = (usb_video_stream_struct_t*)handle;
    usb_status                    status = USB_OK;
    if (video_stream_ptr == NULL)
    {
#ifdef _DEBUG
        USB_PRINTF("usb_class_video_stream_deinit fail\n");
#endif
        return USBERR_ERROR;
    }

    if (video_stream_ptr->stream_iso_in_pipe != NULL)
    {
        status = usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_in_pipe);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_stream_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_iso_out_pipe != NULL)
    {
        status = usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_out_pipe);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_stream_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_bulk_in_pipe != NULL)
    {
        status = usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_in_pipe);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_stream_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_bulk_out_pipe != NULL)
    {
        status = usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_out_pipe);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_stream_deinit to close pipe\n");
#endif
        }
    }
    OS_Mem_free(video_stream_ptr->altrnate_if_ep_descriptor_list);
    OS_Mem_free(handle);
    //USB_PRINTF("Video class driver de-initialized\n");
    return USB_OK;
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_stream_pre_deinit
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_stream_pre_deinit
    (
        /* [IN]  the class driver handle */
        class_handle      handle
     )
{
    usb_video_stream_struct_t*      video_stream_ptr = (usb_video_stream_struct_t*)handle;
    usb_status                   status = USB_OK;

    if (video_stream_ptr == NULL)
    {
#ifdef _DEBUG    
        USB_PRINTF("usb_class_video_stream_pre_deinit fail\n");
#endif
        return USBERR_ERROR;
    }

    if (video_stream_ptr->stream_iso_in_pipe != NULL)
    {
        status = usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_in_pipe, NULL);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_pre_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_iso_out_pipe != NULL)
    {
        status = usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_out_pipe, NULL);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_pre_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_bulk_in_pipe != NULL)
    {
        status = usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_in_pipe, NULL);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_pre_deinit to close pipe\n");
#endif
        }
    }
    if (video_stream_ptr->stream_bulk_out_pipe != NULL)
    {
        status = usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_out_pipe, NULL);
        if (status != USB_OK)
        {
#ifdef _DEBUG
            USB_PRINTF("error in usb_class_video_control_pre_deinit to close pipe\n");
#endif
        }
    }

    //USB_PRINTF("Video class driver pre_deinit\n");
    return USB_OK;
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_set_stream_inf
* Returned Value : None
* Comments       :
*     This function is called by common class to initialize the class driver. It
*     is called in response to a select interface call by application
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_set_stream_inf
(
    class_handle                    class_handle,
    uint8_t                          inf_alternate
)
{ /* Body */
    usb_video_stream_struct_t*      video_stream_ptr = NULL;
    endpoint_descriptor_t*          ep_desc = NULL;
    uint8_t                         ep_num = 0;
    pipe_init_struct_t              pipe_init;
    usb_status                      status;
    
    video_stream_ptr = (usb_video_stream_struct_t*)class_handle;
    if (video_stream_ptr == NULL)
    {
        return USBERR_ERROR;
    }

    if(video_stream_ptr->current_stream_interface_ptr->bAlternateSetting == inf_alternate)
    {
        return USBERR_ERROR;
    }

    if(video_stream_ptr->stream_interface_ptr->alternate_setting_num < inf_alternate)
    {
        return USBERR_ERROR;
    }

    if (video_stream_ptr->stream_iso_in_pipe != NULL)
    {
        usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_in_pipe, NULL);
        usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_in_pipe);
    }
    if (video_stream_ptr->stream_iso_out_pipe != NULL)
    {
        usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_out_pipe, NULL);
        usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_out_pipe);
    }
    if (video_stream_ptr->stream_bulk_in_pipe != NULL)
    {
        usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_in_pipe, NULL);
        usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_in_pipe);
    }
    if (video_stream_ptr->stream_bulk_out_pipe != NULL)
    {
        usb_host_cancel(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_out_pipe, NULL);
        usb_host_close_pipe(video_stream_ptr->host_handle, video_stream_ptr->stream_bulk_out_pipe);
    }
    usb_class_video_fill_endpoint_descriptor(video_stream_ptr, inf_alternate);

    for (ep_num = 0; ep_num < video_stream_ptr->altrnate_if_ep_num; ep_num++)
    {
        ep_desc = video_stream_ptr->altrnate_if_ep_descriptor_list[ep_num].lpEndpointDesc;
        if ((ep_desc->bEndpointAddress & IN_ENDPOINT) && ((ep_desc->bmAttributes & EP_TYPE_MASK) == ISOCH_ENDPOINT))
        {
            pipe_init.endpoint_number  = (ep_desc->bEndpointAddress & ENDPOINT_MASK);
            pipe_init.direction        = USB_RECV;
            pipe_init.pipetype         = USB_ISOCHRONOUS_PIPE;
            pipe_init.max_packet_size  = (uint16_t)(USB_SHORT_UNALIGNED_LE_TO_HOST(ep_desc->wMaxPacketSize) & PACKET_SIZE_MASK);
            pipe_init.interval         = ep_desc->iInterval;
            pipe_init.flags            = 0;
            pipe_init.dev_instance     = video_stream_ptr->dev_handle;
            pipe_init.nak_count        = USBCFG_HOST_DEFAULT_MAX_NAK_COUNT;
            status = usb_host_open_pipe(video_stream_ptr->host_handle, &video_stream_ptr->stream_iso_in_pipe, &pipe_init);
            if (status != USB_OK)
            {
#ifdef _DEBUG
                USB_PRINTF("usb_class_video_set_stream_inf fail to open in pipe\n");
#endif
                return USBERR_ERROR;
            }
        }
        else if ((ep_desc->bEndpointAddress & OUT_ENDPOINT) && ((ep_desc->bmAttributes & EP_TYPE_MASK) == ISOCH_ENDPOINT))
        {
            pipe_init.endpoint_number  = (ep_desc->bEndpointAddress & ENDPOINT_MASK);
            pipe_init.direction        = USB_SEND;
            pipe_init.pipetype         = USB_ISOCHRONOUS_PIPE;
            pipe_init.max_packet_size  = (uint16_t)(USB_SHORT_UNALIGNED_LE_TO_HOST(ep_desc->wMaxPacketSize) & PACKET_SIZE_MASK);
            pipe_init.interval         = ep_desc->iInterval;
            pipe_init.flags            = 0;
            pipe_init.dev_instance     = video_stream_ptr->dev_handle;
            pipe_init.nak_count        = USBCFG_HOST_DEFAULT_MAX_NAK_COUNT;
            status = usb_host_open_pipe(video_stream_ptr->host_handle, &video_stream_ptr->stream_iso_out_pipe, &pipe_init);
            if (status != USB_OK)
            {
#ifdef _DEBUG
                USB_PRINTF("usb_class_video_set_stream_inf fail to open in pipe\n");
#endif
                return USBERR_ERROR;
            }
        }
    }
    //USB_PRINTF("usb_class_video_set_stream_inf\n");
    
    return USB_OK;
   
} /* Endbody */


usb_status usb_class_video_get_format_descriptor
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     sub_type,
    /* [OUT] Data Bffer */
    uint8_t **                  descriptor_ptr
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if(NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }

    if(sub_type == VS_FORMAT_MJPEG)
    {
        if(NULL == video_stream_ptr->vs_mjpeg_format_ptr)
        {
            return USBERR_NO_INTERFACE;
        }
        *descriptor_ptr = (uint8_t *)video_stream_ptr->vs_mjpeg_format_ptr;
        return USB_OK;
    }
    return USBERR_INVALID_PARAM;
}

usb_status usb_class_video_get_frame_descriptor
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     sub_type,
    /* [IN] Request index */
    uint8_t                     index,
    /* [OUT] Data Bffer */
    uint8_t **                   descriptor_ptr
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if(NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }
    
    if(sub_type == VS_FRAME_MJPEG)
    {
        if(NULL == video_stream_ptr->vs_mjpeg_format_ptr)
        {
            return USBERR_NO_INTERFACE;
        }
        *descriptor_ptr = usb_class_video_find_frame_descriptor(video_stream_ptr->vs_mjpeg_format_ptr,
                                                                     video_stream_ptr->vs_mjpeg_format_ptr->bNumFrameDescriptors,
                                                                     CS_INTERFACE, VS_FRAME_MJPEG, index);

        if(*descriptor_ptr)
        {
            return USB_OK;
        }
    }
    return USBERR_INVALID_PARAM;
}


#endif
