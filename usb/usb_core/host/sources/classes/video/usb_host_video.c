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
* $FileName: usb_host_video.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*   This file Contains the implementation of class driver for HID devices.
*
*END************************************************************************/
#include "usb_host_config.h"
#if USBCFG_HOST_VIDEO
#include "usb.h"
#include "usb_host_stack_interface.h"
#include "usb_host_video.h"
#include "usb_host_common.h"
#include "usb_host_ch9.h"

extern usb_status _usb_host_send_setup (usb_host_handle, usb_pipe_handle, tr_struct_t*);
extern usb_host_handle usb_host_dev_mng_get_host(usb_device_instance_handle dev_handle);

extern usb_pipe_handle usb_host_dev_mng_get_control_pipe(usb_device_instance_handle dev_handle);

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_cntrl_callback
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     This is the callback used when HID information is sent or received
*
*END*--------------------------------------------------------------------*/
static void usb_class_video_cntrl_callback
(
    /* [IN] Unused */
    void*     tr_ptr,
    /* [IN] Pointer to the class interface instance */
    void*     param,
    /* [IN] Data buffer */
    uint8_t *   buffer,
    /* [IN] Length of buffer */
    uint32_t     len,
    /* [IN] Error code (if any) */
    usb_status  status
)
{ /* Body */
    usb_video_control_struct_t*      video_control = (usb_video_control_struct_t*)param;

    if (usb_host_release_tr(video_control->host_handle, tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("_usb_host_release_tr failed\n");
#endif
    }
    
    video_control->in_setup = FALSE; 
    if (video_control->ctrl_callback)
    {
        video_control->ctrl_callback(NULL, video_control->ctrl_param, buffer, len, status);
    }
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_cntrl_common
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     This function is used to send a control request
*
*END*--------------------------------------------------------------------*/
static usb_status usb_class_video_cntrl_common
(
    /* [IN] The communication device common command structure */
    video_command_t*         com_ptr,
    /* [IN] Bitmask of the request type */
    uint8_t                  bmrequesttype,
    /* [IN] Request code */
    uint8_t                  brequest,
    /* [IN] Value to copy into wvalue field of the REQUEST */
    uint16_t                 wvalue,
    /* [IN] Value to copy into windex field of the REQUEST */
    uint16_t                 windex,
    /* [IN] Length of the data associated with REQUEST */
    uint16_t                 wlength,
    /* [IN] Pointer to data buffer used to send/recv */
    uint8_t*                 data
)
{ /* Body */
    usb_video_control_struct_t*         video_control = NULL;
    //usb_setup_t                        req;
    usb_status                       status = USB_OK;
    usb_pipe_handle                 pipe_handle;
    tr_struct_t*                    tr_ptr; 
      
    if ((com_ptr == NULL) || (com_ptr->class_control_handle == NULL))
    {
        return USBERR_ERROR;
    }

    video_control = (usb_video_control_struct_t*)com_ptr->class_control_handle;
    if (video_control->in_setup)
    {
        return USBERR_TRANSFER_IN_PROGRESS;
    }

    if (video_control->dev_handle == NULL)
    {
        #ifdef _HOST_DEBUG_
           DEBUG_LOG_TRACE("_usb_hostdev_cntrl_request, invalid device handle");
        #endif
        return USBERR_DEVICE_NOT_FOUND;
    }
    
    video_control->ctrl_callback = com_ptr->callback_fn;
    video_control->ctrl_param = com_ptr->callback_param;
     
    pipe_handle = usb_host_dev_mng_get_control_pipe(video_control->dev_handle);

    if (usb_host_get_tr(video_control->host_handle, usb_class_video_cntrl_callback, video_control, &tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("error to get tr video\n");
#endif
        return USBERR_ERROR;
    }

    /* Set TR buffer length as required */
    if ((REQ_TYPE_IN & bmrequesttype) != 0)
    {
        tr_ptr->rx_buffer = data;
        tr_ptr->rx_length = wlength;
    }
    else
    {
        tr_ptr->tx_buffer = data;
        tr_ptr->tx_length = wlength;
    }
 
    tr_ptr->setup_packet.bmrequesttype = bmrequesttype;
    tr_ptr->setup_packet.brequest      = brequest;
    *(uint16_t*)tr_ptr->setup_packet.wvalue = USB_HOST_TO_LE_SHORT(wvalue);
    *(uint16_t*)tr_ptr->setup_packet.windex = USB_HOST_TO_LE_SHORT(windex);
    *(uint16_t*)tr_ptr->setup_packet.wlength = USB_HOST_TO_LE_SHORT(wlength);

    video_control->in_setup = TRUE;
    status = usb_host_send_setup(video_control->host_handle, pipe_handle, tr_ptr);  
    if (status != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("\nError in usb_class_video_cntrl_common: %x", status);
#endif
        video_control->in_setup = FALSE;
        usb_host_release_tr(video_control->host_handle, tr_ptr);
        return USBERR_ERROR;
    }
    return status;
} /* Endbody */


/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_stream_recv_callback
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*    
*
*END*--------------------------------------------------------------------*/
static void usb_class_video_stream_recv_callback
(
    /* [IN] Unused */
    void*     tr_ptr,
    /* [IN] Pointer to the class interface instance */
    void*     param,
    /* [IN] Data buffer */
    uint8_t *   buffer,
    /* [IN] Length of buffer */
    uint32_t     len,
    /* [IN] Error code (if any) */
    usb_status  status
)
{ /* Body */
    usb_video_stream_struct_t*      video_stream_ptr = (usb_video_stream_struct_t*)param;

    if (usb_host_release_tr(video_stream_ptr->host_handle, tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("_usb_host_release_tr failed\n");
#endif
    }
    
    if (video_stream_ptr->recv_callback)
    {
        video_stream_ptr->recv_callback(NULL, video_stream_ptr->recv_param, buffer, len, status);
    }
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_control_recv_callback
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*    
*
*END*--------------------------------------------------------------------*/
static void usb_class_video_control_recv_callback
(
    /* [IN] Unused */
    void*     tr_ptr,
    /* [IN] Pointer to the class interface instance */
    void*     param,
    /* [IN] Data buffer */
    uint8_t *   buffer,
    /* [IN] Length of buffer */
    uint32_t     len,
    /* [IN] Error code (if any) */
    usb_status  status
)
{ /* Body */
    usb_video_control_struct_t*      video_control_ptr = (usb_video_control_struct_t*)param;

    if (usb_host_release_tr(video_control_ptr->host_handle, tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("_usb_host_release_tr failed\n");
#endif
    }
    
    if (video_control_ptr->recv_callback)
    {
        video_control_ptr->recv_callback(NULL, video_control_ptr->recv_param, buffer, len, status);
    }
} /* Endbody */


/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_stream_recv_data
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     This function is used to recv interrupt data
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_stream_recv_data
(
    /* [IN] Class Interface structure pointer */
    video_command_t*         com_ptr,
    /* [IN] The buffer address */
    uint8_t *               buffer,
    /* [IN] The buffer address */
    uint16_t                length
)
{
    usb_video_stream_struct_t*  video_stream_ptr;
    tr_struct_t*                tr_ptr;
    usb_status                  status;

    if ((com_ptr == NULL) || (com_ptr->class_stream_handle == NULL))
    {
        return USBERR_ERROR;
    }

    video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    
    if ((video_stream_ptr == NULL) || (buffer == NULL))
    {
#ifdef _DEBUG
        USB_PRINTF("input parameter error\n");
#endif
        return USBERR_ERROR;
    }

    video_stream_ptr->recv_callback = com_ptr->callback_fn;
    video_stream_ptr->recv_param = com_ptr->callback_param;

    if (video_stream_ptr->dev_handle == NULL)
    {
        return USBERR_ERROR;
    }

    if (usb_host_get_tr(video_stream_ptr->host_handle, usb_class_video_stream_recv_callback, video_stream_ptr, &tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("error to get tr\n");
#endif
        return USBERR_ERROR;
    }
    
    tr_ptr->rx_buffer = buffer;
    tr_ptr->rx_length = length;
    status = usb_host_recv_data(video_stream_ptr->host_handle, video_stream_ptr->stream_iso_in_pipe, tr_ptr);
    if (status != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("\nError in usb_class_video_recv_data: %x", status);
#endif
        usb_host_release_tr(video_stream_ptr->host_handle, tr_ptr);
        return USBERR_ERROR;
    }
    return USB_OK;
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_control_recv_data
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     This function is used to recv interrupt data
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_control_recv_data
(
    /* [IN] Class Interface structure pointer */
    video_command_t*         com_ptr,
    /* [IN] The buffer address */
    uint8_t *               buffer,
    /* [IN] The buffer address */
    uint16_t                length
)
{
    usb_video_control_struct_t*  video_control_ptr;
    tr_struct_t*             tr_ptr;
    usb_status                status;

    if ((com_ptr == NULL) || (com_ptr->class_control_handle == NULL))
    {
        return USBERR_ERROR;
    }

    video_control_ptr = (usb_video_control_struct_t*)com_ptr->class_control_handle;
    
    if ((video_control_ptr == NULL) || (buffer == NULL))
    {
#ifdef _DEBUG
        USB_PRINTF("input parameter error\n");
#endif
        return USBERR_ERROR;
    }

    video_control_ptr->recv_callback = com_ptr->callback_fn;
    video_control_ptr->recv_param = com_ptr->callback_param;

    if (video_control_ptr->dev_handle == NULL)
    {
        return USBERR_ERROR;
    }

    if (usb_host_get_tr(video_control_ptr->host_handle, usb_class_video_control_recv_callback, video_control_ptr, &tr_ptr) != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("error to get tr\n");
#endif
        return USBERR_ERROR;
    }
    
    tr_ptr->rx_buffer = buffer;
    tr_ptr->rx_length = length;
    status = usb_host_recv_data(video_control_ptr->host_handle, video_control_ptr->control_interrupt_in_pipe, tr_ptr);
    if (status != USB_OK)
    {
#ifdef _DEBUG
        USB_PRINTF("\nError in usb_class_video_recv_data: %x", status);
#endif
        usb_host_release_tr(video_control_ptr->host_handle, tr_ptr);
        return USBERR_ERROR;
    }
    return USB_OK;
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_get_probe
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     Reads the active protocol (boot protocol or report protocol)
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_get_probe
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if(NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }
    if(NULL == video_stream_ptr->current_stream_interface_ptr)
    {
        return USBERR_NO_INTERFACE;
    }
    
    return usb_class_video_cntrl_common(com_ptr,
        REQ_TYPE_IN | REQ_TYPE_CLASS | REQ_TYPE_INTERFACE,
        brequest, VS_PROBE_CONTROL<<8, video_stream_ptr->current_stream_interface_ptr->bInterfaceNumber, 26, probe);
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_set_probe
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     Reads the active protocol (boot protocol or report protocol)
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_set_probe
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if(NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }
    if(NULL == video_stream_ptr->current_stream_interface_ptr)
    {
        return USBERR_NO_INTERFACE;
    }
    if(brequest != SET_CUR)
    {
        return USBERR_INVALID_REQ_TYPE;
    }
    
    return usb_class_video_cntrl_common(com_ptr,
        REQ_TYPE_OUT | REQ_TYPE_CLASS | REQ_TYPE_INTERFACE,
        brequest, VS_PROBE_CONTROL<<8, video_stream_ptr->current_stream_interface_ptr->bInterfaceNumber, 26, probe);
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_get_commit
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     Reads the active protocol (boot protocol or report protocol)
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_get_commit
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if (NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }
    if (NULL == video_stream_ptr->current_stream_interface_ptr)
    {
        return USBERR_NO_INTERFACE;
    }

    if ((brequest == GET_DEF) && (brequest == GET_RES) && (brequest == GET_MIN) && (brequest == GET_MAX))
    {
        return USBERR_INVALID_REQ_TYPE;
    }
    
    return usb_class_video_cntrl_common(com_ptr,
        REQ_TYPE_IN | REQ_TYPE_CLASS | REQ_TYPE_INTERFACE,
        brequest, VS_COMMIT_CONTROL<<8, video_stream_ptr->current_stream_interface_ptr->bInterfaceNumber, 26, probe);
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_class_video_set_commit
* Returned Value : USB_OK if command has been passed on USB.
* Comments       :
*     Reads the active protocol (boot protocol or report protocol)
*
*END*--------------------------------------------------------------------*/
usb_status usb_class_video_set_commit
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
)
{
    usb_video_stream_struct_t * video_stream_ptr = (usb_video_stream_struct_t*)com_ptr->class_stream_handle;
    if(NULL == video_stream_ptr)
    {
        return USBERR_ERROR;
    }
    if(NULL == video_stream_ptr->current_stream_interface_ptr)
    {
        return USBERR_NO_INTERFACE;
    }
    if(brequest != SET_CUR)
    {
        return USBERR_INVALID_REQ_TYPE;
    }
    
    return usb_class_video_cntrl_common(com_ptr,
        REQ_TYPE_OUT | REQ_TYPE_CLASS | REQ_TYPE_INTERFACE,
        brequest, VS_COMMIT_CONTROL<<8, video_stream_ptr->current_stream_interface_ptr->bInterfaceNumber, 26, probe);
}

#endif
