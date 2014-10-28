/**HEADER********************************************************************
*
* Copyright (c) 2008-2014 Freescale Semiconductor;
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
* $FileName: ehci_main.c$
* $Version : 3.8.43.0$
* $Date    : Oct-4-2012$
*
* Comments:
*
*   This file contains the main low-level Host API functions specific to
*   the VUSB chip.
*
*END************************************************************************/
#include "usb_host_config.h"
#if USBCFG_HOST_EHCI
#include "usb.h"
#include "usb_host_stack_interface.h"
#include "usb_host_common.h"
#include "usb_host.h"
#include "ehci_prv.h"
#include "fsl_usb_ehci_hal.h"

#define USBCFG_EHCI_HOST_ENABLE_TASK  1

#if USBCFG_EHCI_HOST_ENABLE_TASK

// EHCI event bits
#define EHCI_EVENT_ATTACH       0x01
#define EHCI_EVENT_RESET        0x02
#define EHCI_EVENT_TOK_DONE     0x04
#define EHCI_EVENT_SOF_TOK      0x08
#define EHCI_EVENT_DETACH       0x10
#define EHCI_EVENT_SYS_ERROR    0x20

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)             /* USB stack running on MQX */
#define USB_EHCI_HOST_TASK_ADDRESS                    _usb_ehci_host_task_stun

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)            /* USB stack running on BM  */
#define USB_EHCI_HOST_TASK_ADDRESS                    _usb_ehci_host_task

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)           /* USB stack running on uCOS */
#if (FSL_RTOS_SELECTED == FSL_RTOS_NONE)
#define USB_EHCI_HOST_TASK_ADDRESS                    _usb_ehci_host_task
#else
#define USB_EHCI_HOST_TASK_ADDRESS                    _usb_ehci_host_task_stun
#endif
#endif

#define USB_EHCI_HOST_TASK_PRIORITY                    (6)
#define USB_EHCI_HOST_TASK_STACKSIZE                   1600
#define USB_EHCI_HOST_TASK_NAME                        "EHCI HOST Task"

#if ((OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK))             /* USB stack running on MQX */
#define USB_NONBLOCKING_MODE 0

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)        /* USB stack running on BM  */
#define USB_NONBLOCKING_MODE 1
#endif

#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)

#if defined( __ICCARM__ )
    #pragma data_alignment=4096
    __no_init static uint8_t usbhs_perodic_list[4096];
#elif defined (__CC_ARM) || defined(__GNUC__)
    __attribute__((aligned(4096))) static uint8_t usbhs_perodic_list[4096];
#else
    #error Unsupported compiler, please use IAR, Keil or arm gcc compiler and rebuild the project.
#endif

#endif



#ifdef __USB_OTG__
#include "otgapi.h"
   extern USB_OTG_STATE_STRUCT_PTR usb_otg_state_struct_ptr;
#endif

usb_instance_t g_usb_instance_echi =    
{
    .name = (uint8_t*)"ehci_usb0",
    .instance = 0,
};

#if USBCFG_EHCI_HS_DISCONNECT_ENABLE
extern void bsp_usb_hs_disconnect_detection_disable(uint8_t controller_id);
extern void bsp_usb_hs_disconnect_detection_enable(uint8_t controller_id);
#endif

static usb_status _usb_ehci_commit_bandwidth_slots(usb_host_handle, ehci_pipe_struct_t*, uint32_t, uint32_t, uint8_t *, uint32_t);
static usb_status _usb_ehci_commit_high_speed_bandwidth(usb_host_handle, ehci_pipe_struct_t*);
static usb_status _usb_ehci_commit_split_bandwidth(usb_host_handle, ehci_pipe_struct_t*);

static void link_interrupt_qh_to_periodiclist(usb_host_handle, ehci_qh_struct_t*, ehci_pipe_struct_t*, uint32_t);
static void unlink_interrupt_qh_from_periodiclist(usb_host_handle, ehci_qh_struct_t*, ehci_pipe_struct_t*, uint32_t);


static usb_status allocate_ss_cs_bandwidth(usb_host_handle, ehci_pipe_struct_t*, uint32_t, uint32_t, uint32_t);

static usb_status split_combinations(usb_host_handle, ehci_pipe_struct_t*, uint8_t [], int, int, int);

void _usb_ehci_get_qtd(usb_host_handle, ehci_qtd_struct_t**);

void _usb_ehci_free_qtd(usb_host_handle, ehci_qtd_struct_t*);

void _usb_ehci_get_qh(usb_host_handle, ehci_qh_struct_t**);

void _usb_ehci_free_qh(usb_host_handle, ehci_qh_struct_t*);

/**************************************************************************
 Local Functions. Note intended to be called from outside this file
**************************************************************************/

const struct usb_host_api_functions_struct _usb_ehci_host_api_table =
{
    /* The Host/Device init function */
    _usb_ehci_preinit,
 
    /* The Host/Device init function */
    _usb_ehci_init,
 
    /* The function to shutdown the host/device */
    _usb_ehci_shutdown,
 
    /* The function to send data */
    _usb_ehci_send_data,
 
    /* The function to send setup data */
    _usb_ehci_send_setup,
 
    /* The function to receive data */
    _usb_ehci_recv_data,
 
    /* The function to cancel the transfer */
    _usb_ehci_cancel_transfer,
 
    /* The function for USB bus control */
    _usb_ehci_bus_control,
 
    //_usb_ehci_allocate_bandwidth,
    NULL,
 
    //_usb_ehci_free_resources,
    NULL,
 
    _usb_ehci_get_frame_number,
 
    _usb_ehci_get_micro_frame_number,
 
    /* The function to open a pipe */
    _usb_ehci_open_pipe,
 
    /* The function to update the maximum packet size */
    _usb_ehci_close_pipe,
 
    /* The function to update the device address */
    NULL,
 
    _usb_ehci_update_dev_address
};


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_send_data
*  Returned Value : None
*  Comments       :
*        Send the data
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_send_data
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle,

      /* [IN] The pipe descriptor to queue */
      pipe_struct_t*               pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                 current_pipe_tr_struct_ptr
   )
{
    usb_status status;
    status = _usb_ehci_queue_pkts(handle, (ehci_pipe_struct_t*) pipe_descr_ptr, (tr_struct_t*) current_pipe_tr_struct_ptr);
    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_send_setup
*  Returned Value : usb_status
*  Comments       :
*        Send Setup packet
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_send_setup
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle,

      /* [IN] The pipe descriptor to queue */
      pipe_struct_t*               pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                 current_pipe_tr_struct_ptr
   )
{
    usb_status status;
    status = _usb_ehci_queue_pkts(handle, (ehci_pipe_struct_t*) pipe_descr_ptr, (tr_struct_t*) current_pipe_tr_struct_ptr);
    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_recv_data
*  Returned Value : None
*  Comments       :
*        Receive data
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_recv_data
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle,

      /* [IN] The pipe descriptor to queue */
      pipe_struct_t*               pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                 current_pipe_tr_struct_ptr
   )
{
    usb_status status;
    status = _usb_ehci_queue_pkts(handle, (ehci_pipe_struct_t*) pipe_descr_ptr, (tr_struct_t*) current_pipe_tr_struct_ptr);
    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_queue_pkts
*  Returned Value : status of the transfer queuing.
*  Comments       :
*        Queue the packet in the hardware
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_queue_pkts
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*            pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                    current_pipe_tr_struct_ptr
   )
{
    usb_status  status;
    if (pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE )
    {
        current_pipe_tr_struct_ptr->setup_first_phase = 0;
    }

    switch (pipe_descr_ptr->common.pipetype)
    {
#if USBCFG_EHCI_MAX_ITD_DESCRS || USBCFG_EHCI_MAX_SITD_DESCRS
        case USB_ISOCHRONOUS_PIPE:
            status = _usb_ehci_add_isochronous_xfer_to_periodic_schedule_list(handle, pipe_descr_ptr, current_pipe_tr_struct_ptr);
            break;
#endif

#ifdef USBCFG_EHCI_MAX_QTD_DESCRS
        case USB_INTERRUPT_PIPE:
            status = _usb_ehci_add_interrupt_xfer_to_periodic_list(handle, pipe_descr_ptr, current_pipe_tr_struct_ptr);
            break;
        case USB_CONTROL_PIPE:
        case USB_BULK_PIPE:
            status = _usb_ehci_add_xfer_to_asynch_schedule_list(handle, pipe_descr_ptr, current_pipe_tr_struct_ptr);
            break;
#endif
        default:
            status = USB_STATUS_ERROR;
            break;
    }

    return status;
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_update_active_async
*  Returned Value : None
*  Comments       :
*        Link the qtd to the qh for interrupt/Bulk/Control transfers.
*END*-----------------------------------------------------------------*/
void _usb_ehci_update_active_async
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle
   )
{
    usb_ehci_host_state_struct_t*     usb_host_ptr;
    ehci_qh_struct_t*                 qh_ptr;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
    qh_ptr       = usb_host_ptr->active_async_list_ptr;

    if (qh_ptr == NULL)
    {
        EHCI_MEM_WRITE(usb_host_ptr->dummy_qh->horiz_link_ptr, (((uint32_t)usb_host_ptr->dummy_qh & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
    }
    else
    {
        EHCI_MEM_WRITE(qh_ptr->horiz_link_ptr, (( (uint32_t)usb_host_ptr->dummy_qh->horiz_link_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK)  | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        EHCI_MEM_WRITE(usb_host_ptr->dummy_qh->horiz_link_ptr, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
    }
    
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_link_qtd_to_qh
*  Returned Value : None
*  Comments       :
*        Link the qtd to the qh for interrupt/Bulk/Control transfers.
*END*-----------------------------------------------------------------*/
void _usb_ehci_link_qtd_to_qh
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      /* [IN] the queue head to initialize */
      ehci_qh_struct_t*               qh_ptr,

      /* [IN] the first QTD to queue for this Queue head */
      ehci_qtd_struct_t*              qtd_ptr
   )
{
    ehci_qtd_struct_t*     temp_qtd_ptr;
    temp_qtd_ptr = (ehci_qtd_struct_t*)qh_ptr->qtd_head;
  
    if (( ((uint32_t)temp_qtd_ptr) & EHCI_QTD_T_BIT) || (temp_qtd_ptr == NULL))
    {
        qh_ptr->qtd_head = (void*)qtd_ptr;
    }
    else
    {
        while ((temp_qtd_ptr != NULL) && (!(usb_hal_ehci_get_next_qtd_ptr(temp_qtd_ptr) & EHCI_QTD_T_BIT)))
        {
            temp_qtd_ptr = (ehci_qtd_struct_t*)usb_hal_ehci_get_next_qtd_ptr(temp_qtd_ptr);
        }
    
        //EHCI_MEM_WRITE(temp_qtd_ptr->next_qtd_ptr, (uint32_t)qtd_ptr);
        usb_hal_ehci_link_qtd(temp_qtd_ptr,(uint32_t)qtd_ptr);
        temp_qtd_ptr->next = (void *)qtd_ptr;
    
    }
   
    if (usb_hal_ehci_get_next_qtd_link_ptr(qh_ptr)& EHCI_QTD_T_BIT)
    {
        //EHCI_MEM_WRITE(qh_ptr->alt_next_qtd_link_ptr, EHCI_QTD_T_BIT);
        usb_hal_ehci_set_qh_next_alt_qtd_link_terminate(qh_ptr);
        //EHCI_MEM_WRITE(qh_ptr->next_qtd_link_ptr, (uint32_t)qtd_ptr);
        usb_hal_ehci_set_qh_next_qtd_link_ptr(qh_ptr, (uint32_t)qtd_ptr);
    }
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_link_qh_to_active_list
*  Returned Value : None
*  Comments       :
*        Link the qtd to the qh for interrupt/Bulk/Control transfers.
*END*-----------------------------------------------------------------*/
void _usb_ehci_link_qh_to_async_active_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      /* [IN] the queue head to initialize */
      ehci_qh_struct_t*               qh_ptr
   )
{
    usb_ehci_host_state_struct_t*     usb_host_ptr;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    if (usb_host_ptr->active_async_list_ptr == NULL)
    {
        usb_host_ptr->active_async_list_ptr = qh_ptr;
        usb_host_ptr->active_async_list_tail_ptr = qh_ptr;
        //EHCI_MEM_WRITE(qh_ptr->horiz_link_ptr, (( (uint32_t)usb_host_ptr->dummy_qh->horiz_link_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK)  | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_hal_ehci_set_qh_horiz_link_ptr(qh_ptr, (( (uint32_t)usb_host_ptr->dummy_qh->horiz_link_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK)  | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        //EHCI_MEM_WRITE(usb_host_ptr->dummy_qh->horiz_link_ptr, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_hal_ehci_set_qh_horiz_link_ptr(usb_host_ptr->dummy_qh, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        //_usb_ehci_update_active_async(handle);
    }
    else
    {
        //EHCI_MEM_WRITE(qh_ptr->horiz_link_ptr, (( (uint32_t)usb_host_ptr->active_async_list_tail_ptr->horiz_link_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK)  | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_hal_ehci_set_qh_horiz_link_ptr(qh_ptr, (( (uint32_t)usb_host_ptr->active_async_list_tail_ptr->horiz_link_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK)  | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        //EHCI_MEM_WRITE(usb_host_ptr->active_async_list_tail_ptr->horiz_link_ptr, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_hal_ehci_set_qh_horiz_link_ptr(usb_host_ptr->active_async_list_tail_ptr, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_host_ptr->active_async_list_tail_ptr->next = (void*)qh_ptr;
        usb_host_ptr->active_async_list_tail_ptr = qh_ptr;
    }
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_unlink_qh_from_active_list
*  Returned Value : None
*  Comments       :
*        Link the qtd to the qh for interrupt/Bulk/Control transfers.
*END*-----------------------------------------------------------------*/
void _usb_ehci_unlink_qh_from_async_active_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      /* [IN] the queue head to initialize */
      ehci_qh_struct_t*               qh_ptr
   )
{
    usb_ehci_host_state_struct_t*     usb_host_ptr;
    ehci_qh_struct_t*                 cur_qh;
    ehci_qh_struct_t*                 prev_qh;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    if (usb_host_ptr->active_async_list_ptr == qh_ptr)
    {
        //EHCI_MEM_WRITE(usb_host_ptr->dummy_qh->horiz_link_ptr, EHCI_MEM_READ((uint32_t)qh_ptr->horiz_link_ptr));
        usb_hal_ehci_set_qh_horiz_link_ptr(usb_host_ptr->dummy_qh, usb_hal_ehci_get_qh_horiz_link_ptr(qh_ptr));
    usb_host_ptr->active_async_list_ptr = qh_ptr->next;
        //_usb_ehci_update_active_async(handle);
        if (usb_host_ptr->active_async_list_tail_ptr == qh_ptr)
        {
            usb_host_ptr->active_async_list_ptr = NULL;
            usb_host_ptr->active_async_list_tail_ptr = NULL;
        }
    }
    else
    {
        cur_qh = prev_qh = usb_host_ptr->active_async_list_ptr;

        while ((cur_qh != NULL) && (cur_qh != qh_ptr))
        {
            prev_qh = cur_qh;
            cur_qh = cur_qh->next;
        }

        if(cur_qh == qh_ptr)
        {
            prev_qh->next = (void*)qh_ptr->next;
            //EHCI_MEM_WRITE(prev_qh->horiz_link_ptr, EHCI_MEM_READ((uint32_t)qh_ptr->horiz_link_ptr));
            usb_hal_ehci_set_qh_horiz_link_ptr(prev_qh, usb_hal_ehci_get_qh_horiz_link_ptr(qh_ptr));
            if (usb_host_ptr->active_async_list_tail_ptr == qh_ptr)
            {
                usb_host_ptr->active_async_list_tail_ptr = prev_qh;
            }
        }
    }
}




/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_init_Q_HEAD
*  Returned Value : None
*  Comments       :
*        Initialize the Queue Head. This routine initializes a queue head
*  for interrupt/Bulk/Control transfers.
*END*-----------------------------------------------------------------*/
void _usb_ehci_init_qh
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*             pipe_descr_ptr,

      /* [IN] the queue head to initialize */
      ehci_qh_struct_t*               qh_ptr
   )
{
    uint32_t                       control_ep_flag = 0, split_completion_mask = pipe_descr_ptr->complete_split;
    uint32_t                       interrupt_schedule_mask = pipe_descr_ptr->start_split;
    uint32_t                       data_toggle_control = 0;
    uint32_t                       temp;
    uint32_t                       h_bit = 0;
    uint8_t                        mult = 1;
    uint8_t                        speed;
    uint8_t                        address;
    uint8_t                        hub_no;
    uint8_t                        port_no;

    /******************************************************************************
     PREPARE THE BASIC FIELDS OF A QUEUE HEAD DEPENDING UPON THE PIPE TYPE
    *******************************************************************************/
    OS_Mem_zero((void *)qh_ptr, sizeof(ehci_qh_struct_t));

    speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance); 
    switch (pipe_descr_ptr->common.pipetype)
    {
        case USB_CONTROL_PIPE:
            if (speed != USB_SPEED_HIGH)
            {
                control_ep_flag = 1;
            }
            data_toggle_control = 1; /* Data toggle in qTD */
            break;
  
        case USB_INTERRUPT_PIPE:
            data_toggle_control = 0; /* Data toggle in QH */
            break;
  
        case USB_BULK_PIPE:
            data_toggle_control = 0; /* Data toggle in QH */
            break;
    }
 
    if (speed == USB_SPEED_HIGH)
    {
        mult = pipe_descr_ptr->common.trs_per_uframe;
    }

    usb_hal_ehci_init_qh(qh_ptr);

    address = (usb_host_dev_mng_get_address(pipe_descr_ptr->common.dev_instance)) & 0x7F; 
    if (usb_host_dev_mng_get_hub_speed(pipe_descr_ptr->common.dev_instance) != USB_SPEED_HIGH)
    {
        hub_no  = (usb_host_dev_mng_get_hs_hub_no(pipe_descr_ptr->common.dev_instance)) & 0x7F;
        port_no = (usb_host_dev_mng_get_hs_port_no(pipe_descr_ptr->common.dev_instance)) & 0x7F;
    }
    else
    {
        hub_no  = (usb_host_dev_mng_get_hubno(pipe_descr_ptr->common.dev_instance)) & 0x7F;
        port_no = (usb_host_dev_mng_get_portno(pipe_descr_ptr->common.dev_instance)) & 0x7F;
    }
    /* Initialize the endpoint capabilities registers */
    temp = (
           ((uint32_t)pipe_descr_ptr->common.nak_count << EHCI_QH_NAK_COUNT_RL_BITS_POS) |
           (control_ep_flag << EHCI_QH_EP_CTRL_FLAG_BIT_POS) |
           (pipe_descr_ptr->common.max_packet_size << EHCI_QH_MAX_PKT_SIZE_BITS_POS) |
           (h_bit << EHCI_QH_HEAD_RECLAMATION_BIT_POS) |
           (data_toggle_control << EHCI_QH_DTC_BIT_POS) |
           ((speed & 0x3) << EHCI_QH_SPEED_BITS_POS) |
           ((uint32_t)pipe_descr_ptr->common.endpoint_number << EHCI_QH_EP_NUM_BITS_POS) |
           address
           );
    //EHCI_MEM_WRITE(qh_ptr->ep_capab_charac1, temp);
    usb_hal_ehci_set_ep_capab_charac1(qh_ptr, temp);
 
    temp = (
           (mult << EHCI_QH_HIGH_BW_MULT_BIT_POS) |
           (port_no << EHCI_QH_HUB_PORT_NUM_BITS_POS) |
           (hub_no << EHCI_QH_HUB_ADDR_BITS_POS) |
           (split_completion_mask << EHCI_QH_SPLIT_COMPLETION_MASK_BITS_POS) |
           interrupt_schedule_mask
           );
    //EHCI_MEM_WRITE(qh_ptr->ep_capab_charac2,temp);
    usb_hal_ehci_set_ep_capab_charac2(qh_ptr, temp);

    //EHCI_MEM_WRITE(qh_ptr->next_qtd_link_ptr, EHCI_QTD_T_BIT);
    usb_hal_ehci_set_qh_next_qtd_link_terminate(qh_ptr);
    //EHCI_MEM_WRITE(qh_ptr->horiz_link_ptr, EHCI_QUEUE_HEAD_POINTER_T_BIT);
    usb_hal_ehci_set_qh_horiz_link_ptr_head_pointer_terminate(qh_ptr);

    return;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_init_QTD
*  Returned Value : None
*  Comments       :
*        Initialize the QTD
*END*-----------------------------------------------------------------*/
void _usb_ehci_init_qtd
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      /* [IN] the address of the QTD to initialize */
      ehci_qtd_struct_t*              qtd_ptr,

      /* The buffer start address for the QTD */
      uint8_t*                         buffer_start_address,

      /* The token value */
      uint32_t                         token
   )
{
    /* Zero the QTD. Leave the scratch pointer */
    OS_Mem_zero((void *)qtd_ptr, sizeof(ehci_qtd_struct_t));

    /* Set the Terminate bit */
    usb_hal_ehci_set_qtd_terminate_bit(qtd_ptr);

 
    /* Set the Terminate bit */
    usb_hal_ehci_set_alt_next_qtd_terminate_bit(qtd_ptr);
    
    usb_hal_ehci_set_qtd_buffer_page_pointer(qtd_ptr, (uint32_t)buffer_start_address);   
    usb_hal_ehci_set_qtd_token(qtd_ptr,token);
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_init_Q_element
*  Returned Value : None
*  Comments       :
*        Initialize the Queue Element descriptor(s)
*END*-----------------------------------------------------------------*/
uint32_t _usb_ehci_init_qtd_link
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*            pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                    current_pipe_tr_struct_ptr
   )
{
    ehci_qtd_struct_t*        qtd_ptr;
    ehci_qtd_struct_t*        prev_qtd_ptr;
    ehci_qtd_struct_t*        first_qtd_ptr = NULL;
    uint32_t                  pid_code;
    uint32_t                  next_pidcode;
    uint32_t                  data_toggle, token;
    int32_t                   total_length, qtd_length;
    unsigned char *           buff_ptr;

    _usb_ehci_get_qtd(handle, &qtd_ptr);
    if (!qtd_ptr)
    {
        goto ERR;
    }

    data_toggle = (uint32_t)((pipe_descr_ptr->common.nextdata01 << EHCI_QTD_DATA_TOGGLE_BIT_POS));

    prev_qtd_ptr = first_qtd_ptr = qtd_ptr;
    current_pipe_tr_struct_ptr->hw_transaction_head = (void*)qtd_ptr;

    if (pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE)
    {
        pid_code = EHCI_QTD_PID_SETUP;
        token = (((uint32_t)8 << EHCI_QTD_LENGTH_BIT_POS) | data_toggle | pid_code | EHCI_QTD_STATUS_ACTIVE);

        /* Initialize a QTD for Setup phase */
        _usb_ehci_init_qtd(handle, qtd_ptr, (uint8_t *)&current_pipe_tr_struct_ptr->setup_packet, token);
        qtd_ptr->length = 8;
        qtd_ptr->pipe   = (void *)pipe_descr_ptr;
        qtd_ptr->tr     = (void *)current_pipe_tr_struct_ptr;

        /* Get a QTD from the queue for the data phase or a status phase */
        _usb_ehci_get_qtd(handle, &qtd_ptr);
        if (!qtd_ptr)
        {
            goto ERR;
        }

        /* The data phase QTD chained to the Setup QTD */
        //EHCI_MEM_WRITE(prev_qtd_ptr->next_qtd_ptr, (uint32_t)qtd_ptr);
        usb_hal_ehci_link_qtd(prev_qtd_ptr, (uint32_t)qtd_ptr);
        prev_qtd_ptr->next = (void*)qtd_ptr;
        data_toggle = (uint32_t)(EHCI_QTD_DATA_TOGGLE);

        if (current_pipe_tr_struct_ptr->send_phase)
        {
            total_length = current_pipe_tr_struct_ptr->tx_length;
            buff_ptr = current_pipe_tr_struct_ptr->tx_buffer;
            pid_code = EHCI_QTD_PID_OUT;
            next_pidcode = EHCI_QTD_PID_IN;
        }
        else
        {
            total_length = current_pipe_tr_struct_ptr->rx_length;
            buff_ptr = current_pipe_tr_struct_ptr->rx_buffer;
            pid_code = EHCI_QTD_PID_IN;
            next_pidcode = EHCI_QTD_PID_OUT;
        }

        if (total_length)
        {
            token = ((total_length << EHCI_QTD_LENGTH_BIT_POS) | data_toggle | pid_code | EHCI_QTD_STATUS_ACTIVE);
            _usb_ehci_init_qtd(handle, qtd_ptr, buff_ptr, token);
            qtd_ptr->length = total_length;
            qtd_ptr->pipe   = (void *)pipe_descr_ptr;
            qtd_ptr->tr     = (void *)current_pipe_tr_struct_ptr;

            /* Use the QTD to chain the next QTD */
            prev_qtd_ptr = qtd_ptr;
    
            /* Get a QTD from the queue for status phase */
            _usb_ehci_get_qtd(handle, &qtd_ptr);
            if (!qtd_ptr)
            {
                goto ERR;
            }
    
            /* Chain the status phase QTD to the data phase QTD */
            //EHCI_MEM_WRITE(prev_qtd_ptr->next_qtd_ptr,(uint32_t)qtd_ptr);
        usb_hal_ehci_link_qtd(prev_qtd_ptr, (uint32_t)qtd_ptr);
            prev_qtd_ptr->next = (void*)qtd_ptr;
        }
        
        /* Zero length IN */
        /* Initialize the QTD for the status phase -- Zero length opposite
             ** direction packet
             */
        token = ((0 << EHCI_QTD_LENGTH_BIT_POS) | (EHCI_QTD_DATA_TOGGLE) | EHCI_QTD_IOC | next_pidcode | EHCI_QTD_STATUS_ACTIVE);
        _usb_ehci_init_qtd(handle, qtd_ptr, NULL, token);
        qtd_ptr->length = 0;
        qtd_ptr->pipe   = (void *)pipe_descr_ptr;
        qtd_ptr->tr     = (void *)current_pipe_tr_struct_ptr;
        
    }
    else
    {
        if (pipe_descr_ptr->common.direction)
        {
            total_length = current_pipe_tr_struct_ptr->tx_length;
            buff_ptr = current_pipe_tr_struct_ptr->tx_buffer;
            pid_code = EHCI_QTD_PID_OUT;
        }
        else
        {
            total_length = current_pipe_tr_struct_ptr->rx_length;
            buff_ptr = current_pipe_tr_struct_ptr->rx_buffer;
            pid_code = EHCI_QTD_PID_IN;
        }

        do
        {
            if (total_length > VUSB_EP_MAX_LENGTH_TRANSFER)
            {
                /* Split OUT transaction to more shorter transactions */
                token = ((VUSB_EP_MAX_LENGTH_TRANSFER  << EHCI_QTD_LENGTH_BIT_POS) | 
                         data_toggle | pid_code | EHCI_QTD_DEFAULT_CERR_VALUE |
                         EHCI_QTD_STATUS_ACTIVE);
                qtd_length = VUSB_EP_MAX_LENGTH_TRANSFER;
                total_length -= VUSB_EP_MAX_LENGTH_TRANSFER;
            }
            else
            {
                token = ((total_length << EHCI_QTD_LENGTH_BIT_POS) | data_toggle |
                         EHCI_QTD_IOC | pid_code | EHCI_QTD_DEFAULT_CERR_VALUE |
                         EHCI_QTD_STATUS_ACTIVE);
                qtd_length = total_length;
                total_length = 0;
            }

            /* Initialize the QTD for the OUT data transaction */
            _usb_ehci_init_qtd(handle, qtd_ptr, buff_ptr, token);
            qtd_ptr->length = qtd_length;
            qtd_ptr->pipe   = (void *)pipe_descr_ptr;
            qtd_ptr->tr     = (void *)current_pipe_tr_struct_ptr;

            buff_ptr += qtd_length;

            if (total_length)
            {
                /* Use the QTD to chain the next QTD */
                prev_qtd_ptr = qtd_ptr;
    
                /* Get a QTD from the queue for status phase */
                _usb_ehci_get_qtd(handle, &qtd_ptr);
                if (!qtd_ptr)
                {
                    return USB_STATUS_TRANSFER_IN_PROGRESS;
                }
    
                /* Chain the status phase QTD to the data phase QTD */
                //EHCI_MEM_WRITE(prev_qtd_ptr->next_qtd_ptr,(uint32_t) qtd_ptr);
                usb_hal_ehci_link_qtd(prev_qtd_ptr, (uint32_t)qtd_ptr);
                prev_qtd_ptr->next = (void*)qtd_ptr;
            }

        } while (total_length);
    }



    return ((uint32_t)first_qtd_ptr);
ERR:
    while (first_qtd_ptr != NULL)
    {
        qtd_ptr = first_qtd_ptr;
        first_qtd_ptr = (ehci_qtd_struct_t*)(usb_hal_ehci_get_next_qtd_ptr(first_qtd_ptr)& USBHS_TD_ADDR_MASK);
        _usb_ehci_free_qtd(handle, qtd_ptr);
    }
    return USBERR_ERROR;
} /* EndBody */


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_add_xfer_to_asynch_schedule_list
*  Returned Value : USB Status
*  Comments       :
*        Queue the packet in the EHCI hardware Asynchronous schedule list
*END*-----------------------------------------------------------------*/
uint32_t _usb_ehci_add_xfer_to_asynch_schedule_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*            pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                    current_pipe_tr_struct_ptr
   )
{
    usb_ehci_host_state_struct_t*               usb_host_ptr;
    ehci_qh_struct_t*                           qh_ptr=NULL;
    ehci_qtd_struct_t*                          first_qtd_ptr;
    bool                                        init_async_list = FALSE;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    first_qtd_ptr = (ehci_qtd_struct_t*)_usb_ehci_init_qtd_link(handle, pipe_descr_ptr, current_pipe_tr_struct_ptr);

    current_pipe_tr_struct_ptr->transfered_length = 0; /* nothing was transferred yet */

    /* If the Asynch Schedule is disabled then initialize a new list */
    if (((!(usb_hal_ehci_get_usb_cmd(usb_host_ptr->usbRegBase)  & EHCI_USBCMD_ASYNC_SCHED_ENABLE)) &&
          (!(usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_ASYNCH_SCHEDULE))
         ) || (!usb_host_ptr->active_async_list_ptr))
    {
        init_async_list = TRUE;
    }

    qh_ptr = pipe_descr_ptr->qh_for_this_pipe;
    
    if (!pipe_descr_ptr->actived)
    {
        _usb_ehci_init_qh(handle, pipe_descr_ptr, qh_ptr);

        qh_ptr->pipe     = (void*)pipe_descr_ptr;
        qh_ptr->next     = NULL;

        _usb_ehci_link_qtd_to_qh(handle, qh_ptr, first_qtd_ptr);
        
        _usb_ehci_link_qh_to_async_active_list(handle, qh_ptr);

        pipe_descr_ptr->actived = 1;
    }
    else
    {
        usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);;
        while (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_ASYNCH_SCHEDULE)
        {
        }

        _usb_ehci_link_qtd_to_qh(handle, qh_ptr, first_qtd_ptr);

        /* Enable the Asynchronous schedule */
        //EHCI_REG_SET_BITS(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_CMD,EHCI_USBCMD_ASYNC_SCHED_ENABLE);
    usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
    }

    if (init_async_list)
    {
        /* Write the QH address to the Current Async List Address */
        //EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.CURR_ASYNC_LIST_ADDR, (uint32_t)usb_host_ptr->dummy_qh);
        usb_hal_ehci_set_qh_to_curr_async_list(usb_host_ptr->usbRegBase, (uint32_t)usb_host_ptr->dummy_qh);
    /* Enable the Asynchronous schedule */
        //EHCI_REG_SET_BITS(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_CMD,EHCI_USBCMD_ASYNC_SCHED_ENABLE);
    usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
    } 
 
    return USB_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_add_interrupt_xfer_to_periodic_list
*  Returned Value : None
*  Comments       :
*        Queue the transfer in the EHCI hardware Periodic schedule list
*END*-----------------------------------------------------------------*/
uint32_t _usb_ehci_add_interrupt_xfer_to_periodic_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*           pipe_descr_ptr,

      /* [IN] the transfer parameters struct */
      tr_struct_t*                   pipe_tr_ptr
   )
{
    usb_ehci_host_state_struct_t*   usb_host_ptr;
    ehci_qh_struct_t*               qh_ptr = NULL;
    ehci_qtd_struct_t*              first_qtd_ptr;
    uint32_t                        cmd_val,sts_val;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /* Initialize the QTDs for the Queue Head */
    first_qtd_ptr = (ehci_qtd_struct_t*)_usb_ehci_init_qtd_link(handle, pipe_descr_ptr, pipe_tr_ptr);

#ifdef DEBUG_INFO
      {
          uint32_t token = usb_hal_ehci_get_qtd_token(first_qtd_ptr);

          USB_PRINTF("QTD queued Top QTD Token=%x\n"
            "   Status=%x,PID code=%x,error code=%x,page=%x,IOC=%x,Bytes=%x,Toggle=%x\n",
          token
          (token&0xFF),
          (token >> 8)&0x3,
          (token >> 10) &0x3,
          (token >> 12)&0x7,
          (token >> 15)&0x1,
          (token >> 16)&0x7FFF,
          (token&EHCI_QTD_DATA_TOGGLE) >>31);
      }
#endif
    qh_ptr = (ehci_qh_struct_t*) pipe_descr_ptr->qh_for_this_pipe;

    if (!pipe_descr_ptr->actived)
    {
        qh_ptr->pipe     = (void*)pipe_descr_ptr;
        //qh_ptr->qtd_head = (void*)first_qtd_ptr;
      //  qh_ptr->next     = NULL;
     //  qh_ptr->interval = pipe_descr_ptr->common.interval;
        
        /*_usb_ehci_init_qh(handle, pipe_descr_ptr, qh_ptr, 0);*/

        _usb_ehci_link_qtd_to_qh(handle, qh_ptr, first_qtd_ptr);
        
        //_usb_ehci_link_qh_to_active_list(handle, qh_ptr, 1);

        pipe_descr_ptr->actived = 1;
    }
    else
    {
        _usb_ehci_link_qtd_to_qh(handle, qh_ptr, first_qtd_ptr);
    }
 
 #ifdef DEBUG_INFO
    {
       uint32_t token = usb_hal_ehci_get_qtd_token(first_qtd_ptr);
       USB_PRINTF("_usb_ehci_add_interrupt_xfer_to_periodic_list: QH =%x\n"
              "  Status=%x,PID code=%x,error code=%x,page=%x,IOC=%x,Bytes=%x,Toggle=%x\n",
              token,
              (token&0xFF),
              (token >> 8)&0x3,
              (token >> 10) &0x3,
              (token >> 12)&0x7,
              (token >> 15)&0x1,
              (token>> 16)&0x7FFF,
              (token)&EHCI_QTD_DATA_TOGGLE) >>31);
    }
 #endif
 
 
    /****************************************************************************
    if periodic schedule is not already enabled, enable it.
    ****************************************************************************/
    sts_val = usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase);
    if(!(sts_val & EHCI_STS_PERIODIC_SCHEDULE))
    {
        cmd_val = usb_hal_ehci_get_usb_cmd(usb_host_ptr->usbRegBase);
 
        /****************************************************************************
        write the address of the periodic list in to the periodic base register
        ****************************************************************************/
        usb_hal_ehci_set_periodic_list_base_addr(usb_host_ptr->usbRegBase, (uint32_t) usb_host_ptr->periodic_list_base_addr);
 
        /****************************************************************************
        wait until we can enable  the periodic schedule.
        ****************************************************************************/
        while ((cmd_val & EHCI_USBCMD_PERIODIC_SCHED_ENABLE) !=
                 (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_PERIODIC_SCHEDULE)) {
        }
 
 
        /****************************************************************************
        enable the schedule now.
        ****************************************************************************/
 
        //EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_CMD,(cmd_val | EHCI_USBCMD_PERIODIC_SCHED_ENABLE));
    usb_hal_ehci_set_usb_cmd(usb_host_ptr->usbRegBase, (cmd_val | EHCI_USBCMD_PERIODIC_SCHED_ENABLE));
    }
    return USB_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_port_change
*  Returned Value : None
*  Comments       :
*        Process port change
*END*-----------------------------------------------------------------*/

bool _usb_ehci_process_port_change
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle
   )
{ /* Body */

   usb_ehci_host_state_struct_t*      usb_host_ptr;
   uint8_t                            i, total_port_numbers;
   uint32_t                           port_control, status;
   //USB_EHCI_HOST_INIT_STRUCT_PTR       param;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

   //param = (USB_EHCI_HOST_INIT_STRUCT_PTR) usb_host_ptr->INIT_PARAM;

   //cap_dev_ptr = (vusb20_reg_struct_t*) param->CAP_BASE_PTR;


   total_port_numbers =
   (uint8_t)(usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) &
      EHCI_HCS_PARAMS_N_PORTS);


   for (i = 0; i < total_port_numbers; i++) {
      port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);

      if (port_control & EHCI_PORTSCX_CONNECT_STATUS_CHANGE) {
         /* Turn on the 125 usec uframe interrupt. This effectively
         ** starts the timer to count 125 usecs
         */
         usb_hal_ehci_enable_interrupts(usb_host_ptr->usbRegBase, EHCI_INTR_SOF_UFRAME_EN);
         do {
            if (port_control & EHCI_PORTSCX_CONNECT_STATUS_CHANGE) {
               usb_host_ptr->uframe_count = 0;
               port_control =  usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
               port_control &= ~EHCI_PORTSCX_W1C_BITS;
               port_control |= EHCI_PORTSCX_CONNECT_STATUS_CHANGE;
               usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, port_control);
            } /* Endif */
            status = (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) &
                      usb_hal_ehci_get_interrupt_enable_status(usb_host_ptr->usbRegBase));
            if (status & EHCI_STS_SOF_COUNT) {
               /* Increment the 125 usecs count */
               usb_host_ptr->uframe_count++;
               usb_hal_ehci_clear_usb_interrupt_status(usb_host_ptr->usbRegBase, status);
            } /* Endif */
            port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
         } while (usb_host_ptr->uframe_count != 2);
         /* Turn off the 125 usec uframe interrupt. This effectively
         ** stops the timer to count 125 usecs
         */
         usb_hal_ehci_disable_interrupts(usb_host_ptr->usbRegBase,EHCI_INTR_SOF_UFRAME_EN);

         port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);

         usb_host_ptr->uframe_count = 0;

         /* We waited to check for stable current connect status */
         if (port_control & EHCI_PORTSCX_CURRENT_CONNECT_STATUS) {
            /* Attach on port i */
            /* send change report to the hub-driver */
            /* The hub driver does GetPortStatus and identifies the new connect */
            /* usb_host_ptr->ROOT_HUB_DRIVER(handle, hub_num, GET_PORT_STATUS); */
            /* reset and enable the port */
            _usb_ehci_reset_and_enable_port(handle, i);
#if USBCFG_EHCI_HS_DISCONNECT_ENABLE
            /*enable HS disconnect detection */
            port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
            if(port_control & EHCI_PORTSCX_PORT_HIGH_SPEED) {
                bsp_usb_hs_disconnect_detection_enable(usb_host_ptr->controller_id);
            }
#endif
         } else {
            /* Detach on port i */
            /* send change report to the hub-driver */
            /* usb_host_ptr->ROOT_HUB_DRIVER(handle, hub_num, GET_PORT_STATUS); */

#if USBCFG_EHCI_HS_DISCONNECT_ENABLE
            /* disable HS disconnect detection */
            bsp_usb_hs_disconnect_detection_disable(usb_host_ptr->controller_id);
#endif
            /* clear the connect status change */
            port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
            port_control &= ~EHCI_PORTSCX_W1C_BITS;
            port_control |= EHCI_PORTSCX_CONNECT_STATUS_CHANGE;
            usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, port_control);

            /* Disable the asynch and periodic schedules */
            usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
            usb_hal_ehci_disable_usb_cmd_periodic_sched(usb_host_ptr->usbRegBase);
           
            /* call device detach (host pointer, speed, hub #, port #) */
#if USBCFG_EHCI_HOST_ENABLE_TASK
            usb_host_ptr->devices_inserted = 0;
            OS_Event_set(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_DETACH);
#else
            usb_host_dev_mng_detach((void*)usb_host_ptr->upper_layer_handle, 0, (uint8_t)((i + 1)));
#endif

            /* Endif */

            if (!i) {
               return TRUE;
            } /* Endif */
         } /* Endif */
      } /* Endif */

      if (port_control & EHCI_PORTSCX_PORT_FORCE_RESUME) {
         port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
         port_control &= ~EHCI_PORTSCX_W1C_BITS;
         port_control &= ~EHCI_PORTSCX_PORT_FORCE_RESUME;
         _usb_host_call_service((usb_host_handle)usb_host_ptr, USB_SERVICE_HOST_RESUME, 0);
      } /* Endif */

      if ((usb_host_ptr->is_resetting != FALSE) &&
         (port_control & EHCI_PORTSCX_PORT_ENABLE))
      {
         usb_host_ptr->is_resetting = FALSE;

         /* reset process complete */
         /* Report the change to the hub driver and enumerate */
         usb_host_ptr->temp_speed = ((port_control & USBHS_SPEED_MASK) >> USBHS_SPEED_BIT_POS);
         //usb_host_ptr->SPEED = USB_SPEED_HIGH;
         usb_host_ptr->port_num = (uint32_t)(i + 1);

         /* Now wait for reset recovery */
         usb_host_ptr->reset_recovery_timer = (USB_RESET_RECOVERY_DELAY*8);

         /* Turn on the 125 usec uframe interrupt. This effectively
         ** starts the timer to count 125 usecs
         */
         usb_hal_ehci_enable_interrupts(usb_host_ptr->usbRegBase, EHCI_INTR_SOF_UFRAME_EN);
      } /* Endif */

   } /* Endfor */


   return FALSE;

} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_reset_and_enable_port
*  Returned Value : None
*  Comments       :
*        Reset and enabled the port
*END*-----------------------------------------------------------------*/

void _usb_ehci_reset_and_enable_port
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] the port number */
      uint8_t                  port_number
   )
{ /* Body */
   usb_ehci_host_state_struct_t*       usb_host_ptr;
   uint32_t                            port_status_control;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

   /* Check the line status bit in the PORTSC register */
   port_status_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);

   port_status_control &= ~EHCI_PORTSCX_W1C_BITS;

   /* reset should wait for 100 Ms debouce period before starting*/
#ifdef __USB_OTG__
   if ((usb_otg_state_struct_ptr->STATE_STRUCT_PTR->STATE != A_SUSPEND) &&
      (usb_otg_state_struct_ptr->STATE_STRUCT_PTR->STATE != B_WAIT_ACON))
#endif

   {
      uint32_t i;

      for (i = 0; i < USB_DEBOUNCE_DELAY; i++) {
         _usb_host_delay(handle, 1);    //wait 1 msec

         /* Check the line status bit in the PORTSC register */
         if (!(usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase) & EHCI_PORTSCX_CURRENT_CONNECT_STATUS))
            break;
      } /* Endfor */
   }

   usb_host_ptr->is_resetting = TRUE;

   /*
   ** Start the reset process
   */

   usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, (port_status_control | EHCI_PORTSCX_PORT_RESET));
   /* Wait for Reset complete */
   do
   {
       port_status_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
   } while (port_status_control & EHCI_PORTSCX_PORT_RESET);
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_host_process_reset_recovery_done
*  Returned Value : None
*  Comments       :
*        Reset and enabled the port
*END*-----------------------------------------------------------------*/

void _usb_host_process_reset_recovery_done
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle
   )
{ /* Body */
   usb_ehci_host_state_struct_t*      usb_host_ptr;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

   /* call device attach (host pointer, speed, hub #, port #) */
#if USBCFG_EHCI_HOST_ENABLE_TASK
   usb_host_ptr->devices_inserted = 1;
   if(usb_host_ptr->devices_attached == 0)
   {
       OS_Event_clear(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_DETACH);
   }
   OS_Event_set(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_ATTACH);
#else
   usb_device_instance_handle         dev_handle = NULL;
   usb_host_dev_mng_attach((void*)usb_host_ptr->upper_layer_handle, NULL, (uint8_t)(usb_host_ptr->temp_speed), 0, (uint8_t)(usb_host_ptr->port_num), 1, &dev_handle);
#endif
   /* Endif */
   /* Turn off the 125 usec uframe interrupt. This effectively
   ** stops the timer to count 125 usecs
   */
   usb_hal_ehci_disable_interrupts(usb_host_ptr->usbRegBase, EHCI_INTR_SOF_UFRAME_EN);
   
} /* EndBody */


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_isr
*  Returned Value : None
*  Comments       :
*        Service all the interrupts in the VUSB1.1 hardware
*END*-----------------------------------------------------------------*/

void _usb_ehci_isr
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle
   )
{ /* Body */
   usb_ehci_host_state_struct_t*      usb_host_ptr;
   uint32_t                           status;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;


   /* We use a while loop to process all the interrupts while we are in the
   ** loop so that we don't miss any interrupts
   */
   while (TRUE) {

      /* Check if any of the enabled interrupts are asserted */
      status = (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & 
                usb_hal_ehci_get_interrupt_enable_status(usb_host_ptr->usbRegBase));

      if (!status) {
         /* No more interrupts pending to be serviced */
         break;
      } /* Endif */

      /* Clear all enabled interrupts */
      usb_hal_ehci_clear_usb_interrupt_status(usb_host_ptr->usbRegBase, status);

      if (status & EHCI_STS_SOF_COUNT) {
         /* Waiting for an interval of 10 ms for reset recovery */
         if (usb_host_ptr->reset_recovery_timer) {
            usb_host_ptr->reset_recovery_timer--;
            if (!usb_host_ptr->reset_recovery_timer) {
               _usb_host_process_reset_recovery_done((usb_host_handle)usb_host_ptr);
            } /* Endif */
         } /* Endif */
      } /* Endif */

      if (status & EHCI_STS_ASYNCH_ADVANCE) {
         /* Process the asynch advance */
      } /* Endif */

      if (status & EHCI_STS_HOST_SYS_ERROR) {
         /* Host system error. Controller halted. Inform the upper layers */
         #if USBCFG_EHCI_HOST_ENABLE_TASK
             OS_Event_set(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_SYS_ERROR);
         #else
             _usb_host_call_service((usb_host_handle)usb_host_ptr, USB_SERVICE_SYSTEM_ERROR, 0);
         #endif
      } /* Endif */

      if (status & EHCI_STS_FRAME_LIST_ROLLOVER) {
         /* Process frame list rollover */
      } /* Endif */

      if (status & EHCI_STS_RECLAIMATION) {
         /* Process reclaimation */
      } /* Endif */

      if (status & EHCI_STS_NAK) {
         #if USBCFG_EHCI_HOST_ENABLE_TASK
              OS_Event_set(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_TOK_DONE);
         #else
              _usb_ehci_process_tr_complete((usb_host_handle)usb_host_ptr);
         #endif
      }

      if (status & (EHCI_STS_USB_INTERRUPT | EHCI_STS_USB_ERROR)) {
         /* Process the USB transaction completion and transaction error
         ** interrupt
         */
         #if USBCFG_EHCI_HOST_ENABLE_TASK
              OS_Event_set(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_TOK_DONE);
         #else
              _usb_ehci_process_tr_complete((usb_host_handle)usb_host_ptr);
         #endif
         
         #ifdef DEBUG_INFO
            USB_PRINTF("TR completed\n");
         #endif

      } /* Endif */

      if (status & EHCI_STS_TIMER0) {
         _usb_ehci_process_timer((usb_host_handle)usb_host_ptr);
      }

      if (status & EHCI_STS_PORT_CHANGE) {
         /* Process the port change detect */
         if (_usb_ehci_process_port_change((usb_host_handle)usb_host_ptr)) {
             /* There was a detach on port 0 so we should return */
             return;
         } /* Endif */
         /* Should return if there was a detach on OTG port */
      } /* Endif */
   } /* EndWhile */

} /* Endbody */

#if USBCFG_EHCI_HOST_ENABLE_TASK
static void _usb_ehci_host_task(void* handle)
{
    static usb_ehci_host_state_struct_t*      usb_host_ptr;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    if (usb_host_ptr->devices_attached)
    {
#if (USB_NONBLOCKING_MODE == 0)
        OS_Event_wait(usb_host_ptr->ehci_event_ptr,  EHCI_EVENT_TOK_DONE|EHCI_EVENT_SYS_ERROR|EHCI_EVENT_DETACH, FALSE,5);
#else
        //OS_Event_wait(usb_host_ptr->ehci_event_ptr,  EHCI_EVENT_TOK_DONE|EHCI_EVENT_SYS_ERROR|EHCI_EVENT_DETACH, FALSE,0);
#endif

        if ((OS_Event_check_bit(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_DETACH)))
        {
            OS_Event_clear(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_DETACH);
            usb_host_dev_mng_detach((void*)usb_host_ptr->upper_layer_handle, 0, (uint8_t)(1));
            usb_host_ptr->devices_attached = 0;
            OS_Event_check_bit(usb_host_ptr->ehci_event_ptr, 0xFE);
            return;
        }

        if ((OS_Event_check_bit(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_SYS_ERROR)))
        {
            OS_Event_clear(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_SYS_ERROR);
            _usb_host_call_service((usb_host_handle)usb_host_ptr, USB_SERVICE_SYSTEM_ERROR, 0);
        }

        if ((OS_Event_check_bit(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_TOK_DONE)))
        {
            OS_Event_clear(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_TOK_DONE);
            _usb_ehci_process_tr_complete((usb_host_handle)usb_host_ptr);
        }
    }
    else
    {
#if (USB_NONBLOCKING_MODE == 0)
        OS_Event_wait(usb_host_ptr->ehci_event_ptr,  EHCI_EVENT_ATTACH, FALSE,5);
#else
        //OS_Event_wait(usb_host_ptr->ehci_event_ptr,  EHCI_EVENT_ATTACH, FALSE,0);
#endif
        if ((OS_Event_check_bit(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_ATTACH)))
        {
            OS_Event_clear(usb_host_ptr->ehci_event_ptr, EHCI_EVENT_ATTACH);
            if(usb_host_ptr->devices_inserted)
            {
                usb_device_instance_handle         dev_handle = NULL;
                usb_host_dev_mng_attach((void*)usb_host_ptr->upper_layer_handle, NULL, (uint8_t)(usb_host_ptr->temp_speed), 0, (uint8_t)(usb_host_ptr->port_num), 1, &dev_handle);
                if(NULL != dev_handle)
                {
                    usb_host_ptr->devices_attached = 0x01;
                }
            }
        }
    }
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_khci_task_stun
*  Returned Value : none
*  Comments       :
*        KHCI task
*END*-----------------------------------------------------------------*/
#if ((OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)) 
static void _usb_ehci_host_task_stun(void* handle)
{
    while (1)
    {
        _usb_ehci_host_task(handle);
    }
}
#endif


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_khci_task_create
*  Returned Value : error or USB_OK
*  Comments       :
*        Create KHCI task
*END*-----------------------------------------------------------------*/
uint32_t ehci_host_task_id;
static usb_status _usb_ehci_host_task_create(usb_host_handle handle) {
    //usb_status status;
    //ehci_host_task_id = _task_create_blocked(0, 0, (uint32_t)&task_template);
    ehci_host_task_id = OS_Task_create(USB_EHCI_HOST_TASK_ADDRESS, (void*)handle, (uint32_t)USB_EHCI_HOST_TASK_PRIORITY, USB_EHCI_HOST_TASK_STACKSIZE, USB_EHCI_HOST_TASK_NAME, NULL);
    
    if (ehci_host_task_id == (uint32_t)OS_TASK_ERROR) 
    {
        return USBERR_ERROR;
    }
    
    return USB_OK;
}

#endif
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_preinit
*  Returned Value : error or USB_OK
*  Comments       :
*        Allocate the structures for EHCI
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_preinit(usb_host_handle upper_layer_handle, usb_host_handle *handle)
{
    usb_ehci_host_state_struct_t* usb_host_ptr = (usb_ehci_host_state_struct_t*) OS_Mem_alloc_zero(sizeof(usb_ehci_host_state_struct_t));
    ehci_pipe_struct_t* p;
    ehci_pipe_struct_t* pp;
    int i;

    if (NULL != usb_host_ptr)
    {
        /* Allocate the USB Host Pipe Descriptors */
        usb_host_ptr->pipe_descriptor_base_ptr = 
        (pipe_struct_t*)OS_Mem_alloc_zero(sizeof(ehci_pipe_struct_t) * USBCFG_HOST_MAX_PIPES);
        if (usb_host_ptr->pipe_descriptor_base_ptr == NULL) {
            OS_Mem_free(usb_host_ptr);

            return USBERR_ALLOC;
        }
        OS_Mem_zero(usb_host_ptr->pipe_descriptor_base_ptr, sizeof(ehci_pipe_struct_t) * USBCFG_HOST_MAX_PIPES);

        p = (ehci_pipe_struct_t*) usb_host_ptr->pipe_descriptor_base_ptr;
        pp = NULL;
        for (i = 0; i < USBCFG_HOST_MAX_PIPES; i++) {
           if (pp != NULL) {
              pp->common.next = (pipe_struct_t*) p;
           }
           pp = p;
           p++;
        }

        //usb_host_ptr->PIPE_SIZE = sizeof(ehci_pipe_struct_t);
        //usb_host_ptr->TR_SIZE = sizeof(EHCI_TR_STRUCT);
        usb_host_ptr->upper_layer_handle = upper_layer_handle;
        *handle = (usb_host_handle) usb_host_ptr;
    } /* Endif */
    else
    {
        *handle = NULL;
        return USBERR_ALLOC;
    }
    
    return USB_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_init
*  Returned Value : error or USB_OK
*  Comments       :
*        Initialize the VUSB_HS controller
*END*-----------------------------------------------------------------*/

usb_status  _usb_ehci_init(uint8_t controller_id, usb_host_handle handle)
{ /* Body */
   usb_ehci_host_state_struct_t*               usb_host_ptr;
   //USB_EHCI_HOST_INIT_STRUCT_PTR                param;
   ehci_qh_struct_t*                           qh_ptr;
   ehci_qtd_struct_t*                          qtd_ptr;
   uint8_t                                      vector;
   uint32_t                                      i, frame_list_size_bits;
#ifndef __USB_OTG__
   uint32_t                                      port_control[16];
   uint8_t                                       portn = 0;
#endif
   uint32_t                                      total_xtd_struct_memory = 0;

   uint32_t                                      endian;
   uint32_t                                      temp = 0;

   #if USBCFG_MEMORY_ENDIANNESS == MQX_LITTLE_ENDIAN
      endian = 0;
   #else
      endian = VUSBHS_MODE_BIG_ENDIAN;
   #endif

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   //param = (USB_EHCI_HOST_INIT_STRUCT_PTR) usb_host_ptr->INIT_PARAM;

   //if (param == NULL)
   //    return USBERR_INIT_DATA;
   

   /* timer_dev_ptr = (vusb20_reg_struct_t*) param->TIMER_BASE_PTR;*/

   /* Get the base address of the VUSB_HS registers */
   usb_host_ptr->usbRegBase = soc_get_usb_base_address(controller_id);
   usb_host_ptr->controller_id = controller_id;
   usb_host_ptr->devices_inserted = 0;
   usb_host_ptr->devices_attached = 0;

    usb_host_ptr->mutex = OS_Mutex_create();
    if (usb_host_ptr->mutex == NULL)
    {
        USB_PRINTF("ehci host create mutex failed\n");
        return USBERR_ALLOC;
    }
#if USBCFG_EHCI_HOST_ENABLE_TASK
    usb_host_ptr->ehci_event_ptr = OS_Event_create(0);
    if (usb_host_ptr->ehci_event_ptr == NULL)
    {
        #if DEV_DEBUG
            USB_PRINTF(" Event create failed in _usb_ehci_init\n");
        #endif
        
        return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
    } /* Endif */

    _usb_ehci_host_task_create(usb_host_ptr);
#endif
    

   /* Stop the controller */
   usb_hal_ehci_initiate_detach_event(usb_host_ptr->usbRegBase);

   /* Configure the VUSBHS has a host controller */
   usb_hal_ehci_set_usb_mode(usb_host_ptr->usbRegBase,USBHS_MODE_CTRL_MODE_HOST);
   usb_hal_ehci_set_usb_mode(usb_host_ptr->usbRegBase,USBHS_MODE_CTRL_MODE_HOST | endian);

   /* Stop the controller */
   usb_hal_ehci_initiate_detach_event(usb_host_ptr->usbRegBase);

   /* Get the interrupt vector number for the VUSB_HS host */
   vector = soc_get_usb_vector_number(controller_id);;

   portn = usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) & 0x0000000f;
   for (i = 0; i < portn; i++)
   {
       port_control[i] = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
   }
#ifndef __USB_OTG__

   if (!(OS_install_isr(vector, _usb_ehci_isr, (void *)usb_host_ptr))) {
      return USB_log_error(__FILE__,__LINE__,USBERR_INSTALL_ISR);
   } /* Endbody */

#endif /* __USB_OTG__ */

   while (!(usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_HC_HALTED)) {
      /* Wait for the controller to stop */
   }

   /* Reset the controller to get default values */
   usb_hal_ehci_set_usb_cmd(usb_host_ptr->usbRegBase, EHCI_CMD_CTRL_RESET);

   while (usb_hal_ehci_get_usb_cmd(usb_host_ptr->usbRegBase) & EHCI_CMD_CTRL_RESET) {
      /* Wait for the controller reset to complete */
   } /* EndWhile */

   /* Configure the VUSBHS has a host controller */
   usb_hal_ehci_set_usb_mode(usb_host_ptr->usbRegBase,USBHS_MODE_CTRL_MODE_HOST | endian);

   //EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.CTRLDSSEGMENT,0);

   /*******************************************************************
    Set the size of frame list in CMD register
   *******************************************************************/
   if (USBCFG_EHCI_FRAME_LIST_SIZE > 512)
   {
       usb_host_ptr->frame_list_size = 1024;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_1024;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 256)
   {
       usb_host_ptr->frame_list_size = 512;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_512;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 128)
   {
       usb_host_ptr->frame_list_size = 256;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_256;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 64)
   {
       usb_host_ptr->frame_list_size = 128;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_128;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 32)
   {
       usb_host_ptr->frame_list_size = 64;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_64;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 16)
   {
       usb_host_ptr->frame_list_size = 32;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_32;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 8)
   {
       usb_host_ptr->frame_list_size = 16;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_16;
   }
   else if (USBCFG_EHCI_FRAME_LIST_SIZE > 0)
   {
       usb_host_ptr->frame_list_size = 8;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_8;
   }
   else
   {
       usb_host_ptr->frame_list_size = 1024;
       frame_list_size_bits = EHCI_CMD_FRAME_SIZE_1024;
   }

   if (!(usb_hal_ehci_get_hccparams(usb_host_ptr->usbRegBase) & EHCI_HCC_PARAMS_PGM_FRM_LIST_FLAG)
        && (frame_list_size_bits != EHCI_CMD_FRAME_SIZE_1024))
   {
      /* Cannot shrink frame list size because it is unsupported by silicon vendor */
      return USB_log_error(__FILE__,__LINE__,USBERR_INIT_FAILED);
   }


   /*
   **   ALL CONTROLLER DRIVER MEMORY NEEDS are allocated here.
   */
   total_xtd_struct_memory += (USBCFG_EHCI_MAX_QH_DESCRS * sizeof(ehci_qh_struct_t))+32;
   total_xtd_struct_memory += (USBCFG_EHCI_MAX_QTD_DESCRS * sizeof(ehci_qtd_struct_t));

#if USBCFG_EHCI_MAX_ITD_DESCRS
   /*memory required by high-speed Iso transfers */
   total_xtd_struct_memory += (USBCFG_EHCI_MAX_ITD_DESCRS * 
   sizeof(ehci_itd_struct_t));
#endif

#if USBCFG_EHCI_MAX_SITD_DESCRS
   /*memory required by full-speed Iso transfers */
   total_xtd_struct_memory += (USBCFG_EHCI_MAX_SITD_DESCRS * sizeof(ehci_sitd_struct_t));
#endif

   usb_host_ptr->xtd_struct_base_addr = (void*)OS_Mem_alloc_uncached(total_xtd_struct_memory);
   if (!usb_host_ptr->xtd_struct_base_addr)
   {
      return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
   }

   OS_Mem_zero(usb_host_ptr->xtd_struct_base_addr, total_xtd_struct_memory);

   

   /* allocate space for frame list aligned at 4kB boundary */
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
   usb_host_ptr->periodic_list_base_addr = (void*)usbhs_perodic_list;
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
   usb_host_ptr->periodic_list_base_addr = (void*)OS_Mem_alloc_uncached_align(sizeof(ehci_frame_list_element_pointer) * usb_host_ptr->frame_list_size, 4096);
#endif
   if (!usb_host_ptr->periodic_list_base_addr)
   {
      OS_Mem_free(usb_host_ptr->xtd_struct_base_addr);
      return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
   }

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
      /* memzero the whole memory */
      OS_Mem_zero(usb_host_ptr->periodic_list_base_addr, sizeof(usbhs_perodic_list));
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
      /* memzero the whole memory */
      OS_Mem_zero(usb_host_ptr->periodic_list_base_addr, sizeof(ehci_frame_list_element_pointer) * usb_host_ptr->frame_list_size);
#endif

   /*
   **   NON-PERIODIC MEMORY DISTRIBUTION STUFF
   */
   usb_host_ptr->async_list_base_addr = usb_host_ptr->qh_base_ptr =
      (void *)USB_MEM32_ALIGN((uint32_t)usb_host_ptr->xtd_struct_base_addr);

   usb_host_ptr->qtd_base_ptr = (ehci_qtd_struct_t*)((uint32_t)usb_host_ptr->qh_base_ptr + (USBCFG_EHCI_MAX_QH_DESCRS * sizeof(ehci_qh_struct_t)));

   /* the first qh is used as dummy qh which is assigned to host's CURR_ASYNC_LIST_ADDR */
   qh_ptr = usb_host_ptr->qh_base_ptr;
   usb_hal_ehci_set_qh_horiz_link_ptr(qh_ptr, (((uint32_t)qh_ptr & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
    
   temp = usb_hal_ehci_get_ep_capab_charac1(qh_ptr);
   temp |= (1 << (EHCI_QH_HEAD_RECLAMATION_BIT_POS));
   usb_hal_ehci_set_ep_capab_charac1(qh_ptr, temp);
   
   usb_hal_ehci_set_qh_next_alt_qtd_link_terminate(qh_ptr);
   usb_hal_ehci_set_qh_next_qtd_link_terminate(qh_ptr);
   usb_host_ptr->dummy_qh = qh_ptr;
   qh_ptr++;

   for (i = 1; i < USBCFG_EHCI_MAX_QH_DESCRS; i++) {
      /* Set the dTD to be invalid */
      usb_hal_ehci_set_qh_horiz_link_ptr_head_pointer_terminate(qh_ptr);
      _usb_ehci_free_qh((usb_host_handle)usb_host_ptr, (void *)qh_ptr);
      qh_ptr++;
   } /* Endfor */

   qtd_ptr = usb_host_ptr->qtd_base_ptr;

   /* Enqueue all the QTDs */
   for (i = 0; i < USBCFG_EHCI_MAX_QTD_DESCRS; i++)
   {
      /*
      usb_host_ptr->qtd_node_link[i].qtd      = qtd_ptr;
      usb_host_ptr->qtd_node_link[i].occupied = 0;
      */
      _usb_ehci_free_qtd((usb_host_handle)usb_host_ptr, (void *)qtd_ptr);
      qtd_ptr++;
   } /* Endfor */


   /*
   **   BANDWIDTH MEMORY DISTRIBUTION STUFF
   */

   /*********************************************************************
   Allocating the memory to store periodic bandwidth. A periodic BW list
   is a two dimensional array with dimension (frame list size x 8 u frames).

   Also note that the following could be a large allocation of memory
   The max value stored in a location will be 125 micro seconds and so we
   use uint8_t
   *********************************************************************/

   usb_host_ptr->periodic_frame_list_bw_ptr = (void*)OS_Mem_alloc_uncached(usb_host_ptr->frame_list_size * 8 * sizeof(uint8_t));
   if (!usb_host_ptr->periodic_frame_list_bw_ptr)
   {
      OS_Mem_free(usb_host_ptr->xtd_struct_base_addr);
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
      OS_Mem_free(usb_host_ptr->periodic_list_base_addr);
#endif
      return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
   }

   OS_Mem_zero(usb_host_ptr->periodic_frame_list_bw_ptr, usb_host_ptr->frame_list_size * 8 * sizeof(uint8_t));

   /*make sure that periodic list is uninitialized */
   usb_host_ptr->periodic_list_initialized = FALSE;

   /* Initialize the list of active structures to NULL initially */
   usb_host_ptr->active_async_list_ptr = NULL;
   usb_host_ptr->active_interrupt_periodic_list_ptr = NULL; 

#if USBCFG_EHCI_MAX_ITD_DESCRS
   /*
   **  HIGH SPEED ISO TRANSFERS MEMORY ALLOCATION STUFF
   */

   usb_host_ptr->itd_base_ptr = (ehci_itd_struct_t*)((uint32_t)usb_host_ptr->qtd_base_ptr + (USBCFG_EHCI_MAX_QTD_DESCRS * sizeof(ehci_qtd_struct_t)));

   usb_host_ptr->itd_list_initialized = FALSE;

   /*****************************************************************************
     ITD QUEUING PREPARATION
   *****************************************************************************/

   /* memory for doubly link list of nodes that keep active ITDs. Head and Tail point to
   same place when list is empty */
   usb_host_ptr->active_iso_itd_periodic_list_head_ptr = 
   (void*)OS_Mem_alloc_uncached(sizeof(list_node_struct_t) * USBCFG_EHCI_MAX_ITD_DESCRS);
   usb_host_ptr->active_iso_itd_periodic_list_tail_ptr = usb_host_ptr->active_iso_itd_periodic_list_head_ptr;

   if (!usb_host_ptr->active_iso_itd_periodic_list_head_ptr)
   {
      OS_Mem_free(usb_host_ptr->xtd_struct_base_addr);
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
      OS_Mem_free(usb_host_ptr->periodic_list_base_addr);
#endif
      OS_Mem_free(usb_host_ptr->periodic_frame_list_bw_ptr);
      return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
   }
  
   OS_Mem_zero(usb_host_ptr->active_iso_itd_periodic_list_head_ptr,
                  sizeof(list_node_struct_t) * USBCFG_EHCI_MAX_ITD_DESCRS);
#endif 

#if USBCFG_EHCI_MAX_SITD_DESCRS
   /*
   **   FULL SPEED ISO TRANSFERS MEMORY ALLOCATION STUFF
   */

   /* Allocate the Split-transactions Isochronous Transfer Descriptors:
   ** 32 bytes aligned
   */
   #if USBCFG_EHCI_MAX_ITD_DESCRS
   usb_host_ptr->sitd_base_ptr = (ehci_sitd_struct_t*)((uint32_t)usb_host_ptr->itd_base_ptr + (USBCFG_EHCI_MAX_ITD_DESCRS * sizeof(ehci_itd_struct_t)));
   #else
   usb_host_ptr->sitd_base_ptr = (ehci_sitd_struct_t*)((uint32_t)usb_host_ptr->qtd_base_ptr + (USBCFG_EHCI_MAX_QTD_DESCRS * sizeof(ehci_qtd_struct_t)));
   #endif
   usb_host_ptr->sitd_list_initialized = FALSE;

   /*****************************************************************************
     SITD QUEUING PREPARATION
   *****************************************************************************/

   /* memory for doubly link list of nodes that keep active SITDs. Head and Tail point to
   same place when list is empty */
   usb_host_ptr->active_iso_sitd_periodic_list_head_ptr = (void*)OS_Mem_alloc_uncached(sizeof(list_node_struct_t) * USBCFG_EHCI_MAX_SITD_DESCRS);
   usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr = usb_host_ptr->active_iso_sitd_periodic_list_head_ptr;

   if (!usb_host_ptr->active_iso_sitd_periodic_list_head_ptr)
   {
      OS_Mem_free(usb_host_ptr->xtd_struct_base_addr);
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
      OS_Mem_free(usb_host_ptr->periodic_list_base_addr);
#endif
      OS_Mem_free(usb_host_ptr->periodic_frame_list_bw_ptr);
      OS_Mem_free(usb_host_ptr->active_iso_itd_periodic_list_head_ptr);
      return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
   }
 
   OS_Mem_zero(usb_host_ptr->active_iso_sitd_periodic_list_head_ptr,
                  sizeof(list_node_struct_t) * USBCFG_EHCI_MAX_SITD_DESCRS);
#endif
   /*
   ** HARDWARE  REGISTERS INITIALIZATION STUFF
   */

#if 0
   /* 4-Gigabyte segment where all of the  interface data structures are allocated. */
   /* If no 64-bit addressing capability then this is zero */
   if ((usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) &
        EHCI_HCC_PARAMS_64_BIT_ADDR_CAP)) {
      EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.CTRLDSSEGMENT,0);
   } else {
      EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.CTRLDSSEGMENT,EHCI_DATA_STRUCTURE_BASE_ADDRESS);
   } /* Endif */

#endif

#ifndef __USB_OTG__
   if (usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) &
       USBHS_HCS_PARAMS_PORT_POWER_CONTROL_FLAG) {
       for (i = 0; i < portn; i++)
       {
           port_control[i] &= ~EHCI_PORTSCX_W1C_BITS; //Do not accidentally clear w1c bits by writing 1 back
           usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, port_control[i] | EHCI_PORTSCX_PORT_POWER);
#if 0
           if (usb_host_ptr->SPEED == USB_SPEED_FULL) {
               EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.PORTSCX[i],port_control[i] | EHCI_PORTSCX_PORT_POWER | VUSBHS_PORTSCX_PORT_PFSC);
           }
           else { //usb_host_ptr->SPEED == USB_SPEED_HIGH
               EHCI_REG_WRITE(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.PORTSCX[i],port_control[i] | EHCI_PORTSCX_PORT_POWER);
           }
#endif
       }
  } /* Endif */
#endif

   usb_hal_ehci_enable_interrupts(usb_host_ptr->usbRegBase, USBHS_HOST_INTR_EN_BITS);

   /* start the controller, setup the frame list size */
   usb_hal_ehci_set_usb_cmd(usb_host_ptr->usbRegBase, (EHCI_INTR_NO_THRESHOLD_IMMEDIATE | frame_list_size_bits | EHCI_CMD_RUN_STOP));
   /* route all ports to the EHCI controller */
   usb_hal_ehci_set_usb_config(usb_host_ptr->usbRegBase, 1);
   
   /* The GPTIMER was disabled. The idea was to use GPTIMER to cancel some transfers that were pending for data.
   ** Since the conception is not right, it was removed, but the code still remains here commented here
   ** for developer to persist backward compatibility. NOTE: GPTimer is not included in the EHCI standard.
   if (timer_dev_ptr) {
      EHCI_REG_WRITE(timer_dev_ptr->REGISTERS.TIMER_REGISTERS.GPTIMER0LD, 10000 - 1);
      EHCI_REG_WRITE(timer_dev_ptr->REGISTERS.TIMER_REGISTERS.GPTIMER0CTL, EHCIFSL_GTCTL_GPTIMER_REPEAT | EHCIFSL_GTCTL_GPTIMER_RESET | EHCIFSL_GTCTL_GPTIMER_RUN);
   }

   EHCI_REG_SET_BITS(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_INTR,EHCI_INTR_TIMER0);
   */

   return USB_OK;

} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_get_QTD
*  Returned Value : void
*  Comments       :
*        Get an QTD from the free QTD ring.
*
*END*-----------------------------------------------------------------*/
void _usb_ehci_get_qtd
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] the QTD to enqueue */
      volatile ehci_qtd_struct_t**   qtd_ptr
   )
{
    usb_ehci_host_state_struct_t*    usb_host_ptr;
    volatile ehci_qtd_struct_t*               qtd;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();

    if (usb_host_ptr->qtd_entries == 0)
    {
#ifdef _DEBUG
        USB_PRINTF("no qtd available\n");
#endif
        USB_EHCI_Host_unlock();
        *qtd_ptr = NULL;
        return ;
    }
    EHCI_QTD_QGET(usb_host_ptr->qtd_head, usb_host_ptr->qtd_tail, qtd);

    if (!qtd)
    {
#ifdef _DEBUG
        USB_PRINTF("get qtd error\n");
#endif
        USB_EHCI_Host_unlock();
        *qtd_ptr = NULL;
        return;
    }
    
    usb_hal_ehci_set_qtd_terminate_bit(qtd);
    *qtd_ptr = qtd;
    usb_host_ptr->qtd_entries--;

    USB_EHCI_Host_unlock();
    return;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_QTD
*  Returned Value : void
*  Comments       :
*        Enqueues an QTD onto the free QTD ring.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_free_qtd
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] the QTD to enqueue */
      ehci_qtd_struct_t*    qtd
   )
{
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();
    
    usb_hal_ehci_set_qtd_terminate_bit(qtd);
 
    EHCI_QTD_QADD(usb_host_ptr->qtd_head, usb_host_ptr->qtd_tail, qtd);
    usb_host_ptr->qtd_entries++;
    
    USB_EHCI_Host_unlock();
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_get_qh
*  Returned Value : void
*  Comments       :
*        Get an QTD from the free qh ring.
*
*END*-----------------------------------------------------------------*/
void _usb_ehci_get_qh
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] the QTD to enqueue */
      ehci_qh_struct_t**    qh_ptr
   )
{
    usb_ehci_host_state_struct_t*    usb_host_ptr;
    ehci_qh_struct_t*                qh;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();
    if (usb_host_ptr->qh_entries == 0)
    {
#ifdef _DEBUG
        USB_PRINTF("no qh available\n");
#endif
        USB_EHCI_Host_unlock();
        *qh_ptr = NULL;
        return ;

    }
    EHCI_QH_QGET(usb_host_ptr->qh_head, usb_host_ptr->qh_tail, qh);

    if (!qh)
    {
#ifdef _DEBUG
        USB_PRINTF("get qh error\n");
#endif
        USB_EHCI_Host_unlock();
        return;
    }
    *qh_ptr = qh;
    usb_host_ptr->qh_entries--;

    USB_EHCI_Host_unlock();
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_QH
*  Returned Value : void
*  Comments       :
*        Enqueues a QH onto the free QH ring.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_free_qh
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] the QH to enqueue */
      ehci_qh_struct_t*     qh
   )
{
    usb_ehci_host_state_struct_t*                    usb_host_ptr;
 
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();

    /*
    ** Add the QH to the free QH queue and increment the tail to the next descriptor
    */

    EHCI_QH_QADD(usb_host_ptr->qh_head, usb_host_ptr->qh_tail, qh);
    usb_host_ptr->qh_entries++;

    // USB_PRINTF("\nQH Add 0x%x, #entries=%d",qh_ptr,usb_host_ptr->qh_entries);

    USB_EHCI_Host_unlock();
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_resources
*  Returned Value : none
*  Comments       :
*        Frees the controller specific resources for a given pipe
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_free_resources
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      pipe_struct_t*                  pipe_ptr
   )
{
    usb_ehci_host_state_struct_t*               usb_host_ptr;
    ehci_qh_struct_t*                           qh_ptr;
    ehci_qh_struct_t*                           temp_qh_ptr = NULL;
    ehci_pipe_struct_t*                         pipe_descr_ptr = (ehci_pipe_struct_t*) pipe_ptr;
    uint32_t                                     need_enable_async_list = 0;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
    qh_ptr = pipe_descr_ptr->qh_for_this_pipe;

    if ((pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE) || (pipe_descr_ptr->common.pipetype == USB_BULK_PIPE))
    {
        USB_EHCI_Host_lock();

        temp_qh_ptr = (ehci_qh_struct_t*)usb_hal_ehci_get_curr_async_list(usb_host_ptr->usbRegBase);

        if(qh_ptr == temp_qh_ptr)
        {
            temp_qh_ptr = (ehci_qh_struct_t*)((uint32_t)temp_qh_ptr->horiz_link_ptr & EHCI_HORIZ_PHY_ADDRESS_MASK);
            if(qh_ptr == temp_qh_ptr)
            {
                temp_qh_ptr = usb_host_ptr->dummy_qh;
            }
            
            usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
            while (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_ASYNCH_SCHEDULE)
            {
            }

            /* Write the QH address to the Current Async List Address */
            usb_hal_ehci_set_qh_to_curr_async_list(usb_host_ptr->usbRegBase, (uint32_t)temp_qh_ptr);
            
            need_enable_async_list = 1;
        }

        _usb_ehci_unlink_qh_from_async_active_list(handle, qh_ptr);

        _usb_ehci_free_qh(handle, (void *) qh_ptr);

        if (usb_host_ptr->active_async_list_ptr == NULL)
        {
            /* If the head is the only one in the queue, disable asynchronous queue */
            usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
        }
        else if(need_enable_async_list > 0)
        {
            /* Enable the Asynchronous schedule */
            usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
        }

        USB_EHCI_Host_unlock();
   }
   else if (pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE)
   {
        _usb_ehci_close_interrupt_pipe(handle, pipe_descr_ptr);
   }
   else if(pipe_descr_ptr->common.pipetype == USB_ISOCHRONOUS_PIPE)
   {
        _usb_ehci_close_isochronous_pipe(handle,pipe_descr_ptr);
   }

   return USB_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_tr_complete
*  Returned Value : None
*  Comments       :
*     Process the Transaction Done interrupt on the EHCI hardware. Note that this
*     routine should be improved later. It is needless to search all the lists
*     since interrupt will belong to only one of them at one time.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_process_tr_complete
   (
      /* [IN] the USB Host state structure */
      usb_host_handle           handle
   )
{
    usb_ehci_host_state_struct_t*               usb_host_ptr;
    ehci_qh_struct_t*                           active_list_member_ptr;
    //qh_link_node_t*                              active_async_list;
 
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /***************************************************************************
     SEARCH THE NON PERIODIC LIST FIRST
    ***************************************************************************/
     /***************************************************************************
     SEARCH THE HIGH SPEED ISOCHRONOUS LIST
    ***************************************************************************/
    if(usb_host_ptr->high_speed_iso_queue_active)
    {
 #if USBCFG_EHCI_MAX_ITD_DESCRS
       _usb_ehci_process_itd_tr_complete(handle);
 #endif //USBCFG_EHCI_MAX_ITD_DESCRS
    }
 
    /***************************************************************************
     SEARCH THE FULL SPEED ISOCHRONOUS LIST
    ***************************************************************************/
    if(usb_host_ptr->full_speed_iso_queue_active)
    {

 #if USBCFG_EHCI_MAX_SITD_DESCRS
       _usb_ehci_process_sitd_tr_complete(handle);
 #endif //USBCFG_EHCI_MAX_SITD_DESCRS
    }
    /* Get the head of the active queue head */
    //active_async_list = usb_host_ptr->active_async_list;//active_async_list_ptr;
    if(usb_host_ptr->active_async_list_ptr)
    {
       _usb_ehci_process_qh_list_tr_complete(handle, usb_host_ptr->active_async_list_ptr);
    }
 
    /***************************************************************************
     SEARCH THE INTERRUPT LIST
    ***************************************************************************/
 
    /* Get the head of the active structures for periodic list*/
    active_list_member_ptr = usb_host_ptr->active_interrupt_periodic_list_ptr;
    if(active_list_member_ptr)
    {
       _usb_ehci_process_qh_interrupt_tr_complete(handle, active_list_member_ptr);
    }

} /* Endbody */



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_qh_list_tr_complete
*  Returned Value : None
*  Comments       :
*     Search the asynchronous or interrupt list to see which QTD had finished and
*     Process the interrupt.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_process_qh_list_tr_complete
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      ehci_qh_struct_t*              active_qh_node_list
   )
{
    ehci_qh_struct_t*                 qh_ptr;
    ehci_qtd_struct_t*                qtd_ptr;
    ehci_qtd_struct_t*                temp_qtd_ptr;
    ehci_pipe_struct_t*               pipe_descr_ptr = NULL;
    tr_struct_t*                      pipe_tr_struct_ptr = NULL;
    uint32_t                          req_bytes = 0;
    uint32_t                          remaining_bytes = 0;
    uint32_t                          status = 0;
    uint8_t*                          buffer_ptr = NULL;

    /* Check all transfer descriptors on all active queue heads */
    do
    {
        /* Get the queue head from the active list */
        qh_ptr = active_qh_node_list;
        /* Get the first QTD for this Queue head */
        qtd_ptr = active_qh_node_list->qtd_head;

#if 0
        if (!(active_list_member_ptr->TIME))
           expired = 1;
        else
           expired = 0;
#endif
        pipe_descr_ptr = active_qh_node_list->pipe;
        active_qh_node_list = active_qh_node_list->next;
        while ((!(((uint32_t)qtd_ptr) & EHCI_QTD_T_BIT)) && (qtd_ptr != NULL))
        {
            /* This is a valid qTD */
            if (!(usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_ACTIVE)) {

#if 0
            uint32_t token = usb_hal_ehci_get_qtd_token(g_usb_instance_echi.instance,QTD_ptr);
                USB_PRINTF("QTD done Token=%x\n"
                   "  Status=%x,PID code=%x,error code=%x,page=%x,IOC=%x,Bytes=%x,Toggle=%x\n",
                   token,
                   ((token)&0xFF),
                   ((token) >> 8)&0x3,
                   ((token) >> 10) &0x3,
                   ((token) >> 12)&0x7,
                   ((token) >> 15)&0x1,
                   ((token) >> 16)&0x7FFF,
                   ((token)&EHCI_QTD_DATA_TOGGLE) >>31);
#endif

                /* Get the pipe descriptor for this transfer */
                pipe_tr_struct_ptr = qtd_ptr->tr;
                
                if (pipe_tr_struct_ptr == NULL)
                {
#ifdef _DEBUG
                    USB_PRINTF("can't find tr by qtd\n");
#endif
                    return;
                }
                pipe_tr_struct_ptr->status = 0;
                /* Check for errors */
                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_ERROR_BITS_MASK)
                {
                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_ERROR_BITS_MASK);
                    usb_hal_ehci_clear_qtd_token_bits(qtd_ptr, EHCI_QTD_ERROR_BITS_MASK);

                }

                /* Check if STALL or endpoint halted because of errors */
                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_HALTED)
                {
                    /* save error count */
                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_CERR_BITS_MASK);

                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_HALTED);
                    usb_hal_ehci_clear_qtd_token_bits(qtd_ptr, EHCI_QTD_STATUS_HALTED);
                    //qh_ptr->status = 0;
                    usb_hal_ehci_clear_qh_status(qh_ptr);
                }

                if (pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE)
                {
                    if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_PID_SETUP)
                    {
                        pipe_tr_struct_ptr->setup_first_phase = TRUE;
                        req_bytes = remaining_bytes = 0;
                    }
                    else if (pipe_tr_struct_ptr->setup_first_phase)
                    {
                        pipe_tr_struct_ptr->setup_first_phase = FALSE;
                        if (pipe_tr_struct_ptr->send_phase)
                        {
                            buffer_ptr = pipe_tr_struct_ptr->tx_buffer;
                            req_bytes  = pipe_tr_struct_ptr->tx_length;
                            //pipe_tr_struct_ptr->send_phase = FALSE;
                        }
                        else
                        {
                            buffer_ptr = pipe_tr_struct_ptr->rx_buffer;
                            req_bytes  = pipe_tr_struct_ptr->rx_length;
                        }

                        remaining_bytes = ((usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_LENGTH_BIT_MASK) >> EHCI_QTD_LENGTH_BIT_POS);

                    }
                    else
                    {
                        //handshake phase, do nothing and setup that we have not received any data in this phase
                        req_bytes = remaining_bytes = 0;
                    }
                }
                else
                {
                    remaining_bytes = ((usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_LENGTH_BIT_MASK) >> EHCI_QTD_LENGTH_BIT_POS);

                    req_bytes = 0;
                    if (pipe_descr_ptr->common.direction)
                    {
                        buffer_ptr = pipe_tr_struct_ptr->tx_buffer;
                        if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC)
                        {
                            if(pipe_tr_struct_ptr->tx_length>0)
                            {
                                req_bytes = ((pipe_tr_struct_ptr->tx_length-1) % VUSB_EP_MAX_LENGTH_TRANSFER) + 1;
                            }
                        }
                        else
                        {
                            req_bytes = VUSB_EP_MAX_LENGTH_TRANSFER;
                        }
                    }
                    else
                    {
                        buffer_ptr = pipe_tr_struct_ptr->rx_buffer;
                        if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC)
                        {
                            if(pipe_tr_struct_ptr->rx_length>0)
                            {
                                req_bytes = ((pipe_tr_struct_ptr->rx_length-1) % VUSB_EP_MAX_LENGTH_TRANSFER) + 1;
                            }
                        }
                        else
                        {
                            req_bytes = VUSB_EP_MAX_LENGTH_TRANSFER;
                        }
                    }
                }

                pipe_tr_struct_ptr->transfered_length += (req_bytes - remaining_bytes);
                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC)
                {
                    status = USB_STATUS_TRANSFER_DONE;

                    if(EHCI_QTD_STATUS_HALTED & pipe_tr_struct_ptr->status)
                    {
                        if(EHCI_QTD_STATUS_BABBLE_DETECTED & pipe_tr_struct_ptr->status)
                        {
                            pipe_tr_struct_ptr->status = USBERR_TR_FAILED;
                        }
                        else if(0 == (EHCI_QTD_CERR_BITS_MASK & pipe_tr_struct_ptr->status))
                        {
                            pipe_tr_struct_ptr->status = USBERR_TR_FAILED;
                        }
                        else
                        {
                            pipe_tr_struct_ptr->status = USBERR_ENDPOINT_STALLED;
                        }
                    }
                    else
                    {
                        pipe_tr_struct_ptr->status = USB_OK;
                    }
                }

                temp_qtd_ptr = qtd_ptr;
                qtd_ptr = (ehci_qtd_struct_t*)usb_hal_ehci_get_next_qtd_ptr( qtd_ptr);

                qh_ptr->qtd_head = (void*)qtd_ptr;

                if (usb_hal_ehci_get_next_qtd_link_ptr( qh_ptr) & EHCI_QTD_T_BIT)
                {
                    /* No alternate descriptor */
                    usb_hal_ehci_set_qh_next_alt_qtd_link_terminate(qh_ptr);
                }

                if (status == USB_STATUS_TRANSFER_DONE)
                {
                    if (_usb_host_unlink_tr(pipe_descr_ptr, pipe_tr_struct_ptr) != USB_OK)
                    {
#if _DEBUG
                        USB_PRINTF("_usb_host_unlink_tr in _usb_ehci_process_qh_list_tr_complete failed\n");
#endif
                    }
                    if (pipe_tr_struct_ptr->callback != NULL)
                    {
                        pipe_tr_struct_ptr->callback((void*)pipe_tr_struct_ptr,
                                                      pipe_tr_struct_ptr->callback_param,
                                                      buffer_ptr,
                                                      pipe_tr_struct_ptr->transfered_length,
                                                      pipe_tr_struct_ptr->status);
                    }
                    status = 0;
                }
                _usb_ehci_free_qtd(handle, (void *)temp_qtd_ptr);
            }
            else
            {
                //qtd_ptr = (ehci_qtd_struct_t*)EHCI_MEM_READ(qtd_ptr->next_qtd_ptr);
                break;
            }
        }
      
    } while (active_qh_node_list);
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_qh_interrupt_tr_complete
*  Returned Value : None
*  Comments       :
*     Search the interrupt list to see which QTD had finished and
*     Process the interrupt.
*
*END*-----------------------------------------------------------------*/
void _usb_ehci_process_qh_interrupt_tr_complete
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                  handle,

      ehci_qh_struct_t*               active_list_member_ptr
   )
{
    ehci_qh_struct_t*                 qh_ptr;
    ehci_qtd_struct_t*                qtd_ptr;
    ehci_qtd_struct_t*                temp_qtd_ptr;
    ehci_pipe_struct_t*               pipe_descr_ptr = NULL;
    tr_struct_t*                       pipe_tr_struct_ptr = NULL;
    uint32_t                           total_req_bytes = 0;
    uint32_t                           remaining_bytes = 0;
    uint32_t                           status = 0;
    uint8_t*                           buffer_ptr = NULL;

    do
    {
        qh_ptr = active_list_member_ptr;
        qtd_ptr = active_list_member_ptr->qtd_head;

        while ((!(((uint32_t)qtd_ptr) & EHCI_QTD_T_BIT)) && (qtd_ptr != NULL))
        {
#ifdef DEBUG_INFO
            uint32_t token = usb_hal_ehci_get_qtd_token(qtd_ptr);
            USB_PRINTF("_usb_ehci_process_qh_interrupt_tr_complete: QTD =%x\n"
                      "  Status=%x,PID code=%x,error code=%x,page=%x,IOC=%x,Bytes=%x,Toggle=%x\n",
                    token,
                    (token&0xFF),
                    (token >> 8)&0x3,
                    (token >> 10) &0x3,
                    (token >> 12)&0x7,
                    (token >> 15)&0x1,
                    (token >> 16)&0x7FFF,
                    (token&EHCI_QTD_DATA_TOGGLE) >>31);
#endif

            if (!(usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_ACTIVE))
            {
                pipe_descr_ptr = (ehci_pipe_struct_t*)qtd_ptr->pipe;
                pipe_tr_struct_ptr = (tr_struct_t*) qtd_ptr->tr;
    
                /* Check for errors */
                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_ERROR_BITS_MASK)
                {
                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_ERROR_BITS_MASK);
            usb_hal_ehci_clear_qtd_token_bits( qtd_ptr, EHCI_QTD_ERROR_BITS_MASK);
                }
    
                /* Check if STALL or endpoint halted because of errors */
                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_HALTED)
                {
                    /* save error count */
                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_CERR_BITS_MASK);
                    pipe_tr_struct_ptr->status |= (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_STATUS_HALTED);
            usb_hal_ehci_clear_qtd_token_bits( qtd_ptr, EHCI_QTD_STATUS_HALTED);
            usb_hal_ehci_clear_qh_status( qh_ptr);
                }
    
                if (pipe_descr_ptr->common.direction)
                {
                    total_req_bytes = pipe_tr_struct_ptr->tx_length;
                    buffer_ptr = pipe_tr_struct_ptr->tx_buffer;
                }
                else
                {
                    total_req_bytes = pipe_tr_struct_ptr->rx_length;
                    buffer_ptr = pipe_tr_struct_ptr->rx_buffer;
                }
                remaining_bytes = ((usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_LENGTH_BIT_MASK) >> EHCI_QTD_LENGTH_BIT_POS);

#ifdef DEBUG_INFO
                USB_PRINTF("_usb_ehci_process_qh_interrupt_tr_complete: Requested Bytes = %d\
                         ,Remaining bytes = %d,",total_req_bytes,remaining_bytes);
#endif

                if (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC)
                {
                    status = USB_STATUS_TRANSFER_DONE;

                    if(EHCI_QTD_STATUS_HALTED & pipe_tr_struct_ptr->status)
                    {
                        if(EHCI_QTD_STATUS_BABBLE_DETECTED & pipe_tr_struct_ptr->status)
                        {
                            pipe_tr_struct_ptr->status = USBERR_TR_FAILED;
                        }
                        else if(0 == (EHCI_QTD_CERR_BITS_MASK & pipe_tr_struct_ptr->status))
                        {
                            pipe_tr_struct_ptr->status = USBERR_TR_FAILED;
                        }
                        else
                        {
                            pipe_tr_struct_ptr->status = USBERR_ENDPOINT_STALLED;
                        }
                    }
                    else
                    {
                        pipe_tr_struct_ptr->status = USB_OK;
                    }
                }
    
                temp_qtd_ptr = qtd_ptr;
                    
        qtd_ptr = (ehci_qtd_struct_t*)usb_hal_ehci_get_next_qtd_ptr(qtd_ptr);
    
                /* Queue the transfer onto the relevant queue head */
                /* should remove this line here !!!!*/              
        usb_hal_ehci_set_qh_next_qtd_link_ptr( qh_ptr, (uint32_t)qtd_ptr);
    
                active_list_member_ptr->qtd_head = (void*)qtd_ptr;
    
                /* Dequeue the used QTD */
                _usb_ehci_free_qtd(handle, (void *)temp_qtd_ptr);
    
                if (status == USB_STATUS_TRANSFER_DONE)
                {
                    if (_usb_host_unlink_tr(pipe_descr_ptr, pipe_tr_struct_ptr) != USB_OK)
                    {
#if _DEBUG
                        USB_PRINTF("_usb_host_unlink_tr in _usb_ehci_process_qh_interrupt_tr_complete failed\n");
#endif
                    }
                    if (pipe_tr_struct_ptr->callback != NULL)
                    {
                        pipe_tr_struct_ptr->callback((void *)pipe_tr_struct_ptr,
                                                      pipe_tr_struct_ptr->callback_param,
                                                      buffer_ptr,
                                                      (total_req_bytes - remaining_bytes),
                                                      pipe_tr_struct_ptr->status);
                    }
                    status = 0;
                }
            }
            else
                break;
            {
            }
         }
         active_list_member_ptr = active_list_member_ptr->next;
    } while (active_list_member_ptr);   
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_host_delay
*  Returned Value : None
*  Comments       :
*        Delay for specified number of milliseconds.
*END*-----------------------------------------------------------------*/

void _usb_host_delay
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* [IN] time to wait in ms */
      uint32_t delay
   )
{ /* Body */
   uint32_t                               start_frame=0,curr_frame=0,diff =0,i=0,j=0;

   /* Get the frame number (not the uframe number */
   start_frame = _usb_ehci_get_frame_number(handle);

   /*wait till frame numer exceeds by delay mili seconds.*/
   do
   {
      curr_frame = _usb_ehci_get_frame_number(handle);
      i++;
      if(curr_frame != start_frame)
      {
         diff++;
         start_frame =  curr_frame;
         j++;
      }

   }while(diff < delay);


} /* Endbody */

uint32_t _get_roundup_pow2(uint32_t data)
{
    uint8_t i = 0;

    if(data == 1 || data == 0 )
        return data;
    while (data != 1)
    {
        data = data >> 1;
        i++;
    }
    return 1 << (i);
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_open_pipe
*  Returned Value : USB Status
*  Comments       :
*        When opening a pipe, this callback ensures for iso / int pipes
*        to allocate bandwidth.
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_open_pipe
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,
      usb_pipe_handle*                pipe_handle_ptr,
      /* The pipe descriptor to queue */
      pipe_init_struct_t*             pipe_init_ptr
   )
{
    usb_status                        error;
    usb_ehci_host_state_struct_t*     usb_host_ptr;
    ehci_pipe_struct_t*               ehci_pipe_ptr;
    pipe_struct_t*                    pipe_ptr = NULL;
    ehci_qh_struct_t*                 qHead = NULL;
    uint8_t                           speed = 0;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
    
    USB_EHCI_Host_lock();
    for (ehci_pipe_ptr = (ehci_pipe_struct_t*)usb_host_ptr->pipe_descriptor_base_ptr; ehci_pipe_ptr != NULL; ehci_pipe_ptr = (ehci_pipe_struct_t*)ehci_pipe_ptr->common.next)
    {
        pipe_ptr = &ehci_pipe_ptr->common;
        if (!pipe_ptr->open)
        {
            pipe_ptr->open = TRUE;
            break;
        }
    }
    USB_EHCI_Host_unlock();
 
    if (ehci_pipe_ptr == NULL)
    {
        #ifdef _HOST_DEBUG_
            DEBUG_LOG_TRACE("_usb_khci_open_pipe failed");
        #endif
        return USB_log_error(__FILE__,__LINE__,USBERR_OPEN_PIPE_FAILED);
    }

    pipe_ptr->endpoint_number = pipe_init_ptr->endpoint_number;
    pipe_ptr->direction       = pipe_init_ptr->direction;
    pipe_ptr->pipetype        = pipe_init_ptr->pipetype;
    pipe_ptr->max_packet_size = pipe_init_ptr->max_packet_size & PACKET_SIZE_MASK;
    pipe_ptr->flags           = pipe_init_ptr->flags;
    pipe_ptr->nak_count       = pipe_init_ptr->nak_count;
    pipe_ptr->nextdata01      = 0;
    pipe_ptr->tr_list_ptr     = NULL;
    pipe_ptr->dev_instance    = pipe_init_ptr->dev_instance;
    pipe_ptr->trs_per_uframe  = 1;

    speed = usb_host_dev_mng_get_speed(ehci_pipe_ptr->common.dev_instance);

    if (pipe_ptr->pipetype == USB_ISOCHRONOUS_PIPE)
    {
        if(speed == USB_SPEED_HIGH)
            pipe_ptr->interval = 1 << (pipe_init_ptr->interval - 1);
        else
            pipe_ptr->interval = pipe_init_ptr->interval;
    }   
    else
    {
        if(speed == USB_SPEED_HIGH)
        {
            if(pipe_ptr->pipetype == USB_INTERRUPT_PIPE)
            {
                if(pipe_init_ptr->interval <= 8)
                {
                    pipe_ptr->interval = 1 << (pipe_init_ptr->interval - 1);
                }
                else
                {
                    pipe_ptr->interval = 128;
                }
            }
            else
            {
                pipe_ptr->interval = 
                _get_roundup_pow2(pipe_init_ptr->interval);
            }
        }
        else
        {
            pipe_ptr->interval = _get_roundup_pow2(pipe_init_ptr->interval);
        }
    }

    
    if ((pipe_init_ptr->pipetype == USB_INTERRUPT_PIPE) || ((pipe_init_ptr->pipetype == USB_ISOCHRONOUS_PIPE)))
    {
        /* Call the low-level routine to send a setup packet */
        error = _usb_ehci_allocate_bandwidth (handle, (pipe_struct_t*)ehci_pipe_ptr);
        if (error != USB_OK)
        {
            goto ERR;
        }
    }
    else
    {
        _usb_ehci_get_qh(handle, &qHead);
        if (qHead == NULL)
        {
            goto ERR;
        }
        ehci_pipe_ptr->qh_for_this_pipe = (void*)qHead;
    }
    
    *pipe_handle_ptr = ehci_pipe_ptr;
    return USB_OK;

ERR:
    pipe_ptr->open = FALSE;
    return USBERR_BANDWIDTH_ALLOC_FAILED;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_close_pipe
*  Returned Value : error or USB_OK
*  Comments       :
*        The function to open a pipe
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_close_pipe
    (
        usb_host_handle          handle, 
        /* [in] Handle of opened pipe */
        usb_pipe_handle          pipe_handle
    )
{
    usb_ehci_host_state_struct_t* usb_host_ptr = (usb_ehci_host_state_struct_t*) handle;
    ehci_pipe_struct_t*           ehci_pipe_ptr_temp = NULL;
    pipe_struct_t*                 pipe_ptr = NULL;
    uint32_t                       offset = (uint32_t)&(((ehci_pipe_struct_t*)0)->actived);
    uint32_t                       offset_zero_set = (uint32_t)&(((ehci_pipe_struct_t*)0)->current_nak_count);
    uint8_t                        matched = (uint8_t)FALSE;
    
    ehci_pipe_ptr_temp =  (ehci_pipe_struct_t*)pipe_handle;
    pipe_ptr           = &ehci_pipe_ptr_temp->common;
    
    USB_EHCI_Host_lock();    
    if ((pipe_ptr != NULL) && (pipe_ptr->open == (uint8_t)TRUE))
    {
        for (ehci_pipe_ptr_temp = (ehci_pipe_struct_t*)usb_host_ptr->pipe_descriptor_base_ptr; ehci_pipe_ptr_temp != NULL; ehci_pipe_ptr_temp = (ehci_pipe_struct_t*)ehci_pipe_ptr_temp->common.next)
        {
            pipe_ptr = &ehci_pipe_ptr_temp->common;
            if ((pipe_ptr->open) && (ehci_pipe_ptr_temp == pipe_handle))
            {
                matched = (uint8_t)TRUE;
                break;
            }
        }
        if (matched)
        {
            _usb_ehci_free_resources(handle, (pipe_struct_t*)ehci_pipe_ptr_temp);
            OS_Mem_zero((void*)((uint32_t)ehci_pipe_ptr_temp + offset_zero_set), offset - offset_zero_set + 1);
            pipe_ptr->open            = (uint8_t)FALSE;
            pipe_ptr->dev_instance    = NULL;
            pipe_ptr->endpoint_number = 0;
            pipe_ptr->direction       = 0;
            pipe_ptr->pipetype        = 0;
            pipe_ptr->max_packet_size = 0;
            pipe_ptr->interval        = 0;
            pipe_ptr->flags           = 0;
            pipe_ptr->nak_count       = 0;
            pipe_ptr->nextdata01      = 0;
            pipe_ptr->tr_list_ptr     = NULL;
        }
        else
        {
#ifdef _DEBUG        
            USB_PRINTF("_usb_ehci_close_pipe can't find target pipe\n");
#endif
            USB_EHCI_Host_unlock();
            return USBERR_INVALID_PIPE_HANDLE;
        }
    }
    else
    {
#ifdef _DEBUG
        USB_PRINTF("_usb_ehci_close_pipe invalid pipe \n");
#endif
        USB_EHCI_Host_unlock();
        return USBERR_INVALID_PIPE_HANDLE;
    }
 
    USB_EHCI_Host_unlock();
    return USB_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_update_dev_address
*  Returned Value : USB Status
*  Comments       :
*        Update the queue in the EHCI hardware Asynchronous schedule list
*        to reflect new device address.
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_update_dev_address
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      pipe_struct_t*                  pipe_ptr
   )
{
    ehci_qh_struct_t*                  qh_ptr = NULL;//, QH_prev_ptr = NULL;
    bool                               active_async;
    ehci_pipe_struct_t*                pipe_descr_ptr = (ehci_pipe_struct_t*) pipe_ptr;
    uint32_t                           temp;
    uint8_t                            address;
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    usb_host_ptr = (usb_ehci_host_state_struct_t*) handle;
 

    qh_ptr = pipe_descr_ptr->qh_for_this_pipe;
 
#if 0 
    while ((EHCI_MEM_READ(qh_ptr->horiz_link_ptr) & EHCI_QH_HORIZ_PHY_ADDRESS_MASK) != (uint32_t) usb_host_ptr->active_async_list_ptr)
    {
        //QH_prev_ptr = qh_ptr;
        qh_ptr = (void *) EHCI_MEM_READ((qh_ptr->horiz_link_ptr) & EHCI_QH_HORIZ_PHY_ADDRESS_MASK);
        if (qh_ptr == pipe_descr_ptr->qh_for_this_pipe)
        {
            break;
        }
    }
 #endif
    /* Queue head is already on the active list. Simply update the queue */
    active_async = usb_hal_ehci_get_usb_cmd(usb_host_ptr->usbRegBase) & EHCI_USBCMD_ASYNC_SCHED_ENABLE;
    if (active_async)
    {
        /* stop async schedule */
    usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
        while (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_ASYNCH_SCHEDULE)
        {
        }
    }
    //_usb_ehci_init_qh(handle, pipe_descr_ptr, qh_ptr, QH_prev_ptr, (void *)EHCI_MEM_READ(qh_ptr->next_qtd_link_ptr));
    address = (usb_host_dev_mng_get_address(pipe_descr_ptr->common.dev_instance)) & 0x7F; 
    
    temp = usb_hal_ehci_get_ep_capab_charac1(qh_ptr);
    
    temp &= ~(0x07FF007F);
    /* Initialize the endpoint capabilities registers */
    temp |= ((pipe_descr_ptr->common.max_packet_size << EHCI_QH_MAX_PKT_SIZE_BITS_POS) | address);
    usb_hal_ehci_set_ep_capab_charac1(qh_ptr, temp);
 
    if (active_async)
    {
    usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
    }
 
    return USB_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_timer
*  Returned Value : USB Status
*  Comments       :
*        Updates Asynchronous list QTDs timer information. Removes QTDs
*        with timeout.
*END*-----------------------------------------------------------------*/
void _usb_ehci_process_timer
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle
   )
{
#if 0
   usb_ehci_host_state_struct_t*               usb_host_ptr;
   ACTIVE_QH_MGMT_STRUCT_PTR                    active_list_member;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

   for (active_list_member = usb_host_ptr->active_async_list_ptr; active_list_member != NULL; active_list_member = active_list_member->NEXT_ACTIVE_QH_MGMT_STRUCT_PTR)
   {
      if (active_list_member->TIME > 0)
         if (!--active_list_member->TIME)
            _usb_ehci_process_qh_list_tr_complete(handle, usb_host_ptr->active_async_list_ptr);
   }
#endif
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_cancel_transfer
*  Returned Value : None
*  Comments       :
*        Cancel a transfer
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_cancel_transfer
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle,

      /* The pipe descriptor to queue */            
      pipe_struct_t*               pipe_ptr,
      
      /* [IN] the transfer parameters struct */
      tr_struct_t*                 current_pipe_tr_struct_ptr
   )
{
    usb_ehci_host_state_struct_t*               usb_host_ptr;
    ehci_qh_struct_t*                           qh_ptr;
    ehci_qtd_struct_t*                          qtd_ptr;
    ehci_qtd_struct_t*                          temp_qtd_ptr;
    ehci_qtd_struct_t*                           start_qtd_ptr;
    ehci_qh_struct_t*                           active_list_member_ptr = NULL;
    ehci_qtd_struct_t*                          prev_qtd_ptr = NULL;
    tr_struct_t*                                temp_tr = NULL;
    uint32_t                                    temp = 0;
    ehci_pipe_struct_t*                         pipe_descr_ptr = (ehci_pipe_struct_t*) pipe_ptr;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*) handle;

    if (pipe_descr_ptr->common.open == 0)
        return USB_OK;
    if ((pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE) || (pipe_descr_ptr->common.pipetype == USB_BULK_PIPE))
    {
        active_list_member_ptr = usb_host_ptr->active_async_list_ptr;
    }
    else if (pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE)
    {
        active_list_member_ptr = usb_host_ptr->active_interrupt_periodic_list_ptr;
    }

    while (!active_list_member_ptr && (active_list_member_ptr != pipe_descr_ptr->qh_for_this_pipe))
    {
        active_list_member_ptr = active_list_member_ptr->next;
    }

    if (active_list_member_ptr == NULL)
    {
#ifdef _DEBUG
        USB_PRINTF("can't find target qh in _usb_ehci_cancel_transfer\n");
#endif
        return USBERR_INVALID_PIPE_HANDLE;
    }
    
    if ((pipe_descr_ptr->common.pipetype == USB_CONTROL_PIPE) || (pipe_descr_ptr->common.pipetype == USB_BULK_PIPE))
    {
        qh_ptr = pipe_descr_ptr->qh_for_this_pipe;
        qtd_ptr = qh_ptr->qtd_head;

        prev_qtd_ptr = NULL;
        start_qtd_ptr = NULL;
        /* remove all the qtd except the tr which is partial completed */
        while ((((uint32_t)qtd_ptr & EHCI_QTD_T_BIT) == 0 ) && (qtd_ptr != NULL))
        {
            temp_tr = (tr_struct_t*)(qtd_ptr->tr);

            if((NULL == current_pipe_tr_struct_ptr) || (current_pipe_tr_struct_ptr == temp_tr))
            {
                /* Set Asynch_Enable bit = 0 */
                usb_hal_echi_disable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
                while (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_ASYNCH_SCHEDULE)
                {
                }
                    
                while(0 == (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC))
                {
                    temp_qtd_ptr = qtd_ptr->next;
                    /* Dequeue the used QTD */
                    _usb_ehci_free_qtd(handle, (void*)qtd_ptr);
                    qtd_ptr = temp_qtd_ptr;
                }
                temp_qtd_ptr = qtd_ptr->next;
                /* Dequeue the used QTD */
                _usb_ehci_free_qtd(handle, (void*)qtd_ptr);
                qtd_ptr = temp_qtd_ptr;
                if((NULL!= temp_tr) && (NULL != temp_tr->callback))
                {
                    temp_tr->callback((void*)temp_tr,
                                              temp_tr->callback_param,
                                              temp_tr->send_phase ? temp_tr->tx_buffer : temp_tr->rx_buffer,
                                              0,
                                              USBERR_TR_CANCEL);
                }
                if(NULL != prev_qtd_ptr)
                {
                    prev_qtd_ptr->next = (void *)qtd_ptr;
                    if(NULL == qtd_ptr)
                    {
                        usb_hal_ehci_set_qtd_terminate_bit( prev_qtd_ptr);
                    }
                    else
                    {
                        usb_hal_ehci_link_qtd( prev_qtd_ptr, (uint32_t)qtd_ptr);
                    }
                }

                if(NULL == start_qtd_ptr)
                {
                    start_qtd_ptr = qtd_ptr;
                }               
                usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
            }
            else
            {
                if(NULL == start_qtd_ptr)
                {
                    start_qtd_ptr = qtd_ptr;
                }
                while(0 == (usb_hal_ehci_get_qtd_token(qtd_ptr) & EHCI_QTD_IOC))
                {
                    qtd_ptr = qtd_ptr->next;
                }
                prev_qtd_ptr = qtd_ptr;
                qtd_ptr = qtd_ptr->next;
            }
        }

        if (NULL != start_qtd_ptr)
        {
            usb_hal_ehci_set_qh_next_qtd_link_ptr(qh_ptr, (uint32_t)start_qtd_ptr);
        }
        else
        {
            usb_hal_ehci_set_qh_next_qtd_link_terminate(qh_ptr);
        }
        qh_ptr->qtd_head = (void*)start_qtd_ptr;

    usb_hal_ehci_enable_usb_cmd_async_sched(usb_host_ptr->usbRegBase);
    } /* Endif */
   
    /****************************************************************************
    SGARG: Add the ability to cancel the transfers for the interrupt pipe. Note that
    interrupts QHs are in the periodic list and they must be unlinked from all
    the possible frame lists that are linked to them.   
    *****************************************************************************/
 
    else if (pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE) 
    {
        /* Get the queue head for this pipe */
        qh_ptr = pipe_descr_ptr->qh_for_this_pipe;
        /* Get the first QTD for this Queue head */
        qtd_ptr = qh_ptr->qtd_head;

        if ((((uint32_t)qtd_ptr & EHCI_QTD_T_BIT) == 0 ) && (qtd_ptr != NULL))
        {         
            /* Now we can disable the QTD list from this QH */
            
        usb_hal_ehci_set_qh_next_qtd_link_terminate(qh_ptr);
                   
            /* start pointer points to the first QTD in the final list*/
            start_qtd_ptr = NULL;
                      
            /* previous pointer points to NULL in the beginning */
            prev_qtd_ptr =  NULL;
 
            do 
            {
                if ((NULL == current_pipe_tr_struct_ptr) || (qtd_ptr->tr == (void*)current_pipe_tr_struct_ptr))
                {
                    /* if list already started we connect previous QTD with next one*/
                    if(prev_qtd_ptr != NULL) 
                    {
            usb_hal_ehci_link_qtd(prev_qtd_ptr, usb_hal_ehci_get_next_qtd_ptr(qtd_ptr));
                        prev_qtd_ptr->next = qtd_ptr->next;
                    }
                    
                     /* if list already started we link previous pointer*/
                    temp_qtd_ptr = qtd_ptr;
                    
                    /* advance the QTD pointer */
                    qtd_ptr = (ehci_qtd_struct_t*) usb_hal_ehci_get_next_qtd_ptr(qtd_ptr);
                    
                    /* Dequeue the used QTD */
                    _usb_ehci_free_qtd(handle, (void*)temp_qtd_ptr);

                    if((NULL!= temp_qtd_ptr->tr) && (NULL != ((tr_struct_t*)temp_qtd_ptr->tr)->callback))
                    {
                        ((tr_struct_t*)temp_qtd_ptr->tr)->callback(temp_qtd_ptr->tr,
                                                  ((tr_struct_t*)temp_qtd_ptr->tr)->callback_param,
                                                  ((tr_struct_t*)temp_qtd_ptr->tr)->send_phase ? ((tr_struct_t*)temp_qtd_ptr->tr)->tx_buffer : ((tr_struct_t*)temp_qtd_ptr->tr)->rx_buffer,
                                                  0,
                                                  USBERR_TR_CANCEL);
                    }
                }
                else 
                {
                    if(start_qtd_ptr == NULL)
                    {
                       /* Initialize the start pointer */
                       start_qtd_ptr =  qtd_ptr;
                    }
     
                    /* store the previous qtd pointer */
                    prev_qtd_ptr = qtd_ptr;
                                      
                    /* advance the QTD pointer */
                    qtd_ptr = (ehci_qtd_struct_t*) usb_hal_ehci_get_next_qtd_ptr(qtd_ptr);
                    
                }
            } while ((((uint32_t)qtd_ptr & EHCI_QTD_T_BIT) == 0 ) && (qtd_ptr != NULL));
  
            if(start_qtd_ptr != NULL) 
            {
                /* Queue the transfer onto the relevant queue head */
        usb_hal_ehci_set_qh_next_qtd_link_ptr(qh_ptr, (uint32_t)start_qtd_ptr);
            }
            
            qh_ptr->qtd_head = (void*)start_qtd_ptr;
               
            /* Clear all error conditions */
        temp = usb_hal_ehci_get_qh_status(qh_ptr);
        usb_hal_ehci_set_qh_status(qh_ptr, temp & EHCI_QH_TR_OVERLAY_DT_BIT);
        }
    }
    return USB_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_calculate_uframe_tr_time
*  Returned Value : bus time in nanoseconds or -1 if error
*  Comments       :
*        Calculate the high speed bus transaction time (USB2.0 Spec 5.11.3)
* in micro seconds for given number of bytes and type of pipe. Handshake
time is included in this and worst case bt stuffing time is take in to
account.
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_calculate_uframe_tr_time (

      uint32_t     bytes,

      uint8_t      pipetype

   )
{
    uint32_t  uframe_tr_time = 0;
    /* only high speed is considered, we need implement the ls/fs calculation */

    /******************************************************************
    The following formulae taken from USB specification are used
    Non-Isochronous Transfer (Handshake Included)
    = (55 * 8 * 2.083) + (2.083 * Floor(3.167 + BitStuffTime(Data_bc))) +
    Host_Delay
    Isochronous Transfer (No Handshake)
    = (38 * 8 * 2.083) + (2.083 * Floor(3.167 + BitStuffTime(Data_bc))) +
    Host_Delay
    ******************************************************************/

    if (pipetype == USB_ISOCHRONOUS_PIPE)
    {
        /******************************************************************
        Host delay has been taken as 3 nano seconds (3000 pico seconds to guess).
        Linux code takes a rough guess of 5 nano seconds.
        ******************************************************************/

        uframe_tr_time = 38 * 8 * 2083 +
                         2083 * ((3167 + BitStuffTime(1000*bytes))/1000) +
                         VUSB_HS_DELAY;
    }
    else
    {
        /******************************************************************
        Host delay has been taken as 3 nano seconds (3000 pico seconds to guess).
        Linux code takes a rough guess of 5 nano seconds.
        ******************************************************************/
        uframe_tr_time = 55 * 8 * 2083 +
                       2083 * ((3167 + BitStuffTime(1000*bytes))/1000) +
                       VUSB_HS_DELAY; 
    }

    /******************************************************************
    convert the times back to micro seconds
    ******************************************************************/
    uframe_tr_time = uframe_tr_time/1000000;

    /******************************************************************
    if time is less than 1 micro seconds we take an assumption of 1 Micro sec.
    This is true for transfers that are few bytes esp. Interrupt transfers.
    Actually if the bytes are less than 36 bytes, it will always be less than
    1 Micro seconds.
    ******************************************************************/


    if(uframe_tr_time < 1)  uframe_tr_time = 1;

    return uframe_tr_time;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_get_frame_number
*  Returned Value : uint32_t
*  Comments       :
*        Get the frame number
*END*-----------------------------------------------------------------*/

uint32_t _usb_ehci_get_frame_number
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle
   )
{

    usb_ehci_host_state_struct_t*       usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    /* Get the frame number (not the uframe number */
    return((usb_hal_ehci_get_frame_index(usb_host_ptr->usbRegBase) & 0x1FFF)>> 3);
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_get_micro_frame_number
*  Returned Value : uint32_t
*  Comments       :
*        Get the micro frame number
*END*-----------------------------------------------------------------*/

uint32_t _usb_ehci_get_micro_frame_number
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle
   )
{
    usb_ehci_host_state_struct_t*       usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;


    /* Get the uframe number */
    return(usb_hal_ehci_get_frame_index(usb_host_ptr->usbRegBase) & 0x07);
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_init_periodic_list
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the type of transfer (Iso or Periodic) and
   allocatest the slots in periodic list. Once all slots are found,
   it updates the bandwidth list to reflect the change.
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_init_periodic_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      pipe_struct_t*                  pipe_ptr
   )
{
    usb_ehci_host_state_struct_t*       usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
    uint32_t*                            temp_periodic_list_ptr = NULL;
    ehci_itd_struct_t*                  itd_ptr;
    void*                                prev_ptr;
    list_node_struct_t*                 temp_itd_node_ptr;
    list_node_struct_t*                 temp_sitd_node_ptr;
    ehci_sitd_struct_t*                 sitd_ptr;
    ehci_pipe_struct_t*                 pipe_descr_ptr = (ehci_pipe_struct_t*) pipe_ptr;
    uint8_t                              speed;
    uint32_t                             i;
    speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);
    if(!usb_host_ptr->periodic_list_initialized)
    {
        /* Set T-Bits of all elements in periodic frame list to 1 */
        temp_periodic_list_ptr = (uint32_t *)usb_host_ptr->periodic_list_base_addr;

        for (i=0;i<usb_host_ptr->frame_list_size; i++)
        {
            EHCI_MEM_WRITE(*temp_periodic_list_ptr,EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT);
            temp_periodic_list_ptr++;
        }

        usb_host_ptr->periodic_list_initialized = TRUE;
    }

    /*******************************************************************************
    Initialize the ITDS list if this is High speed pipe.
    *******************************************************************************/
    if ((!usb_host_ptr->itd_list_initialized) && (speed == USB_SPEED_HIGH))
    {
        /* Enqueue all the ITDs */
        itd_ptr = usb_host_ptr->itd_base_ptr;
#if USBCFG_EHCI_MAX_ITD_DESCRS
        /* Enqueue all the ITDs */
        for (i=0; i<USBCFG_EHCI_MAX_ITD_DESCRS; i++)
        {
            /* Set the dTD to be invalid */
        usb_hal_ehci_set_ITD_terminate_bit(itd_ptr);
            /* Set the Reserved fields to 0 */
            itd_ptr->scratch_ptr = (void *)usb_host_ptr;
            _usb_ehci_free_ITD((usb_host_handle)usb_host_ptr, (void *)itd_ptr);
            itd_ptr++;
        }

         /* initialize all nodes and link them */
        temp_itd_node_ptr =  (list_node_struct_t*) usb_host_ptr->active_iso_itd_periodic_list_head_ptr;
        prev_ptr = NULL;
        for(i=0;i<USBCFG_EHCI_MAX_ITD_DESCRS;i++)
        {
            /* next node is not an active node */
            temp_itd_node_ptr->next_active = FALSE;
 
            /* previous node connetcion */
            temp_itd_node_ptr->prev = prev_ptr;
 
            /* move pointer */
            prev_ptr =  temp_itd_node_ptr;
 
            /* move next */
            temp_itd_node_ptr++;
 
            /* store the next pointer in previous node */
            ((list_node_struct_t*) prev_ptr)->next = temp_itd_node_ptr;
        }
#endif

        usb_host_ptr->itd_list_initialized = TRUE;
    }
    
    /*******************************************************************************
    Initialize the SITDS list if this is full speed or low speed pipe.
    *******************************************************************************/
    if ((!usb_host_ptr->sitd_list_initialized) && (speed != USB_SPEED_HIGH))
    {
        sitd_ptr = usb_host_ptr->sitd_base_ptr;
#if USBCFG_EHCI_MAX_SITD_DESCRS

        /* Enqueue all the SITDs */
        for (i = 0; i < USBCFG_EHCI_MAX_SITD_DESCRS; i++)
        {
            /* Set the dTD to be invalid */
        usb_hal_ehci_set_sitd_next_link_terminate_bit(sitd_ptr);
            /* Set the Reserved fields to 0 */
            sitd_ptr->scratch_ptr = (void *)usb_host_ptr;
            _usb_ehci_free_SITD((usb_host_handle)usb_host_ptr, (void *)sitd_ptr);
            sitd_ptr++;
        }

        /* initialize all nodes and link them */
        temp_sitd_node_ptr = (list_node_struct_t*) usb_host_ptr->active_iso_sitd_periodic_list_head_ptr;
        prev_ptr = NULL;
        for(i = 0; i < USBCFG_EHCI_MAX_SITD_DESCRS; i++)
        {
            /* next node is not an active node */
            temp_sitd_node_ptr->next_active = FALSE;

            /* previous node connection */
            temp_sitd_node_ptr->prev = prev_ptr;

            /* move pointer */
            prev_ptr =  temp_sitd_node_ptr;

            /* move next */
            temp_sitd_node_ptr++;

            /* store the next pointer in previous node */
            ((list_node_struct_t*) prev_ptr)->next = temp_sitd_node_ptr;
        }
#endif
        usb_host_ptr->sitd_list_initialized = TRUE;
    }
    return USB_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_allocate_bandwidth
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the type of transfer (Iso or Periodic) and
   allocatest the slots in periodic list. Once all slots are found,
   it updates the bandwidth list to reflect the change.
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_allocate_bandwidth
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      pipe_struct_t*                  pipe_ptr
   )
{
    //uint32_t                             max_ss_slots;
    usb_status                           status = USB_OK;
    //usb_ehci_host_state_struct_t*       usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   // uint32_t*                            temp_periodic_list_ptr = NULL;
    //ehci_itd_struct_t*                  itd_ptr;
    //void*                                prev_ptr;
    //list_node_struct_t*                 temp_itd_node_ptr,temp_sitd_node_ptr;
    //ehci_sitd_struct_t*                 sitd_ptr;
    ehci_pipe_struct_t*                 pipe_descr_ptr = (ehci_pipe_struct_t*) pipe_ptr;
    uint8_t                              speed;
 
    speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance); 

    /*******************************************************************************
    Initialize the periodic list if it is not initialized already. Note that
    this code could have been put under Host_init routine but since host must
    initialze witthin 1 mili second under OTG timing restrictions, this code
    has been moved here.
    *******************************************************************************/
    status = _usb_ehci_init_periodic_list(handle, pipe_ptr);
    if (USB_OK != status)
    {
#if _DEBUG
        USB_PRINTF("_usb_ehci_init_periodic_list failed\n");
#endif
        return status;
    }

    /*******************************************************************************
    Go Through the  PERIODIC_FRAME_LIST_BW record to find the available slots. Here
    starts the complexity of the process. See the following from USB specifications.
 
    An isochronous endpoint must specify its required bus access period. Full-/high-speed
    endpoints must specify a desired period as (2^(bInterval-1)) x F, where bInterval is
    in the range one to (and including) 16 and F is 125 µs for high-speed and 1ms for
    full-speed. This allows full-/high-speed isochronous transfers to have rates slower
    than one transaction per (micro)frame.
 
    An endpoint for an interrupt pipe specifies its desired bus access period. A
    full-speed endpoint can specify a desired period from 1 ms to 255 ms. Low-speed
    endpoints are limited to specifying only 10 ms to 255 ms. High-speed endpoints can
    specify a desired period (2^(bInterval-1))x125 µs, where bInterval is in the range 1 to
    (including) 16.
 
 
    For high speed interrupts, Since we must have slots available at uframe_interval
    periods, if we can not find any slot starting from anywhere in Micro frame number
    1 to Micro frame number equal to uframe_interval....till the end of
    periodic list we are out of bandwidth. For low and full speed device we do the same
    starting from frame 1 to frame frame_interval.
 
    ********************************************************************************/
 
    /*************************************************************************
    If it is a high speed interrupt, allocate the slots at micro frame interval.
    Or if low or full speed, allocate slots at frame intervals. Note that
    the possible values of the interval for high speed are 1,2,4,8,16.
    **************************************************************************/

    if (speed == USB_SPEED_HIGH)
    {
        status = _usb_ehci_commit_high_speed_bandwidth(handle, pipe_descr_ptr);
        if (USB_OK != status)
        {
#if _DEBUG
            USB_PRINTF("alloc high speed bandwidth failed\n");
#endif
            return status;
        }
   }
   else
   {

        /***************************************************************************
        Find the possible SS and CS that this endpoint could have. Check  the
        periodic list for each such slot. If it fails, restart searching the periodic
        list for new slot.
        ****************************************************************************/
        /********************************************************************
        It is no simple game as it sounds in the code here. We should consider
        the time taken by the transaction and set the start split and complete
        split transactions for low and full speed devices. Read the following
        rules carefully to understand the code simplifying assumptions.
        ********************************************************************/
  
        /********************************************************************
        We are allowed to use only 80% of a frame for full speed and low speed
        periodic transfers. This is to provide space for Bulk and Control
        transfers. This means that if a frame slot exceeds .8x1000 micro seconds
        ,it is out of space. Since this transaction is a low or full speed
        transaction we need to find which start split and complete split could
        be scheduled. The following rules are followed in this code. See section
        11.18 in USB specs for details. Y0-Y7 refers to micro frames on B bus
        which is 1 Microframe behind H Bus.
  
        1) Host never schedules a start split in Y6.
  
         If We can not schedule the SS in H0-H6 for OUT, we will move to next frame.
         For IN transactions we must schedule SS early enough to ensure that
         all CS are scheduled within H7. Simplification is that we don't want to
         cross the microframe boundry in our implementation of CS.
  
        ISO OUT
        ========
        2) for Iso OUTs we must determine how many 188 bytes chunks it will
        take and schedule each of them in a Microframe. Each such microframe
        (also called budgeted microframe) should have a SS in previous
        micro frame. We always try to do a SS in H0-H6.
        If this is not possible we just move over to next frame.
  
        3) There is no complete split scheduled for Iso OUT
  
  
        ISO IN
        =======
  
        4) For ISO IN, we must schedule a complete split in each micro frame
        following the micro frame in which transaction is budgeted. Also
        if the last micro frame in which complete-split is scheduled is
        less that Y6, we schedule two more in Y6 and Y7.
  
        5)We take a simplyfying assumption that we budget the transaction
        between H0-H5 and schedule a complete split in rest of the
        micro frames till H7.
  
  
        Interrupt IN/OUT
        ================
        6)For interrupt IN/OUT host must schedule a complete split in the
        next two micro frames after transaction is budgeted. An additional
        complete split must be scheduled in the 3rd micro frame unless the
        transaction was budgeted to start in Y6.
  
        7)We make a simplyfying assumption that if the transaction is not
        within H0-H4, we will go to the next frame (we will never schedule
        interrupt transactions to the H5-H7). This will allow us to
        schedule 3 CS in H5-H7 and 1 SS in the H0-H3.
  
        8)Max data size for low speed interrupt is  8 bytes
         Max data size for full speed interrupt is 64 bytes
  
         This means that we will never need more than a Microframe for
         an Interrupt OUT or IN.
  
  
        9) Host never schedules for than 16 SS (start splits) in a Micro
        frame for a given TT.
  
        ********************************************************************/
        status = _usb_ehci_commit_split_bandwidth(handle, pipe_descr_ptr);
        if (USB_OK != status)
        {
#if _DEBUG
            USB_PRINTF("alloc low/full speed bandwidth failed\n");
#endif
            return status;
        }
    }

    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_commit_bandwidth_slots
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static usb_status _usb_ehci_commit_split_bandwidth
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*              pipe_descr_ptr
)
{

    uint32_t                            max_ss_slots;
    usb_status                          status = USB_OK;
    


    switch (pipe_descr_ptr->common.pipetype)
    {
        case USB_ISOCHRONOUS_PIPE:
           /* Get max. allocated start split slots, at least 1 */
           max_ss_slots = (pipe_descr_ptr->common.max_packet_size + EHCI_START_SPLIT_MAX_BUDGET - 1) / EHCI_START_SPLIT_MAX_BUDGET;
    
           if (pipe_descr_ptr->common.direction == USB_SEND)
           {
                /***********************************************************
                For ISO OUT we want to schedule the SS in H0-H6.
                See the rules above. This means that we can do
                188*7 = 1316 Max packetsize in one frame, return fail if this
                is not the case.
                ***********************************************************/
                if(max_ss_slots > 6)
                {
                    return USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                }

                /***********************************************************************
               If the packet size is within budget, try to allocate bandwidth for
               all possible start split combinations starting from microframe 0 to 7.
               ************************************************************************/
                status = allocate_ss_cs_bandwidth(handle,pipe_descr_ptr,max_ss_slots,0,5);
            }
            else
            {
                /***********************************************************************
                For ISO IN, we want to schedule SS between H0-H5 so that complete splits
                could be scheduled latest by H6. In best case we will have 7 slots
                to complete split transaction.
                ************************************************************************/
                if(max_ss_slots > 6)
                {
                    return USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                }

                /* Note that ideally we should start SS as late as possible in this
                case because we want to minimize the number of slots when a complete
                transaction is scheduled but in this case we start from 0 always
                for simplicity. This should be addressed if we run in bandwidth
                problems because of more ISO IN pipes.*/
  
                /* schedule a SS from 0 to as late as 7-max_ss_slots */
                status = allocate_ss_cs_bandwidth(handle,pipe_descr_ptr,1,0,(6-max_ss_slots));
            }
            break;
                
        case USB_INTERRUPT_PIPE:
            /*****************************************************************************
            For interrupt pipes, life is cool because we need only 1 microframe max to
            send all data (see the data limits on interrupt pipes in comments above).
            We still make a simplifying assumption to schedule SS in Y0-Y3.
            *****************************************************************************/
            if (pipe_descr_ptr->common.direction == USB_SEND)
            {
                /* 1 slot required between microframes 0-3 to send data and we can schedule CS in next 3 of them*/
                status = allocate_ss_cs_bandwidth(handle,pipe_descr_ptr,1,0,3);
            }
            else
            {
                /* 1 slot required between microframes 0-3 to so SS and we can schedule CS in next 3 of them*/
                status = allocate_ss_cs_bandwidth(handle,pipe_descr_ptr,1,0,3);
            }
            break;
        default:
            break;
    }
    
    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_split_bandwidth
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static usb_status _usb_ehci_free_split_bandwidth
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*              pipe_descr_ptr
)
{
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    uint8_t*                            frame_list_bw_ptr;
    ehci_qh_struct_t*                  qh_ptr = NULL;
    uint32_t                            i,j;//,uframe;
    uint32_t                            time_for_action;
    uint8_t                             interval = pipe_descr_ptr->common.interval;
    //bool                                bandwidth_slots[8] = {FALSE,FALSE,FALSE,FALSE, \
    //                                                            FALSE,FALSE,FALSE,FALSE};
                                                                
    //uint8_t                             slot_found = 1;
    uint32_t                            start;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

    time_for_action = pipe_descr_ptr->bwidth;
    start = pipe_descr_ptr->start_frame;
    qh_ptr = pipe_descr_ptr->qh_for_this_pipe;

    for (i = start; i < usb_host_ptr->frame_list_size; i += interval)
    {
        for (j = 0; j < 8; j++)
        {
            if (pipe_descr_ptr->bwidth_slots[j])
            {
                frame_list_bw_ptr[8 * i + j] -= time_for_action;
            }
        }
        unlink_interrupt_qh_from_periodiclist(handle, qh_ptr, pipe_descr_ptr, i);
    }

    _usb_ehci_free_qh(handle, qh_ptr);

    return USB_OK;
}



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_commit_high_speed_bandwidth
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static usb_status _usb_ehci_commit_high_speed_bandwidth
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*              pipe_descr_ptr
)
{
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    uint8_t*                            frame_list_bw_ptr;
    ehci_qh_struct_t*                  qh_ptr = NULL;
    uint32_t                            i, j, uframe;
    uint32_t                            time_for_action;
    uint8_t                             interval = pipe_descr_ptr->common.interval;
    uint8_t                              bandwidth_slots[8] = {0,0,0,0, \
                                                                0,0,0,0};
                                                                
    uint8_t                             slot_found = 1;
    uint32_t                            start = 0;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

    time_for_action = _usb_ehci_calculate_uframe_tr_time(
                                 pipe_descr_ptr->common.max_packet_size * pipe_descr_ptr->common.trs_per_uframe,
                                 pipe_descr_ptr->common.pipetype);

    for (i = 0; (i < pipe_descr_ptr->common.interval) && (i < usb_host_ptr->frame_list_size * 8); i++)
    {
        slot_found = 1;
        for (j = i; j < usb_host_ptr->frame_list_size * 8; j += interval)
        {
            uframe = j % 8;
            bandwidth_slots[uframe] = 1;

            if((frame_list_bw_ptr[j] + time_for_action) > 100)
            {
                slot_found = 0;
                break;
            }
            
        }
        if (slot_found == 1)
        {
            start = i;
            break;
        }
    }

    if (slot_found == 1)
    {
        pipe_descr_ptr->bwidth = (uint16_t)time_for_action;

        pipe_descr_ptr->start_uframe = start;

        pipe_descr_ptr->start_split = (1 << (start%8));
        
        if (pipe_descr_ptr->common.pipetype == USB_ISOCHRONOUS_PIPE)
        {
            for(j = 0; j < 8; j++)
            {
                pipe_descr_ptr->bwidth_slots[j] = (uint8_t)bandwidth_slots[j];
            }
            for (i = start; i < usb_host_ptr->frame_list_size * 8; i += interval)
                frame_list_bw_ptr[i] += time_for_action;
        }

        if (pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE)
        {
            _usb_ehci_get_qh(handle, &qh_ptr);
            if (qh_ptr == NULL)
            {
                goto ERR;
            }
    
            _usb_ehci_init_qh(handle, pipe_descr_ptr, qh_ptr);
            qh_ptr->interval = interval;
            pipe_descr_ptr->qh_for_this_pipe = (void*)qh_ptr;
            
            for (i = start; i < usb_host_ptr->frame_list_size * 8; i += interval)
            {
                frame_list_bw_ptr[i] += time_for_action;
                link_interrupt_qh_to_periodiclist(handle, qh_ptr, pipe_descr_ptr, i/8);
            }
            
            if (usb_host_ptr->active_interrupt_periodic_list_ptr == NULL)
            {
                usb_host_ptr->active_interrupt_periodic_list_ptr = qh_ptr;
                usb_host_ptr->active_interrupt_periodic_list_tail_ptr = qh_ptr;
            }
            else
            {
                usb_host_ptr->active_interrupt_periodic_list_tail_ptr->next = (void*)qh_ptr;
                usb_host_ptr->active_interrupt_periodic_list_tail_ptr = qh_ptr;
            }
        }  
    }

    return USB_OK;
ERR:
    return USBERR_ERROR;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_high_speed_bandwidth
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static usb_status _usb_ehci_free_high_speed_bandwidth
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*              pipe_descr_ptr
)
{
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    uint8_t*                            frame_list_bw_ptr;
    ehci_qh_struct_t*                  qh_ptr = NULL;
    uint32_t                            i;//,j,uframe;
    uint32_t                            time_for_action;
    uint8_t                             interval = pipe_descr_ptr->common.interval;
    //bool                                bandwidth_slots[8] = {FALSE,FALSE,FALSE,FALSE, \
    //                                                            FALSE,FALSE,FALSE,FALSE};                                                          
    //uint8_t                             slot_found = 1;
    uint32_t                            start;
    
    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

    time_for_action = pipe_descr_ptr->bwidth;
    start = pipe_descr_ptr->start_uframe;
    qh_ptr = pipe_descr_ptr->qh_for_this_pipe;
    
    for (i = start; i < (usb_host_ptr->frame_list_size * 8); i+= interval)
    {
        frame_list_bw_ptr[i] -= time_for_action;
        unlink_interrupt_qh_from_periodiclist(handle, qh_ptr, pipe_descr_ptr, i/8);
    }
    
    _usb_ehci_free_qh(handle, qh_ptr);

    return USB_OK;
}



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_commit_bandwidth_slots
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static usb_status _usb_ehci_commit_bandwidth_slots
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*              pipe_descr_ptr,

      /* The starting slot from the periodic list*/
      uint32_t                          start,

      /* The interval for the pipe              */
      uint32_t                          interval,

      /* start split slots asked for this pipe, NULL for high speed pipe*/
      uint8_t*                          ss,

      /* Max number of SS slots required*/
      uint32_t                          max_ss_slots


)
{
    usb_ehci_host_state_struct_t*      usb_host_ptr;
    uint8_t*                            frame_list_bw_ptr;
    ehci_qh_struct_t*                  qh_ptr = NULL;
    uint32_t                            i,j,count;
    uint32_t                            status = USB_OK, time_for_action;
    uint8_t                             bandwidth_slots[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t                             speed = 0;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

    speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);
    /**************************************************************************
    If the speed is low or full,we have to find slots where we can issue
    split transactions. Read the following from USB 2.0 specs.
 
    11.18.2 TT Microframe Pipeline
 
    The TT implements a microframe pipeline of split transactions in support of
    a full-/low-speed bus. Startsplit transactions are scheduled a microframe
    before the earliest time that their corresponding full-/lowspeed transaction
    s expected to start. Complete-split transactions are scheduled in
    microframes that the full-/low-speed transaction can finish.
 
    This clearly says that we must check the budget in next microframe for
    Start splits and same microframe for complete splits.
    ***************************************************************************/
 
    /*************************************************************************
    If this pipe takes less than 188 bytes of time, we use lower number to
    save bandwidth time.
    **************************************************************************/
    if (max_ss_slots <= 1) //max_ss_slots should be always > 0
    {
        /* splitted transaction will have size less than max_packet_size */
        time_for_action = _usb_ehci_calculate_uframe_tr_time(
                             pipe_descr_ptr->common.max_packet_size,
                             pipe_descr_ptr->common.pipetype);
    }
    else
    {
        /* splitted transaction will have size less than EHCI_START_SPLIT_MAX_BUDGET */
        time_for_action = _usb_ehci_calculate_uframe_tr_time(
                             EHCI_START_SPLIT_MAX_BUDGET,
                             pipe_descr_ptr->common.pipetype);
    }


    /**********************************************************
    Check bandwidth in periodic list for the asked combination
    of start split transaction.
    ***********************************************************/
    for (i = start; i < usb_host_ptr->frame_list_size; i += interval)
    {
        /**********************************************************
        Interrupt OUT budget the next microframe after SS
        ***********************************************************/
        if ((pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE) &&
            (pipe_descr_ptr->common.direction == USB_SEND))
        {
            /* for the full / low interrupt transfer, only one start split, fine one slot from 0-3 */
            for (j = 0; j < max_ss_slots; j++)
            {
                if ((frame_list_bw_ptr[8 * i + ss[j] + 1] + time_for_action) > 100)
                {
                    status =  USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                    return status;
                }

                /* we never allocate last microframe for the slot so ss[j] is always less than 7 */
                bandwidth_slots[ss[j] + 1] = TRUE; /* at ss[j] the SS will be potentionally placed */
            }
        }
        /* for interrupt IN check 3 micro frames (starting from second) for CS*/
        else if ((pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE) &&
                 (pipe_descr_ptr->common.direction == USB_RECV))
        {
            for (j = 0; j < max_ss_slots; j++)
            {
                /* look for the budget in the microframe*/
                if (((frame_list_bw_ptr[8*i+ss[j]+1] + time_for_action) > 100) ||
                    ((frame_list_bw_ptr[8*i+ss[j]+2] + time_for_action) > 100) ||
                    ((frame_list_bw_ptr[8*i+ss[j]+3] + time_for_action) > 100))
                {
                    status =  USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                    return status;
                }

                bandwidth_slots[ss[j]+1] = TRUE;
                bandwidth_slots[ss[j]+2] = TRUE;
                bandwidth_slots[ss[j]+3] = TRUE;
            }
        }
        /* for ISO OUT check next micro frame */
        else if ((pipe_descr_ptr->common.pipetype == USB_ISOCHRONOUS_PIPE) &&
                 (pipe_descr_ptr->common.direction == USB_SEND))
        {
            for (j = 0; j < max_ss_slots; j++)
            {
                if ((frame_list_bw_ptr[8 * i + ss[j] + 1] + time_for_action) > 100)
                {
                    status =  USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                    return status;
                }

                bandwidth_slots[ss[j] + 1] = TRUE;
            }
        }
        /* for ISO IN check next all micro frames after first SS. We ignore next SS positions */
        else if ((pipe_descr_ptr->common.pipetype == USB_ISOCHRONOUS_PIPE) &&
                 (pipe_descr_ptr->common.direction == USB_RECV))
        {
            for (j = ss[0] + 1; j < 7; j++) //make sure that the last uframe is used for CS and not for iso
            {
                if ((frame_list_bw_ptr[8 * i + j] + time_for_action) > 100)
                {
                    status =  USB_log_error(__FILE__,__LINE__,USBERR_BANDWIDTH_ALLOC_FAILED);
                    return status;
                }
                bandwidth_slots[j] = TRUE;
            }
        }
    }
    
    pipe_descr_ptr->bwidth = (uint16_t)time_for_action;

    for(j = 0; j < 8; j++)
    {
        pipe_descr_ptr->bwidth_slots[j] = (uint8_t)bandwidth_slots[j];
    }

    pipe_descr_ptr->start_frame = start;


    /********************************************************************
    When Bandwidth is available,
    For ISOCHRONOUS full speed TRANSACTIONS, we just need to store the
    SS and CS positions inside frames.
    ********************************************************************/
    if (pipe_descr_ptr->common.pipetype == USB_ISOCHRONOUS_PIPE)
    {
        if (pipe_descr_ptr->common.direction == USB_SEND)
        {
            count = 0;
            for (j = 1; j < 8; j++)
            {
                if (bandwidth_slots[j])
                {
                    /* bandwidth is allocated at next position to SS so we use j-1 to start a split */
                    pipe_descr_ptr->start_split |= (1 << (j - 1));
                    count++;
                }
            }
            /* there is no complete split on a ISO OUT so we set the field to 0 */
            pipe_descr_ptr->complete_split = 0;
            pipe_descr_ptr->no_of_start_split = count;
        }
        else
        {
            pipe_descr_ptr->start_split = (uint8_t)(1 << ss[0]);
            /* we do complete split on all positions after first SS */
            for (j = 0; j < 8; j++)
            {
                if (bandwidth_slots[j])
                {
                    /* CS is scheduled in the next micro frame in which transaction is budgeted so we use j + 1 */
                    pipe_descr_ptr->complete_split |= (1 << (j + 1));
                }
            }
        }
        for (i = start; i < usb_host_ptr->frame_list_size; i += interval)
        {
            for (j = 0; j < 8; j++)
            {
                if (bandwidth_slots[j])
                {
                    frame_list_bw_ptr[8 * i + j] += time_for_action;
                }
            }
        }
    }

    /********************************************************************
    Since bandwidth is available, letus make the life easy for interrupt
    transfers. We will allocate a QH here and link it to the available
    slots in periodic slots. This is the best place to do it since we
    already know the slots (micro frames) that are available. We will however
    keep the QH inactive until a transfer is really queued on it. Life is
    not easy for Iso transfers however because they have no queue heads.
    In case of Iso transfers we should remember the slots available inside
    the pipe decriptor and use them when a transfer is queued.
    ********************************************************************/
    if (pipe_descr_ptr->common.pipetype == USB_INTERRUPT_PIPE)
    {
        for(j = 0; j < max_ss_slots; j++)
        {
            /*schedule a SS */
            pipe_descr_ptr->start_split |=  (1 << ss[j]);

            /*schedule a CS at the next three positions*/
            pipe_descr_ptr->complete_split |=  (1 << (ss[j]+2));
            pipe_descr_ptr->complete_split |=  (1 << (ss[j]+3));
            pipe_descr_ptr->complete_split |=  (1 << (ss[j]+4));
        }
        
        /******************************************************************
        Allocate a new queue head
        *******************************************************************/
        _usb_ehci_get_qh(handle, &qh_ptr);
        if (qh_ptr == NULL)
        {
            return USB_log_error(__FILE__,__LINE__,USBERR_ALLOC);
        }

        _usb_ehci_init_qh(handle, pipe_descr_ptr, qh_ptr);

        qh_ptr->interval = interval;
        
        if(speed != USB_SPEED_HIGH)
        {
            qh_ptr->interval = qh_ptr->interval << 3;
        }
        
        for (i = start; i < usb_host_ptr->frame_list_size; i += interval)
        {
            for (j = 0; j < 8; j++)
            {
                if (bandwidth_slots[j])
                {
                    frame_list_bw_ptr[8 * i + j] += time_for_action;
                }
            }
            link_interrupt_qh_to_periodiclist(handle, qh_ptr, pipe_descr_ptr, i);
        }
        if (usb_host_ptr->active_interrupt_periodic_list_ptr == NULL)
        {
            usb_host_ptr->active_interrupt_periodic_list_ptr = qh_ptr;
            usb_host_ptr->active_interrupt_periodic_list_tail_ptr = qh_ptr;
        }
        else
        {
            usb_host_ptr->active_interrupt_periodic_list_tail_ptr->next = (void*)qh_ptr;
            usb_host_ptr->active_interrupt_periodic_list_tail_ptr = qh_ptr;
        }
        //pipe_descr_ptr->actived = 1;
        pipe_descr_ptr->qh_for_this_pipe = (void*)qh_ptr;
    }
    return status;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_commit_bandwidth_slots
*  Returned Value : USB_OK or error
*  Comments       :
*        Allocates the slots for scheduling transactions on this pipe.
   This routine looks at the BW list and finds regular slots at the
   asked interval starting from 'start' slot number.
*END*-----------------------------------------------------------------*/

static uint32_t _usb_ehci_get_link_obj_interval
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                   handle,

      /* The pipe descriptor to queue */
      uint32_t*                         link_obj
   )
{
    ehci_itd_struct_t*                 itd_ptr = NULL;
    ehci_sitd_struct_t*                sitd_ptr = NULL;
    ehci_qh_struct_t*                  qh_ptr = NULL;
    uint32_t                            item_type;

    item_type = EHCI_GET_TYPE(link_obj);

    if (item_type == EHCI_ELEMENT_TYPE_ITD)
    {
        itd_ptr = (ehci_itd_struct_t*)(EHCI_MEM_READ(*link_obj) & EHCI_HORIZ_PHY_ADDRESS_MASK);
        return itd_ptr->interval;
    }
    else if (item_type == EHCI_ELEMENT_TYPE_SITD)
    {
        sitd_ptr = (ehci_sitd_struct_t*)(EHCI_MEM_READ(*link_obj) & EHCI_HORIZ_PHY_ADDRESS_MASK);
        return sitd_ptr->interval;
    }
    else if (item_type == EHCI_ELEMENT_TYPE_QH)
    {
        qh_ptr = (ehci_qh_struct_t*)(EHCI_MEM_READ(*link_obj) & EHCI_HORIZ_PHY_ADDRESS_MASK);
        return qh_ptr->interval;
    }

    return (uint32_t)-1;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : link_interrupt_qh_to_periodiclist
*  Returned Value : USB_OK or error
*  Comments       :
*        Links the QH to the given slot in periodic list. If pipe is
a high speed, slot number is the micro frame number to link QH in. If
pipe is a low speed, slot number is the frame number to link QH in. In
all cases, we link in frame except that we also update the microframe
schedule mask number.
*END*-----------------------------------------------------------------*/

static void link_interrupt_qh_to_periodiclist
   (

      /* [IN] the USB Host state structure */
      usb_host_handle                handle,

      /* QH that will be scheduled. */
      ehci_qh_struct_t*             qh_ptr,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*           pipe_descr_ptr,

      /* slot in which this QH should be scheduled */
      uint32_t                       slot_number

   )
{
    usb_ehci_host_state_struct_t*   usb_host_ptr;
    volatile uint32_t*               transfer_data_struct_ptr = NULL;
    volatile uint32_t*               prev_transfer_data_struct_ptr = NULL;
    uint32_t                         interval;
    //ehci_qh_struct_t*               active_qh_list;

    usb_host_ptr = (usb_ehci_host_state_struct_t*) handle;

    //active_qh_list = usb_host_ptr->active_interrupt_periodic_list_ptr;

    transfer_data_struct_ptr = (volatile uint32_t *)(usb_host_ptr->periodic_list_base_addr);
    transfer_data_struct_ptr += slot_number;

    if (usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT)
    {
        /* this item is the first one in this frame */
        //EHCI_MEM_WRITE(*transfer_data_struct_ptr,((uint32_t)qh_ptr | (EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS)));
        usb_hal_ehci_set_periodic_list_addr( transfer_data_struct_ptr, qh_ptr);
    }
    else
    {
        /* insert item into the link */
        while (!(usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_QUEUE_HEAD_POINTER_T_BIT))
        {
            if ((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) != (uint32_t)qh_ptr)
            {
                interval = _usb_ehci_get_link_obj_interval(handle, (uint32_t*)transfer_data_struct_ptr);
                if (interval >= qh_ptr->interval)
                {
                    prev_transfer_data_struct_ptr = transfer_data_struct_ptr;
                    transfer_data_struct_ptr = (uint32_t *)(usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
                    if ((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_QUEUE_HEAD_POINTER_T_BIT))
                        usb_hal_ehci_set_periodic_list_addr( transfer_data_struct_ptr, qh_ptr);
                }
                else
                {
                    if (prev_transfer_data_struct_ptr == NULL)
                    {
                        /* should insert this item as the first one */
                        usb_hal_ehci_set_qh_horiz_link_ptr( qh_ptr, usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) | EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS);
                        usb_hal_ehci_set_periodic_list_addr( transfer_data_struct_ptr, qh_ptr);
                    }
                    else
                    {
                        /* should insert this item after prev one */
                        usb_hal_ehci_set_qh_horiz_link_ptr( qh_ptr, usb_hal_ehci_get_periodic_list_addr( prev_transfer_data_struct_ptr) | EHCI_FRAME_LIST_ELEMENT_TYPE_QH << EHCI_QH_ELEMENT_TYPE_BIT_POS);         
                        usb_hal_ehci_set_periodic_list_addr( prev_transfer_data_struct_ptr, qh_ptr);
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }
    }
    return;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : unlink_interrupt_qh_from_periodiclist
*  Returned Value : USB_OK or error
*  Comments       :
*        Links the QH to the given slot in periodic list. If pipe is
a high speed, slot number is the micro frame number to link QH in. If
pipe is a low speed, slot number is the frame number to link QH in. In
all cases, we link in frame except that we also update the microframe
schedule mask number.
*END*-----------------------------------------------------------------*/

static void unlink_interrupt_qh_from_periodiclist
   (

      /* [IN] the USB Host state structure */
      usb_host_handle                handle,

      /* QH that will be scheduled. */
      ehci_qh_struct_t*             qh_ptr,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*           pipe_descr_ptr,

      /* slot in which this QH should be scheduled */
      uint32_t                       slot_number

   )
{
    usb_ehci_host_state_struct_t*   usb_host_ptr;
    uint32_t*                        transfer_data_struct_ptr = NULL;
    uint32_t*                        prev_transfer_data_struct_ptr = NULL;
    uint32_t*                        next_transfer_data_struct_ptr = NULL;
    //uint32_t                         interval;
    ehci_qh_struct_t*               cur_qh;
    ehci_qh_struct_t*               prev_qh;

    usb_host_ptr = (usb_ehci_host_state_struct_t*) handle;

    transfer_data_struct_ptr = (uint32_t *)(usb_host_ptr->periodic_list_base_addr);
    transfer_data_struct_ptr += slot_number;

    if (usb_host_ptr->active_interrupt_periodic_list_ptr == qh_ptr)
    {
        usb_host_ptr->active_interrupt_periodic_list_ptr = qh_ptr->next;
        if (usb_host_ptr->active_interrupt_periodic_list_tail_ptr == qh_ptr)
        {
            usb_host_ptr->active_interrupt_periodic_list_tail_ptr = NULL;
        }
    }
    else
    {
        if(NULL != usb_host_ptr->active_interrupt_periodic_list_ptr)
        {
            cur_qh = prev_qh = usb_host_ptr->active_interrupt_periodic_list_ptr;

            while ((cur_qh != NULL) && (cur_qh != qh_ptr))
            {
                prev_qh = cur_qh;
                cur_qh = cur_qh->next;
            }
            if(cur_qh == qh_ptr)
            {
                prev_qh->next = (void*)qh_ptr->next;
                if (usb_host_ptr->active_interrupt_periodic_list_tail_ptr == qh_ptr)
                {
                    usb_host_ptr->active_interrupt_periodic_list_tail_ptr = prev_qh;
                }
            }
        }
    }

    if ((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) == (uint32_t)qh_ptr)
    {
        /* this item is the first one in the frame list */
        next_transfer_data_struct_ptr = (uint32_t*)usb_hal_ehci_get_qh_horiz_link_ptr( qh_ptr);
    usb_hal_ehci_set_transfer_data_struct( transfer_data_struct_ptr, ((uint32_t)next_transfer_data_struct_ptr));
    }
    else
    {
        while (!(usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_QUEUE_HEAD_POINTER_T_BIT))
        {
            if ((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) != (uint32_t)qh_ptr)
            {
                prev_transfer_data_struct_ptr = transfer_data_struct_ptr;
                transfer_data_struct_ptr = (uint32_t *)(usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            }
            else
            {
                if (prev_transfer_data_struct_ptr == NULL)
                {
                }
                else
                {                   
                    usb_hal_ehci_set_transfer_data_struct( transfer_data_struct_ptr, (uint32_t)usb_hal_ehci_get_qh_horiz_link_ptr( qh_ptr));
                }
            }
        }
    }
    return;
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : unlink_periodic_data_structure_from_frame
*  Returned Value : None
*  Comments       :
*        unlinks the data structure from periodic list
*END*-----------------------------------------------------------------*/
void unlink_periodic_data_structure_from_frame(
                                          volatile uint32_t  *prev_transfer_data_struct_ptr,
                                          volatile uint32_t  *transfer_data_struct_ptr
)
{
         ehci_itd_struct_t*                          itd_ptr;
         ehci_sitd_struct_t*                         sitd_ptr;
         ehci_fstn_struct_t*                         FSTN_ptr;
         ehci_qh_struct_t*                           qh_ptr;
         uint32_t                                     next=0;

         /**************************************************************
          Find the void *to the next structure to be pointed in the list
         **************************************************************/

         switch (EHCI_GET_TYPE(transfer_data_struct_ptr))
         {

            case EHCI_ELEMENT_TYPE_ITD:
               itd_ptr = (ehci_itd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
               next = usb_hal_ehci_get_itd_next_link_pointer( itd_ptr);
               break;
            case EHCI_ELEMENT_TYPE_QH:
               qh_ptr = (ehci_qh_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
               next = usb_hal_ehci_get_qh_horiz_link_ptr( qh_ptr);
               break;
            case EHCI_ELEMENT_TYPE_SITD:
               sitd_ptr = (ehci_sitd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
               next = usb_hal_ehci_get_sitd_next_link_pointer( sitd_ptr);
               break;

            case EHCI_ELEMENT_TYPE_FSTN:
               FSTN_ptr = (ehci_fstn_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
               next = usb_hal_ehci_get_fstn_normal_path_link_ptr( FSTN_ptr);
               break;
            default:
               break;
         }


          /**************************************************************
            Assign the previous to new one
          **************************************************************/
         if(prev_transfer_data_struct_ptr != transfer_data_struct_ptr)
         {
               switch (EHCI_GET_TYPE(prev_transfer_data_struct_ptr))
               {

                  case EHCI_ELEMENT_TYPE_ITD:
                     itd_ptr = (ehci_itd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( prev_transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
             usb_hal_ehci_set_itd_next_link_pointer( itd_ptr, next);
                     break;
                  case EHCI_ELEMENT_TYPE_QH:
                     qh_ptr = (ehci_qh_struct_t*)((usb_hal_ehci_get_periodic_list_addr( prev_transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
             usb_hal_ehci_set_qh_horiz_link_ptr( qh_ptr, next);
                     break;
                  case EHCI_ELEMENT_TYPE_SITD:
                     sitd_ptr = (ehci_sitd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( prev_transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
             usb_hal_ehci_set_sitd_next_link_pointer( sitd_ptr, next);
                     break;

                  case EHCI_ELEMENT_TYPE_FSTN:
                     FSTN_ptr = (ehci_fstn_struct_t*)((usb_hal_ehci_get_periodic_list_addr( prev_transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
             usb_hal_ehci_set_fstn_normal_path_link_ptr( FSTN_ptr, next);
                     break;
                  default:
                     break;
               }
           }
           else
           {
                 EHCI_MEM_WRITE(*prev_transfer_data_struct_ptr,next);
           }

          return;

}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : reclaim_band_width
*  Returned Value : None
*  Comments       :
*        updates the bandwidth list with released time
*END*-----------------------------------------------------------------*/

void reclaim_band_width(
         /* [IN] the USB Host state structure */
         usb_host_handle                 handle,

        /*Frame number that will get the bandwidth*/
        uint32_t                          frame_number,

        /*data structure that is releasing the bandwidth */
        volatile uint32_t                 *transfer_data_struct_ptr,

        /* The pipe descriptor to queue */
        ehci_pipe_struct_t*              pipe_descr_ptr

)
{
      usb_ehci_host_state_struct_t*               usb_host_ptr;
      uint32_t                                     time_for_action,uframe_index;
      uint8_t                                   *frame_list_bw_ptr;
      uint8_t                              speed;
      speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);


      usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
      frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

      /**************************************************************
       Find the void *to the next structure to be pointed in the list
      **************************************************************/

      switch (EHCI_GET_TYPE(transfer_data_struct_ptr))
      {

         case EHCI_ELEMENT_TYPE_ITD:
            //itd_ptr = (ehci_itd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            break;
         case EHCI_ELEMENT_TYPE_QH:
            //qh_ptr = (ehci_qh_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            time_for_action =  pipe_descr_ptr->bwidth;

            if (speed == USB_SPEED_HIGH){
               uframe_index = frame_number % 8;
               if (pipe_descr_ptr->bwidth_slots[uframe_index]) {
                  frame_list_bw_ptr[frame_number] -= time_for_action;
               }
            }
            else {
               for (uframe_index = 0; uframe_index < 8; uframe_index++) {
                  /* if bandwidth is allocated in this microframe subtract (free) it */
                  if (pipe_descr_ptr->bwidth_slots[uframe_index]) {
                      frame_list_bw_ptr[8 * frame_number + uframe_index] -= time_for_action;
                  }
               }
            }
            break;
         case EHCI_ELEMENT_TYPE_SITD:
            //sitd_ptr = (ehci_sitd_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            break;
         case EHCI_ELEMENT_TYPE_FSTN:
            //FSTN_ptr = (ehci_fstn_struct_t*)((usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr)) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            break;
         default:
            break;
      }


      return;
}



/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : allocate_ss_cs_bandwidth
*  Returned Value : None
*  Comments       :
*  Allocates the slots for scheduling split transactions on this pipe.
*END*-----------------------------------------------------------------*/

static usb_status allocate_ss_cs_bandwidth (

     /* [IN] the USB Host state structure */
        usb_host_handle                  handle,

     /* [IN] The pipe descriptor to queue */
        ehci_pipe_struct_t*              pipe_descr_ptr,

        /*Number of slots required by this pipe*/
        uint32_t                           slots_asked,

        /* starting slot out of 8 */
        uint32_t                           start_slot,

        /* ending slot out of 8 */
        uint32_t                           end_slot

)
{
   /* Generate a pattern where */
   uint8_t  microframes[8] = {0,0,0,0,0,0,0,0};

   /**********************************************************
   Generate all start split combinations for this pipe and
   check the bandwidth allocation for each of them. If this
   fails return bandwidth alloctaion failed. Notice that by
   nature of the work, this is time consuming part of the code
   because of several combinations available and huge
   periodic list to iterate through to find if a combination
   is available. If it is not, we go to the next one.In general
   this code will take time when more devices are plugged in.
   *************************************************************/
   return split_combinations(handle,
                      pipe_descr_ptr,
                      microframes,
                      (int)start_slot,
                      (int)end_slot,
                      (int)slots_asked);
}


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : split_combinations
*  Returned Value : None
*  Comments       :
*  Finds all combinations for k elements from a set of n elements format.
*END*-----------------------------------------------------------------*/

static usb_status split_combinations
(
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* [IN] The pipe descriptor to queue */
      ehci_pipe_struct_t*             pipe_descr_ptr,

     /* [IN] Array to use to generate combinations */
      uint8_t   v[],

      /* generate combinations starting from */
      int start,

      /* generate combinations until */
      int end,

      /* generate combination for max. n microframes */
      int n
)
{
    int  i, j;
    uint32_t frame_number;
    //uint32_t mask;
    uint32_t status = USB_OK;
    usb_ehci_host_state_struct_t* usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /* the original algorithm supposed the transaction can be not continuous, so in the example [3,5] is shown */
    /* while in fact, [3,5] should never happen, the start split can be not continuous, but once it starts, it */
    /* must be continuous */
#if 0
    /* Prepare slots for bandwidth allocation.
    ** This algorithm tries to allocate as much as possible microframes.
    **
    **
    ** start = 3, end = 5, n = 1
    ** NORMAL             REVERSE
    ** --------------------------
    ** [3]                [5]
    ** [4]                [4]
    ** [5]                [3]
    **
    **
    ** start = 3, end = 5, n = 2
    ** NORMAL             REVERSE
    ** --------------------------
    ** [3, 4]             [4, 5]  110 6
    ** [3, 5]             [3, 5]  101 5
    ** [4, 5]             [3, 4]  011 3
    ** [3]                [5]
    ** [4]                [4]
    ** [5]                [3]
    **
    */
    for (; n > 0; n--) {
       for (j = n; j < 8; j++) { // zero vector unused fields
          v[j] = 0;
       }
 
       for (num = 1 << start; num < (1 << (end + 1)) - 1; num++) { //search for number having j slots in the slots interval [start, end]
          //test if number i contains j slots (bits set to one)
          j = 0;
          for (i = 0; i < 8; i++) {
             if (num & (1 << i)) {
                j++;
             }
          }
          if (j != n) {
              continue;
          }
 
          //Found number i having the number of slots as high as requested ('j'). Now, create vector v for slots
          j = 0;
          for (i = 0; i < 8; i++) {
             if (num & (1 << i)) {
                v[j++] = i;
             }
          }
 
          /* we prepared a combination so we now check if this is available in periodic list if bandwidth is not already allocated */
          if (pipe_descr_ptr->bwidth == 0)
          {
             for (frame_number = 0; (frame_number < pipe_descr_ptr->common.interval) && (frame_number < usb_host_ptr->frame_list_size); frame_number++)
             {
                status = _usb_ehci_commit_bandwidth_slots(handle,
                                                          pipe_descr_ptr,
                                                          frame_number,
                                                          pipe_descr_ptr->common.interval,
                                                          v, /* combination to try (and commit if possible) */
                                                          n /*number of slots */
                                                          );
 
                if (status != USBERR_BANDWIDTH_ALLOC_FAILED)
                {
                   /***************************************************************************
                   Slot allocation was either unsuccessful or there was some other error.
                   ****************************************************************************/
                   return status;
                }
             }
          }
       }
    }
#else
    /* Prepare slots for bandwidth allocation.
    ** This algorithm tries to allocate as much as possible microframes.
    **
    **
    ** start = 3, end = 5, n = 1
    ** mask = 0b'1
    ** --------------------------
    ** num = 1 << 3           v[0] = 3
    ** num = 1 << 4           v[0] = 4
    ** num = 1 << 5           v[0] = 5
    **
    **
    ** start = 3, end = 5, n = 2
    ** mask = 0b'11
    ** --------------------------
    ** num = 0b'11 << 3      v[0] = 3 v[1] = 4
    ** num = 0b'11 << 4      v[0] = 4 v[1] = 5
    **
    */
    //mask = (1 << n) - 1;
    for (i = start; i <= (end + 1 - n); i++)
    {
        //num = mask << i;
        for (j = 0; j < n; j++)
             v[j] = i + j;
        for (j = 0; j < n; j++)
        {
            /* we prepared a combination so we now check if this is available in periodic list if bandwidth is not already allocated */
            if (pipe_descr_ptr->bwidth == 0)
            {
                for (frame_number = 0; (frame_number < pipe_descr_ptr->common.interval) && (frame_number < usb_host_ptr->frame_list_size); frame_number++)
                {
                    status = _usb_ehci_commit_bandwidth_slots(handle,
                                                              pipe_descr_ptr,
                                                              frame_number,
                                                              pipe_descr_ptr->common.interval,
                                                              v, /* combination to try (and commit if possible) */
                                                              n /*number of slots */
                                                              );
     
                    if (status == USB_OK)
                    {
                        /***************************************************************************
                        Slot allocation was either unsuccessful or there was some other error.
                        ****************************************************************************/
                        return status;
                    }
                }
            }
        }
    }
    
#endif

   return status;
}





/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_close_interrupt_pipe
*  Returned Value : None
*  Comments       :
*        Close the Interrupt pipe and update the bandwidth list.
Here are the notes. In EHCI, closing an interrupt pipe involves removing
the queue head from the periodic list to make sure that none of the
frames refer to this queue head any more. It is also important to remember
that we must start removing the queue head link from a safe place which
is not currently being executed by EHCi controller. Apart from this we
should free all QTDs associated with it and QH Managements structure.
*END*-----------------------------------------------------------------*/

void _usb_ehci_close_interrupt_pipe (

      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */
      ehci_pipe_struct_t*             pipe_descr_ptr

)
{
    //usb_ehci_host_state_struct_t*               usb_host_ptr;
    //ehci_qh_struct_t*                           qh_ptr,temp_qh_ptr;
    //ACTIVE_QH_MGMT_STRUCT_PTR                    active_list_member_ptr;
    //ACTIVE_QH_MGMT_STRUCT_PTR                    temp_active_list_member_ptr,prev_active_list_member_ptr;
    //ehci_qtd_struct_t*                          qtd_ptr, temp_qtd_ptr;
    //volatile uint32_t                            *periodic_list_base_ptr;
    //volatile uint32_t                            *transfer_data_struct_ptr;
    //volatile uint32_t                            *prev_transfer_data_struct_ptr;
    //ehci_itd_struct_t*                          itd_ptr;
    //ehci_sitd_struct_t*                         sitd_ptr;
    //ehci_fstn_struct_t*                         FSTN_ptr;
    //int                                          i;
    //bool                                         unlinked;
    //uint32_t                                     start;
    //uint32_t                                     frame_list_size, interval, frame_number;
    uint8_t                              speed;
    speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);
 
    //usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
    /****************************************************************************
    Obtain the QH for this pipe
    ****************************************************************************/
    //qh_ptr = (ehci_qh_struct_t*) pipe_descr_ptr->qh_for_this_pipe;

    /****************************************************************************
    First Search the periodic list and unlink this QH from the list.
    ****************************************************************************/
 
    //periodic_list_base_ptr  = (volatile uint32_t *)(usb_host_ptr->periodic_list_base_addr);
 
    /*******************************************************************
    Start from fram 0 till end of the list and unlink the QH if found. Note
    that we should not unlink when the QH is active but current code does
    not take this in account.
    ********************************************************************/
 
    /* Get frame list size and interval */
    if (speed == USB_SPEED_HIGH)
    {
        _usb_ehci_free_high_speed_bandwidth(handle, pipe_descr_ptr);
    }
    else
    {
        _usb_ehci_free_split_bandwidth(handle, pipe_descr_ptr);
    }

    return;
}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_link_structure_in_periodic_list
*  Returned Value : None
*  Comments       :
*        This routine adds the given list of ITD or SITD structures in
*  periodic list starting at the earliest available slot in the slots (micro frames)
*  allocated for the given pipe.
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_link_structure_in_periodic_list (
      /* [IN] the USB_dev_initialize state structure */
      usb_host_handle                 handle,
     
      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*             pipe_descr_ptr,
      
      uint32_t                      *struct_to_link_list,
      

      /* [IN] this is one more than actual length */
      uint32_t                          no_of_structs

)
{
   usb_ehci_host_state_struct_t*     usb_host_ptr;
   ehci_itd_struct_t*                itd_ptr;
   ehci_sitd_struct_t*               sitd_ptr;
   uint32_t                            i;
   uint32_t                            earliest_frame_slot;
   uint32_t                            current_frame_number;
   uint32_t                        *transfer_data_struct_ptr = NULL;
   uint8_t                              speed;
   speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

   /* we can not have interrupts when adding data structures to the 
   hardware list */
   USB_EHCI_Host_lock();

   /**********************************************************************
   What frame number controller is executing at the moment. We start
   from the slot allocated to this pipe after this frame number.
   **********************************************************************/
   current_frame_number = _usb_ehci_get_frame_number(handle) % usb_host_ptr->frame_list_size;
      
   /**********************************************************************
   Loop the periodic list and find the earliest frame higher than this
   frame number.We take  a margin of 3 frames for safety.
   **********************************************************************/
   if(speed  == USB_SPEED_HIGH)
   {
            i =  pipe_descr_ptr->start_uframe; /* note that i is microframe number */
   
            while(i/8 < (current_frame_number + 3) % usb_host_ptr->frame_list_size)
            {
               i += pipe_descr_ptr->common.interval;
            }

            /**********************************************************************
            If eariest microframe where we could schedule is more than
            the size of periodic list, we schedule at the start slot
            **********************************************************************/
            earliest_frame_slot = (i/8) % usb_host_ptr->frame_list_size;

            /**********************************************************************
            Link all structures in periodic list. Note that ITds are added
            in the beginning in the list. Interrupts QH are linked to multiple
            lists so they are always added at the end.
            **********************************************************************/

            /*****************************************************************
            transfer_data_struct_ptr is the void *to earliest frame slot.
            *****************************************************************/
            transfer_data_struct_ptr = (uint32_t *)(usb_host_ptr->periodic_list_base_addr);
            transfer_data_struct_ptr += earliest_frame_slot;

            for(i = 0; i < no_of_structs; i++)
            {
                  itd_ptr = (ehci_itd_struct_t*) (struct_to_link_list[i]);

                  /* store the frame list pointer */      
                  itd_ptr->frame_list_ptr = transfer_data_struct_ptr;
                     
                  /* save the next one */
                  /*next_data_struct  =  (*transfer_data_struct_ptr);*/

                                          
                  /*restore the previous link back */
                  usb_hal_ehci_set_itd_next_link_pointer( itd_ptr, usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr));
                  #ifdef  __USB_OTG__ 

                     #ifdef HOST_TESTING
                     
                     USB_mem_copy((unsigned char *)itd_ptr,
                                &usb_otg_state_struct_ptr->ITD_QUEUE_LOG[usb_otg_state_struct_ptr->LOG_FRAME_COUNT]
                                  ,80);

                     usb_otg_state_struct_ptr->LOG_FRAME_COUNT++;
                     if(usb_otg_state_struct_ptr->LOG_FRAME_COUNT > HOST_LOG_MOD)
                     usb_otg_state_struct_ptr->LOG_FRAME_COUNT = 0;
                     #endif

                     #endif

                  /* add this one to the start of the frame list pointer */
                  usb_hal_ehci_add_frame_list_pointer_itd( transfer_data_struct_ptr, itd_ptr);
                  
                  earliest_frame_slot +=  (pipe_descr_ptr->common.interval / 8 > 1) ?
                                               pipe_descr_ptr->common.interval / 8 : 1;

         
                  /* if we did not reach the end of the list we move on*/
                  if(earliest_frame_slot <  usb_host_ptr->frame_list_size)
                  {  
                        /* move to next frame interval */
                        transfer_data_struct_ptr += (pipe_descr_ptr->common.interval / 8 > 1) ?
                                               (pipe_descr_ptr->common.interval / 8) : 1;

                  }
                  else
                  {
                     /* start from the first frame allocated to this pipe */
                     earliest_frame_slot = pipe_descr_ptr->start_uframe/8;
                     transfer_data_struct_ptr = (uint32_t *)(usb_host_ptr->periodic_list_base_addr);
                     transfer_data_struct_ptr += earliest_frame_slot;
                  }
         
            } /* end for loop */

   }
   /******************************************************************************
   For full speed we connect SITDs in periodic list   
   ******************************************************************************/
   else
   {
         i =  pipe_descr_ptr->start_frame; /* note that i is frame nubmer */
		if(((i > current_frame_number)? (i-current_frame_number):(current_frame_number - 
   					i)) > 5)
		{
			i = 0;
			/* we take a safety margin of 3 frames (3 milliseconds) to find the right slot */
			while(i < (current_frame_number + 3) % usb_host_ptr->frame_list_size)
			{
				i += pipe_descr_ptr->common.interval;
			}
		}
		else
		 	i += pipe_descr_ptr->common.interval;
         /**********************************************************************
         If eariest microframe where we could schedule is more than
         the size of periodic list, we schedule at the start slot
         **********************************************************************/
         earliest_frame_slot = i % usb_host_ptr->frame_list_size;

		 pipe_descr_ptr->start_frame = earliest_frame_slot;
		 if(pipe_descr_ptr->start_frame == usb_host_ptr->frame_list_size)
		 	pipe_descr_ptr->start_frame = 0;
         /**********************************************************************
         Link all structures in periodic list. Note that SITds are added
         in the beginning in the list. Interrupts QH are linked to multiple
         lists so they are always added at the end.
         **********************************************************************/

         /*****************************************************************
         transfer_data_struct_ptr is the void *to earliest frame slot.
         *****************************************************************/
         transfer_data_struct_ptr = (uint32_t *)(usb_host_ptr->periodic_list_base_addr);
         transfer_data_struct_ptr += earliest_frame_slot;

         for(i = 0; i < no_of_structs; i++)
         {

               sitd_ptr = (ehci_sitd_struct_t*) (struct_to_link_list[i]);

               /* store the frame list pointer */      
               sitd_ptr->frame_list_ptr = transfer_data_struct_ptr;
               
               /*restore the previous link back */
               usb_hal_ehci_set_sitd_next_link_pointer( sitd_ptr, usb_hal_ehci_get_periodic_list_addr( transfer_data_struct_ptr));
               #ifdef  __USB_OTG__ 

               #ifdef HOST_TESTING
               
               USB_mem_copy((unsigned char *)sitd_ptr,
                          &usb_otg_state_struct_ptr->SITD_QUEUE_LOG[usb_otg_state_struct_ptr->LOG_FRAME_COUNT]
                            ,44);

               usb_otg_state_struct_ptr->LOG_FRAME_COUNT++;
               if(usb_otg_state_struct_ptr->LOG_FRAME_COUNT > HOST_LOG_MOD)
               usb_otg_state_struct_ptr->LOG_FRAME_COUNT = 0;
               #endif

               #endif
               
               /* add this one to the start of the frame list pointer */

               usb_hal_ehci_add_frame_list_pointer_sitd( transfer_data_struct_ptr, sitd_ptr);
               earliest_frame_slot += pipe_descr_ptr->common.interval;
         
               /* if we did not reach the end of the list we move on*/
               if(earliest_frame_slot <  usb_host_ptr->frame_list_size)
               {  
                     /* move to next frame interval */
                     transfer_data_struct_ptr += pipe_descr_ptr->common.interval;

               }
               else
               {
                     /* start from the first frame allocated to this pipe */
                     earliest_frame_slot = (pipe_descr_ptr->start_frame + 3) % usb_host_ptr->frame_list_size;
            
                     
                      transfer_data_struct_ptr = (uint32_t *)(usb_host_ptr->periodic_list_base_addr);
                      transfer_data_struct_ptr += earliest_frame_slot;
               }
         
         } /* end for loop */

   }
   
   USB_EHCI_Host_unlock();
   
   return USB_OK;   
}

#if USBCFG_EHCI_MAX_ITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_add_ITD
*  Returned Value : None
*  Comments       :
*        Adds Isochronous transfer desriptors to the Periodic list for
   the given pipe.
*
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_add_ITD
   (
      /* [IN] the USB_dev_initialize state structure */
      usb_host_handle                 handle,
     
      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*             pipe_descr_ptr,
      
      /* [IN] the transfer parameters struct */
      tr_struct_t*               pipe_tr_ptr
   )
{ /* Body */
   usb_ehci_host_state_struct_t*                  usb_host_ptr;
   ehci_itd_struct_t*                             itd_ptr;
   uint32_t                                         total_length = 0, pg_select,remaining_length;
   uint32_t                                         length_scheduled;
   unsigned char                                    *buff_ptr,*curr_page_ptr=NULL;
   uint8_t                                          itd_direction_bit;
   uint32_t                                         no_of_itds=0,no_of_uframes=0;
   uint32_t                                         i,j,offset,next;
   uint8_t                                          max_slots_per_frame=0;
   uint32_t                                         struct_to_link_list[USBCFG_EHCI_MAX_ITD_DESCRS];
   usb_status                                      status;
   uint8_t                                         address;
   address = (usb_host_dev_mng_get_address(pipe_descr_ptr->common.dev_instance)) & 0x7F; 

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;   

   /*assert max packet size, interval, buffers, length etc.*/

   /******************************************************************************
   This list will contain all ITDs that will be linked in periodic list.
   We ensure that all fields inside it are NULL before we try to link it.
   ******************************************************************************/
   next = 0;
   OS_Mem_zero(struct_to_link_list, USBCFG_EHCI_MAX_ITD_DESCRS * sizeof(uint32_t));    
    
   /******************************************************************************
   A client buffer request to an isochronous endpoint may span 1 to N microframes.
   When N is larger than one, system software may have to use multiple iTDs to
   read or write data with the buffer (if N is larger than eight, it must use
   more than one iTD).
   
   Each iTD can be initialized to service up to 24 transactions, organized into
   eight groups of up to three transactions each. Each group maps to one 
   micro-frame's worth of transactions. The EHCI controller does not provide
   per-transaction results within a micro-frame. It treats the per-micro-frame
   transactions as a single logical transfer.
   *******************************************************************************/

   /*******************************************************************************
   find out how big is the transaction
   *******************************************************************************/

   if (pipe_descr_ptr->common.direction == USB_SEND) 
   {
         total_length = pipe_tr_ptr->tx_length;
         buff_ptr = pipe_tr_ptr->tx_buffer;
         /*see EHCI specs. direction is 0 for an ISO OUT */
         itd_direction_bit = 0;
   } 
   else 
   {
         total_length = pipe_tr_ptr->rx_length;
         buff_ptr = pipe_tr_ptr->rx_buffer;
         itd_direction_bit = 1;
   }

   /*******************************************************************************
    How many micro frame this transaction will it take? If some bytes are remaining
    we increase it by 1.
   *******************************************************************************/
   no_of_uframes = ((total_length%(pipe_descr_ptr->common.max_packet_size*pipe_descr_ptr->common.trs_per_uframe)) > 0) ?
                (total_length/(pipe_descr_ptr->common.max_packet_size*pipe_descr_ptr->common.trs_per_uframe) + 1):
                (total_length/(pipe_descr_ptr->common.max_packet_size*pipe_descr_ptr->common.trs_per_uframe));
                

   /**********************************************************************************
    How many micro frame slots (per frame) this transaction will take?. We cannot use all 8 
    microframe slots of a ITD because this ITD should execute only in the micro frames
    allocated to it.  
    **********************************************************************************/
   for(j = 0; j < 8; j++)
   {
      
     if(pipe_descr_ptr->bwidth_slots[j])
      {
          max_slots_per_frame++;
      }
   }

   no_of_itds = ((no_of_uframes%max_slots_per_frame) > 0)?
                ((no_of_uframes/max_slots_per_frame) + 1) :
                (no_of_uframes/max_slots_per_frame);

   /* error check */             
   if (no_of_itds >= usb_host_ptr->itd_entries)
   {
      return USB_log_error(__FILE__,__LINE__,USBERR_TRANSFER_IN_PROGRESS);
   }
   /*******************************************************************************
    Allocate as many ITDs and schedule it in periodic list   
   *******************************************************************************/
   remaining_length = total_length;
   length_scheduled = 0;
   //curr_page_ptr = buff_ptr;

   /* save on how mant ITDS are required for this transfer */   
   pipe_tr_ptr->no_of_itds_sitds = no_of_itds; 
   
   for(i = 0; i < no_of_itds; i++)
   {
        USB_EHCI_Host_lock();
        /*********************************************************************
        Get an ITD from the queue   
        **********************************************************************/

        EHCI_ITD_QGET(usb_host_ptr->itd_head, usb_host_ptr->itd_tail, itd_ptr);

        if (!itd_ptr) {
        return USB_STATUS_TRANSFER_IN_PROGRESS;
        } /* Endif */

        usb_host_ptr->itd_entries--;

        /*********************************************************************
        Add the ITD to the list of active ITDS (note that it is assumed that
        space is available in the queue because itd_ptr was allocated and
        number of nodes available always match the number of ITD_ENTRIES
        **********************************************************************/
        EHCI_ACTIVE_QUEUE_ADD_NODE(usb_host_ptr->active_iso_itd_periodic_list_tail_ptr,itd_ptr);
        USB_EHCI_Host_unlock();
        /*********************************************************************
        Zero the ITD. Leave everything else expect first 16 int bytes (which are
        defined by EHCI specs). Fill the necessary fields.
        **********************************************************************/
        OS_Mem_zero((unsigned char *)itd_ptr,16*sizeof(uint32_t));

        /* Initialize the ITD private fields*/
        itd_ptr->pipe_descr_for_this_itd = pipe_descr_ptr;
        itd_ptr->pipe_tr_descr_for_this_itd = pipe_tr_ptr;
        itd_ptr->scratch_ptr = handle;

        /* Set the Terminate bit */
        usb_hal_ehci_set_ITD_terminate_bit(itd_ptr);

        /*store endpoint number and device address */
        usb_hal_ehci_store_endpoint_number_and_device_addr(itd_ptr, (uint32_t)((pipe_descr_ptr->common.endpoint_number << EHCI_ITD_EP_BIT_POS) | address));

        /*store max packet size and direction */
        usb_hal_ehci_store_max_packet_size_and_direction(itd_ptr, (uint32_t)(pipe_descr_ptr->common.max_packet_size | (itd_direction_bit << EHCI_ITD_DIRECTION_BIT_POS)));

        /*A High-Bandwidth transaction is allowed only if the length of bytes to schedule are same
            or more than mutiple bandwidth factor times the Max packet size */

        if(remaining_length >=  
        pipe_descr_ptr->common.trs_per_uframe * pipe_descr_ptr->common.max_packet_size)
        {
            //EHCI_MEM_WRITE(itd_ptr->buffer_page_ptr_list[2],pipe_descr_ptr->common.trs_per_uframe);  
            usb_hal_ehci_set_transaction_number_per_micro_frame(itd_ptr, pipe_descr_ptr->common.trs_per_uframe);
        }
        else
        {
            //EHCI_MEM_WRITE(itd_ptr->buffer_page_ptr_list[2],1);           
            usb_hal_ehci_set_transaction_number_per_micro_frame(itd_ptr, 1);
        }

        /*initialize the number of transactions on this ITD */
        itd_ptr->number_of_transactions = 0;

        /*********************************************************************
        One ITD can address 7 4K pages. Find how many pages this transaction
        will take and schedule each of them in this ITD. It is assumed that
        buffer is large enough for the given transaction and we don't check
        for its validlity here.
        ***********************************************************************/
        pg_select = 0;

        /**************************************************************
        Prepare the ITD for the slots that are scheduled for it.
        **************************************************************/
         for(j = 0; j < 8; j++)
         {
           
            /*only if this micro frame is allocated to this pipe*/
            if(pipe_descr_ptr->bwidth_slots[j])
            {
                /*********************************************************************
                Set the location of Page 0 (4K aligned)
                *********************************************************************/


                //EHCI_MEM_WRITE(itd_ptr->buffer_page_ptr_list[pg_select],(EHCI_MEM_READ(itd_ptr->buffer_page_ptr_list[pg_select] ) | (uint32_t)buff_ptr) & 0xFFFFF000);
                usb_hal_ehci_set_buffer_page_pointer(itd_ptr, pg_select, (usb_hal_ehci_get_buffer_page_pointer(itd_ptr, pg_select) | (uint32_t)buff_ptr) & 0xFFFFF000);                              

                offset = (uint32_t)buff_ptr & 0xFFF;


                /**************************************************************
                For IN transaction, we always use Max packet size but for an
                OUT we can use the length remained in the buffer if it is less
                than a maxk packet size.
                **************************************************************/
                if (pipe_descr_ptr->common.direction == USB_SEND) 
                {
                    length_scheduled = 
                    (pipe_descr_ptr->common.trs_per_uframe * pipe_descr_ptr->common.max_packet_size > remaining_length) ?
                    remaining_length : pipe_descr_ptr->common.trs_per_uframe * pipe_descr_ptr->common.max_packet_size;
                    remaining_length -=  length_scheduled;
                }
                else
                {
                    /* on a ISO IN, we still schedule Max packet size but makes sure that remaining
                    length is set to 0 so no more slots are scheduled */

                    length_scheduled =  (uint32_t)(pipe_descr_ptr->common.trs_per_uframe * pipe_descr_ptr->common.max_packet_size);
                    remaining_length =  (length_scheduled > remaining_length) ?
                    0 : (remaining_length - length_scheduled);
                }

                /**************************************************************
                Fill the fields inside ITD
                **************************************************************/


                usb_hal_ehci_set_transcation_status_and_control_bit(itd_ptr,j, ((length_scheduled << EHCI_ITD_LENGTH_BIT_POS) | (pg_select << EHCI_ITD_PG_SELECT_BIT_POS) | EHCI_ITD_IOC_BIT | offset));
                usb_hal_ehci_set_transcation_status_active_bit(itd_ptr,j);
                /*update the transaction number queued*/
                itd_ptr->number_of_transactions++;

                /* if remaining length is 0 we break from the loop because we
                don't need to schedule any more slots*/
                if(remaining_length == 0) 
                {
                    break;
                }

                /* move buffer pointer */   
                buff_ptr += length_scheduled;

                /**************************************************************
                if a page boundry has been crossed we move to next page
                **************************************************************/
                if(((uint32_t)buff_ptr & 0xFFFFF000) != ((uint32_t)curr_page_ptr & 0xFFFFF000))
                {
                    pg_select++; /* move the  page count*/
                    curr_page_ptr = buff_ptr; /* start with this page now*/
                }

            }
            
         } /*end for loop */

         usb_hal_ehci_set_itd_length_scheduled(itd_ptr, length_scheduled);

         /**********************************************************************
         ITD ready. We store it in out list of ITDs
         **********************************************************************/
         #ifdef  __USB_OTG__ 

         #ifdef HOST_TESTING
               usb_otg_state_struct_ptr->NEXT_LINK[usb_otg_state_struct_ptr->NEXT_LINK_COUNT] =
                  EHCI_MEM_READ(itd_ptr->next_link_ptr);

               usb_otg_state_struct_ptr->NEXT_LINK_COUNT++;
               if(usb_otg_state_struct_ptr->NEXT_LINK_COUNT > HOST_LOG_MOD)
               usb_otg_state_struct_ptr->NEXT_LINK_COUNT = 0;

         #endif
         #endif
         
         struct_to_link_list[next] = (uint32_t)itd_ptr;
         next++;
   }  /*end for loop*/

   /**********************************************************************
   List ready. We link it into periodic list now.
   **********************************************************************/
   status = _usb_ehci_link_structure_in_periodic_list(handle,pipe_descr_ptr,struct_to_link_list,next);
          
   if(status == USB_OK)
   {
      usb_host_ptr->high_speed_iso_queue_active = TRUE;
   }

   return status;
} /* EndBody */
#endif //USBCFG_EHCI_MAX_ITD_DESCRS

#if USBCFG_EHCI_MAX_SITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_add_SITD
*  Returned Value : None
*  Comments       :
*        Adds Isochronous transfer desriptors to the Periodic list for
   the given full speed pipe.
*
*END*-----------------------------------------------------------------*/
usb_status _usb_ehci_add_SITD
   (
      /* [IN] the USB_dev_initialize state structure */
      usb_host_handle                 handle,
     
      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*             pipe_descr_ptr,
      
      /* [IN] the transfer parameters struct */
      tr_struct_t*               pipe_tr_ptr
   )
{ /* Body */
   usb_ehci_host_state_struct_t*                  usb_host_ptr;
   ehci_sitd_struct_t*                            sitd_ptr;
   uint32_t                                        total_length = 0, pg_select,remaining_length;
   uint32_t                                        length_scheduled;
   unsigned char                                   *buff_ptr;
   uint8_t                                         sitd_direction_bit, tp_bits=0;
   uint32_t                                        no_of_sitds=0;
   uint32_t                                        i,next;
   uint32_t                                        struct_to_link_list[USBCFG_EHCI_MAX_SITD_DESCRS];
   usb_status                                      status;
   uint8_t                                         address;
   uint8_t                                         hub_no;
   uint8_t                                         port_no;
   address = (usb_host_dev_mng_get_address(pipe_descr_ptr->common.dev_instance)) & 0x7F; 

    if (usb_host_dev_mng_get_hub_speed(pipe_descr_ptr->common.dev_instance) != USB_SPEED_HIGH)
    {
        hub_no  = (usb_host_dev_mng_get_hs_hub_no(pipe_descr_ptr->common.dev_instance)) & 0x7F;
        port_no = (usb_host_dev_mng_get_hs_port_no(pipe_descr_ptr->common.dev_instance)) & 0x7F;
    }
    else
    {
        hub_no  = (usb_host_dev_mng_get_hubno(pipe_descr_ptr->common.dev_instance)) & 0x7F;
        port_no = (usb_host_dev_mng_get_portno(pipe_descr_ptr->common.dev_instance)) & 0x7F;
    }
   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;   

   /*assert max packet size, interval, buffers, length etc.*/

   /******************************************************************************
   This list will contain all SITDs that will be linked in periodic list.
   We ensure that all fields inside it are NULL before we try to link it.
   Notice that the following could be a time consuming line if number of
   SITDS is large. Reconsider removing this line after proper testing.
   ******************************************************************************/
   next = 0;
   OS_Mem_zero(struct_to_link_list, sizeof(struct_to_link_list));
    

   /*******************************************************************************
   find out how big is the transaction
   *******************************************************************************/

   if (pipe_descr_ptr->common.direction == USB_SEND) 
   {
         total_length = pipe_tr_ptr->tx_length;
         buff_ptr = pipe_tr_ptr->tx_buffer;
         /*see EHCI specs. direction is 0 for an ISO OUT */
         sitd_direction_bit = 0;
   } 
   else 
   {
         total_length = pipe_tr_ptr->rx_length;
         buff_ptr = pipe_tr_ptr->rx_buffer;
         sitd_direction_bit = 1;
   }

   /*******************************************************************************
   Full speed devices have a minimum frequency of 1 mili second so 1 SITD can 
   completely describe 1 Max_packet_size transaction. Thus number of SITD is same
   as number of max packet size transactions required for the given transfer.
   *******************************************************************************/
  
   no_of_sitds = ((total_length%pipe_descr_ptr->common.max_packet_size) > 0) ?
                (total_length/pipe_descr_ptr->common.max_packet_size + 1):
                (total_length/pipe_descr_ptr->common.max_packet_size);
  
   /* error check , if we are going to run out of SITDS, we reject transfer here itself*/             
   if (no_of_sitds >= usb_host_ptr->sitd_entries)
   {
      return USB_log_error(__FILE__,__LINE__,USBERR_TRANSFER_IN_PROGRESS);
   }
   
   /*******************************************************************************
    Allocate as many SITDs and schedule it in periodic list   
   *******************************************************************************/
   remaining_length = total_length;
   length_scheduled = 0;

   /* save on how many SITDS are required for this transfer */   
   pipe_tr_ptr->no_of_itds_sitds = no_of_sitds;
   USB_EHCI_Host_lock();
   for(i = 0; i < no_of_sitds; i++)
   {
		 
		/*********************************************************************
		Get an SITD from the queue   
		**********************************************************************/
		EHCI_SITD_QGET(usb_host_ptr->sitd_head, usb_host_ptr->sitd_tail, sitd_ptr);

		if (!sitd_ptr) {
			 USB_EHCI_Host_unlock();
		     return USB_STATUS_TRANSFER_IN_PROGRESS;
		} /* Endif */

        usb_host_ptr->sitd_entries--;

        /*********************************************************************
        Zero the SITD. Leave everything else expect first 7 int bytes (which are
        defined by EHCI specs). Fill the necessary fields.
        **********************************************************************/
        OS_Mem_zero((void *)sitd_ptr, 7 * sizeof(uint32_t));

        /* Initialize the scratch pointer inside SITD */
        sitd_ptr->pipe_descr_for_this_sitd = pipe_descr_ptr;
        sitd_ptr->pipe_tr_descr_for_this_sitd = pipe_tr_ptr;
        sitd_ptr->scratch_ptr = handle;

        /*********************************************************************
        Add the SITD to the list of active SITDS (note that it is assumed that
        space is available in the queue because sitd_ptr was allocated and
        number of nodes available always match the number of SITD_ENTRIES
        **********************************************************************/
         EHCI_ACTIVE_QUEUE_ADD_NODE(usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr,sitd_ptr);
		
         /* Set the Terminate bit */
         usb_hal_ehci_set_sitd_next_link_terminate_bit(sitd_ptr);

         /* Set the Back pointer */
         usb_hal_ehci_set_sitd_back_pointer_terminate_bit(sitd_ptr);
         
         /*store direction, port number, hub, endpoint number device address etc.*/

         usb_hal_ehci_set_sitd_ep_capab_charac(sitd_ptr, \
                     (uint32_t)((sitd_direction_bit              << EHCI_SITD_DIRECTION_BIT_POS) |
                               (port_no    << EHCI_SITD_PORT_NUMBER_BIT_POS) |
                               (hub_no << EHCI_SITD_HUB_ADDR_BIT_POS) |
                               (pipe_descr_ptr->common.endpoint_number << EHCI_SITD_EP_ADDR_BIT_POS) |
                               address));


         /* store the split transaction schedule */                              
         usb_hal_ehci_set_sitd_uframe_sched_ctl(sitd_ptr, \
                     (uint32_t)((pipe_descr_ptr->complete_split << EHCI_SITD_COMPLETE_SPLIT_MASK_BIT_POS) | pipe_descr_ptr->start_split));
                                       
        /*store the buffer pointer 1 and offset*/
        /*EHCI_MEM_WRITE(sitd_ptr->buffer_ptr_0,(uint32_t)buff_ptr);*/

        /*********************************************************************
        One SITD can only carry a max of 1023 bytes so two page pointers are
        enough to describe it. We always set the page select to 0 in SITD and
        controller will move to next page and update the pg_select bit.
        ***********************************************************************/
        pg_select = 0;

        /**************************************************************
        For IN transaction, we always use Max packet size but for an
        OUT we can use the length remained in the buffer if it is less
        than a maxk packet size.
        **************************************************************/
         if (pipe_descr_ptr->common.direction == USB_SEND) 
         {
             length_scheduled = 
                      (pipe_descr_ptr->common.max_packet_size > remaining_length) ?
                       remaining_length : pipe_descr_ptr->common.max_packet_size;
                       
             remaining_length -=  length_scheduled;
             
         }
         else
         {
               /* on a ISO IN, we still schedule Max packet size but makes sure that remaining
                    length is set to 0 so no more slots are scheduled */
                     
               length_scheduled =  pipe_descr_ptr->common.max_packet_size;
               
               remaining_length =  (length_scheduled > remaining_length) ?
                       0 : (remaining_length - length_scheduled);
         }
         

        /**************************************************************
        Fill the fields inside SITD
        **************************************************************/
        /*store total bytes to transfer and status. Note that a total of
        1023 bytes max can be transferred on a SITD*/
        usb_hal_ehci_set_sitd_transfer_state(sitd_ptr, EHCI_SITD_IOC_BIT_SET | \
                     (pg_select << EHCI_SITD_PAGE_SELECT_BIT_POS) | \
                     (length_scheduled << EHCI_SITD_TRANSFER_LENGTH_BIT_POS) |  \
                     EHCI_SITD_STATUS_ACTIVE);

         /* Depending upon if this SITD will carry less than 188 bytes, we have to code the TP bits*/
         tp_bits = (uint8_t)((length_scheduled < EHCI_START_SPLIT_MAX_BUDGET) ? EHCI_SITD_TP_ALL : EHCI_SITD_TP_BEGIN);

         /* next buffer pointer is at 4096 bytes ahead */
        usb_hal_ehci_set_sitd_length_scheduled(sitd_ptr, length_scheduled);
            
        usb_hal_ehci_set_sitd_buffer_ptr_0(sitd_ptr, (uint32_t)buff_ptr);
            
        usb_hal_ehci_set_sitd_buffer_ptr_1(sitd_ptr,(((uint32_t)buff_ptr +  4096) & 0xFFFFF000) |
                      (tp_bits << EHCI_SITD_TP_BIT_POS) | pipe_descr_ptr->no_of_start_split);
         
            
         /* move buffer pointer */   
         buff_ptr += length_scheduled;


         /**********************************************************************
         SITD ready. We store it in out list of SITDs
         **********************************************************************/
         struct_to_link_list[next] = (uint32_t)sitd_ptr;
         next++;
         
   }  /*end for loop*/

   /**********************************************************************
   List ready. We keep the SITD pointers in our active list of SITds
   **********************************************************************/
  USB_EHCI_Host_unlock();
   /**********************************************************************
   List ready. We link it into periodic list now.
   **********************************************************************/
   status = _usb_ehci_link_structure_in_periodic_list(handle,pipe_descr_ptr,struct_to_link_list,next);
          
   if(status == USB_OK)
   {
      usb_host_ptr->full_speed_iso_queue_active = TRUE;
   }

   return status;

} /* EndBody */
#endif //USBCFG_EHCI_MAX_SITD_DESCRS

#if USBCFG_EHCI_MAX_ITD_DESCRS || USBCFG_EHCI_MAX_SITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_add_xfer_to_periodic_schedule_list
*  Returned Value : USB_OK or error
*  Comments       :
*        Queue the packet in the EHCI hardware Periodic schedule list
*END*-----------------------------------------------------------------*/

uint32_t _usb_ehci_add_isochronous_xfer_to_periodic_schedule_list
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*             pipe_descr_ptr,
      
      /* [IN] the transfer parameters struct */
      tr_struct_t*               pipe_tr_ptr
   )
{ /* Body */
   uint32_t                             cmd_val,sts_val;
   usb_status                          status;
   uint8_t                              speed;
   usb_ehci_host_state_struct_t*    usb_host_ptr;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);


   /****************************************************************************
   If it is a high-speed device we use ITds for transfer and if it is a
   full speed device, we use SITds for transfers.
   ****************************************************************************/

   if(speed == USB_SPEED_HIGH)
   {
#if USBCFG_EHCI_MAX_ITD_DESCRS
      status = _usb_ehci_add_ITD(handle, pipe_descr_ptr, pipe_tr_ptr);
#endif //USBCFG_EHCI_MAX_ITD_DESCRS
   }
   else
   {
#if USBCFG_EHCI_MAX_SITD_DESCRS
      status = _usb_ehci_add_SITD(handle, pipe_descr_ptr, pipe_tr_ptr);
#endif //USBCFG_EHCI_MAX_SITD_DESCRS
   }
   
   
   if(status != USB_OK)
   {
      return status;
   }

    /****************************************************************************
    if periodic schedule is not already enabled, enable it.
    ****************************************************************************/
    sts_val = usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase);

    if(!(sts_val & EHCI_STS_PERIODIC_SCHEDULE))
    {

        cmd_val = usb_hal_ehci_get_usb_cmd(usb_host_ptr->usbRegBase);

	/****************************************************************************
	write the address of the periodic list in to the periodic base register
	****************************************************************************/
	usb_hal_ehci_set_periodic_list_base_addr(usb_host_ptr->usbRegBase,(uint32_t) 
		usb_host_ptr->periodic_list_base_addr); 
	 
    /****************************************************************************
    wait until we can enable  the periodic schedule.
    ****************************************************************************/
    while((cmd_val & EHCI_USBCMD_PERIODIC_SCHED_ENABLE) !=
        (usb_hal_ehci_get_usb_interrupt_status(usb_host_ptr->usbRegBase) & EHCI_STS_PERIODIC_SCHEDULE)) {
        }


    /****************************************************************************
    enable the schedule now.
    ****************************************************************************/
      
    usb_hal_ehci_set_usb_cmd(usb_host_ptr->usbRegBase, (cmd_val | EHCI_USBCMD_PERIODIC_SCHED_ENABLE));
    }

   return USB_OK;

} /* EndBody */
#endif //USBCFG_EHCI_MAX_ITD_DESCRS || USBCFG_EHCI_MAX_SITD_DESCRS



#if USBCFG_EHCI_MAX_ITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_itd_tr_complete
*  Returned Value : None
*  Comments       :
*     Search the ITD list to see which ITD had finished and 
*     Process the interrupt.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_process_itd_tr_complete
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle
   )
{ /* Body */
   usb_ehci_host_state_struct_t*               usb_host_ptr;
   ehci_itd_struct_t*                          itd_ptr;
   ehci_pipe_struct_t*                         pipe_descr_ptr = NULL;
   tr_struct_t*                           pipe_tr_struct_ptr = NULL;
   uint32_t                                     no_of_scheduled_slots;
   uint32_t                                     status = 0;
   unsigned char                                *buffer_ptr = NULL;
   list_node_struct_t*                          node_ptr;
   list_node_struct_t*                          prev_node_ptr;
   list_node_struct_t*                          next_node_ptr;
   uint8_t                                      transaction_number;
   bool                                         pending_transaction;
   uint32_t                                     length_transmitted;
   uint32_t                                     *prev_link_ptr = NULL, *next_link_ptr = NULL;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   length_transmitted = 0;
   UNUSED_ARGUMENT(prev_link_ptr)

   /******************************************************************
   Search the ITD list starting from head till we find inactive nodes.
   Note that for Head there is no previous node so we can disntiguish
   it from rest of the list.
   ******************************************************************/
   prev_node_ptr = node_ptr =  usb_host_ptr->active_iso_itd_periodic_list_head_ptr; 

   /* loop till current node is active or node is a head node*/
   while ((prev_node_ptr->next_active && (prev_node_ptr->next != NULL))
          || ((node_ptr->prev == NULL) && (node_ptr->member != NULL)))
   {
      
        itd_ptr =  (ehci_itd_struct_t*) node_ptr->member;
#ifdef  __USB_OTG__

#ifdef HOST_TESTING
        /*
        usb_otg_state_struct_ptr->status_STARTS[usb_otg_state_struct_ptr->LOG_ITD_COUNT]
                  = EHCI_MEM_READ(itd_ptr->status);
        */            
#endif
#endif

        if (!itd_ptr) {
        continue;
        }

        pipe_tr_struct_ptr =  (tr_struct_t*) itd_ptr->pipe_tr_descr_for_this_itd;

        pipe_descr_ptr = (ehci_pipe_struct_t*) itd_ptr->pipe_descr_for_this_itd;

        /* assume that no transactions are pending on this ITD */
        pending_transaction = FALSE;

        no_of_scheduled_slots = 0;
      
        /**************************************************************
        Check the status of every transaction inside the ITD.
        **************************************************************/
        for(transaction_number = 0; transaction_number < 8; transaction_number++)
        {

         /**************************************************************
         Note that each iteration of this loop checks the micro frame 
         number on which transaction is scheduled. If a micro frame was
         not allocated for this pipe, we don't need to check it. But
         caution is that, there could be a transaction that was too
         small and even though many bandwidth slots are available 
         but this transaction was finished only in 1 of the slots. Thus
         we also keep a check of how many transactions were allocated for
         this ITD.
         **************************************************************/
      
         if ((pipe_descr_ptr->bwidth_slots[transaction_number]) &&
            (no_of_scheduled_slots < itd_ptr->number_of_transactions))
         {  
            no_of_scheduled_slots++;
            status = usb_hal_ehci_get_transcation_status_ctl_list(itd_ptr, transaction_number) & EHCI_ITD_STATUS;
            /* if transaction is not active and IOC was set we look in to it else we move on */
            if ((!(status & EHCI_ITD_STATUS_ACTIVE)) &&
            (usb_hal_ehci_get_transcation_status_ctl_list(itd_ptr, transaction_number) & EHCI_ITD_IOC_BIT)) {

                         length_transmitted += (itd_ptr->tr_status_ctl_list[transaction_number] &
                         EHCI_ITD_LENGTH_TRANSMITTED) >> EHCI_ITD_LENGTH_BIT_POS;
                     
            }
            /* if IOC is set and status is active, we have a pending transaction */
            else if ((status & EHCI_ITD_STATUS_ACTIVE) &&
            (usb_hal_ehci_get_transcation_status_ctl_list(itd_ptr, transaction_number) & EHCI_ITD_IOC_BIT)) {
                /* This means that this ITD has a pending transaction */
                pending_transaction = TRUE;
                break;
            }
         
         }

      }

       /* If this ITD is done with all transactions, time to free it */
      if(!pending_transaction)
      {

            /* if we are freeing a head node, we move node_ptr to next head 
                      or else we move normally*/
            USB_EHCI_Host_lock();
            if(node_ptr ==  usb_host_ptr->active_iso_itd_periodic_list_head_ptr)
            {
               /*free this node */
               EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_itd_periodic_list_head_ptr,
                                    usb_host_ptr->active_iso_itd_periodic_list_tail_ptr,
                                    node_ptr);

               prev_node_ptr = node_ptr = usb_host_ptr->active_iso_itd_periodic_list_head_ptr;
            }
            else
            {
                 /*save next node */
                 next_node_ptr = node_ptr->next;
                 
                 /*free current node */
                 EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_itd_periodic_list_head_ptr,
                                    usb_host_ptr->active_iso_itd_periodic_list_tail_ptr,
                                    node_ptr);

                 /*move to next node now */
                 node_ptr = next_node_ptr;                   
                 prev_node_ptr = node_ptr->prev;

            }
            
         /* EHCI_MEM_WRITE((*EHCI_MEM_READ(itd_ptr->prev_data_struct_ptr)),EHCI_MEM_READ(itd_ptr->next_data_struct_value));*/
           #ifdef  __USB_OTG__ 
              #ifdef HOST_TESTING
                        usb_otg_state_struct_ptr->status[usb_otg_state_struct_ptr->LOG_ITD_COUNT]
                         = status;
                         
                        USB_mem_copy((uchar_ptr)itd_ptr,
                                   &usb_otg_state_struct_ptr->LOG_INTERRUPT_ITDS[usb_otg_state_struct_ptr->LOG_ITD_COUNT]
                                     ,8);

                        usb_otg_state_struct_ptr->LOG_ITD_COUNT++;
                        if(usb_otg_state_struct_ptr->LOG_ITD_COUNT > HOST_LOG_MOD)
                        usb_otg_state_struct_ptr->LOG_ITD_COUNT = 0;
                     
               #endif
          #endif
            
            /*remove the ITD from periodic list */
            prev_link_ptr = next_link_ptr =  itd_ptr->frame_list_ptr;
            /*iterate the list while we find valid pointers (1 means invalid pointer) */

            while(!((uint32_t)next_link_ptr & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT) && !(EHCI_MEM_READ(*next_link_ptr) & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT))
            {
               /*if a pointer matches our ITD we remove it from list*/
               if((EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) == (uint32_t) itd_ptr)
               {
                  EHCI_MEM_WRITE(*next_link_ptr, usb_hal_ehci_get_itd_next_link_pointer(itd_ptr));
                  break;
               }
               
               prev_link_ptr = next_link_ptr;
               next_link_ptr = (uint32_t *) (EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            }

            /* subtract on how many ITDs are pending from this transfer */
            pipe_tr_struct_ptr->no_of_itds_sitds -= 1;
            
            /* if all ITDS are served free the TR INDEX */
            if(pipe_tr_struct_ptr->no_of_itds_sitds == 0)
            {
                 /* Mark TR as unused so that next request can use it */
                 pipe_tr_struct_ptr->tr_index = 0;
                 
            }
         
          /* Get buffer ftom ITD, aligned */
          buffer_ptr = (unsigned char *) (usb_hal_ehci_get_buffer_page_pointer(itd_ptr, 0) & EHCI_ITD_BUFFER_POINTER);
          /* Add offset of unaligned buffer */
          buffer_ptr += usb_hal_ehci_get_transcation_status_ctl_list(itd_ptr, 0) & EHCI_ITD_BUFFER_OFFSET;

          /* free the ITD used */
          _usb_ehci_free_ITD((usb_host_handle)usb_host_ptr, (void *)itd_ptr);
          USB_EHCI_Host_unlock();
           if (_usb_host_unlink_tr(pipe_descr_ptr, pipe_tr_struct_ptr) != USB_OK)
           {
#if _DEBUG
                USB_PRINTF("_usb_host_unlink_tr in _usb_ehci_process_qh_list_tr_complete failed\n");
#endif
            }
          if (pipe_tr_struct_ptr->callback != NULL) 
          {
              pipe_tr_struct_ptr->callback(
                             (void *)pipe_tr_struct_ptr,
                             pipe_tr_struct_ptr->callback_param,
                             buffer_ptr,
                             length_transmitted,
                             status);
          }
         
      }
      else
      {
         /* move to next ITD in the list */      
         prev_node_ptr = node_ptr;
         node_ptr = node_ptr->next;

      }

   }/* end while loop */
   
   
} /* Endbody */
#endif //USBCFG_EHCI_MAX_ITD_DESCRS

#if USBCFG_EHCI_MAX_SITD_DESCRS   
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_process_sitd_tr_complete
*  Returned Value : None
*  Comments       :
*     Search the SITD list to see which SITD had finished and 
*     Process the interrupt.
*
*END*-----------------------------------------------------------------*/
void _usb_ehci_process_sitd_tr_complete
   (
      /* [IN] the USB Host state structure */
      usb_host_handle                 handle
   )
{ /* Body */
   usb_ehci_host_state_struct_t*               usb_host_ptr;
   ehci_sitd_struct_t*                         sitd_ptr;
   ehci_pipe_struct_t*                         pipe_descr_ptr = NULL;
   tr_struct_t*                                 pipe_tr_struct_ptr = NULL;
   uint32_t                                     status = 0;
   unsigned char                                *buffer_ptr = NULL;
   list_node_struct_t*                          node_ptr;
   list_node_struct_t*                          prev_node_ptr;
   list_node_struct_t*                          next_node_ptr;
   uint32_t                                     length_scheduled, length_remaining;
   uint32_t                                     *prev_link_ptr = NULL, *next_link_ptr = NULL;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   UNUSED_ARGUMENT(prev_link_ptr)
   
   /******************************************************************
   Search the SITD list starting from head till we find inactive nodes.
   Note that for Head there is no previous node so we can disntiguish
   it from rest of the list.
   ******************************************************************/
   prev_node_ptr = node_ptr =  usb_host_ptr->active_iso_sitd_periodic_list_head_ptr; 
   
   /* loop till current node is active or node is a head node*/
   while ((prev_node_ptr->next_active && (prev_node_ptr->next != NULL))
          || ((node_ptr->prev == NULL) && (node_ptr->member != NULL)))
   {
      
      sitd_ptr =  (ehci_sitd_struct_t*) node_ptr->member;
      
      pipe_tr_struct_ptr = (tr_struct_t*) sitd_ptr->pipe_tr_descr_for_this_sitd;
      
      pipe_descr_ptr = (ehci_pipe_struct_t*) sitd_ptr->pipe_descr_for_this_sitd;

      /*grab the status and check it */
      status = usb_hal_ehci_get_sitd_transfer_state(sitd_ptr) & EHCI_SITD_STATUS;
               
      /* if transaction is not active we look in to it else we move on */
      if(!(status & EHCI_SITD_STATUS_ACTIVE))
      {

            USB_EHCI_Host_lock();
            /*********************************************************************
            Since status is Non active for this SITD, time to delete it.           
            *********************************************************************/

            /* if we are freeing a head node, we move node_ptr to next head 
            or else we move normally */
            if(node_ptr ==  usb_host_ptr->active_iso_sitd_periodic_list_head_ptr)
            {
               /*free this node */
               EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_sitd_periodic_list_head_ptr,
                                    usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr,
                                    node_ptr);

               prev_node_ptr = node_ptr = usb_host_ptr->active_iso_sitd_periodic_list_head_ptr;
            }
            else
            {
                 /*save next node */
                 next_node_ptr = node_ptr->next;
                 
                 /*free current node */
                 EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_sitd_periodic_list_head_ptr,
                                    usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr,
                                    node_ptr);

                 /*move to next node now */
                 node_ptr = next_node_ptr;                   
                 prev_node_ptr = node_ptr->prev;

            }

            if (_usb_host_unlink_tr(pipe_descr_ptr, pipe_tr_struct_ptr) != USB_OK)
            {
#if _DEBUG
                USB_PRINTF("_usb_host_unlink_tr in _usb_ehci_process_qh_list_tr_complete failed\n");
#endif
            }
            /* send callback to app with the status*/

            if (pipe_tr_struct_ptr->callback != NULL) 
            {
                  length_scheduled = usb_hal_ehci_get_sitd_length_scheduled(sitd_ptr);
                  length_remaining =((usb_hal_ehci_get_sitd_transfer_state(sitd_ptr) & EHCI_SITD_LENGTH_TRANSMITTED) >> 16);
                  buffer_ptr = (unsigned char *) usb_hal_ehci_get_sitd_buffer_ptr_0(sitd_ptr);
                  buffer_ptr = buffer_ptr - (length_scheduled - length_remaining);

                  pipe_tr_struct_ptr->callback(
                                 (void *)pipe_tr_struct_ptr,
                                 pipe_tr_struct_ptr->callback_param,
                                 buffer_ptr,
                                 length_scheduled - length_remaining,
                                 status);
            }
           #ifdef  __USB_OTG__ 
 
              #ifdef HOST_TESTING
                        usb_otg_state_struct_ptr->status[usb_otg_state_struct_ptr->LOG_SITD_COUNT]
                         = status;
                         
                        USB_mem_copy((uchar_ptr)sitd_ptr,
                                   &usb_otg_state_struct_ptr->LOG_INTERRUPT_SITDS[usb_otg_state_struct_ptr->LOG_SITD_COUNT]
                                     ,44);

                        usb_otg_state_struct_ptr->LOG_SITD_COUNT++;
                        if(usb_otg_state_struct_ptr->LOG_SITD_COUNT > HOST_LOG_MOD)
                        usb_otg_state_struct_ptr->LOG_SITD_COUNT = 0;
                     
               #endif
            #endif
                        
            /*remove the SITD from periodic list, TODO: remove prev_link_ptr variable */
            prev_link_ptr = next_link_ptr =  sitd_ptr->frame_list_ptr;
            /*iterate the list while we find valid pointers (1 means invalid pointer) */
            while(!((uint32_t)next_link_ptr & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT) && !(EHCI_MEM_READ(*next_link_ptr) & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT))
            {
               /*if a pointer matches our SITD we remove it from list*/
               if((EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) == (uint32_t) sitd_ptr)
               {
                  EHCI_MEM_WRITE(*next_link_ptr, usb_hal_ehci_get_sitd_next_link_pointer(sitd_ptr));
                  break;
               }
               
               prev_link_ptr = next_link_ptr;
               next_link_ptr = (uint32_t *) (EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
            }



            /* free the SITD used */
            _usb_ehci_free_SITD((usb_host_handle)usb_host_ptr, (void *)sitd_ptr);
            USB_EHCI_Host_unlock();
            /* subtract on how many SITDs are pending from this transfer */
            pipe_tr_struct_ptr->no_of_itds_sitds -= 1;
            
            /* if all SITDS are served free the TR INDEX */
            if(pipe_tr_struct_ptr->no_of_itds_sitds == 0)
            {
                 /* Mark TR as unused so that next request can use it */
                 pipe_tr_struct_ptr->tr_index = 0;
                 
            }
         

                           
      }
      /* else move on to the next node in the queue */
      else
      {
      
         prev_node_ptr = node_ptr;
         node_ptr = node_ptr->next;

      }



     
   }/* end while loop */
   
   
} /* Endbody */
#endif //USBCFG_EHCI_MAX_SITD_DESCRS

#if USBCFG_EHCI_MAX_ITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_ITD
*  Returned Value : void
*  Comments       :
*        Enqueues an ITD onto the free ITD ring.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_free_ITD
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,
      
      /* [IN] the ITD to enqueue */
      void                    *itd_ptr
   )
{ /* Body */
    usb_ehci_host_state_struct_t*                usb_host_ptr;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();

    /*
    ** Add the ITD to the free ITD queue (linked via PRIVATE) and
    ** increment the tail to the next descriptor
    */
    EHCI_ITD_QADD(usb_host_ptr->itd_head, usb_host_ptr->itd_tail, (ehci_itd_struct_t*)itd_ptr);
    usb_host_ptr->itd_entries++;

    USB_EHCI_Host_unlock();

} /* Endbody */
#endif //USBCFG_EHCI_MAX_ITD_DESCRS

#if USBCFG_EHCI_MAX_SITD_DESCRS
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_free_SITD
*  Returned Value : void
*  Comments       :
*        Enqueues an SITD onto the free SITD ring.
*
*END*-----------------------------------------------------------------*/

void _usb_ehci_free_SITD
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,
      
      /* [IN] the SITD to enqueue */
      void                    *sitd_ptr
   )
{ /* Body */
    usb_ehci_host_state_struct_t*                usb_host_ptr;

    usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

    /*
    ** This function can be called from any context, and it needs mutual
    ** exclusion with itself.
    */
    USB_EHCI_Host_lock();

    /*
    ** Add the SITD to the free SITD queue (linked via PRIVATE) and
    ** increment the tail to the next descriptor
    */
    EHCI_SITD_QADD(usb_host_ptr->sitd_head, usb_host_ptr->sitd_tail, 
        (ehci_sitd_struct_t*)sitd_ptr);
    usb_host_ptr->sitd_entries++;

    USB_EHCI_Host_unlock();

} /* Endbody */
#endif //USBCFG_EHCI_MAX_SITD_DESCRS

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_close_isochronous_pipe
*  Returned Value : None
*  Comments       :
*        Close the Iso pipe and update the bandwidth list.
Here are the notes. In EHCI, closing an ISO pipe involves removing
the ITD or SITD from the periodic list to make sure that none of the
frames refer to this  any more.
*END*-----------------------------------------------------------------*/

void _usb_ehci_close_isochronous_pipe (

      /* [IN] the USB Host state structure */
      usb_host_handle                 handle,

      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*             pipe_descr_ptr

)
{
   usb_ehci_host_state_struct_t*            usb_host_ptr;
   uint8_t*                                 frame_list_bw_ptr;
   ehci_itd_struct_t*                       itd_ptr;
   ehci_sitd_struct_t*                      sitd_ptr = NULL;
   uint32_t                                  i,j,frame,uframe,start;
   uint32_t                                  interval = pipe_descr_ptr->common.interval;
   list_node_struct_t*                      node_ptr;
   list_node_struct_t*                      next_node_ptr;
   list_node_struct_t*                       prev_node_ptr = NULL;
   tr_struct_t*                               pipe_tr_struct_ptr = NULL;
   uint32_t                                  *prev_link_ptr=NULL, *next_link_ptr=NULL;
   uint8_t                              speed;
   speed = usb_host_dev_mng_get_speed(pipe_descr_ptr->common.dev_instance);

   UNUSED_ARGUMENT(prev_node_ptr)

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
 
   /****************************************************************************
   if this is a HS pipe, search ITDs or else search SITD list to free them.
   ****************************************************************************/
   if(speed  == USB_SPEED_HIGH)
   {
#if USBCFG_EHCI_MAX_ITD_DESCRS
        prev_node_ptr = node_ptr =  usb_host_ptr->active_iso_itd_periodic_list_head_ptr; 

        /* loop until we find an invalid node or if this is a head node*/ 
        while (node_ptr->member != NULL)
        {
       
            itd_ptr =  (ehci_itd_struct_t*) node_ptr->member;

            pipe_tr_struct_ptr =  itd_ptr->pipe_tr_descr_for_this_itd;
            pipe_descr_ptr = (ehci_pipe_struct_t*) sitd_ptr->pipe_descr_for_this_sitd;
       
            if(itd_ptr->pipe_descr_for_this_itd == pipe_descr_ptr)
            {
                     /* if we are freeing a head node, we move node_ptr to next head 
                     or else we move normally*/

                     if(node_ptr ==  usb_host_ptr->active_iso_itd_periodic_list_head_ptr)
                     {
                        /*free this node */
                        EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_itd_periodic_list_head_ptr,
                                             usb_host_ptr->active_iso_itd_periodic_list_tail_ptr,
                                             node_ptr);

                        prev_node_ptr = node_ptr = usb_host_ptr->active_iso_itd_periodic_list_head_ptr;
                     }
                     else
                     {
                          /*save next node */
                          next_node_ptr = node_ptr->next;
                          
                          /*free current node */
                          EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_itd_periodic_list_head_ptr,
                                             usb_host_ptr->active_iso_itd_periodic_list_tail_ptr,
                                             node_ptr);

                          /*move to next node now */
                          node_ptr = next_node_ptr;                   
                          prev_node_ptr = node_ptr->prev;

                     }
                     pipe_tr_struct_ptr->callback((void *)pipe_tr_struct_ptr,
                                    pipe_tr_struct_ptr->callback_param,
                                    (unsigned char *)usb_hal_ehci_get_sitd_buffer_ptr_0(sitd_ptr),
                                    0,
                                    USBERR_TR_CANCEL);
                     /*remove the ITD from periodic list */
                     prev_link_ptr = next_link_ptr =  itd_ptr->frame_list_ptr;
                     /*iterate the list while we find valid pointers (1 means invalid pointer) */
                     while(!(EHCI_MEM_READ(*next_link_ptr) & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT))
                     {
                        /*if a pointer matches our ITD we remove it from list*/
                        if((EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) ==  (uint32_t) itd_ptr)
                        {
                           EHCI_MEM_WRITE(*prev_link_ptr,usb_hal_ehci_get_itd_next_link_pointer(itd_ptr));
                           break;
                        }
         
                        prev_link_ptr = next_link_ptr;
                        next_link_ptr = (uint32_t *) (EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
                     }

                     /* free the ITD used */
                     _usb_ehci_free_ITD((usb_host_handle)usb_host_ptr, (void *)itd_ptr);

                     /* subtract on how many ITDs are pending from this transfer */
                     pipe_tr_struct_ptr->no_of_itds_sitds -= 1;
      
                     /* if all ITDS are served free the TR INDEX */
                     if(pipe_tr_struct_ptr->no_of_itds_sitds == 0)
                     {
                          /* Mark TR as unused so that next request can use it */
                          pipe_tr_struct_ptr->tr_index = 0;
                          
                     }
         
            }
            else
            {
               /* move to next ITD in the list */      
               prev_node_ptr = node_ptr;
               node_ptr = node_ptr->next;
            }
       } /* while */
       

#endif //USBCFG_EHCI_MAX_ITD_DESCRS
   } /* end if */
   else
   {
#if USBCFG_EHCI_MAX_SITD_DESCRS
        prev_node_ptr = node_ptr =  usb_host_ptr->active_iso_sitd_periodic_list_head_ptr; 

        /* loop until we find an invalid node or if this is a head node*/ 
        while (node_ptr->member != NULL)
        {
       
            sitd_ptr =  (ehci_sitd_struct_t*) node_ptr->member;

            pipe_tr_struct_ptr = sitd_ptr->pipe_tr_descr_for_this_sitd;

            pipe_descr_ptr = (ehci_pipe_struct_t*) sitd_ptr->pipe_descr_for_this_sitd;
       
            if(sitd_ptr->pipe_descr_for_this_sitd == pipe_descr_ptr)
            {
                     /* if we are freeing a head node, we move node_ptr to next head 
                     or else we move normally*/

                     if(node_ptr ==  usb_host_ptr->active_iso_sitd_periodic_list_head_ptr)
                     {
                        /*free this node */
                        EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_sitd_periodic_list_head_ptr,
                                             usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr,
                                             node_ptr);

                        prev_node_ptr = node_ptr = usb_host_ptr->active_iso_sitd_periodic_list_head_ptr;
                     }
                     else
                     {
                          /*save next node */
                          next_node_ptr = node_ptr->next;
                          
                          /*free current node */
                          EHCI_QUEUE_FREE_NODE(usb_host_ptr->active_iso_sitd_periodic_list_head_ptr,
                                             usb_host_ptr->active_iso_sitd_periodic_list_tail_ptr,
                                             node_ptr);

                          /*move to next node now */
                          node_ptr = next_node_ptr;                   
                          prev_node_ptr = node_ptr->prev;

                     }
                    pipe_tr_struct_ptr->callback((void *)pipe_tr_struct_ptr,
                                  pipe_tr_struct_ptr->callback_param,
                                  (unsigned char *)usb_hal_ehci_get_sitd_buffer_ptr_0(sitd_ptr),
                                  0,
                                  USBERR_TR_CANCEL);

                     /*remove the SITD from periodic list */
                     prev_link_ptr = next_link_ptr =  sitd_ptr->frame_list_ptr;
                     /*iterate the list while we find valid pointers (1 means invalid pointer) */
                     while(!(EHCI_MEM_READ(*next_link_ptr) & EHCI_FRAME_LIST_ELEMENT_POINTER_T_BIT))
                     {
                        /*if a pointer matches our SITD we remove it from list*/
                        if((EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK) ==  (uint32_t) sitd_ptr)
                        {
                           EHCI_MEM_WRITE(*prev_link_ptr,usb_hal_ehci_get_sitd_next_link_pointer(sitd_ptr));
                           break;
                        }
         
                        prev_link_ptr = next_link_ptr;
                        next_link_ptr = (uint32_t *) (EHCI_MEM_READ(*next_link_ptr) & EHCI_HORIZ_PHY_ADDRESS_MASK);
                     }

                     /* free the ITD used */
                     _usb_ehci_free_SITD((usb_host_handle)usb_host_ptr, (void *)sitd_ptr);

                     /* subtract on how many ITDs are pending from this transfer */
                     pipe_tr_struct_ptr->no_of_itds_sitds -= 1;
      
                     /* if all ITDS are served free the TR INDEX */
                     if(pipe_tr_struct_ptr->no_of_itds_sitds == 0)
                     {
                          /* Mark TR as unused so that next request can use it */
                          pipe_tr_struct_ptr->tr_index = 0;
                          
                     }
         
            }
            else
            {
               /* move to next ITD in the list */      
               prev_node_ptr = node_ptr;
               node_ptr = node_ptr->next;

            }
   

       } /* while */
       
#endif //USBCFG_EHCI_MAX_SITD_DESCRS
   }



   /********************************************************************
   if status is fine we should free the slots now by updating the 
   bandwidth list.
   ********************************************************************/
   frame_list_bw_ptr = usb_host_ptr->periodic_frame_list_bw_ptr;

   if (speed == USB_SPEED_HIGH) 
   {
      start = pipe_descr_ptr->start_uframe;
    
      for(i = start; i < usb_host_ptr->frame_list_size * 8; i+= interval)
      {
         /********************************************************************
         We are allowed to use only 80% of a micro frame for 
         periodic transfers. This is to provide space for Bulk and Control
         transfers. This means that if a micro frame slot exceeds .8 * 125 = 100
         micro seconds, it is out of space.
         ********************************************************************/
         frame  = i/8;
         uframe = i%8;

         frame_list_bw_ptr[frame+uframe] -= pipe_descr_ptr->bwidth;
         if(frame_list_bw_ptr[frame+uframe] == 0)
            pipe_descr_ptr->bwidth_slots[uframe] = 0;
        
      }
   
   }
   else
   {
   
      start = pipe_descr_ptr->start_frame;
      
       for(i = start; i < usb_host_ptr->frame_list_size; i+= interval)
      {
            /********************************************************************
            Update the bandwidth in all frames in which transaction is scheduled.
            ********************************************************************/

            for(j = 0; j < 8; j++)
            {

               if(pipe_descr_ptr->bwidth_slots[j])
               {
                  frame_list_bw_ptr[i*8+j] -= pipe_descr_ptr->bwidth;
               }      
            }
       }
       
   }

   return;

}

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_shutdown
*  Returned Value : None
*  Comments       :
*     Shutdown and de-initialize the VUSB1.1 hardware
*
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_shutdown
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle
   )
{ /* Body */
   usb_ehci_host_state_struct_t*    usb_host_ptr;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;

#if USBCFG_EHCI_HOST_ENABLE_TASK
   if (ehci_host_task_id != (uint32_t)OS_TASK_ERROR)
   {
       OS_Task_delete(ehci_host_task_id);
   }
#endif

   
   /* Disable interrupts */
   usb_hal_ehci_disable_interrupts(usb_host_ptr->usbRegBase, USBHS_HOST_INTR_EN_BITS);
      
   /* Stop the controller */
   usb_hal_ehci_initiate_detach_event(usb_host_ptr->usbRegBase);
   
   /* Reset the controller to get default values */
   usb_hal_ehci_reset_controller(usb_host_ptr->usbRegBase);
   
   /**********************************************************
   ensure that periodic list in uninitilized.
   **********************************************************/
   usb_host_ptr->itd_list_initialized = FALSE;
   usb_host_ptr->sitd_list_initialized = FALSE;
   usb_host_ptr->periodic_list_initialized = FALSE;
   usb_host_ptr->periodic_list_base_addr = NULL;
   
   /**********************************************************
   Free all memory occupied by active transfers   
   **********************************************************/
   if(NULL != usb_host_ptr->xtd_struct_base_addr)
   {
      /* Free all controller non-periodic specific memory */
      OS_Mem_free((void *)usb_host_ptr->xtd_struct_base_addr);
   }
   
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
   if(NULL != usb_host_ptr->periodic_list_base_addr)
   {
      /* Free all controller non-periodic specific memory */
      OS_Mem_free((void *)usb_host_ptr->periodic_list_base_addr);
   }
#endif
      
   if(NULL != usb_host_ptr->periodic_frame_list_bw_ptr)
   {
      /* Free all controller periodic frame list bandwidth and other specific memory */
      OS_Mem_free((void *)usb_host_ptr->periodic_frame_list_bw_ptr);
   }

   if(NULL != usb_host_ptr->active_iso_itd_periodic_list_head_ptr)
   {
      OS_Mem_free((void *)usb_host_ptr->active_iso_itd_periodic_list_head_ptr);
   }

   if(NULL != usb_host_ptr->active_iso_sitd_periodic_list_head_ptr)
   {
      OS_Mem_free((void *)usb_host_ptr->active_iso_sitd_periodic_list_head_ptr);
   }

   if(NULL != usb_host_ptr->pipe_descriptor_base_ptr)
   {
      OS_Mem_free(usb_host_ptr->pipe_descriptor_base_ptr);
   }

#if USBCFG_EHCI_HOST_ENABLE_TASK
   if (NULL != usb_host_ptr->ehci_event_ptr)
   {
      OS_Event_destroy(usb_host_ptr->ehci_event_ptr);
   }
#endif
   if (usb_host_ptr->mutex != NULL)
   {
       OS_Mutex_destroy(usb_host_ptr->mutex);
   }
   
   OS_Mem_free(usb_host_ptr);
   
   return USB_OK;
   
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_bus_control
*  Returned Value : None
*  Comments       :
*        Bus control
*END*-----------------------------------------------------------------*/

usb_status _usb_ehci_bus_control
   (
      /* [IN] the USB Host state structure */
      usb_host_handle        handle,

      /* The operation to be performed on the bus */
      uint8_t                  bControl
   )
{ /* Body */
   
   switch(bControl) {
      case USB_ASSERT_BUS_RESET:
         break;
      case USB_ASSERT_RESUME:
         break;
      case USB_SUSPEND_SOF:
         _usb_ehci_bus_suspend(handle);
         break;
      case USB_DEASSERT_BUS_RESET:
      case USB_RESUME_SOF:
      case USB_DEASSERT_RESUME:
      default:
         break;
   } /* EndSwitch */
   
   return USB_OK;
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_bus_suspend
*  Returned Value : None
*  Comments       :
*        Suspend the bus
*END*-----------------------------------------------------------------*/

void _usb_ehci_bus_suspend
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle
   )
{ /* Body */
   uint8_t                                       i, total_port_numbers;
   uint32_t                                      port_control;
   usb_ehci_host_state_struct_t*    usb_host_ptr;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   //param = (USB_EHCI_HOST_INIT_STRUCT_PTR) usb_host_ptr->INIT_PARAM;

   //cap_dev_ptr = (vusb20_reg_struct_t*) param->CAP_BASE_PTR;
   //dev_ptr = (vusb20_reg_struct_t*) usb_host_ptr->dev_ptr;
      
   total_port_numbers = 
   (uint8_t)((usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) & 
         EHCI_HCS_PARAMS_N_PORTS));
         
   USB_EHCI_Host_lock();
      

   /* Suspend all ports */
   for (i=0;i<total_port_numbers;i++) {
      port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
      
      if (port_control & EHCI_PORTSCX_PORT_ENABLE) {
      
         port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
         port_control &= ~EHCI_PORTSCX_W1C_BITS;
         
     usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, (port_control | EHCI_PORTSCX_PORT_SUSPEND));
      } /* Endif */
      
   } /* Endfor */
   
   /* Stop the controller 
   SGarg: This should not be done. If the controller is stopped, we will
   get no attach or detach interrupts and this will stop all operatings
   including the OTG state machine which is still running assuming that
   Host is alive.
   
   EHCI_REG_CLEAR_BITS(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_CMD,EHCI_CMD_RUN_STOP);
   */   
      
   USB_EHCI_Host_unlock();
   
} /* EndBody */

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : _usb_ehci_bus_resume
*  Returned Value : None
*  Comments       :
*        Resume the bus
*END*-----------------------------------------------------------------*/

void _usb_ehci_bus_resume
   (
      /* [IN] the USB Host state structure */
      usb_host_handle              handle,

      /* The pipe descriptor to queue */            
      ehci_pipe_struct_t*          pipe_descr_ptr
   )
{ /* Body */
   uint8_t                                       i, total_port_numbers;
   uint32_t                                      port_control;
   usb_ehci_host_state_struct_t*    usb_host_ptr;

   usb_host_ptr = (usb_ehci_host_state_struct_t*)handle;
   
   total_port_numbers = (uint8_t)(usb_hal_ehci_get_hcsparams(usb_host_ptr->usbRegBase) & EHCI_HCS_PARAMS_N_PORTS);

   USB_EHCI_Host_lock();
      
   /* Resume all ports */
   for (i=0;i<total_port_numbers;i++) {
      port_control = usb_hal_ehci_get_port_status(usb_host_ptr->usbRegBase);
      
      if (port_control & EHCI_PORTSCX_PORT_ENABLE) {
         port_control &= ~EHCI_PORTSCX_W1C_BITS;
     usb_hal_ehci_set_port_status(usb_host_ptr->usbRegBase, (port_control | EHCI_PORTSCX_PORT_SUSPEND));
      } /* Endif */
      
   } /* Endfor */
   
   /* 
    S Garg: This should not be done. See comments in suspend.
      Restart the controller   
      EHCI_REG_SET_BITS(dev_ptr->REGISTERS.OPERATIONAL_HOST_REGISTERS.USB_CMD,EHCI_CMD_RUN_STOP);
     */

   USB_EHCI_Host_unlock();
   
} /* EndBody */


#endif
/* EOF */
