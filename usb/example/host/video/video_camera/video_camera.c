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
* $FileName: video_camera.c$
* $Version :
* $Date    :
*
* Comments:
*
*   This file is an example of device drivers for the Video class. This example
*   demonstrates the keyboard functionality. Note that a real keyboard driver also
*   needs to distinguish between intentionally repeated and unrepeated key presses.
*   This example simply demonstrates how to receive data from a USB Keyboard. 
*   Interpretation of data is upto the application and customers can add the code
*   for it.
*
*END************************************************************************/
#include "usb_host_config.h"
#include "usb.h"
#include "usb_host_stack_interface.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
#include <mqx.h>
#include <lwevent.h>
#include <bsp.h>
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_port_hal.h"
#include "board.h"
#include "fsl_debug_console.h"
#include <stdio.h>
#include <stdlib.h>
#include "fsl_uart_driver.h"
#include "fsl_hwtimer.h"
#else
#include <types.h>
#include "derivative.h"
#include "hidef.h"
#include "mem_util.h"
#include "user_config.h"
#include "rtc_kinetis.h"
#endif


#include <usb_host_hub_sm.h>
#include "usb_host_video.h"

#include "video_camera.h"
#include "comm.h"
#include "audio.h"

#define USB_EVENT_CTRL           (0x01)

/*
** Globals
*/
void usb_host_video_control_event
(
    /* [IN] pointer to device instance */
    usb_device_instance_handle dev_handle,
    /* [IN] pointer to interface descriptor */
    usb_interface_descriptor_handle intf_handle,
    /* [IN] code number for event causing callback */
    uint32_t event_code
);
void usb_host_video_stream_event
(
    /* [IN] pointer to device instance */
    usb_device_instance_handle dev_handle,
    /* [IN] pointer to interface descriptor */
    usb_interface_descriptor_handle intf_handle,
    /* [IN] code number for event causing callback */
    uint32_t event_code
);
void get_video_data();

video_camera_struct_t g_video_camera;

/* Table of driver capabilities this application wants to use */
static  usb_host_driver_info_t DriverInfoTable[] =
{
    {
        {0x00,0x00},                  /* Vendor ID per USB-IF             */
        {0x00,0x00},                  /* Product ID per manufacturer      */
        USB_CLASS_VIDEO,              /* Class code                       */
        USB_SUBCLASS_VIDEO_CONTROL,   /* Sub-Class code                   */
        0xFF,                         /* Protocol                         */
        0,                            /* Reserved                         */
        usb_host_video_control_event  /* Application call back function   */
    },
    {
        {0x00,0x00},                  /* Vendor ID per USB-IF             */
        {0x00,0x00},                  /* Product ID per manufacturer      */
        USB_CLASS_VIDEO,              /* Class code                       */
        USB_SUBCLASS_VIDEO_STREAMING, /* Sub-Class code                   */
        0xFF,                         /* Protocol                         */
        0,                            /* Reserved                         */
        usb_host_video_stream_event   /* Application call back function   */
    },
    {
        {0x00,0x00},                  /* All-zero entry terminates        */
        {0x00,0x00},                  /* driver info list.                */
        0,
        0,
        0,
        0,
        NULL
    }
};

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)

static uint32_t time_now = 0;
hwtimer_t hwtimer;

void hwtimer_callback(void* data)
{
    g_video_camera.stream_transfer.is_1ms++;
}

void time_init(void)
{
    extern const hwtimer_devif_t kSystickDevif;
    extern const hwtimer_devif_t kPitDevif;
    #define HWTIMER_LL_DEVIF    kPitDevif      // Use hardware timer PIT
    #define HWTIMER_LL_SRCCLK   kBusClock     // Source Clock for PIT
    #define HWTIMER_LL_ID       1
    #define HWTIMER_PERIOD      1000      // 1 ms interval

    if (kHwtimerSuccess != HWTIMER_SYS_Init(&hwtimer, &HWTIMER_LL_DEVIF, HWTIMER_LL_ID, 5, NULL))
    {
        USB_PRINTF("\r\nError: hwtimer initialization.\r\n");
    }
    if (kHwtimerSuccess != HWTIMER_SYS_SetPeriod(&hwtimer, HWTIMER_LL_SRCCLK, HWTIMER_PERIOD))
    {
        USB_PRINTF("\r\nError: hwtimer set period.\r\n");
    }
    if (kHwtimerSuccess != HWTIMER_SYS_RegisterCallback(&hwtimer, hwtimer_callback, NULL))
    {
        USB_PRINTF("\r\nError: hwtimer callback registration.\r\n");
    }
    if (kHwtimerSuccess != HWTIMER_SYS_Start(&hwtimer))
    {
        USB_PRINTF("\r\nError: hwtimer start.\r\n");
    }
}

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)

void SysTick_Handler()
{
    g_video_camera.stream_transfer.is_1ms++;
}

void time_init(void)
{
    SYST_RVR = 0x0000BB80;
    SYST_CVR = 0x0000BB80;
    SYST_CSR = 0x00000007;
}

#else
#error MQX unsupported!
#endif
/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_host_audio_ctrl_callback
* Returned Value : None
* Comments       :
*     Called when a command is completed
*END*--------------------------------------------------------------------*/
static void usb_host_audio_ctrl_callback
(
    /* [IN] no used */
    void*             unused,
    /* [IN] user-defined parameter */
    void*             user_parm,
    /* [IN] buffer address */
    uint8_t *         buffer,
    /* [IN] length of data transferred */
    uint32_t          buflen,
    /* [IN] status, hopefully USB_OK or USB_DONE */
    usb_status        status
)
{
    g_video_camera.in_ctrl = 0;
    g_video_camera.ctrl_status = status;
}
__root int usbget = 0;
/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_host_audio_stream_callback
* Returned Value : None
* Comments       :
*     Called when a command is completed
*END*--------------------------------------------------------------------*/
static void usb_host_audio_stream_callback
(
    /* [IN] no used */
    void*             unused,
    /* [IN] user-defined parameter */
    void*             user_parm,
    /* [IN] buffer address */
    uint8_t *         buffer,
    /* [IN] length of data transferred */
    uint32_t          buflen,
    /* [IN] status, hopefully USB_OK or USB_DONE */
    usb_status        status
)
{
    g_video_camera.stream_transfer.stream_transfer = 0;
    g_video_camera.stream_transfer.stream_status = status;

    usbget++;
    
	if (buflen >= 12)
	{
	    g_video_data_pool[g_video_data_rx_index].len = buflen + 3;
		g_video_data_pool[g_video_data_rx_index].flag = 1;
		g_video_data_pool[g_video_data_rx_index].type = 1;
		if (++g_video_data_rx_index >= MAX_VIDEO_DATA_BUF)
		{
			g_video_data_rx_index = 0;
		}
	}
}



/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_host_video_unsupported_device_event
* Returned Value : None
* Comments       :
*     Called when unsupported device has been attached.
*END*--------------------------------------------------------------------*/
void usb_host_video_unsupported_device_event
   (
      /* [IN] pointer to device instance */
      usb_device_instance_handle      dev_handle,

      /* [IN] pointer to interface descriptor */
      usb_interface_descriptor_handle intf_handle,

      /* [IN] code number for event causing callback */
      uint32_t                          event_code
   )
{
    usb_device_interface_struct_t* pDeviceIntf;
    interface_descriptor_t* intf_ptr;

    if (USB_ATTACH_INTF_NOT_SUPPORT == event_code)
    {
        pDeviceIntf = (usb_device_interface_struct_t*)intf_handle;
        intf_ptr    = pDeviceIntf->lpinterfaceDesc;
        USB_PRINTF("----- Unsupported Interface of attached Device -----\r\n");
        USB_PRINTF("  Interface Number = %d", intf_ptr->bInterfaceNumber);
        USB_PRINTF("  Alternate Setting = %d", intf_ptr->bAlternateSetting);
        USB_PRINTF("  Class = %d", intf_ptr->bInterfaceClass);
        USB_PRINTF("  SubClass = %d", intf_ptr->bInterfaceSubClass);
        USB_PRINTF("  Protocol = %d\r\n", intf_ptr->bInterfaceProtocol);
    }
    else if (USB_ATTACH_DEVICE_NOT_SUPPORT == event_code)
    {
        USB_PRINTF("----- Unsupported Device attached -----\r\n");
    }
}


/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_host_video_control_event
* Returned Value : None
* Comments       :
*     Called when video device has been attached, detached, etc.
*END*--------------------------------------------------------------------*/

void usb_host_video_control_event
(
    /* [IN] pointer to device instance */
    usb_device_instance_handle dev_handle,
    /* [IN] pointer to interface descriptor */
    usb_interface_descriptor_handle intf_handle,
    /* [IN] code number for event causing callback */
    uint32_t event_code
)
{
    usb_device_interface_struct_t* pHostIntf = (usb_device_interface_struct_t*)intf_handle;
    interface_descriptor_t* intf_ptr = pHostIntf->lpinterfaceDesc;

    switch (event_code)
    {
        case USB_ATTACH_EVENT:
            g_video_camera.interface_ptr[g_video_camera.interface_number] = pHostIntf;
            g_video_camera.interface_type[g_video_camera.interface_number] = USB_SUBCLASS_VIDEO_CONTROL;
            g_video_camera.interface_number++;
            USB_PRINTF("----- Attach Event -----\r\n");
            USB_PRINTF("State = %d", g_video_camera.control_state);
            USB_PRINTF("  Interface Number = %d", intf_ptr->bInterfaceNumber);
            USB_PRINTF("  Alternate Setting = %d", intf_ptr->bAlternateSetting);
            USB_PRINTF("  Class = %d", intf_ptr->bInterfaceClass);
            USB_PRINTF("  SubClass = %d", intf_ptr->bInterfaceSubClass);
            USB_PRINTF("  Protocol = %d\r\n", intf_ptr->bInterfaceProtocol);
            break;
        case USB_CONFIG_EVENT:
            if(g_video_camera.control_state == USB_DEVICE_IDLE)
            {
                if ((NULL != g_video_camera.dev_handle) && (g_video_camera.dev_handle != dev_handle))
                {
                    USB_PRINTF("Video device already attached - Control DEV_STATE = %d\r\n", g_video_camera.control_state);
                    USB_PRINTF("Video device already attached - Stream DEV_STATE = %d\r\n", g_video_camera.stream_state);
                    return;
                }
                g_video_camera.dev_handle = dev_handle;
                for(int i = 0;i < g_video_camera.interface_number;i++)
                {
                    if(g_video_camera.interface_type[i] == USB_SUBCLASS_VIDEO_CONTROL)
                    {
                        g_video_camera.control_intf_handle = g_video_camera.interface_ptr[i];
                        break;
                    }
                }
                g_video_camera.control_state = USB_DEVICE_ATTACHED;
            }
            else
            {
                 USB_PRINTF("Video device already attached - Control DEV_STATE = %d\r\n", g_video_camera.control_state);
            }
            break;
    
        case USB_INTF_OPENED_EVENT:
            USB_PRINTF("----- Interfaced Event -----\r\n");
            g_video_camera.control_state = USB_DEVICE_INTERFACE_OPENED;
            break;
    
        case USB_DETACH_EVENT:
            /* Use only the interface with desired protocol */
            USB_PRINTF("\r\n----- Detach Event -----\r\n");
            USB_PRINTF("State = %d", g_video_camera.control_state);
            USB_PRINTF("  Interface Number = %d", intf_ptr->bInterfaceNumber);
            USB_PRINTF("  Alternate Setting = %d", intf_ptr->bAlternateSetting);
            USB_PRINTF("  Class = %d", intf_ptr->bInterfaceClass);
            USB_PRINTF("  SubClass = %d", intf_ptr->bInterfaceSubClass);
            USB_PRINTF("  Protocol = %d\r\n", intf_ptr->bInterfaceProtocol);
            g_video_camera.interface_number--;
            g_video_camera.control_state = USB_DEVICE_DETACHED;
            break;
        default:
            USB_PRINTF("Video Device state = %d??\r\n", g_video_camera.control_state);
            g_video_camera.control_state = USB_DEVICE_IDLE;
            break;
    }

    /* notify application that status has changed */
    OS_Event_set(g_video_camera.video_camera_control_event, USB_EVENT_CTRL);
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : usb_host_video_stream_event
* Returned Value : None
* Comments       :
*     Called when video device has been attached, detached, etc.
*END*--------------------------------------------------------------------*/

void usb_host_video_stream_event
(
    /* [IN] pointer to device instance */
    usb_device_instance_handle dev_handle,
    /* [IN] pointer to interface descriptor */
    usb_interface_descriptor_handle intf_handle,
    /* [IN] code number for event causing callback */
    uint32_t event_code
)
{
    usb_device_interface_struct_t* pHostIntf = (usb_device_interface_struct_t*)intf_handle;
    interface_descriptor_t* intf_ptr = pHostIntf->lpinterfaceDesc;

    switch (event_code)
    {
        case USB_ATTACH_EVENT:
            g_video_camera.interface_ptr[g_video_camera.interface_number] = pHostIntf;
            g_video_camera.interface_type[g_video_camera.interface_number] = USB_SUBCLASS_VIDEO_STREAMING;
            g_video_camera.interface_number++;
            USB_PRINTF("----- Attach Event -----\r\n");
            USB_PRINTF("State = %d", g_video_camera.stream_state);
            USB_PRINTF("  Interface Number = %d", intf_ptr->bInterfaceNumber);
            USB_PRINTF("  Alternate Setting = %d", intf_ptr->bAlternateSetting);
            USB_PRINTF("  Class = %d", intf_ptr->bInterfaceClass);
            USB_PRINTF("  SubClass = %d", intf_ptr->bInterfaceSubClass);
            USB_PRINTF("  Protocol = %d\r\n", intf_ptr->bInterfaceProtocol);
            break;
        case USB_CONFIG_EVENT:
            if(g_video_camera.stream_state == USB_DEVICE_IDLE)
            {
                if ((NULL != g_video_camera.dev_handle) && (g_video_camera.dev_handle != dev_handle))
                {
                    USB_PRINTF("Video device already attached - Control DEV_STATE = %d\r\n", g_video_camera.control_state);
                    USB_PRINTF("Video device already attached - Stream DEV_STATE = %d\r\n", g_video_camera.stream_state);
                    return;
                }
                g_video_camera.dev_handle = dev_handle;
                for(int i = 0;i < g_video_camera.interface_number;i++)
                {
                    if(g_video_camera.interface_type[i] == USB_SUBCLASS_VIDEO_STREAMING)
                    {
                        g_video_camera.stream_intf_handle = g_video_camera.interface_ptr[i];
                        break;
                    }
                }
                g_video_camera.stream_state = USB_DEVICE_ATTACHED;
            }
            else
            {
                 USB_PRINTF("Video device already attached - Stream DEV_STATE = %d\r\n", g_video_camera.stream_state);
            }
            break;
    
        case USB_INTF_OPENED_EVENT:
            USB_PRINTF("----- Interfaced Event -----\r\n");
            g_video_camera.stream_state = USB_DEVICE_INTERFACE_OPENED;
            break;
    
        case USB_DETACH_EVENT:
            /* Use only the interface with desired protocol */
            USB_PRINTF("\r\n----- Detach Event -----\r\n");
            USB_PRINTF("State = %d", g_video_camera.stream_state);
            USB_PRINTF("  Interface Number = %d", intf_ptr->bInterfaceNumber);
            USB_PRINTF("  Alternate Setting = %d", intf_ptr->bAlternateSetting);
            USB_PRINTF("  Class = %d", intf_ptr->bInterfaceClass);
            USB_PRINTF("  SubClass = %d", intf_ptr->bInterfaceSubClass);
            USB_PRINTF("  Protocol = %d\r\n", intf_ptr->bInterfaceProtocol);
            g_video_camera.interface_number --;
            g_video_camera.stream_state = USB_DEVICE_DETACHED;
            break;
        default:
            USB_PRINTF("Video Device state = %d??\r\n", g_video_camera.stream_state);
            g_video_camera.stream_state = USB_DEVICE_IDLE;
            break;
    }

    /* notify application that status has changed */
    OS_Event_set(g_video_camera.video_camera_stream_event, USB_EVENT_CTRL);
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : main (Main_Task if using MQX)
* Returned Value : none
* Comments       :
*     Execution starts here
*
*END*--------------------------------------------------------------------*/
void APP_init ()
{ /* Body */
    
    usb_status           status = USB_OK;
    
    status = usb_host_init(CONTROLLER_ID, &g_video_camera.host_handle);
    if (status != USB_OK) 
    {
        USB_PRINTF("\r\nUSB Host Initialization failed! STATUS: 0x%x", status);
        return;
    }
    /*
     ** since we are going to act as the host driver, register the driver
     ** information for wanted class/subclass/protocols
     */
    status = usb_host_register_driver_info(g_video_camera.host_handle, (void *)DriverInfoTable);
    if (status != USB_OK) 
    {         
        USB_PRINTF("\r\nUSB Initialization driver info failed! STATUS: 0x%x", status);
          return;
    }

    status = usb_host_register_unsupported_device_notify(g_video_camera.host_handle, usb_host_video_unsupported_device_event);
    if (status != USB_OK) 
    {         
        USB_PRINTF("\r\nUSB Initialization driver info failed! STATUS: 0x%x", status);
          return;
    }
    
    g_video_camera.video_camera_control_event = OS_Event_create(0);/* manually clear */
    if (g_video_camera.video_camera_control_event == NULL)
    {
        USB_PRINTF("\r\nOS_Event_create failed!\r\n");
        return;
    }
    
    g_video_camera.video_camera_stream_event = OS_Event_create(0);/* manually clear */
    if (g_video_camera.video_camera_stream_event == NULL)
    {
        USB_PRINTF("\r\nOS_Event_create failed!\r\n");
        return;
    }
    g_video_camera.video_command_ptr = (video_command_t*)OS_Mem_alloc_zero(sizeof(video_command_t));
    if (g_video_camera.video_command_ptr == NULL)
    {
        USB_PRINTF("\r\nOS_Mem_alloc_zero failed!\r\n");
        return;
    }
    g_video_camera.stream_interface_alternate = 0;
   
    time_init();

    USB_PRINTF("Video camera starting...\r\n");
} /* Endbody */

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : APP_task
* Returned Value : none
* Comments       :
*     Used to execute the whole camera state machine running and camera control process.
*
*END*--------------------------------------------------------------------*/
void video_camera_control_task()
{
    usb_status              status = USB_OK;

    // Wait for insertion or removal event
    OS_Event_wait(g_video_camera.video_camera_control_event, USB_EVENT_CTRL, FALSE, 0);
    if (OS_Event_check_bit(g_video_camera.video_camera_control_event, USB_EVENT_CTRL))
        OS_Event_clear(g_video_camera.video_camera_control_event, USB_EVENT_CTRL);
    
    switch ( g_video_camera.control_state)
    {
        case USB_DEVICE_IDLE:
            break;

        case USB_DEVICE_ATTACHED:
            USB_PRINTF("Video device attached\r\n");
            g_video_camera.control_state = USB_DEVICE_SET_INTERFACE_STARTED;
            status = usb_host_open_dev_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.control_intf_handle, (class_handle*)&g_video_camera.video_control_handle);
            if (status != USB_OK)
            {
                USB_PRINTF("\r\nError in _usb_hostdev_open_interface: %x\r\n", status);
                return;
            }
            g_video_camera.video_command_ptr->class_control_handle = g_video_camera.video_control_handle;
            break;

        case USB_DEVICE_INTERFACE_OPENED:
            break;
        case USB_DEVICE_DETACHED:
            status = usb_host_close_dev_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.control_intf_handle, g_video_camera.video_control_handle);
            if (status != USB_OK)
            {
                USB_PRINTF("error in _usb_hostdev_close_interface %x\n", status);
            }
            g_video_camera.control_intf_handle = NULL;
            g_video_camera.video_control_handle = NULL;
            USB_PRINTF("Going to idle state\r\n");
            g_video_camera.control_state = USB_DEVICE_IDLE;
            break;
        case USB_DEVICE_OTHER:
            break;
        default:
            break;
        } /* Endswitch */
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : APP_task
* Returned Value : none
* Comments       :
*     Used to execute the whole camera state machine running and camera stream process.
*
*END*--------------------------------------------------------------------*/
void video_camera_stream_task()
{
    usb_status              status = USB_OK;

    // Wait for insertion or removal event
    OS_Event_wait(g_video_camera.video_camera_stream_event, USB_EVENT_CTRL, FALSE, 0);
    if (OS_Event_check_bit(g_video_camera.video_camera_stream_event, USB_EVENT_CTRL))
        OS_Event_clear(g_video_camera.video_camera_stream_event, USB_EVENT_CTRL);
    
    switch ( g_video_camera.stream_state)
    {
        case USB_DEVICE_IDLE:
            break;

        case USB_DEVICE_ATTACHED:
            USB_PRINTF("Video device attached\r\n");
            g_video_camera.stream_state = USB_DEVICE_SET_INTERFACE_STARTED;
            status = usb_host_open_dev_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.stream_intf_handle, (class_handle*)&g_video_camera.video_stream_handle);
            if (status != USB_OK)
            {
                USB_PRINTF("\r\nError in _usb_hostdev_open_interface: %x\r\n", status);
                return;
            }
            g_video_camera.video_command_ptr->class_stream_handle = g_video_camera.video_stream_handle;
            //g_video_camera.stream_interface_status = (uint8_t)STREAM_INTERFACE_IDLE;
            break;

        case USB_DEVICE_INTERFACE_OPENED:
            if (g_video_camera.stream_interface_status == (uint8_t)STREAM_INTERFACE_IDLE)
            {
                g_video_camera.stream_interface_status = (uint8_t)STREAM_INTERFACE_SET_DEFAULT_INTRFACE;
            }
            else if (g_video_camera.stream_interface_status == (uint8_t)STREAM_INTERFACE_INTERFACE_OPENING)
            {
                g_video_camera.stream_interface_status = (uint8_t)STREAM_INTERFACE_FIND_OPTIMAL_SETTING;
            }
            else if (g_video_camera.stream_interface_status == (uint8_t)STREAM_INTERFACE_SET_INTERFACE)
            {
                g_video_camera.stream_interface_status = (uint8_t)STREAM_INTERFACE_STREAMING;
                g_video_camera.stream_state = USB_DEVICE_VIDEO_START;
                g_video_camera.stream_pipe_opened = 1;
                g_video_camera.stream_transfer.is_1ms = 0;
                g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                g_video_camera.video_command_ptr->callback_fn = usb_host_audio_stream_callback;
            }
            else
            {
                
            }
            break;
        case USB_DEVICE_INTERFACE_OPENING:
            break;
        case USB_DEVICE_VIDEO_START:
//            if(g_video_camera.stream_transfer.stream_transfer == 0)
//            {
//                g_video_camera.video_command_ptr->callback_param = &g_video_camera;
//                g_video_camera.video_command_ptr->callback_fn = usb_host_audio_stream_callback;
//                g_video_camera.stream_transfer.stream_transfer = 1;
//            }
//            else
//            {
//                
//            }
            break;
        case USB_DEVICE_DETACHED:
            status = usb_host_close_dev_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.stream_intf_handle, g_video_camera.video_stream_handle);
            if (status != USB_OK)
            {
                USB_PRINTF("error in _usb_hostdev_close_interface %x\n", status);
            }
            g_video_camera.stream_pipe_opened = 0;
            g_video_camera.stream_interface_status = STREAM_INTERFACE_IDLE;
            g_video_camera.stream_intf_handle = NULL;
            g_video_camera.video_stream_handle = NULL;
            g_video_camera.stream_transfer.stream_transfer = 0;
            USB_PRINTF("Going to idle state\r\n");
            g_video_camera.stream_state = USB_DEVICE_IDLE;
            break;
        case USB_DEVICE_OTHER:
            break;
        default:
            break;
        } /* Endswitch */
}

/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : APP_task
* Returned Value : none
* Comments       :
*     Used to execute the whole keyboard state machine running and keyboard data process.
*
*END*--------------------------------------------------------------------*/
void APP_task()
{
    static usb_status status;
    static video_probe_and_commit_controls_t probe[3];
    static uint32_t spet = 0;
    
    switch (g_video_camera.stream_interface_status)
    {
    case STREAM_INTERFACE_IDLE:
        break;
    case STREAM_INTERFACE_SET_DEFAULT_INTRFACE:
        {
            status = usb_host_open_dev_alternate_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.stream_intf_handle, g_video_camera.stream_interface_alternate);
            if (status != USB_OK)
            {
                USB_PRINTF("\r\nError in usb_host_open_dev_alternate_interface: %x\r\n", status);
                return;
            }
            g_video_camera.stream_interface_status = STREAM_INTERFACE_INTERFACE_OPENING;
            g_video_camera.stream_state = USB_DEVICE_INTERFACE_OPENING;
            g_video_camera.stream_pipe_opened = 0;
        }
        break;
    case STREAM_INTERFACE_INTERFACE_OPENING:
        break;
    case STREAM_INTERFACE_FIND_OPTIMAL_SETTING:
        {
            uint32_t min_frame_size = 0xFFFFFFFF;
            uint32_t temp = 0;
            uint8_t index;
            uint8_t min_frame_index = 0xFF;
            video_payload_mjpeg_video_format_descriptor_t*  format_desc_ptr = NULL;
            video_payload_mjpeg_video_frame_descriptor_t*  frame_desc_ptr = NULL;
            
            status = usb_class_video_get_format_descriptor(g_video_camera.video_command_ptr, VS_FORMAT_MJPEG, (void*)&format_desc_ptr);
            if (status == USB_OK)
            {
                g_video_camera.video_format_desc_ptr = (void*)format_desc_ptr;
                for (index = 1;index < format_desc_ptr->bNumFrameDescriptors;index++)
                {
                    status = usb_class_video_get_frame_descriptor(g_video_camera.video_command_ptr, VS_FRAME_MJPEG, index, (void*)&frame_desc_ptr);
                    if (status == USB_OK)
                    {
                        temp = frame_desc_ptr->wHeight * frame_desc_ptr->wWitd;
                        //if(min_frame_size > temp)
                        if(index == 3)
                        {
                            min_frame_size = temp;
                            min_frame_index = index;
                            g_video_camera.video_frame_desc_ptr = (void*)frame_desc_ptr;
                        }
                    }
                }

                if(min_frame_index!= 0xFF)
                {
                    g_video_camera.stream_interface_status = (uint8_t)STREAM_INTERFACE_GET_SET_PROBE_COMMIT;
                    spet = 0;
                }
            }
        }
        break;
    case STREAM_INTERFACE_GET_SET_PROBE_COMMIT:
        switch (spet)
        {
        case 0:
            if (g_video_camera.in_ctrl == 0)
            {
                g_video_camera.in_ctrl = 1;
                g_video_camera.ctrl_status = USB_OK;
                g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_CUR, (void*)&probe[0]);
                spet++;
            }
            break;
        case 1:
            if (g_video_camera.in_ctrl == 0)
            {
                g_video_camera.in_ctrl = 1;
                g_video_camera.ctrl_status = USB_OK;
                g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_MAX, (void*)&probe[1]);
                spet++;
            }
            break;
        case 2:
            if (g_video_camera.in_ctrl == 0)
            {
                g_video_camera.in_ctrl = 1;
                g_video_camera.ctrl_status = USB_OK;
                g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_MIN, (void*)&probe[2]);
                spet++;
            }
            break;
        case 3:
            {
                video_payload_mjpeg_video_format_descriptor_t*  format_desc_ptr = NULL;
                video_payload_mjpeg_video_frame_descriptor_t*   frame_desc_ptr = NULL;
                uint32_t *                                      dwFrameInterval_ptr = NULL;
                uint8_t                                         i = 0;
                uint32_t                                        frame = 0;
                uint32_t                                        min_frame = 0xFFFFFFFF;
                uint32_t                                        frame_interval = 0;
                if (g_video_camera.in_ctrl == 0)
                {
                    format_desc_ptr = (video_payload_mjpeg_video_format_descriptor_t*)g_video_camera.video_format_desc_ptr;
                    frame_desc_ptr = (video_payload_mjpeg_video_frame_descriptor_t*)g_video_camera.video_frame_desc_ptr;
                    if (g_video_camera.video_probe_ptr == NULL)
                    {
                        g_video_camera.video_probe_ptr = OS_Mem_alloc_uncached_zero(sizeof(video_probe_and_commit_controls_t));
                    }
                    OS_Mem_copy((void*)&probe[2],g_video_camera.video_probe_ptr,26);
                    dwFrameInterval_ptr = (uint32_t*)&frame_desc_ptr->dwFrameInterval;
                    for(i = 0;i < frame_desc_ptr->bFrameIntervalType; i++)
                    {
                        OS_Mem_copy((uint8_t*)(dwFrameInterval_ptr + i), (uint8_t*)&frame_interval, 4);
                        frame = 10000000/frame_interval;
                        //if(min_frame > frame)
                        if (frame == 30)
                        //if (0x00051615 == dwFrameInterval_ptr[i]) // 30pfs
                        {
                            min_frame = frame;
                            g_video_camera.video_probe_ptr->dwFrameInterval = frame_interval;
                        }
                    }
                    g_video_camera.video_probe_ptr->bFormatIndex = format_desc_ptr->bFormatIndex;
                    g_video_camera.video_probe_ptr->bFrameIndex = frame_desc_ptr->bFrameIndex;
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    status = usb_class_video_set_probe(g_video_camera.video_command_ptr, SET_CUR, (void*)g_video_camera.video_probe_ptr);
                    spet++;
                }
            }
            break;
        case 4:
        case 5:
        case 6:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    if(spet == 4)
                    {
                        status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_CUR, (void*)&probe[0]);
                    }
                    else if(spet == 5)
                    {
                        status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_MAX, (void*)&probe[1]);
                    }
                    else
                    {
                        status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_MIN, (void*)&probe[2]);
                    }
                    spet++;
                }
            }
            break;
        case 7:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    OS_Mem_copy((void*)&probe[0],g_video_camera.video_probe_ptr,22);
                    status = usb_class_video_set_probe(g_video_camera.video_command_ptr, SET_CUR, (void*)g_video_camera.video_probe_ptr);
                    spet++;
                }
            }
            break;
        case 8:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_CUR, (void*)&probe[0]);
                    spet++;
                }
            }
            break;
        case 9:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    OS_Mem_copy((void*)&probe[0],g_video_camera.video_probe_ptr,26);
                    status = usb_class_video_set_probe(g_video_camera.video_command_ptr, SET_CUR, (void*)g_video_camera.video_probe_ptr);
                    spet++;
                }
            }
            break;
        case 10:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    status = usb_class_video_get_probe(g_video_camera.video_command_ptr, GET_CUR, (void*)&probe[0]);
                    spet++;
                }
            }
            break;
        case 11:
            {
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.in_ctrl = 1;
                    g_video_camera.ctrl_status = USB_OK;
                    g_video_camera.video_command_ptr->callback_param = &g_video_camera;
                    g_video_camera.video_command_ptr->callback_fn = usb_host_audio_ctrl_callback;
                    OS_Mem_copy((void*)&probe[0],g_video_camera.video_probe_ptr,26);
                    status = usb_class_video_set_commit(g_video_camera.video_command_ptr, SET_CUR, (void*)g_video_camera.video_probe_ptr);
                    spet++;
                }
            }
            break;
        case 12:
            {
                uint32_t param1 = 0;
                uint32_t param2 = 0;
                void*    descriptor = NULL;
                interface_descriptor_t*             intf;
                usb_device_interface_struct_t*      pDeviceIntf;
                
                if (g_video_camera.in_ctrl == 0)
                {
                    g_video_camera.stream_interface_alternate = 2;
                    
                    pDeviceIntf = (usb_device_interface_struct_t*)g_video_camera.stream_intf_handle;
                    intf = pDeviceIntf->lpinterfaceDesc;
                    param1 = (uint32_t)((uint32_t)(intf->bInterfaceNumber << 8) | (0x00000000));
                    usb_host_get_dev_descriptor(g_video_camera.stream_intf_handle, USB_DESC_TYPE_IF, &param1, &param2, &descriptor);
                    usb_host_get_dev_descriptor(descriptor, USB_DESC_TYPE_EP, &param2, NULL, &descriptor);             
                    
                    status = usb_class_video_set_stream_inf(g_video_camera.video_stream_handle, g_video_camera.stream_interface_alternate);
                    if (status != USB_OK)
                    {
                        USB_PRINTF("\r\nError in usb_class_video_set_stream_inf: %x\r\n", status);
                        return;
                    }
                    status = usb_host_open_dev_alternate_interface(g_video_camera.host_handle, g_video_camera.dev_handle, g_video_camera.stream_intf_handle, g_video_camera.stream_interface_alternate);
                    if (status != USB_OK)
                    {
                        USB_PRINTF("\r\nError in usb_host_open_dev_alternate_interface: %x\r\n", status);
                        return;
                    }
                    g_video_camera.stream_interface_status = STREAM_INTERFACE_SET_INTERFACE;
                    g_video_camera.stream_state = USB_DEVICE_INTERFACE_OPENING;
                }
            }
            break;
        default:
            break;
        }        
        break;
    }
    video_camera_control_task();
    video_camera_stream_task();
    
    if(g_video_camera.stream_transfer.is_1ms)
    {   
        g_video_camera.stream_transfer.is_1ms = 0;
        get_video_data();
    }
}

void get_video_data()
{
    if(g_video_camera.stream_pipe_opened)
    {
        if(!g_video_camera.stream_transfer.stream_transfer)
        {
			if (g_video_data_pool[g_video_data_rx_index].flag == 0)
			{
                g_video_camera.stream_transfer.stream_transfer = 1;
                
	            usb_class_video_stream_recv_data(g_video_camera.video_command_ptr, 
												 &g_video_data_pool[g_video_data_rx_index].rawdata[0],
												 g_video_camera.video_probe_ptr->dwMaxPayloadTransferSize);
			}
        }
    }
}


#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
    
#if defined(FSL_RTOS_MQX)
void Main_Task(uint32_t param);
TASK_TEMPLATE_STRUCT  MQX_template_list[] =
    {
   { 1L,     Main_Task,      2500L,  MQX_MAIN_TASK_PRIORITY, "Main",      MQX_AUTO_START_TASK},
   { 0L,     0L,             0L,    0L, 0L,          0L }
};
    #endif
    
static void Task_Start(void *arg)
{
#if (USE_RTOS)
    APP_init();
   
    for ( ; ; ) {
#endif
        APP_task();
#if (USE_RTOS)
    } /* Endfor */
#endif
}

#if defined(FSL_RTOS_MQX)
void Main_Task(uint32_t param)
#else
int main(void)
#endif
{
    OSA_Init();
    hardware_init();
    dbg_uart_init();
    comm_init();
	//Audio_Init();
    
#if !(USE_RTOS)
    APP_init();
#endif

    OS_Task_create(Task_Start, NULL, 9L, 3000L, "task_start", NULL);
    OSA_Start();
#if !defined(FSL_RTOS_MQX)
    return 1;
#endif
}
#endif
/* EOF */

