/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file virtual_camera.h
 *
 * @author 
 *
 * @version 
 *
 * @date Jul-20-2010
 *
 * @brief The file contains Macro's and functions required for Virtual Camera
 *        Loopback Application
 *
 *****************************************************************************/

#ifndef _VIDEO_CAMERA_H
#define _VIDEO_CAMERA_H

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define  SD_CARD_FATFS                      (0)

#ifndef  SYST_CSR
#define  SYST_CSR      *((volatile uint32_t *)0xE000E010)                   /* Data Watchpoint and Trace (DWT) Cycle Count Register */
#endif

#ifndef  SYST_RVR
#define  SYST_RVR      *((volatile uint32_t *)0xE000E014)                   /* Data Watchpoint and Trace (DWT) Control Register     */                
#endif

#ifndef  SYST_CVR
#define  SYST_CVR      *((volatile uint32_t *)0xE000E018)                   /* Data Watchpoint and Trace (DWT) Control Register     */                
#endif

#define  PICTURE_COUNT                      (300)

#define  HIGH_SPEED                         (0)

#if HIGH_SPEED
#define CONTROLLER_ID                       USB_CONTROLLER_EHCI_0
#else
#define CONTROLLER_ID                       USB_CONTROLLER_KHCI_0
#endif
   
#define  USB_DEVICE_IDLE                   (0)
#define  USB_DEVICE_ATTACHED               (1)
#define  USB_DEVICE_CONFIGURED             (2)
#define  USB_DEVICE_SET_INTERFACE_STARTED  (3)
#define  USB_DEVICE_INTERFACE_OPENED       (4)
#define  USB_DEVICE_SETTING_PROTOCOL       (5)
#define  USB_DEVICE_INUSE                  (6)
#define  USB_DEVICE_DETACHED               (7)
#define  USB_DEVICE_OTHER                  (8)
#define  USB_DEVICE_INTERFACE_CLOSED       (9)
#define  USB_DEVICE_VIDEO_START            (0x0A)
#define  USB_DEVICE_INTERFACE_OPENING       (0x0B)

enum
{
    STREAM_INTERFACE_IDLE                   = 0x00,
    STREAM_INTERFACE_SET_DEFAULT_INTRFACE = 0x01,
    STREAM_INTERFACE_GET_SET_PARAM         = 0x02,
    STREAM_INTERFACE_FIND_OPTIMAL_SETTING = 0x03,
    STREAM_INTERFACE_GET_SET_PROBE_COMMIT = 0x04,
    STREAM_INTERFACE_SET_INTERFACE         = 0x05,
    STREAM_INTERFACE_STREAMING             = 0x06,
    STREAM_INTERFACE_INTERFACE_OPENING     = 0x07,
};

typedef struct _video_camera_stream_struct
{
    uint32_t         stream_transfer;
    usb_status       stream_status;
    uint32_t         transfer_length;
    uint32_t         is_1ms;
} video_camera_stream_struct_t;

typedef struct _video_camera_struct
{
    usb_host_handle                         host_handle;
    usb_device_interface_struct_t*          interface_ptr[USBCFG_HOST_MAX_INTERFACE_PER_CONFIGURATION];
    usb_device_instance_handle              dev_handle;
    usb_interface_descriptor_handle         control_intf_handle;
    usb_interface_descriptor_handle         stream_intf_handle;
    class_handle                            video_control_handle; /* Class-specific info */
    class_handle                            video_stream_handle; /* Class-specific info */
    video_command_t*                        video_command_ptr;
    video_probe_and_commit_controls_t*      video_probe_ptr;
    video_probe_and_commit_controls_t*      video_commit_ptr;
    void*                                   video_format_desc_ptr;
    void*                                   video_frame_desc_ptr;
    os_event_handle                         video_camera_control_event;
    os_event_handle                         video_camera_stream_event;
    uint32_t                                control_state;  /* Attach/detach state */
    uint32_t                                stream_state;  /* Attach/detach state */
    usb_status                              ctrl_status;
    video_camera_stream_struct_t            stream_transfer;
    uint8_t                                 stream_pipe_opened;
    uint8_t                                 interface_type[USBCFG_HOST_MAX_INTERFACE_PER_CONFIGURATION];
    uint8_t                                 interface_number;
    uint8_t                                 stream_interface_alternate;
    uint8_t                                 stream_interface_status;
    uint8_t                                 in_ctrl;
    
} video_camera_struct_t;
/*****************************************************************************
 * Global variables
 *****************************************************************************/

/*****************************************************************************
 * Global Functions
 *****************************************************************************/


#endif 
