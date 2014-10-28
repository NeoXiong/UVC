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
* $FileName: usb_host_video.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*   This file defines a template structure for Class Drivers.
*
*END************************************************************************/
#ifndef __usb_host_video_h__
#define __usb_host_video_h__

/* Video Tierminal Types Code */

/* USB Terminal Type Codes */
#define TT_VENDOR_SPECIFIC                           (0x0100)
#define TT_STREAMING                                  (0x0101)

/* Input Terminal Type Codes */
#define ITT_VENDOR_SPECIFIC                           (0x0200)
#define ITT_CAMERA                                     (0x0201)
#define ITT_MEDIA_TRANSPORT_INPUT                       (0x0202)

/* Output Terminal Type Codes */
#define OTT_VENDOR_SPECIFIC                           (0x0300)
#define OTT_DISPLAY                                   (0x0301)
#define OTT_MEDIA_TRANSPORT_ONTPUT                    (0x0302)

/* External Terminal Type Codes */
#define EXTERNAL_VENDOR_SPECIFIC                        (0x0400)
#define COMPOSITE_CONNECTOR                            (0x0401)
#define SVIDEO_CONNECTOR                               (0x0402)
#define COMPONENT_CONNECTOR                            (0x0403)

/* Video Interface Class Code */
#define CC_VIDEO                          (0x0E)

/* Video Interface Subclass Codes */
#define SC_UNDEFINED                      (0x00)
#define SC_VIDEOCONTROL                   (0x01)
#define SC_VIDEOSTREAMING                 (0x02)
#define SC_VIDEO_INTERFACE_COLLECTION     (0x03)

/* Video Interface Protocol Codes */
#define PC_PROTOCOL_UNDEFINED             (0x00)

/* Video Class-Specific Descriptor Types */
#define CS_UNDEFINED                      (0x20)
#define CS_DEVICE                         (0x21)
#define CS_CONFIGURATION                  (0x22)
#define CS_STRING                         (0x23)
#define CS_INTERFACE                      (0x24)
#define CS_ENDPOINT                       (0x25)

/* Video Class-Specific VC Interface Descriptor Subtypes */
#define VC_DESCRIPTOR_UNDEFINED           (0x00)
#define VC_HEADER                         (0x01)
#define VC_INPUT_TERMINAL                 (0x02)
#define VC_OUTPUT_TERMINAL                (0x03)
#define VC_SECLECTOR_UNIT                 (0x04)
#define VC_PROCESSING_UNIT                (0x05)
#define VC_EXTENSION_UNIT                 (0x06)

/* Video Class-specific VS Interface Desriptor Subtypes */
#define VS_UNDEFINED                      (0x00)
#define VS_INPUT_HEADER                   (0x01)
#define VS_OUTPUT_HEADER                  (0x02)
#define VS_STILL_IMAGE_FRAME              (0x03)
#define VS_FORMAT_UNCOMPRESSED            (0x04)
#define VS_FRAME_UNCOMPRESSED             (0x05)
#define VS_FORMAT_MJPEG                   (0x06)
#define VS_FRAME_MJPEG                    (0x07)
#define VS_FORMAT_MPEG2TS                 (0x0A)
#define VS_FORMAT_DV                      (0x0C)
#define VS_COLORFORMAT                    (0x0D)
#define VS_FORMAT_FRAME_BASED             (0x10)
#define VS_FRAME_FRAME_BASED              (0x11)
#define VS_FORMAT_STREAM_BASED            (0x12)

/* Video Class-Specific Endpoint Descriptor Subtypes */
#define EP_UNDEFINED                      (0x00)
#define EP_GENERAL                        (0x01)
#define EP_ENDPOINT                       (0x02)
#define EP_INTERRUPT                      (0x03)

/* Video Class-Specific Request Codes */
#define EP_UNDEFINED                      (0x00)
#define SET_CUR                           (0x01)
#define GET_CUR                           (0x81)
#define GET_MIN                           (0x82)
#define GET_MAX                           (0x83)
#define GET_RES                           (0x84)
#define GET_LEN                           (0x85)
#define GET_INFO                          (0x86)
#define GET_DEF                           (0x87)


/* VideoControl Interface Control Selector Codes */
#define VC_CONTROL_UNDEFINED              (0x00)
#define VC_VIDEO_POWER_MODE_CONTROL       (0x01)
#define VC_REQUEST_ERROR_CODE_CONTROL     (0x02)

/* Terminal Control Selector Codes */
#define TE_CONTROL_UNDEFINED              (0x00)

/* Selector Unit Control Selector Codes */
#define SU_CONTROL_UNDEFINED               (0x00)
#define SU_INPUT_SELECT_CONTROL            (0x01)

/* Camera Terminal Control Selector Codes */
#define CT_CONTROL_UNDEFINED                (0x00)
#define CT_SCANNING_MODE_CONTROL            (0x01)
#define CT_AE_MODE_CONTROL                  (0x02)
#define CT_AE_PRIORITY_CONTROL              (0x03)
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL   (0x04)
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL   (0x05)
#define CT_FOCUS_ABSOLUTE_CONTROL           (0x06)
#define CT_FOCUS_RELATIVE_CONTROL           (0x07)
#define CT_FOCUS_AUTO_CONTROL               (0x08)
#define CT_IRIS_ABSOLUTE_CONTROL            (0x09)
#define CT_IRIS_RELATIVE_CONTROL            (0x0A)
#define CT_ZOOM_ABSOLUTE_CONTROL            (0x0B)
#define CT_ZOOM_RELATIVE_CONTROL            (0x0C)
#define CT_PANTILT_ABSOLUTE_CONTROL         (0x0D)
#define CT_PANTILT_RELATIVE_CONTROL         (0x0E)
#define CT_ROLL_ABSOLUTE_CONTROL            (0x0F)
#define CT_ROLL_RELATIVE_CONTROL            (0x10)
#define CT_PRIVACY_CONTROL                  (0x11)

/* Processing Unit Control Selector Codes */
#define PU_CONTROL_UNDEFINED                      (0x00)
#define PU_BACKLIGHT_COMPENSATION_CONTROL         (0x01)
#define PU_BRIGHTNESS_CONTROL                     (0x02)
#define PU_CONTRACT_CONTROL                       (0x03)
#define PU_GAIN_CONTROL                           (0x04)
#define PU_POWER_LINE_FREQUENCY_CONTROL           (0x05)
#define PU_HUE_CONTROL                            (0x06)
#define PU_SATURATION_CONTROL                     (0x07)
#define PU_SHARRNESS_CONTROL                      (0x08)
#define PU_GAMMA_CONTROL                          (0x09)
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL      (0x0A)
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL (0x0B)
#define PU_WHITE_BALANCE_COMPONENT_CONTROL        (0x0C)
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL   (0x0D)
#define PU_DIGITAL_MULTIPLIER_CONTROL             (0x0E)
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL       (0x0F)
#define PU_HUE_AUTO_CONTROL                       (0x10)
#define PU_ANALOG_VIDEO_STANDARD_CONTROL          (0x11)
#define PU_ANALOG_LOCK_STATUS_CONTROL             (0x12)

/* Extension Unit Control Selectors Codes */
#define XU_CONTROL_UNDEFINED                       (0x00)

/* VideoStreming Unit Control Selector Codes */
#define VS_CONTROL_UNDEFINED                     (0x00)
#define VS_PROBE_CONTROL                         (0x01)
#define VS_COMMIT_CONTROL                        (0x02)
#define VS_STILL_PROBE_CONTROL                   (0x03)
#define VS_STILL_COMMIT_CONTROL                  (0x04)
#define VS_STILL_IMAGE_TRIGGER_CONTROL           (0x05)
#define VS_STREAM_ERROR_CODE_CONTROL             (0x06)
#define VS_GENERATE_KEY_FRAME_CONTROL            (0x07)
#define VS_UPDATE_FRAME_SEGMENT_CONTROL          (0x08)
#define VS_SYNCH_DELAY_CONTROL                   (0x09)

/* Class-specific VC  Interface Header Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vc_interface_header_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint16_t bcdVDC;
    uint16_t wTotalLength;
    uint32_t dwClockFrequency;
    uint8_t  bInCollection;
} PACKED_STRUCT_END;
typedef struct _video_vc_interface_header_descriptor video_vc_interface_header_descriptor_t;


/* Input Terminal Descriptor */
PACKED_STRUCT_BEGIN
struct _video_input_terminal_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  iTerminal;
} PACKED_STRUCT_END;
typedef struct _video_input_terminal_descriptor video_input_terminal_descriptor_t;

/* Output Terminal Descriptor */
PACKED_STRUCT_BEGIN
struct _video_output_terminal_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  iTerminal;
} PACKED_STRUCT_END;
typedef struct _video_output_terminal_descriptor video_output_terminal_descriptor_t;

/* Camera Terminal Descriptor */
PACKED_STRUCT_BEGIN
struct _video_camera_terminal_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  iTerminal;
    uint16_t wObjectiveFocalLengthMin;
    uint16_t wObjectiveFocalLengthMax;
    uint16_t wOcularFocalLength;
    uint8_t  bControlSize;
    uint16_t bmControls;
} PACKED_STRUCT_END;
typedef struct _video_camera_terminal_descriptor video_camera_terminal_descriptor_t;

/* Selector Unit Descriptor */
PACKED_STRUCT_BEGIN
struct _video_selector_unit_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;
    uint8_t  bNrInPins;
} PACKED_STRUCT_END;
typedef struct _video_selector_unit_descriptor video_selector_unit_descriptor_t;

/* Processing Unit Descriptor */
PACKED_STRUCT_BEGIN
struct _video_processing_unit_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;
    uint8_t  bSourceID;
    uint16_t wMaxMultiplier;
    uint8_t  bControlSize;
} PACKED_STRUCT_END;
typedef struct _video_processing_unit_descriptor video_processing_unit_descriptor_t;

/* Extension Unit Descriptor */
PACKED_STRUCT_BEGIN
struct _video_extension_unit_descriptor 
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;
    uint8_t  guidExtensionCode[16];
    uint8_t  bNumControls;
    uint8_t  bNrInPins;
} PACKED_STRUCT_END;
typedef struct _video_extension_unit_descriptor video_extension_unit_descriptor_t;

PACKED_STRUCT_BEGIN
struct _video_extension_unit_descriptor_list
{
    uint32_t extension_num;
    void*   extension_descriptor_list_ptr;
} PACKED_STRUCT_END;
typedef struct _video_extension_unit_descriptor_list video_extension_unit_descriptor_list_t;

/* Class-specific VC Interrupt Endpo int Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vc_interrupt_endpoint_descriptor 
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint16_t wMaxTransferSize;
} PACKED_STRUCT_END;
typedef struct _video_vc_interrupt_endpoint_descriptor video_vc_interrupt_endpoint_descriptor_t;

/* Class-specific VS  Interface Input Header Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vs_interface_input_header_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bNumFormats;
    uint16_t wTotalLength;
    uint8_t  bEndpointAddress;
    uint8_t  bmInfo;
    uint8_t  bTerminalLink;
    uint8_t  bStillCaptureMethod;
    uint8_t  bTriggerSupport;
    uint8_t  bTriggerUsage;
    uint8_t  bControlSize;
} PACKED_STRUCT_END;
typedef struct _video_vs_interface_input_header_descriptor video_vs_interface_input_header_descriptor_t;

/* Class-specific VS  Interface Output Header Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vs_interface_output_header_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bNumFormats;
    uint16_t wTotalLength;
    uint8_t  bEndpointAddress;
    uint8_t  bTerminalLink;
    uint8_t  bControlSize;
} PACKED_STRUCT_END;
typedef struct _video_vs_interface_output_header_descriptor video_vs_interface_output_header_descriptor_t;

/* Payload Format Descriptor */
PACKED_STRUCT_BEGIN
struct _video_payload_format_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatIndex;
} PACKED_STRUCT_END;
typedef struct _video_payload_format_descriptor video_payload_format_descriptor_t;

/* Motion-JPEG Video Format Descriptor */
PACKED_STRUCT_BEGIN
struct _video_payload_mjpeg_video_format_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatIndex;
    uint8_t  bNumFrameDescriptors;
    uint8_t  bmFlags;
    uint8_t  bDefaultFrameIndex;
    uint8_t  bAspectRatioX;
    uint8_t  bAspectRatioY;
    uint8_t  bmInterlaceFlags;
    uint8_t  bCopyProtect;
} PACKED_STRUCT_END;
typedef struct _video_payload_mjpeg_video_format_descriptor video_payload_mjpeg_video_format_descriptor_t;

/* MPEG-2 TS Format Descriptor */
PACKED_STRUCT_BEGIN
struct _video_payload_mpeg2ts_video_format_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatIndex;
    uint8_t  bDataOffset;
    uint8_t  bPacketLength;
    uint8_t  bStrideLength;
    uint8_t  guidStrideFormat[16];
} PACKED_STRUCT_END;
typedef struct _video_payload_mpeg2ts_video_format_descriptor video_payload_mpeg2ts_video_format_descriptor_t;

/* Motion-JPEG Video Frame Descriptor */
PACKED_STRUCT_BEGIN 
struct _video_payload_mjpeg_video_frame_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFrameIndex;
    uint8_t  bmCapabilities;
    uint16_t wWitd;
    uint16_t wHeight;
    uint32_t dwMinBitRate;
    uint32_t dwMaxBitRate;
    uint32_t dwMaxVideoFrameBufferSize;
    uint32_t dwDefaultFrameInterval;
    uint8_t  bFrameIntervalType;
    uint32_t dwFrameInterval;
} PACKED_STRUCT_END;
typedef struct _video_payload_mjpeg_video_frame_descriptor video_payload_mjpeg_video_frame_descriptor_t;

/* Still Image Frame Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vs_still_image_frame_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bEndpointAddress;
    uint8_t  bNumImageSizePatterns;
} PACKED_STRUCT_END;
typedef struct _video_vs_still_image_frame_descriptor video_vs_still_image_frame_descriptor_t;

/* Color Matching Descriptor */
PACKED_STRUCT_BEGIN
struct _video_vs_color_matching_descriptor 
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bColorPrimaries;
    uint8_t  bTransferCharacteristics;
    uint8_t  bMatrixCoefficients;
} PACKED_STRUCT_END;
typedef struct _video_vs_color_matching_descriptor video_vs_color_matching_descriptor_t;

typedef struct _video_vs_mjpeg_frame_descriptor_list
{
    uint32_t frame_num;
    video_payload_mjpeg_video_frame_descriptor_t*   frame_descriptor_list_ptr;
} video_vs_mjpeg_frame_descriptor_list_t;

/* Payload Format Descriptor */
PACKED_STRUCT_BEGIN
struct _video_common_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
} PACKED_STRUCT_END;
typedef struct _video_common_descriptor video_common_descriptor_t;

PACKED_STRUCT_BEGIN
struct _video_frame_common_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFrameIndex;
} PACKED_STRUCT_END;
typedef struct _video_frame_common_descriptor video_frame_common_descriptor_t;

PACKED_STRUCT_BEGIN
struct _video_probe_and_commit_controls
{
    uint16_t bmHint;
    uint8_t  bFormatIndex;
    uint8_t  bFrameIndex;
    uint32_t dwFrameInterval;
    uint16_t wKeyFrameRate;
    uint16_t wPFrameRate;
    uint16_t wCompQuality;
    uint16_t wCompWindowSize;
    uint16_t wDelay;
    uint32_t dwMaxVideoFrameSize;
    uint32_t dwMaxPayloadTransferSize;
    uint32_t dwClockFrequency;
    uint8_t  bmFramingInfo;
    uint8_t  bPreferedVersion;
    uint8_t  bMinVersion;
    uint8_t  bMaxVersion;    
} PACKED_STRUCT_END;
typedef struct _video_probe_and_commit_controls video_probe_and_commit_controls_t;

PACKED_STRUCT_BEGIN
struct _video_payload_header_struct
{
    uint8_t          bHeaderLength;
    union
    {
        uint8_t      bmHeaderInfo;
        struct
        {
          uint8_t frame_id:1;
          uint8_t end_of_frame:1;
          uint8_t presentation_time:1;
          uint8_t source_clock:1;
          uint8_t reserved:1;
          uint8_t still_image:1;
          uint8_t error:1;
          uint8_t end_of_header:1;
        }bitMap;
    }HeaderInfo;
    uint32_t         dwPresentationTime;
    uint32_t         dwClockFrequency;
    uint16_t         scrSourceClock;
} PACKED_STRUCT_END;
typedef struct _video_payload_header_struct video_payload_header_struct_t;

typedef union _video_descriptor_union
{
   uint32_t                      word;
   uint8_t *                     bufr;
   void*                         pntr;
   device_descriptor_t         dvic;
   usb_configuration_descriptor_t*  cfig;
   interface_descriptor_t*      intf;
   endpoint_descriptor_t*       ndpt;
   qualifier_descriptor_t*      qual;
   otg_descriptor_t*            otg;
   common_descriptor_t*         common;
   video_common_descriptor_t*   video_common;
   video_frame_common_descriptor_t * video_frame_common;
} video_descriptor_union_t;

/*
** HID Class Interface structure. This structure will be passed to
** all commands to this class driver.
*/

typedef struct _usb_video_class_control_strcut
{
    usb_host_handle                 host_handle; 
    usb_device_instance_handle      dev_handle;
    usb_interface_descriptor_handle intf_handle;
     /* Only 1 command can be issued at one time */
    usb_pipe_handle                 control_pipe;
    usb_pipe_handle                 control_interrupt_in_pipe;

    usb_device_interface_struct_t*         control_interface_ptr;
    video_vc_interface_header_descriptor_t* vc_interface_header_ptr;
    video_input_terminal_descriptor_t * vc_input_terminal_ptr;
    video_output_terminal_descriptor_t * vc_output_terminal_ptr;
    video_processing_unit_descriptor_t * vc_processing_unit_ptr;
    video_extension_unit_descriptor_list_t vc_extension_unit_list_ptr;
    
    /* Here we store callback and parameter from higher level */
    tr_callback                     recv_callback;
    void*                           recv_param;
    tr_callback                     ctrl_callback;
    void*                           ctrl_param;
    uint32_t                        running;
    bool                            in_setup;
} usb_video_control_struct_t;

typedef struct _usb_video_class_stream_struct
{
    usb_host_handle                 host_handle; 
    usb_device_instance_handle      dev_handle;
    usb_interface_descriptor_handle intf_handle;
     /* Only 1 command can be issued at one time */
    usb_pipe_handle                 control_pipe;
    usb_pipe_handle                 stream_iso_in_pipe;
    usb_pipe_handle                 stream_iso_out_pipe;
    usb_pipe_handle                 stream_bulk_in_pipe;
    usb_pipe_handle                 stream_bulk_out_pipe;
    usb_device_interface_struct_t*         stream_interface_ptr;
    interface_descriptor_t*         current_stream_interface_ptr;
    video_vs_interface_input_header_descriptor_t * vs_input_header_ptr;
    video_payload_mjpeg_video_format_descriptor_t* vs_mjpeg_format_ptr;
    video_vs_mjpeg_frame_descriptor_list_t vs_mjpeg_frame_list;
    
    /* Here we store callback and parameter from higher level */
    tr_callback                     recv_callback;
    void*                           recv_param;
    uint32_t                        running;
    usb_device_ep_struct_t*         altrnate_if_ep_descriptor_list;
    uint8_t                         altrnate_if_ep_num;
} usb_video_stream_struct_t;

/* Audio Command */
typedef struct {
   	class_handle            class_control_handle;
	class_handle            class_stream_handle;
    tr_callback             callback_fn;
    void*                   callback_param;
} video_command_t;



/* Class specific functions exported by HID class driver */
#ifdef __cplusplus
extern "C" {
#endif

extern usb_status usb_class_video_control_init(usb_device_instance_handle, usb_interface_descriptor_handle, class_handle*);
extern usb_status usb_class_video_control_deinit(class_handle);
extern usb_status usb_class_video_control_pre_deinit(class_handle);

extern usb_status usb_class_video_stream_init(usb_device_instance_handle, usb_interface_descriptor_handle, class_handle*);
extern usb_status usb_class_video_stream_deinit(class_handle);
extern usb_status usb_class_video_stream_pre_deinit(class_handle);


extern usb_status usb_class_video_set_stream_inf
(
    class_handle                    class_handle,
    uint8_t                          inf_alternate
);

extern usb_status usb_class_video_get_format_descriptor
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     sub_type,
    /* [OUT] Data Bffer */
    uint8_t **                  descriptor_ptr
);

extern usb_status usb_class_video_get_frame_descriptor
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     sub_type,
    /* [IN] Request index */
    uint8_t                     index,
    /* [OUT] Data Bffer */
    uint8_t **                   descriptor_ptr
);

usb_status usb_class_video_get_probe
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
);

usb_status usb_class_video_set_probe
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
);

usb_status usb_class_video_get_commit
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
);

usb_status usb_class_video_set_commit
(
    /* [IN] Class Interface structure pointer */
    video_command_t*            com_ptr,
    /* [IN] Request code */
    uint8_t                     brequest,
    /* [IN] Data Bffer */
    uint8_t *                   probe
);

usb_status usb_class_video_stream_recv_data
(
    /* [IN] Class Interface structure pointer */
    video_command_t*         com_ptr,
    /* [IN] The buffer address */
    uint8_t *               buffer,
    /* [IN] The buffer address */
    uint16_t                length
);

usb_status usb_class_video_control_recv_data
(
    /* [IN] Class Interface structure pointer */
    video_command_t*         com_ptr,
    /* [IN] The buffer address */
    uint8_t *               buffer,
    /* [IN] The buffer address */
    uint16_t                length
);

#ifdef __cplusplus
}
#endif

#endif
