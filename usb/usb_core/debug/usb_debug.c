/**HEADER**********************************************************************
*
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved
*
*******************************************************************************
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
* $FileName: usb_debug.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*   
*
*END************************************************************************/
#include "usb_device_config.h"
#if USBCFG_DEV_KHCI
#include "types.h"
#include "usb_types.h" //USB error definitions
#include "compiler.h"
#include "usb_desc.h"  //USB descriptor macros
#include "usb_misc.h"
#include "usb_device_stack_interface.h"
#include "khci_dev.h"
#include "usb_dev.h"

#include "usb_debug.h"

volatile uint32_t g_usb_record_list_i = 0;
_RECORD_LIST_T g_usb_record_list[USB_RECORD_LIST_MAX] = {0};

/**************************************************************************//*!
 *
 * @name  _dump_record_list
 *
 * @brief The funtion output the trace points in memory.
 *
 *****************************************************************************/
void _dump_record_list()
{
	uint32_t i;
	USB_PRINTF("\ng_usb_record_list_i: %d", g_usb_record_list_i);
	for(i = 0; i < USB_RECORD_LIST_MAX; i++)
	{
		USB_PRINTF("\n[%.4d] %s: 0x%.8x, 0x%.8x, 0x%.8x, 0x%.8x",
			i,
			g_usb_record_list[i].event_string,
			g_usb_record_list[i].record[0],
			g_usb_record_list[i].record[1],
			g_usb_record_list[i].record[2],
			g_usb_record_list[i].record[3]
		  );
	}

}

#endif