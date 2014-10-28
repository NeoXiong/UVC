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
* $FileName: usb_debug.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*   
*
*END************************************************************************/

#ifndef __usb_debug_h__
#define __usb_debug_h__ 1
/******************************************************************************
 * Macro's
 *****************************************************************************/
#define USB_RECORD_ENTRY_MAX 4
#define USB_RECORD_LIST_MAX 500
/******************************************************************************
 * Types
 *****************************************************************************/
typedef struct{
	char* event_string; /*the event description*/
	uint32_t record[USB_RECORD_ENTRY_MAX]; /*variables need to be recorded*/
}_RECORD_LIST_T;

/******************************************************************************
 * This macro is used to log the points into memory.
 *
 * Usage: (example)
 *           _RECORD_LIST("TOKEN_DONE", var0, var1, var2, var3);
 *
 * Anytime you want to dump the records, using:
 *
 *           _dump_record_list();
 *
 *****************************************************************************/
#define _RECORD_LIST(a, b, c, d, e) \
do{ \
	OS_Lock(); \
	if(g_usb_record_list_i >= USB_RECORD_LIST_MAX) g_usb_record_list_i = 0; \
	g_usb_record_list[g_usb_record_list_i].event_string = a; \
	g_usb_record_list[g_usb_record_list_i].record[0] = b; \
	g_usb_record_list[g_usb_record_list_i].record[1] = c; \
	g_usb_record_list[g_usb_record_list_i].record[2] = d; \
	g_usb_record_list[g_usb_record_list_i].record[3] = e; \
	g_usb_record_list_i++; \
	OS_Unlock(); \
}while(0)

extern volatile uint32_t g_usb_record_list_i;
extern _RECORD_LIST_T g_usb_record_list[USB_RECORD_LIST_MAX];
/******************************************************************************
 * Global Functions
 *****************************************************************************/
extern void _dump_record_list();
#endif
