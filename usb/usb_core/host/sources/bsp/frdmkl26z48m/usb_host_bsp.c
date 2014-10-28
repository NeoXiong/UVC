/**HEADER********************************************************************
* 
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
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
* Comments:  
*
*END************************************************************************/
#include "adapter.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#elif ((OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)||(OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX))
    #include "MKL26Z4.h"	
    #include "MKL26Z4/MKL26Z4_sim.h"
    #include "MKL26Z4/MKL26Z4_usb.h"
#endif

extern uint8_t soc_get_usb_vector_number(uint8_t);
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#define BSP_USB_INT_LEVEL                (1)


static int32_t bsp_usb_host_io_init
(
    int32_t i
)
{
    if (i == 0)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
        HW_SIM_SCGC4_SET(SIM_SCGC4_USBOTG_MASK);
        HW_SIM_SOPT2_SET(SIM_SOPT2_USBSRC_MASK);
        BW_SIM_SOPT2_PLLFLLSEL(1);
#endif
    }
    else
    {
        return -1; //unknow controller
    }

    return 0;
    
}


int32_t bsp_usb_host_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_host_io_init(controller_id);
    if (result != 0)
    {
        return result;
    }
    if (0 == controller_id)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
        /* Do not configure enable USB regulator for host */
        HW_SIM_SOPT1CFG_SET(SIM_SOPT1_USBREGEN_MASK);
        HW_SIM_SOPT1_SET(SIM_SOPT1_USBREGEN_MASK);
        /* reset USB CTRL register */
        HW_USB_USBCTRL_WR(0);
        /* setup interrupt */
        OS_intr_init(soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
#endif
    }
    else
    {
        /* unknown controller */
        result = -1;
    }

    return result;
}
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#define BSP_USB_INT_LEVEL                (4)

static int32_t bsp_usb_host_io_init
(
   int32_t i
)
{
	if ( i == 0)
	{
    	/* PLL/FLL selected as CLK source */
        CLOCK_SYS_SetUsbfsSrc(i, kClockUsbfsSrcPllFllSel);
        CLOCK_SYS_SetPllfllSel(kClockPllFllSelPll);
        /* USB Clock Gating */
        CLOCK_SYS_EnableUsbfsClock(i);
		
        /* Enable clock gating to all ports, A, B, C, D, E*/
        CLOCK_SYS_EnablePortClock(0);
        CLOCK_SYS_EnablePortClock(1);
        CLOCK_SYS_EnablePortClock(2);
        CLOCK_SYS_EnablePortClock(3);
        CLOCK_SYS_EnablePortClock(4);


        /* Weak pull downs */
        HW_USB_USBCTRL_WR(USB0_BASE, 0x40);

    }
    else
    {
        return -1; //unknow controller
    }

    return 0;
}

int32_t bsp_usb_host_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_host_io_init(controller_id);
    if (result != 0)
        return result;

    /* MPU is disabled. All accesses from all bus masters are allowed */
	//	MPU_CESR=0;
    if (0 == controller_id)
    {
        /* Do not configure enable USB regulator for host */
        // SIM_SOPT1CFG_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
        // SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;

        /* reset USB CTRL register */
		HW_USB_USBCTRL_WR(USB0_BASE, 0);

        /* setup interrupt */
        OS_intr_init((IRQn_Type)soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
    }
    else
    {
        /* unknown controller */
        result = -1;
    }

    return result;
}

#endif
/* EOF */
