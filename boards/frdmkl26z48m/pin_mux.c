/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : pin_mux.c
**     Project     : twrkl25z48m
**     Processor   : MKL25Z48M
**     Component   : PinSettings
**     Version     : Component 01.002, Driver 1.1, CPU db: 3.00.000
**     Compiler    : IAR ARM C Compiler
**     Date/Time   : 2013-12-10, 14:58, # CodeGen: 7
**     Abstract    :
**
**     Settings    :
**
**     Contents    :
**         GPIO                - void pin_mux_GPIO(uint32_t instance);
**         I2C                 - void pin_mux_I2C(uint32_t instance);
**         RTC                 - void pin_mux_RTC(uint32_t instance);
**         SPI                 - void pin_mux_SPI(uint32_t instance);
**         UART                - void pin_mux_UART(uint32_t instance);
**
**     Copyright : 1997 - 2013 Freescale Semiconductor, Inc. All Rights Reserved.
**     SOURCE DISTRIBUTION PERMISSIBLE as directed in End User License Agreement.
**
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/
/*!
** @file pin_mux.c
** @version 1.1
** @brief
**
*/
/*!
**  @addtogroup pin_mux_module pin_mux module documentation
**  @{
*/

/* MODULE pin_mux. */

#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "pin_mux.h"


void configure_gpio_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* PTA */
      break;
    case 1:                             /* PTB */
      break;
    case 2:                             /* PTC */
      /* PORTC_PCR3    LLWU_P7*/
      PORT_HAL_SetMuxMode(PORTC_BASE,3u,kPortMuxAsGpio);
      /* PORTC_PCR5 */
      PORT_HAL_SetMuxMode(PORTC_BASE,5u,kPortMuxAsGpio);
      /* PORTC_PCR12 */
      PORT_HAL_SetMuxMode(PORTC_BASE,12u,kPortMuxAsGpio);
      break;
    case 3:                             /* PTD */
      /* PORTD_PCR1 */
      PORT_HAL_SetMuxMode(PORTD_BASE,1u,kPortMuxAsGpio);
	  /* PORTD_PCR5 */
      PORT_HAL_SetMuxMode(PORTD_BASE,5u,kPortMuxAsGpio);
      break;
    case 4:                             /* PTE */
	  /* PORTE_PCR29 */
      PORT_HAL_SetMuxMode(PORTE_BASE,29u,kPortMuxAsGpio);
      break;
    default:
      break;
  }
}

void configure_i2c_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                          /* I2C0 */
      /* PORTB_PCR2 */
      PORT_HAL_SetMuxMode(PORTB_BASE,2u,kPortMuxAlt2);
      PORT_HAL_SetPullCmd(PORTB_BASE, 2u, true);
      PORT_HAL_SetPullMode(PORTB_BASE, 2u, kPortPullUp);
      PORT_HAL_SetPassiveFilterCmd(PORTB_BASE, 2u, false);
      /* PORTB_PCR3 */
      PORT_HAL_SetMuxMode(PORTB_BASE,3u,kPortMuxAlt2);
      PORT_HAL_SetPullCmd(PORTB_BASE, 3u, true);
      PORT_HAL_SetPullMode(PORTB_BASE, 3u, kPortPullUp);
      PORT_HAL_SetPassiveFilterCmd(PORTB_BASE, 3u, false);

      break;
    case 1:
      /* PORTE_PCR0 */
      PORT_HAL_SetMuxMode(PORTE_BASE,0u,kPortMuxAlt6);
      PORT_HAL_SetPullCmd(PORTE_BASE, 0u, true);
      PORT_HAL_SetPullMode(PORTE_BASE, 0u, kPortPullUp);
      PORT_HAL_SetPassiveFilterCmd(PORTE_BASE, 0u, false);
      /* PORTE_PCR1 */
      PORT_HAL_SetMuxMode(PORTE_BASE,1u,kPortMuxAlt6);
      PORT_HAL_SetPullCmd(PORTE_BASE, 1u, true);
      PORT_HAL_SetPullMode(PORTE_BASE, 1u, kPortPullUp);
      PORT_HAL_SetPassiveFilterCmd(PORTE_BASE, 1u, false);
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    default:
      break;
  }
}

void configure_rtc_pins(uint32_t instance)
{
      /* PORTE_PCR26 */
    PORT_HAL_SetMuxMode(PORTE_BASE,26u,kPortMuxAlt6);

}

void configure_lpsci_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* LPSCI0 */
      /* PORTA_PCR1 */
      PORT_HAL_SetMuxMode(PORTA_BASE,1u,kPortMuxAlt2);
      /* PORTA_PCR2 */
      PORT_HAL_SetMuxMode(PORTA_BASE,2u,kPortMuxAlt2);
      break;
    default:
      break;
  }
}

void configure_uart_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* UART0 */
      /* PORTE_PCR0 */
      PORT_HAL_SetMuxMode(PORTE_BASE,0u,kPortMuxAlt3);
      /* PORTE_PCR1 */
      PORT_HAL_SetMuxMode(PORTE_BASE,1u,kPortMuxAlt3);
      break;
    default:
      break;
  }
}

void configure_gpio_i2c_pins(uint32_t instance)
{
  PORT_HAL_SetMuxMode(PORTE_BASE,24u,kPortMuxAsGpio);
  PORT_HAL_SetMuxMode(PORTE_BASE,25u,kPortMuxAsGpio);
}


/* Setup TSI pins for on board electrodes */
void configure_tsi_pins(uint32_t instance) // todo
{
  switch(instance) {
    case 0:                             /* TSI0 */
      /* PORTB_PCR16 */
      PORT_HAL_SetMuxMode(PORTB_BASE,16u,kPortPinDisabled);
      /* PORTB_PCR17 */
      PORT_HAL_SetMuxMode(PORTB_BASE,17u,kPortPinDisabled);
      break;
  default:
      break;
  }
}
void configure_spi_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                          /* SPI0 */
      /* PORTE_PCR19 */
      PORT_HAL_SetMuxMode(PORTC_BASE, 7u,kPortMuxAlt2); /* MISO */
      /* PORTE_PCR18 */
      PORT_HAL_SetMuxMode(PORTC_BASE, 6u,kPortMuxAlt2); /* MOSI */
      /* PORTE_PCR17 */
      PORT_HAL_SetMuxMode(PORTC_BASE, 5u,kPortMuxAlt2); /* SCK */
      /* PORTE_PCR16 */
      PORT_HAL_SetMuxMode(PORTC_BASE, 4u,kPortMuxAlt2); /* PCS0 */
      break;
    case 1:                          /* SPI1 */
      /* PORTD_PCR7 */
      PORT_HAL_SetMuxMode(PORTD_BASE,7u,kPortMuxAlt2); /* MISO */
      /* PORTD_PCR6 */
      PORT_HAL_SetMuxMode(PORTD_BASE,6u,kPortMuxAlt2); /* MOSI */
      /* PORTD_PCR5 */
      PORT_HAL_SetMuxMode(PORTD_BASE,5u,kPortMuxAlt2); /* SCK */
      /* PORTD_PCR4 */
      PORT_HAL_SetMuxMode(PORTD_BASE,4u,kPortMuxAlt2); /* PCS0 */
      break;
    default:
      break;
  }
}

void configure_tpm_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* TPM0 */
        /* PTD_PCR5 TPM0 channel 5 */
        PORT_HAL_SetMuxMode(PORTD_BASE, 5u, kPortMuxAlt4);
      break;
    case 1:                             /* TPM1 */
      break;
    default:
      break;
  }
}

void configure_cmp_pins(uint32_t instance)
{
    switch (instance)
    {
        case 0U:
            PORT_HAL_SetMuxMode(PORTC_BASE, 6U, kPortPinDisabled); /* PTC6 - CMP0_IN0. */
            break;
        default:
            break;
    }
}

void configure_i2s_pins(uint32_t instance)
{
  /* Affects PORTB_PCR18 register */
  PORT_HAL_SetMuxMode(PORTB_BASE,18u,kPortMuxAlt4);
  /* Affects PORTB_PCR19 register */
  PORT_HAL_SetMuxMode(PORTB_BASE,19u,kPortMuxAlt4);
  /* Affects PORTC_PCR0 register */
  PORT_HAL_SetMuxMode(PORTC_BASE,0u,kPortMuxAlt6);
  /* Affects PORTC_PCR4 register */
  PORT_HAL_SetMuxMode(PORTC_BASE,4u,kPortMuxAlt5);
  /* Affects PORTC_PCR6 register */
  PORT_HAL_SetMuxMode(PORTC_BASE,6,kPortMuxAlt4);
  /* Affects PORTC_PCR10 register */
  PORT_HAL_SetMuxMode(PORTC_BASE,10u,kPortMuxAlt4);
  /* Affects PORTC_PCR11 register */
  PORT_HAL_SetMuxMode(PORTC_BASE,11u,kPortMuxAlt4);
}

void configure_dac_pin(uint32_t instance)
{
	PORT_HAL_SetMuxMode(PORTE_BASE, 30u, kPortPinDisabled);
}

/* END pin_mux. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
