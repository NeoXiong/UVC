/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__FSL_DMA_REQUEST_H__)
#define __FSL_DMA_REQUEST_H__

/*!
 * @addtogroup edma_request
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Structure for the DMA hardware request
 *
 * Defines the structure for the DMA hardware request collections. The user can configure the
 * hardware request in DMAMUX to trigger the DMA transfer accordingly. The index
 * of the hardware request varies according  to the to SoC.
 */

typedef enum _dma_request_source {
#if defined(CPU_MKL16Z256VLH4) || defined(CPU_MKL16Z256VMP4) || defined(CPU_MKL26Z256VLH4) || defined(CPU_MKL26Z128VLL4) || \
    defined(CPU_MKL26Z256VLL4) || defined(CPU_MKL26Z128VMC4) || defined(CPU_MKL26Z256VMC4) || defined(CPU_MKL26Z256VMP4) || \
    defined(CPU_MKL36Z64VLH4) || defined(CPU_MKL36Z128VLH4) || defined(CPU_MKL36Z256VLH4) || defined(CPU_MKL36Z64VLL4) || \
    defined(CPU_MKL36Z128VLL4) || defined(CPU_MKL36Z256VLL4) || defined(CPU_MKL36Z128VMC4) || defined(CPU_MKL36Z256VMC4) || \
    defined(CPU_MKL36Z256VMP4) || defined(CPU_MKL46Z128VLH4) || defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z128VLL4) || \
    defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z128VMC4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    kDmaRequestMux0Disable = 0|0x100U, /*!< Disable */
    kDmaRequestMux0Reserved1 = 1|0x100U, /*!< Reserved1 */
    kDmaRequestMux0UART0Rx = 2|0x100U, /*!< UART0 Rx */
    kDmaRequestMux0UART0Tx = 3|0x100U, /*!< UART0 Tx */
    kDmaRequestMux0UART1Rx = 4|0x100U, /*!< UART1 Rx */
    kDmaRequestMux0UART1Tx = 5|0x100U, /*!< UART1 Tx */
    kDmaRequestMux0UART2Rx = 6|0x100U, /*!< UART2 Rx */
    kDmaRequestMux0UART2Tx = 7|0x100U, /*!< UART2 Tx */
    kDmaRequestMux0Reserved8 = 8|0x100U, /*!< Reserved8 */
    kDmaRequestMux0Reserved9 = 9|0x100U, /*!< Reserved9 */
    kDmaRequestMux0Reserved10 = 10|0x100U, /*!< Reserved10 */
    kDmaRequestMux0Reserved11 = 11|0x100U, /*!< Reserved11 */
    kDmaRequestMux0Reserved12 = 12|0x100U, /*!< Reserved12 */
    kDmaRequestMux0Reserved13 = 13|0x100U, /*!< Reserved13 */
    kDmaRequestMux0I2S0Rx = 14|0x100U, /*!< I2S0 Rx */
    kDmaRequestMux0I2S0Tx = 15|0x100U, /*!< I2S0 Tx */
    kDmaRequestMux0SPI0Rx = 16|0x100U, /*!< SPI0 Rx */
    kDmaRequestMux0SPI0Tx = 17|0x100U, /*!< SPI0 Tx */
    kDmaRequestMux0SPI1Rx = 18|0x100U, /*!< SPI1 Rx */
    kDmaRequestMux0SPI1Tx = 19|0x100U, /*!< SPI1 Tx */
    kDmaRequestMux0Reserved20 = 20|0x100U, /*!< Reserved20 */
    kDmaRequestMux0Reserved21 = 21|0x100U, /*!< Reserved21 */
    kDmaRequestMux0I2C0 = 22|0x100U, /*!< I2C0 */
    kDmaRequestMux0I2C1 = 23|0x100U, /*!< I2C1 */
    kDmaRequestMux0TPM0Channel0 = 24|0x100U, /*!< TPM0 Channel0 */
    kDmaRequestMux0TPM0Channel1 = 25|0x100U, /*!< TPM0 Channel1 */
    kDmaRequestMux0TPM0Channel2 = 26|0x100U, /*!< TPM0 Channel2 */
    kDmaRequestMux0TPM0Channel3 = 27|0x100U, /*!< TPM0 Channel3 */
    kDmaRequestMux0TPM0Channel4 = 28|0x100U, /*!< TPM0 Channel4 */
    kDmaRequestMux0TPM0Channel5 = 29|0x100U, /*!< TPM0 Channel5 */
    kDmaRequestMux0Reserved30 = 30|0x100U, /*!< Reserved30 */
    kDmaRequestMux0Reserved31 = 31|0x100U, /*!< Reserved31 */
    kDmaRequestMux0TPM1Channel0 = 32|0x100U, /*!< TPM1 Channel0 */
    kDmaRequestMux0TPM1Channel1 = 33|0x100U, /*!< TPM1 Channel1 */
    kDmaRequestMux0TPM2Channel0 = 34|0x100U, /*!< TPM2 Channel0 */
    kDmaRequestMux0TPM2Channel1 = 35|0x100U, /*!< TPM2 Channel1 */
    kDmaRequestMux0Reserved36 = 36|0x100U, /*!< Reserved36 */
    kDmaRequestMux0Reserved37 = 37|0x100U, /*!< Reserved37 */
    kDmaRequestMux0Reserved38 = 38|0x100U, /*!< Reserved38 */
    kDmaRequestMux0Reserved39 = 39|0x100U, /*!< Reserved39 */
    kDmaRequestMux0ADC0 = 40|0x100U, /*!< ADC0 */
    kDmaRequestMux0Reserved41 = 41|0x100U, /*!< Reserved41 */
    kDmaRequestMux0CMP0 = 42|0x100U, /*!< CMP0 */
    kDmaRequestMux0Reserved43 = 43|0x100U, /*!< Reserved43 */
    kDmaRequestMux0Reserved44 = 44|0x100U, /*!< Reserved44 */
    kDmaRequestMux0DAC0 = 45|0x100U, /*!< DAC0 */
    kDmaRequestMux0Reserved46 = 46|0x100U, /*!< Reserved46 */
    kDmaRequestMux0Reserved47 = 47|0x100U, /*!< Reserved47 */
    kDmaRequestMux0Reserved48 = 48|0x100U, /*!< Reserved48 */
    kDmaRequestMux0PortA = 49|0x100U, /*!< PortA */
    kDmaRequestMux0Reserved50 = 50|0x100U, /*!< Reserved50 */
    kDmaRequestMux0PortC = 51|0x100U, /*!< PortC */
    kDmaRequestMux0PortD = 52|0x100U, /*!< PortD */
    kDmaRequestMux0Reserved53 = 53|0x100U, /*!< Reserved53 */
    kDmaRequestMux0TPM0Overflow = 54|0x100U, /*!< TPM0 Overflow */
    kDmaRequestMux0TPM1Overflow = 55|0x100U, /*!< TPM1 Overflow */
    kDmaRequestMux0TPM2Overflow = 56|0x100U, /*!< TPM2 Overflow */
    kDmaRequestMux0TSI = 57|0x100U, /*!< TSI */
    kDmaRequestMux0Reserved58 = 58|0x100U, /*!< Reserved58 */
    kDmaRequestMux0Reserved59 = 59|0x100U, /*!< Reserved59 */
    kDmaRequestMux0AlwaysOn60 = 60|0x100U, /*!< AlwaysOn60 */
    kDmaRequestMux0AlwaysOn61 = 61|0x100U, /*!< AlwaysOn61 */
    kDmaRequestMux0AlwaysOn62 = 62|0x100U, /*!< AlwaysOn62 */
    kDmaRequestMux0AlwaysOn63 = 63|0x100U, /*!< AlwaysOn63 */
#elif defined(CPU_MKL17Z128VFM4) || defined(CPU_MKL17Z256VFM4) || defined(CPU_MKL17Z128VFT4) || defined(CPU_MKL17Z256VFT4) || \
    defined(CPU_MKL17Z128VLH4) || defined(CPU_MKL17Z256VLH4) || defined(CPU_MKL17Z128VMP4) || defined(CPU_MKL17Z256VMP4) || \
    defined(CPU_MKL27Z128VFM4) || defined(CPU_MKL27Z256VFM4) || defined(CPU_MKL27Z128VFT4) || defined(CPU_MKL27Z256VFT4) || \
    defined(CPU_MKL27Z128VLH4) || defined(CPU_MKL27Z256VLH4) || defined(CPU_MKL27Z128VMP4) || defined(CPU_MKL27Z256VMP4) || \
    defined(CPU_MKL33Z128VLH4) || defined(CPU_MKL33Z256VLH4) || defined(CPU_MKL33Z128VMP4) || defined(CPU_MKL33Z256VMP4) || \
    defined(CPU_MKL43Z128VLH4) || defined(CPU_MKL43Z256VLH4) || defined(CPU_MKL43Z128VMP4) || defined(CPU_MKL43Z256VMP4)
    kDmaRequestMux0Disable = 0|0x100U, /*!< Disable */
    kDmaRequestMux0Reserved1 = 1|0x100U, /*!< Reserved1 */
    kDmaRequestMux0LPUART0Rx = 2|0x100U, /*!< LPUART0 Rx */
    kDmaRequestMux0LPUART0Tx = 3|0x100U, /*!< LPUART0 Tx */
    kDmaRequestMux0LPUART1Rx = 4|0x100U, /*!< LPUART1 Rx */
    kDmaRequestMux0LPUART1Tx = 5|0x100U, /*!< LPUART1 Tx */
    kDmaRequestMux0UART2Rx = 6|0x100U, /*!< UART2 Rx */
    kDmaRequestMux0UART2Tx = 7|0x100U, /*!< UART2 Tx */
    kDmaRequestMux0Reserved8 = 8|0x100U, /*!< Reserved8 */
    kDmaRequestMux0Reserved9 = 9|0x100U, /*!< Reserved9 */
    kDmaRequestMux0FlexIOChannel0 = 10|0x100U, /*!< FlexIO Channel0 */
    kDmaRequestMux0FlexIOChannel1 = 11|0x100U, /*!< FlexIO Channel1 */
    kDmaRequestMux0FlexIOChannel2 = 12|0x100U, /*!< FlexIO Channel2 */
    kDmaRequestMux0FlexIOChannel3 = 13|0x100U, /*!< FlexIO Channel3 */
    kDmaRequestMux0I2S0Rx = 14|0x100U, /*!< I2S0 Rx */
    kDmaRequestMux0I2S0Tx = 15|0x100U, /*!< I2S0 Tx */
    kDmaRequestMux0SPI0Rx = 16|0x100U, /*!< SPI0 Rx */
    kDmaRequestMux0SPI0Tx = 17|0x100U, /*!< SPI0 Tx */
    kDmaRequestMux0SPI1Rx = 18|0x100U, /*!< SPI1 Rx */
    kDmaRequestMux0SPI1Tx = 19|0x100U, /*!< SPI1 Tx */
    kDmaRequestMux0Reserved20 = 20|0x100U, /*!< Reserved20 */
    kDmaRequestMux0Reserved21 = 21|0x100U, /*!< Reserved21 */
    kDmaRequestMux0I2C0 = 22|0x100U, /*!< I2C0 */
    kDmaRequestMux0I2C1 = 23|0x100U, /*!< I2C1 */
    kDmaRequestMux0TPM0Channel0 = 24|0x100U, /*!< TPM0 Channel0 */
    kDmaRequestMux0TPM0Channel1 = 25|0x100U, /*!< TPM0 Channel1 */
    kDmaRequestMux0TPM0Channel2 = 26|0x100U, /*!< TPM0 Channel2 */
    kDmaRequestMux0TPM0Channel3 = 27|0x100U, /*!< TPM0 Channel3 */
    kDmaRequestMux0TPM0Channel4 = 28|0x100U, /*!< TPM0 Channel4 */
    kDmaRequestMux0TPM0Channel5 = 29|0x100U, /*!< TPM0 Channel5 */
    kDmaRequestMux0Reserved30 = 30|0x100U, /*!< Reserved30 */
    kDmaRequestMux0Reserved31 = 31|0x100U, /*!< Reserved31 */
    kDmaRequestMux0TPM1Channel0 = 32|0x100U, /*!< TPM1 Channel0 */
    kDmaRequestMux0TPM1Channel1 = 33|0x100U, /*!< TPM1 Channel1 */
    kDmaRequestMux0TPM2Channel0 = 34|0x100U, /*!< TPM2 Channel0 */
    kDmaRequestMux0TPM2Channel1 = 35|0x100U, /*!< TPM2 Channel1 */
    kDmaRequestMux0Reserved36 = 36|0x100U, /*!< Reserved36 */
    kDmaRequestMux0Reserved37 = 37|0x100U, /*!< Reserved37 */
    kDmaRequestMux0Reserved38 = 38|0x100U, /*!< Reserved38 */
    kDmaRequestMux0Reserved39 = 39|0x100U, /*!< Reserved39 */
    kDmaRequestMux0ADC0 = 40|0x100U, /*!< ADC0 */
    kDmaRequestMux0Reserved41 = 41|0x100U, /*!< Reserved41 */
    kDmaRequestMux0CMP0 = 42|0x100U, /*!< CMP0 */
    kDmaRequestMux0Reserved43 = 43|0x100U, /*!< Reserved43 */
    kDmaRequestMux0Reserved44 = 44|0x100U, /*!< Reserved44 */
    kDmaRequestMux0DAC0 = 45|0x100U, /*!< DAC0 */
    kDmaRequestMux0Reserved46 = 46|0x100U, /*!< Reserved46 */
    kDmaRequestMux0Reserved47 = 47|0x100U, /*!< Reserved47 */
    kDmaRequestMux0Reserved48 = 48|0x100U, /*!< Reserved48 */
    kDmaRequestMux0PortA = 49|0x100U, /*!< Port A */
    kDmaRequestMux0Reserved50 = 50|0x100U, /*!< Reserved50 */
    kDmaRequestMux0PortC = 51|0x100U, /*!< Port C */
    kDmaRequestMux0PortD = 52|0x100U, /*!< Port D */
    kDmaRequestMux0Reserved53 = 53|0x100U, /*!< Reserved53 */
    kDmaRequestMux0TPM0Overflow = 54|0x100U, /*!< TPM0 Overflow */
    kDmaRequestMux0TPM1Overflow = 55|0x100U, /*!< TPM1 Overflow */
    kDmaRequestMux0TPM2Overflow = 56|0x100U, /*!< TPM2 Overflow */
    kDmaRequestMux0Reserved57 = 57|0x100U, /*!< Reserved57 */
    kDmaRequestMux0Reserved58 = 58|0x100U, /*!< Reserved58 */
    kDmaRequestMux0Reserved59 = 59|0x100U, /*!< Reserved59 */
    kDmaRequestMux0AlwaysOn60 = 60|0x100U, /*!< AlwaysOn60 */
    kDmaRequestMux0AlwaysOn61 = 61|0x100U, /*!< AlwaysOn61 */
    kDmaRequestMux0AlwaysOn62 = 62|0x100U, /*!< AlwaysOn62 */
    kDmaRequestMux0AlwaysOn63 = 63|0x100U, /*!< AlwaysOn63 */
#elif defined(CPU_MKL17Z32VDA4) || defined(CPU_MKL17Z64VDA4) || defined(CPU_MKL17Z32VFM4) || defined(CPU_MKL17Z64VFM4) || \
    defined(CPU_MKL17Z32VFT4) || defined(CPU_MKL17Z64VFT4) || defined(CPU_MKL17Z32VLH4) || defined(CPU_MKL17Z64VLH4) || \
    defined(CPU_MKL17Z32VMP4) || defined(CPU_MKL17Z64VMP4) || defined(CPU_MKL27Z32VDA4) || defined(CPU_MKL27Z64VDA4) || \
    defined(CPU_MKL27Z32VFM4) || defined(CPU_MKL27Z64VFM4) || defined(CPU_MKL27Z32VFT4) || defined(CPU_MKL27Z64VFT4) || \
    defined(CPU_MKL27Z32VLH4) || defined(CPU_MKL27Z64VLH4) || defined(CPU_MKL27Z32VMP4) || defined(CPU_MKL27Z64VMP4)
    kDmaRequestMux0Disable = 0|0x100U, /*!< Disable */
    kDmaRequestMux0Reserved1 = 1|0x100U, /*!< Reserved1 */
    kDmaRequestMux0LPUART0Rx = 2|0x100U, /*!< LPUART0 Rx */
    kDmaRequestMux0LPUART0Tx = 3|0x100U, /*!< LPUART0 Tx */
    kDmaRequestMux0LPUART1x = 4|0x100U, /*!< LPUART1 x */
    kDmaRequestMux0LPUART1Tx = 5|0x100U, /*!< LPUART1 Tx */
    kDmaRequestMux0UART2Rx = 6|0x100U, /*!< UART2 Rx */
    kDmaRequestMux0UART2Tx = 7|0x100U, /*!< UART2 Tx */
    kDmaRequestMux0Reserved8 = 8|0x100U, /*!< Reserved8 */
    kDmaRequestMux0Reserved9 = 9|0x100U, /*!< Reserved9 */
    kDmaRequestMux0FlexIOChannel0 = 10|0x100U, /*!< FlexIO Channel0 */
    kDmaRequestMux0FlexIOChannel1 = 11|0x100U, /*!< FlexIO Channel1 */
    kDmaRequestMux0FlexIOChannel2 = 12|0x100U, /*!< FlexIO Channel2 */
    kDmaRequestMux0FlexIOChannel3 = 13|0x100U, /*!< FlexIO Channel3 */
    kDmaRequestMux0Reserved14 = 14|0x100U, /*!< Reserved14 */
    kDmaRequestMux0Reserved15 = 15|0x100U, /*!< Reserved15 */
    kDmaRequestMux0SPI0Rx = 16|0x100U, /*!< SPI0 Rx */
    kDmaRequestMux0SPI0Tx = 17|0x100U, /*!< SPI0 Tx */
    kDmaRequestMux0SPI1Rx = 18|0x100U, /*!< SPI1 Rx */
    kDmaRequestMux0SPI1Tx = 19|0x100U, /*!< SPI1 Tx */
    kDmaRequestMux0Reserved20 = 20|0x100U, /*!< Reserved20 */
    kDmaRequestMux0Reserved21 = 21|0x100U, /*!< Reserved21 */
    kDmaRequestMux0I2C0 = 22|0x100U, /*!< I2C0 */
    kDmaRequestMux0I2C1 = 23|0x100U, /*!< I2C1 */
    kDmaRequestMux0TPM0Channel0 = 24|0x100U, /*!< TPM0 Channel0 */
    kDmaRequestMux0TPM0Channel1 = 25|0x100U, /*!< TPM0 Channel1 */
    kDmaRequestMux0TPM0Channel2 = 26|0x100U, /*!< TPM0 Channel2 */
    kDmaRequestMux0TPM0Channel3 = 27|0x100U, /*!< TPM0 Channel3 */
    kDmaRequestMux0TPM0Channel4 = 28|0x100U, /*!< TPM0 Channel4 */
    kDmaRequestMux0TPM0Channel5 = 29|0x100U, /*!< TPM0 Channel5 */
    kDmaRequestMux0Reserved30 = 30|0x100U, /*!< Reserved30 */
    kDmaRequestMux0Reserved31 = 31|0x100U, /*!< Reserved31 */
    kDmaRequestMux0TPM1Channel0 = 32|0x100U, /*!< TPM1 Channel0 */
    kDmaRequestMux0TPM1Channel1 = 33|0x100U, /*!< TPM1 Channel1 */
    kDmaRequestMux0TPM2Channel0 = 34|0x100U, /*!< TPM2 Channel0 */
    kDmaRequestMux0TPM2Channel1 = 35|0x100U, /*!< TPM2 Channel1 */
    kDmaRequestMux0Reserved36 = 36|0x100U, /*!< Reserved36 */
    kDmaRequestMux0Reserved37 = 37|0x100U, /*!< Reserved37 */
    kDmaRequestMux0Reserved38 = 38|0x100U, /*!< Reserved38 */
    kDmaRequestMux0Reserved39 = 39|0x100U, /*!< Reserved39 */
    kDmaRequestMux0ADC0 = 40|0x100U, /*!< ADC0 */
    kDmaRequestMux0Reserved41 = 41|0x100U, /*!< Reserved41 */
    kDmaRequestMux0CMP0 = 42|0x100U, /*!< CMP0 */
    kDmaRequestMux0Reserved43 = 43|0x100U, /*!< Reserved43 */
    kDmaRequestMux0Reserved44 = 44|0x100U, /*!< Reserved44 */
    kDmaRequestMux0Reserved45 = 45|0x100U, /*!< Reserved45 */
    kDmaRequestMux0Reserved46 = 46|0x100U, /*!< Reserved46 */
    kDmaRequestMux0Reserved47 = 47|0x100U, /*!< Reserved47 */
    kDmaRequestMux0Reserved48 = 48|0x100U, /*!< Reserved48 */
    kDmaRequestMux0GPIOPortA = 49|0x100U, /*!< GPIO Port A */
    kDmaRequestMux0Reserved50 = 50|0x100U, /*!< Reserved50 */
    kDmaRequestMux0GPIOPortC = 51|0x100U, /*!< GPIO Port C */
    kDmaRequestMux0GPIOPortD = 52|0x100U, /*!< GPIO Port D */
    kDmaRequestMux0Reserved53 = 53|0x100U, /*!< Reserved53 */
    kDmaRequestMux0TPM0Overflow = 54|0x100U, /*!< TPM0 Overflow */
    kDmaRequestMux0TPM1Overflow = 55|0x100U, /*!< TPM1 Overflow */
    kDmaRequestMux0TPM2Overflow = 56|0x100U, /*!< TPM2 Overflow */
    kDmaRequestMux0Reserved57 = 57|0x100U, /*!< Reserved57 */
    kDmaRequestMux0Reserved58 = 58|0x100U, /*!< Reserved58 */
    kDmaRequestMux0Reserved59 = 59|0x100U, /*!< Reserved59 */
    kDmaRequestMux0AlwaysOn60 = 60|0x100U, /*!< AlwaysOn60 */
    kDmaRequestMux0AlwaysOn61 = 61|0x100U, /*!< AlwaysOn61 */
    kDmaRequestMux0AlwaysOn62 = 62|0x100U, /*!< AlwaysOn62 */
    kDmaRequestMux0AlwaysOn63 = 63|0x100U, /*!< AlwaysOn63 */
#elif defined(CPU_MKL34Z64VLH4) || defined(CPU_MKL34Z64VLL4)
    kDmaRequestMux0Disable = 0|0x100U, /*!< Disable */
    kDmaRequestMux0Reserved1 = 1|0x100U, /*!< Reserved1 */
    kDmaRequestMux0UART0Rx = 2|0x100U, /*!< UART0 Rx */
    kDmaRequestMux0UART0Tx = 3|0x100U, /*!< UART0 Tx */
    kDmaRequestMux0UART1Rx = 4|0x100U, /*!< UART1 Rx */
    kDmaRequestMux0UART1Tx = 5|0x100U, /*!< UART1 Tx */
    kDmaRequestMux0UART2Rx = 6|0x100U, /*!< UART2 Rx */
    kDmaRequestMux0UART2Tx = 7|0x100U, /*!< UART2 Tx */
    kDmaRequestMux0Reserved8 = 8|0x100U, /*!< Reserved8 */
    kDmaRequestMux0Reserved9 = 9|0x100U, /*!< Reserved9 */
    kDmaRequestMux0Reserved10 = 10|0x100U, /*!< Reserved10 */
    kDmaRequestMux0Reserved11 = 11|0x100U, /*!< Reserved11 */
    kDmaRequestMux0Reserved12 = 12|0x100U, /*!< Reserved12 */
    kDmaRequestMux0Reserved13 = 13|0x100U, /*!< Reserved13 */
    kDmaRequestMux0Reserved14 = 14|0x100U, /*!< Reserved14 */
    kDmaRequestMux0Reserved15 = 15|0x100U, /*!< Reserved15 */
    kDmaRequestMux0SPI0Rx = 16|0x100U, /*!< SPI0 Rx */
    kDmaRequestMux0SPI0Tx = 17|0x100U, /*!< SPI0 Tx */
    kDmaRequestMux0SPI1Rx = 18|0x100U, /*!< SPI1 Rx */
    kDmaRequestMux0SPI1Tx = 19|0x100U, /*!< SPI1 Tx */
    kDmaRequestMux0Reserved20 = 20|0x100U, /*!< Reserved20 */
    kDmaRequestMux0Reserved21 = 21|0x100U, /*!< Reserved21 */
    kDmaRequestMux0I2C0 = 22|0x100U, /*!< I2C0 */
    kDmaRequestMux0I2C1 = 23|0x100U, /*!< I2C1 */
    kDmaRequestMux0TPM0Channel0 = 24|0x100U, /*!< TPM0 Channel0 */
    kDmaRequestMux0TPM0Channel1 = 25|0x100U, /*!< TPM0 Channel1 */
    kDmaRequestMux0TPM0Channel2 = 26|0x100U, /*!< TPM0 Channel2 */
    kDmaRequestMux0TPM0Channel3 = 27|0x100U, /*!< TPM0 Channel3 */
    kDmaRequestMux0TPM0Channel4 = 28|0x100U, /*!< TPM0 Channel4 */
    kDmaRequestMux0TPM0Channel5 = 29|0x100U, /*!< TPM0 Channel5 */
    kDmaRequestMux0Reserved30 = 30|0x100U, /*!< Reserved30 */
    kDmaRequestMux0Reserved31 = 31|0x100U, /*!< Reserved31 */
    kDmaRequestMux0TPM1Channel0 = 32|0x100U, /*!< TPM1 Channel0 */
    kDmaRequestMux0TPM1Channel1 = 33|0x100U, /*!< TPM1 Channel1 */
    kDmaRequestMux0TPM2Channel0 = 34|0x100U, /*!< TPM2 Channel0 */
    kDmaRequestMux0TPM2Channel1 = 35|0x100U, /*!< TPM2 Channel1 */
    kDmaRequestMux0Reserved36 = 36|0x100U, /*!< Reserved36 */
    kDmaRequestMux0Reserved37 = 37|0x100U, /*!< Reserved37 */
    kDmaRequestMux0Reserved38 = 38|0x100U, /*!< Reserved38 */
    kDmaRequestMux0Reserved39 = 39|0x100U, /*!< Reserved39 */
    kDmaRequestMux0ADC0 = 40|0x100U, /*!< ADC0 */
    kDmaRequestMux0Reserved41 = 41|0x100U, /*!< Reserved41 */
    kDmaRequestMux0CMP0 = 42|0x100U, /*!< CMP0 */
    kDmaRequestMux0Reserved43 = 43|0x100U, /*!< Reserved43 */
    kDmaRequestMux0Reserved44 = 44|0x100U, /*!< Reserved44 */
    kDmaRequestMux0Reserved45 = 45|0x100U, /*!< Reserved45 */
    kDmaRequestMux0Reserved46 = 46|0x100U, /*!< Reserved46 */
    kDmaRequestMux0Reserved47 = 47|0x100U, /*!< Reserved47 */
    kDmaRequestMux0Reserved48 = 48|0x100U, /*!< Reserved48 */
    kDmaRequestMux0PortA = 49|0x100U, /*!< PortA */
    kDmaRequestMux0Reserved50 = 50|0x100U, /*!< Reserved50 */
    kDmaRequestMux0PortC = 51|0x100U, /*!< PortC */
    kDmaRequestMux0PortD = 52|0x100U, /*!< PortD */
    kDmaRequestMux0Reserved53 = 53|0x100U, /*!< Reserved53 */
    kDmaRequestMux0TPM0Overflow = 54|0x100U, /*!< TPM0 Overflow */
    kDmaRequestMux0TPM1Overflow = 55|0x100U, /*!< TPM1 Overflow */
    kDmaRequestMux0TPM2Overflow = 56|0x100U, /*!< TPM2 Overflow */
    kDmaRequestMux0Reserved57 = 57|0x100U, /*!< Reserved57 */
    kDmaRequestMux0Reserved58 = 58|0x100U, /*!< Reserved58 */
    kDmaRequestMux0Reserved59 = 59|0x100U, /*!< Reserved59 */
    kDmaRequestMux0AlwaysOn60 = 60|0x100U, /*!< AlwaysOn60 */
    kDmaRequestMux0AlwaysOn61 = 61|0x100U, /*!< AlwaysOn61 */
    kDmaRequestMux0AlwaysOn62 = 62|0x100U, /*!< AlwaysOn62 */
    kDmaRequestMux0AlwaysOn63 = 63|0x100U, /*!< AlwaysOn63 */
#else
    #error "No valid CPU defined!"
#endif
} dma_request_source_t;

/* @} */

#endif /* __FSL_DMA_REQUEST_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/

