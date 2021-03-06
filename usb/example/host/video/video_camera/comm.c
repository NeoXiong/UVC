#include "board.h"
#include "fsl_spi_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_dma_driver.h"
#include "comm.h"

video_data_t g_video_data_pool[MAX_VIDEO_DATA_BUF];
uint8_t g_video_data_tx_index;
uint8_t g_video_data_rx_index;

#define GPIO_TEST                  1

#define CMD_RECV_AUDIOVIDEO_STREAM 0xFA
#define CMD_SEND_AUDIO_STREAM      0x05
const uint8_t CMD_NO_DATA        = 0x00;

static dma_state_t   dmaState;

void MySPIslave_DRV_Init(void);

/*
    comm_init

    Note:
        init comm related things
*/
void comm_init(void)
{
    DMA_DRV_Init(&dmaState);

    // init spi
    MySPIslave_DRV_Init();

#if GPIO_TEST
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_PCR0 = PORT_PCR_MUX(1);
    FGPIOB_PDDR |= 0x00000001;
    FGPIOB_PCOR = 0x00000001;
#endif
}

    /* 
    MySPIslave_DRV_DMARecv
    
    Note:
        SPI DMA recv routine, non-blocking call
*/
void MySPIslave_DRV_DMARecv(uint8_t *p_recv_buffer, uint32_t len)
{
    DMA_HAL_SetSourceAddr   (DMA_BASE, 0, SPI_HAL_GetDataRegAddr(SPI1_BASE));
    DMA_HAL_SetDestAddr     (DMA_BASE, 0, (uint32_t)p_recv_buffer);
    DMA_HAL_SetTransferCount(DMA_BASE, 0, len);

    // Enable the DMA peripheral request
    DMA_HAL_SetDmaRequestCmd(DMA_BASE, 0, true);
    // Enable the SPI TX DMA Request
    SPI_HAL_SetRxDmaCmd(SPI1_BASE, true);
}

/* 
    MySPIslave_DRV_DMASend
    
    Note:
        SPI DMA send routine, non-blocking call
*/
void MySPIslave_DRV_DMASend(const uint8_t *p_send_buffer, uint32_t len)
{
    // Per the reference manual, before enabling the SPI transmit DMA request, we first need
    // to read the status register and then write to the SPI data register.  Afterwards, we need
    // to decrement the sendByteCount and perform other driver maintenance functions.
    // Read the SPI Status register
    while (SPI_HAL_IsTxBuffEmptyPending(SPI1_BASE) == 0);
    SPI_HAL_WriteDataLow(SPI1_BASE, *p_send_buffer++);
    if (--len > 0)
    {
        DMA_HAL_SetSourceAddr   (DMA_BASE, 1, (uint32_t)p_send_buffer);
        DMA_HAL_SetDestAddr     (DMA_BASE, 1, SPI_HAL_GetDataRegAddr(SPI1_BASE));
        DMA_HAL_SetTransferCount(DMA_BASE, 1, len);

        // Enable the DMA peripheral request
        DMA_HAL_SetDmaRequestCmd(DMA_BASE, 1, true);
        // Enable the SPI TX DMA Request
        SPI_HAL_SetTxDmaCmd(SPI1_BASE, true);
    }
}

/* 
    MySPIslave_DRV_Init
    
    Note:
        SPI1 slave Init routine with associated DMA channel enabled for TX and RX. 
        First enable interrupt mode to receive CMD byte from Master. 
        Fixed to SPI1 because it can achieve 24MHz. SPI0 is only 12MHz in Max.
        For the timing-efficiency requirement, we didn't use any KSDK Driver code. 
*/
void MySPIslave_DRV_Init(void)
{
    // Enable clock for SPI1
    CLOCK_SYS_EnableSpiClock(1);

    // Reset the SPI module to its default settings including disabling SPI1
    SPI_HAL_Init(SPI1_BASE);

	// Config SPI1 to 8-bit mode, Don't use FIFO, slave mode, idle high, rising edge sample data, msb first, pin normal mode 
	SPI_HAL_Set8or16BitMode(SPI1_BASE, kSpi8BitMode);
	SPI_HAL_SetFifoMode    (SPI1_BASE, false, kSpiTxFifoOneHalfEmpty, kSpiRxFifoOneHalfFull);
    SPI_HAL_SetMasterSlave (SPI1_BASE, kSpiSlave);
    SPI_HAL_SetDataFormat  (SPI1_BASE, kSpiClockPolarity_ActiveLow, kSpiClockPhase_SecondEdge, kSpiMsbFirst);
    SPI_HAL_SetPinMode     (SPI1_BASE, kSpiPinMode_Normal);

    // Config 2 DMA channel for Tx and Rx
    CLOCK_SYS_EnableDmaClock(0);
    CLOCK_SYS_EnableDmamuxClock(0);
   
    INT_SYS_EnableIRQ(DMA0_IRQn);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 0, false);
    DMAMUX_HAL_SetTriggerSource(DMAMUX0_BASE, 0, (uint32_t)kDmaRequestMux0SPI1Rx % (uint32_t)kDmamuxDmaRequestSource);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 0, true);
    
    INT_SYS_EnableIRQ(DMA1_IRQn);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 1, false);
    DMAMUX_HAL_SetTriggerSource(DMAMUX0_BASE, 1, (uint32_t)kDmaRequestMux0SPI1Tx % (uint32_t)kDmamuxDmaRequestSource);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 1, true);

    DMA_HAL_ClearStatus(DMA_BASE, 0);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 0, false);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 0, true);    
    DMA_HAL_ClearStatus(DMA_BASE, 1);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 1, false);
    DMAMUX_HAL_SetChannelCmd(DMAMUX0_BASE, 1, true);

    /* Common configuration, we don't need to modify these configs after each DMA DONE event */
    dma_channel_link_config_t config;
    config.channel1 = 0;
    config.channel2 = 0;
    config.linkType = kDmaChannelLinkDisable;

    DMA_HAL_SetAutoAlignCmd              (DMA_BASE, 0, false);
    DMA_HAL_SetCycleStealCmd             (DMA_BASE, 0, true);
    DMA_HAL_SetAsyncDmaRequestCmd        (DMA_BASE, 0, false);
    DMA_HAL_SetDisableRequestAfterDoneCmd(DMA_BASE, 0, true);
    DMA_HAL_SetChanLink                  (DMA_BASE, 0, &config);
    DMA_HAL_SetIntCmd                    (DMA_BASE, 0, true);
    DMA_HAL_SetSourceModulo              (DMA_BASE, 0, kDmaModuloDisable);
    DMA_HAL_SetDestModulo                (DMA_BASE, 0, kDmaModuloDisable);
    DMA_HAL_SetSourceTransferSize        (DMA_BASE, 0, kDmaTransfersize8bits);
    DMA_HAL_SetDestTransferSize          (DMA_BASE, 0, kDmaTransfersize8bits);
    DMA_HAL_SetSourceIncrementCmd        (DMA_BASE, 0, false);
    DMA_HAL_SetDestIncrementCmd          (DMA_BASE, 0, true);
    
    DMA_HAL_SetAutoAlignCmd              (DMA_BASE, 1, false);
    DMA_HAL_SetCycleStealCmd             (DMA_BASE, 1, true);
    DMA_HAL_SetAsyncDmaRequestCmd        (DMA_BASE, 1, false);
    DMA_HAL_SetDisableRequestAfterDoneCmd(DMA_BASE, 1, true);
    DMA_HAL_SetChanLink                  (DMA_BASE, 1, &config);
    DMA_HAL_SetIntCmd                    (DMA_BASE, 1, true);
    DMA_HAL_SetSourceModulo              (DMA_BASE, 1, kDmaModuloDisable);
    DMA_HAL_SetDestModulo                (DMA_BASE, 1, kDmaModuloDisable);
    DMA_HAL_SetSourceTransferSize        (DMA_BASE, 1, kDmaTransfersize8bits);
    DMA_HAL_SetDestTransferSize          (DMA_BASE, 1, kDmaTransfersize8bits);
    DMA_HAL_SetSourceIncrementCmd        (DMA_BASE, 1, true);
    DMA_HAL_SetDestIncrementCmd          (DMA_BASE, 1, false);
    
	// SPI module interrupt enable in NVIC 
    INT_SYS_EnableIRQ(SPI1_IRQn);

	// Enable receive interrupts to make the slave can keep up with the master 
	SPI_HAL_SetReceiveAndFaultIntCmd(SPI1_BASE, true);

    // SPI module enable
    SPI_HAL_Enable(SPI1_BASE);
}

__root volatile int dma_done_event  = 0;
__root volatile int interrupt  = 0;
//uint8_t dummybuffer[100];

/* 
    SPI1_IRQHandler

    Note:
        SPI1 interrupt handle, use to respone to single byte command only
*/
void SPI1_IRQHandler(void)
{
    interrupt++;
#if GPIO_TEST
    FGPIOB_PSOR = 0x00000001;
#endif
    static uint16_t s_frame_num = 0;
   
    //static uint8_t i = 0;
    
    if (SPI_HAL_IsReadBuffFullPending(SPI1_BASE))
    {
        uint8_t byte = SPI_HAL_ReadDataLow(SPI1_BASE);
        switch (byte)
        {
        case CMD_RECV_AUDIOVIDEO_STREAM:
            {
                if (g_video_data_pool[g_video_data_tx_index].flag)
                {
                    g_video_data_pool[g_video_data_tx_index].frame = s_frame_num++;
                    MySPIslave_DRV_DMASend((const uint8_t *)&g_video_data_pool[g_video_data_tx_index], 
                                           g_video_data_pool[g_video_data_tx_index].len + 3);

                    SPI_HAL_SetReceiveAndFaultIntCmd(SPI1_BASE, false);
#if GPIO_TEST
                    FGPIOB_PCOR = 0x00000001;
#endif
                }
                else
                {
                    MySPIslave_DRV_DMASend(&CMD_NO_DATA, 1);
                }
            }
            break;
            
        case CMD_SEND_AUDIO_STREAM:
            {
                //MySPIslave_DRV_DMARecv()
                SPI_HAL_SetReceiveAndFaultIntCmd(SPI1_BASE, false);
            }
            break;
            
        default:
            //dummybuffer[i++] = byte;
            // !! We may get dummy byte from host after re-enable interrupt in DMA routine
            break;
        }
    }
#if GPIO_TEST
    FGPIOB_PCOR = 0x00000001;
#endif    
}

/* 
    DMA0_IRQHandler

    Note:
        re-enable receive interrupt in this routine to receive further single byte command from host
*/
void DMA0_IRQHandler(void)
{
    DMA_HAL_ClearStatus(DMA_BASE, 0);
    
    /* Disable DMA requests and interrupts. */
    SPI_HAL_SetRxDmaCmd(SPI1_BASE, false);

	// Stop DMA to recevei further request from peripheral
	DMA_HAL_SetDmaRequestCmd(DMA_BASE, 0, false);

	// Re-enable interrupts to receive CMD
	SPI_HAL_SetReceiveAndFaultIntCmd(SPI1_BASE, true);
}

/* 
    DMA1_IRQHandler

    Note:
        re-enable receive interrupt in this routine to receive further single byte command from host
*/
void DMA1_IRQHandler(void)
{
    dma_done_event++;
    
    uint8_t dummy;
    
    // write DONE to clear interrupt and error bits
    DMA_HAL_ClearStatus(DMA_BASE, 1);

	/// Disable DMA requests and interrupts
	SPI_HAL_SetTxDmaCmd(SPI1_BASE, false);

	// Stop DMA to recevei further request from peripheral
	DMA_HAL_SetDmaRequestCmd(DMA_BASE, 1, false);
    
	// Re-enable interrupts to receive CMD, will receive 1-2 bytes dummy data from Host after re-enable interrupt as SPRF is set
    // must make sure dummy byte from host is a special pattern and will not conflict to current CMD byte 
    /*while (SPI_HAL_IsReadBuffFullPending(SPI1_BASE))
    {
        dummy = SPI_HAL_ReadDataLow(SPI1_BASE);
    }*/
    
	SPI_HAL_SetReceiveAndFaultIntCmd(SPI1_BASE, true);	

	g_video_data_pool[g_video_data_tx_index].flag = 0;
	if (++g_video_data_tx_index >= MAX_VIDEO_DATA_BUF)
	{
		g_video_data_tx_index = 0;
	}
}
