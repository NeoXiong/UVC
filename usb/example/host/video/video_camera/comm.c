#include "board.h"
#include "fsl_spi_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_dma_driver.h"
#include "comm.h"

video_data_t g_video_data_pool[MAX_VIDEO_DATA_BUF];
uint8_t g_video_data_tx_index;
uint8_t g_video_data_rx_index;

spi_status_t MySPIslave_DMARecv(uint32_t instance, uint8_t *p_recv_buffer, uint32_t len);

#define CMD_RECV_AUDIOVIDEO_STREAM	0xFA
#define CMD_SEND_AUDIO_STREAM		0x75

static dma_state_t   dmaState;
static dma_channel_t dmaReceive;
static dma_channel_t dmaTransmit;
uint8_t sdata[64];

void MySPIslave_DRV_Sendbyte(uint32_t instance, uint8_t data);
spi_status_t MySPIslave_DRV_Init(uint32_t instance);

void comm_init(void)
{
    /* Init the DMA module */
    (void)DMA_DRV_Init(&dmaState);

	/* Init SPI module */
    (void)MySPIslave_DRV_Init(BOARD_SPI_SLAVE_INSTANCE);
}

/*! @brief Table of base addresses for SPI instances. */
extern const uint32_t g_spiBaseAddr[];
/*! @brief Table to save SPI IRQ enumeration numbers defined in the CMSIS header file. */
extern const IRQn_Type g_spiIrqId[];


static void MySPIslave_DRV_OnDMARecvDone(void *param, dma_channel_status_t chanStatus)
{
	uint32_t instance = (uint32_t)(param);
    uint32_t baseAddr = g_spiBaseAddr[instance];
    uint8_t  dummy;
    
    /* Disable DMA requests and interrupts. */
    SPI_HAL_SetRxDmaCmd(baseAddr, false);

    // Stop DMA channels
    DMA_DRV_StopChannel(&dmaReceive);
    
    // Perform a dummy read to clear SPRF flag 
    if (SPI_HAL_IsReadBuffFullPending(baseAddr))
    {
		dummy = SPI_HAL_ReadDataLow(baseAddr);
    }

	// Re-enable interrupts to receive CMD
	SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, true);
}
__root int transfered = 0;
static void MySPIslave_DRV_OnDMASendDone(void *param, dma_channel_status_t chanStatus)
{
	uint32_t instance = (uint32_t)(param);
	uint32_t baseAddr = g_spiBaseAddr[instance];
    uint8_t  dummy;

	/* Disable DMA requests and interrupts. */
	SPI_HAL_SetTxDmaCmd(baseAddr, false);

	// Stop DMA channels
	DMA_DRV_StopChannel(&dmaTransmit);
    
    // Perform a dummy read to clear SPRF flag 
    if (SPI_HAL_IsReadBuffFullPending(baseAddr))
    {
		dummy = SPI_HAL_ReadDataLow(baseAddr);
    }

	// Re-enable interrupts to receive CMD
	SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, true);	

    transfered++;
    
	g_video_data_pool[g_video_data_tx_index].flag = 0;
	if (++g_video_data_tx_index >= MAX_VIDEO_DATA_BUF)
	{
		g_video_data_tx_index = 0;
	}
}


spi_status_t MySPIslave_DRV_DMARecv(uint32_t instance, uint8_t *p_recv_buffer, uint32_t len)
{
	if ((instance >= HW_SPI_INSTANCE_COUNT) || (p_recv_buffer == NULL) || (len == 0))
	{
		return kStatus_SPI_InvalidParameter;
	}

	uint32_t baseAddr = g_spiBaseAddr[instance];

    uint8_t rxChannel = dmaReceive.channel;
    uint32_t dmaBaseAddr = g_dmaRegBaseAddr[rxChannel / FSL_FEATURE_DMA_DMAMUX_CHANNELS];
    uint32_t dmamuxBaseAddr = g_dmamuxRegBaseAddr[rxChannel / FSL_FEATURE_DMAMUX_MODULE_CHANNEL];

    // The DONE needs to be cleared before programming the channel's TCDs for the next
    // transfer.
    DMA_HAL_ClearStatus(dmaBaseAddr, rxChannel);

    // Disable and enable the TX DMA channel at the DMA mux. Doing so will prevent an
    // inadvertent DMA transfer when the TX DMA channel ERQ bit is set after having been
    // cleared from a previous DMA transfer (clearing of the ERQ bit is automatically performed
    // at the end of a transfer when D_REQ is set).
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, rxChannel, false);
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, rxChannel, true);

    // Set up this channel's control which includes enabling the DMA interrupt
    DMA_DRV_ConfigTransfer(&dmaReceive,
                           kDmaPeripheralToMemory,
                           1,
                           SPI_HAL_GetDataRegAddr(baseAddr),     // source is data register
                           (uint32_t)(p_recv_buffer),  			 // detination is rx buffer
                           (uint32_t)(len));

    // Enable the cycle steal mode which forces a single read/write transfer per request
    DMA_HAL_SetCycleStealCmd(dmaBaseAddr, rxChannel, true);

    // Enable the DMA peripheral request
    DMA_DRV_StartChannel(&dmaReceive);

    // Register callback for DMA interrupt
    DMA_DRV_RegisterCallback(&dmaReceive, MySPIslave_DRV_OnDMARecvDone, (void *)instance);

    // Enable the SPI RX DMA Request
    SPI_HAL_SetRxDmaCmd(baseAddr, true);

	return kStatus_SPI_Success;
}

spi_status_t MySPIslave_DRV_DMASend(uint32_t instance, const uint8_t *p_send_buffer, uint32_t len)
{
	if ((instance >= HW_SPI_INSTANCE_COUNT) || (p_send_buffer == NULL) || (len == 0))
	{
		return kStatus_SPI_InvalidParameter;
	}

	uint32_t baseAddr = g_spiBaseAddr[instance];

    uint8_t txChannel = dmaTransmit.channel;
    uint32_t dmaBaseAddr = g_dmaRegBaseAddr[txChannel / FSL_FEATURE_DMA_DMAMUX_CHANNELS];
    uint32_t dmamuxBaseAddr = g_dmamuxRegBaseAddr[txChannel / FSL_FEATURE_DMAMUX_MODULE_CHANNEL];	

    // The DONE needs to be cleared before programming the channel's TCDs for the next
    // transfer.
    DMA_HAL_ClearStatus(dmaBaseAddr, txChannel);

    // Disable and enable the TX DMA channel at the DMA mux. Doing so will prevent an
    // inadvertent DMA transfer when the TX DMA channel ERQ bit is set after having been
    // cleared from a previous DMA transfer (clearing of the ERQ bit is automatically performed
    // at the end of a transfer when D_REQ is set).
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, txChannel, false);
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, txChannel, true);

    // Per the reference manual, before enabling the SPI transmit DMA request, we first need
    // to read the status register and then write to the SPI data register.  Afterwards, we need
    // to decrement the sendByteCount and perform other driver maintenance functions.

    // Read the SPI Status register
    while (SPI_HAL_IsTxBuffEmptyPending(baseAddr) == 0);
    SPI_HAL_WriteDataLow(baseAddr, *p_send_buffer++);
    len--;				// Decrement the send byte count for use in DMA setup
    if (len == 0)
    {
        return kStatus_SPI_Success;
    }
    else  // Since there are more bytes to send, set up the TX DMA channel
    {
        // Set up this channel's control which includes enabling the DMA interrupt
        DMA_DRV_ConfigTransfer(&dmaTransmit, 
                               kDmaMemoryToPeripheral,
                               1,
                               (uint32_t)(p_send_buffer),
                               SPI_HAL_GetDataRegAddr(baseAddr),
                               (uint32_t)(len));

        // Enable the cycle steal mode which forces a single read/write transfer per request
        DMA_HAL_SetCycleStealCmd(dmaBaseAddr, txChannel, true);

        // Enable the DMA peripheral request
        DMA_DRV_StartChannel(&dmaTransmit);

		// Register callback for DMA interrupt
    	DMA_DRV_RegisterCallback(&dmaTransmit, MySPIslave_DRV_OnDMASendDone, (void *)instance);

        // Enable the SPI TX DMA Request
        SPI_HAL_SetTxDmaCmd(baseAddr, true);
    }

    return kStatus_SPI_Success;
}

void MySPIslave_DRV_Sendbyte(uint32_t instance, uint8_t data)
{
	uint32_t baseAddr = g_spiBaseAddr[instance];
	SPI_HAL_WriteDataLow(baseAddr, data);
}

spi_status_t MySPIslave_DRV_Init(uint32_t instance)
{
	if (instance >= HW_SPI_INSTANCE_COUNT)
	{
		return kStatus_SPI_InvalidParameter;
	}

	uint32_t baseAddr = g_spiBaseAddr[instance];
	
    // Enable clock for SPI
    CLOCK_SYS_EnableSpiClock(1);

    // Reset the SPI module to its default settings including disabling SPI
    SPI_HAL_Init(baseAddr);

	// Set SPI to 8-bit mode
	SPI_HAL_Set8or16BitMode(baseAddr, kSpi8BitMode);

	// Don't use FIFO
	SPI_HAL_SetFifoMode(baseAddr, false, kSpiTxFifoOneHalfEmpty, kSpiRxFifoOneHalfFull);

    // Set SPI to slave mode
    SPI_HAL_SetMasterSlave(baseAddr, kSpiSlave);

    // Configure the slave clock polarity, phase and data direction
    SPI_HAL_SetDataFormat(baseAddr, kSpiClockPolarity_ActiveLow, 
    								kSpiClockPhase_SecondEdge, 
    								kSpiMsbFirst);

    // Set the SPI pin mode to normal mode
    SPI_HAL_SetPinMode(baseAddr, kSpiPinMode_Normal);

	if (instance == 0)
	{
	    // Request DMA channel for RX FIFO
	    if (kDmaInvalidChannel == DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI0Rx, &dmaReceive))
	    {
	        return kStatus_SPI_DMAChannelInvalid;
	    }
	    // Request DMA channel for TX FIFO
	    if (kDmaInvalidChannel == DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI0Tx, &dmaTransmit))
	    {
	        return kStatus_SPI_DMAChannelInvalid;
	    }
	}
	else // instace == 1
	{
	    // Request DMA channel for RX FIFO
	    if (kDmaInvalidChannel == DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI1Rx, &dmaReceive))
	    {
	        return kStatus_SPI_DMAChannelInvalid;
	    }
	    // Request DMA channel for TX FIFO
	    if (kDmaInvalidChannel == DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI1Tx, &dmaTransmit))
	    {
	        return kStatus_SPI_DMAChannelInvalid;
	    }
	}

	// SPI module interrupt enable in NVIC 
    INT_SYS_EnableIRQ(g_spiIrqId[instance]);

	// Enable receive interrupts to make the slave can keep up with the master 
	SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, true);

    // SPI module enable
    SPI_HAL_Enable(baseAddr);

    return kStatus_SPI_Success;
}

void MySPIslave_DRV_IRQHandler(uint32_t instance)
{
	static uint16_t s_frame_num = 0;
    assert(instance < HW_SPI_INSTANCE_COUNT);
    uint32_t baseAddr = g_spiBaseAddr[instance];
    
    if (SPI_HAL_IsReadBuffFullPending(baseAddr))
    {
		uint8_t byteReceived = SPI_HAL_ReadDataLow(baseAddr);
        
        switch (byteReceived)
        {
		case CMD_RECV_AUDIOVIDEO_STREAM:
			{
                //debug_printf("byte:%x\r\n", byteReceived);
                
				if (g_video_data_pool[g_video_data_tx_index].flag)
				{
					g_video_data_pool[g_video_data_tx_index].frame = s_frame_num++;
					MySPIslave_DRV_DMASend(BOARD_SPI_SLAVE_INSTANCE, 
									       (const uint8_t *)&g_video_data_pool[g_video_data_tx_index], 
									       g_video_data_pool[g_video_data_tx_index].len + 3);
				
                    SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, false);
                }
				else
				{
					MySPIslave_DRV_Sendbyte(BOARD_SPI_SLAVE_INSTANCE, 0);
				}
			}
			break;
			
		case CMD_SEND_AUDIO_STREAM:
			{
				SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, false);
				MySPIslave_DRV_DMASend(BOARD_SPI_SLAVE_INSTANCE, sdata, 64);
			}
			break;
        } 
    }
}

void SPI0_IRQHandler(void)
{
   MySPIslave_DRV_IRQHandler(HW_SPI0);
}

void SPI1_IRQHandler(void)
{
   MySPIslave_DRV_IRQHandler(HW_SPI1);
}
