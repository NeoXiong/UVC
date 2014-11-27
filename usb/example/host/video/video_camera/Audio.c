#include "board.h"
#include "fsl_adc16_driver.h"
#include "fsl_dma_driver.h"
#include "fsl_dac_hal.h"
#include "fsl_tpm_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "Audio.h"

#define DAC_MID_POINT		2048
#define DAC0_DATA0_L_ADDR   0x4003F000

static bool s_IsAudioInOn;
static bool s_IsAudioOutOn;
static dma_channel_t dmaADC;

static uint8_t s_adc_index;
static uint16_t s_adc_result[10][100];

void MyTPM08KHz_DRV_Init(void);
void MyTPM08KHz_DRV_Start(void);
void MyTPM08KHz_DRV_Stop(void);

void MyAudioDAC_DRV_Init(void);

adc16_status_t MyAudioADC_DRV_Init(void);
unsigned short mysin[20];
#include <math.h>
void Audio_Init(void)
{
	s_IsAudioInOn  = true;
	s_IsAudioOutOn = true;
	
	MyTPM08KHz_DRV_Init();
	MyTPM08KHz_DRV_Start();
	MyAudioDAC_DRV_Init();
	MyAudioADC_DRV_Init();
    
    for (int i = 0; i < 20; i++)
    {
        mysin[i] = (sin(i * 2 * 3.1415926 / 20) + 1) * 2048;
    }

    LED2_EN;
}

static void MyAudioADC_DRV_OnDMADone(void *param, dma_channel_status_t chanStatus)
{
	uint8_t chn = dmaADC.channel;
	uint32_t dmaBaseAddr = g_dmaRegBaseAddr[chn / FSL_FEATURE_DMA_DMAMUX_CHANNELS];
	uint32_t dmamuxBaseAddr = g_dmamuxRegBaseAddr[chn / FSL_FEATURE_DMAMUX_MODULE_CHANNEL];

	if (++s_adc_index >= 10)
	{
		s_adc_index = 0;
	}
	
    // Stop DMA channels
    DMA_DRV_StopChannel(&dmaADC);

	// Set up this channel's control which includes enabling the DMA interrupt
	DMA_DRV_ConfigTransfer(&dmaADC,
						   kDmaPeripheralToMemory,
						   2,
						   HW_ADC_Rn_ADDR(ADC0_BASE, 0),	 // source is data register
						   (uint32_t)(s_adc_result[s_adc_index]), 	 // detination is rx buffer
						   (uint32_t)(100 * 2));

	// Enable the cycle steal mode which forces a single read/write transfer per request
	DMA_HAL_SetCycleStealCmd(dmaBaseAddr, chn, true);

	// Enable the DMA peripheral request
	DMA_DRV_StartChannel(&dmaADC);
}


adc16_status_t MyAudioADC_DRV_Init(void)
{
	adc16_calibration_param_t dummy;
	adc16_user_config_t config;
	
	config.intEnable = false;
	config.lowPowerEnable = false;
	config.clkDividerMode = kAdcClkDividerInputOf8;
	config.resolutionMode = kAdcResolutionBitOfSingleEndAs8;
	config.clkSrcMode = kAdcClkSrcOfBusClk;
	config.asyncClkEnable = false;
	config.highSpeedEnable = false;
	config.hwTriggerEnable = false;
	config.dmaEnable = true;
	config.refVoltSrcMode = kAdcRefVoltSrcOfVref;
	config.continuousConvEnable = false;

	ADC16_DRV_GetAutoCalibrationParam(BOARD_ADC_INSTANCE, &dummy);
	ADC16_DRV_Init(BOARD_ADC_INSTANCE, &config);
	ADC16_DRV_EnableLongSample(BOARD_ADC_INSTANCE, kAdcLongSampleCycleOf16);
	ADC16_DRV_EnableHwAverage(BOARD_ADC_INSTANCE, kAdcHwAverageCountOf8);

	if (kDmaInvalidChannel == DMA_DRV_RequestChannel(2, kDmaRequestMux0ADC0, &dmaADC))
	{
		return kStatus_ADC16_Failed;
	}

	uint8_t chn = dmaADC.channel;
	uint32_t dmaBaseAddr = g_dmaRegBaseAddr[chn / FSL_FEATURE_DMA_DMAMUX_CHANNELS];
	uint32_t dmamuxBaseAddr = g_dmamuxRegBaseAddr[chn / FSL_FEATURE_DMAMUX_MODULE_CHANNEL];

	// The DONE needs to be cleared before programming the channel's TCDs for the next
	// transfer.
	DMA_HAL_ClearStatus(dmaBaseAddr, chn);
	// Disable and enable the TX DMA channel at the DMA mux. Doing so will prevent an
	// inadvertent DMA transfer when the TX DMA channel ERQ bit is set after having been
	// cleared from a previous DMA transfer (clearing of the ERQ bit is automatically performed
	// at the end of a transfer when D_REQ is set).
	DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, chn, false);
	DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, chn, true);

	// Set up this channel's control which includes enabling the DMA interrupt
	DMA_DRV_ConfigTransfer(&dmaADC,
						   kDmaPeripheralToMemory,
						   2,
						   HW_ADC_Rn_ADDR(ADC0_BASE, 0),	 // source is data register
						   (uint32_t)(s_adc_result[0]),		 // detination is rx buffer
						   (uint32_t)(100 * 2));

	// Enable the cycle steal mode which forces a single read/write transfer per request
	DMA_HAL_SetCycleStealCmd(dmaBaseAddr, chn, true);

	// Register callback for DMA interrupt
	DMA_DRV_RegisterCallback(&dmaADC, MyAudioADC_DRV_OnDMADone, (void *)NULL);

	// Enable the DMA peripheral request
	DMA_DRV_StartChannel(&dmaADC);

    return kStatus_ADC16_Success;
}

static inline void MyAudioDAC_DRV_Output(uint16_t val)
{
	*(uint16_t *)DAC0_DATA0_L_ADDR = val;
}

void MyAudioDAC_DRV_Init(void)
{
	CLOCK_SYS_EnableDacClock(0);

	DAC_HAL_Init(DAC0_BASE);

	DAC_HAL_SetBuffCmd(DAC0_BASE, false);
	DAC_HAL_SetDmaCmd(DAC0_BASE, false);

	DAC_HAL_SetRefVoltSrcMode(DAC0_BASE, kDacRefVoltSrcOfVref1);
	DAC_HAL_SetLowPowerCmd(DAC0_BASE, false);

	DAC_HAL_Enable(DAC0_BASE);

	MyAudioDAC_DRV_Output(DAC_MID_POINT);
}

void MyTPM08KHz_DRV_Init(void)
{
	uint32_t tpm_clock;
	
    CLOCK_SYS_EnableTpmClock(0);
	CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcPllFllSel);

	TPM_HAL_Reset(TPM0_BASE, 0);
	TPM_HAL_SetClockMode(TPM0_BASE, kTpmClockSourceNoneClk);
	TPM_HAL_SetClockDiv(TPM0_BASE, kTpmDividedBy1);
	TPM_HAL_SetMod(TPM0_BASE, 48000000 / 1000);

	INT_SYS_EnableIRQ(TPM0_IRQn);
	NVIC_SetPriority(TPM0_IRQn, 0);

	TPM_HAL_EnableTimerOverflowInt(TPM0_BASE);
	TPM_HAL_SetClockMode(TPM0_BASE, kTpmClockSourceModuleClk);
}

void MyTPM08KHz_DRV_Start(void)
{
	TPM_HAL_SetClockMode(TPM0_BASE, kTpmClockSourceModuleClk);
}

void MyTPM08KHz_DRV_Stop(void)
{
	TPM_HAL_SetClockMode(TPM0_BASE, kTpmClockSourceNoneClk);
}

static const adc16_chn_config_t s_chn_config = {kAdcChnMuxOfDefault, 0, false, false};
uint16_t adc_data_raw[100];
uint16_t adc_data_index = 0;
#include "fsl_spi_hal.h"
void TPM0_IRQHandler(void)
{
	static int i = 0;
    if (TPM_HAL_GetTimerOverflowStatus(TPM0_BASE))
    {
		TPM_HAL_ClearTimerOverflowFlag(TPM0_BASE);
        
		if (s_IsAudioInOn)
		{
			ADC16_DRV_ConfigConvChn(BOARD_ADC_INSTANCE, 0, (adc16_chn_config_t *)&s_chn_config);
		}

		if (s_IsAudioOutOn)
		{
			MyAudioDAC_DRV_Output(mysin[i++]);
			if (i == 20) i = 0;
		}
    }
}

