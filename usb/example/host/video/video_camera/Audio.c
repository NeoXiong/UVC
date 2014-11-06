#include "board.h"
#include "fsl_adc16_driver.h"
#include "fsl_dac_hal.h"
#include "fsl_tpm_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "Audio.h"

#define DAC_MID_POINT		2048
#define DAC0_DATA0_L_ADDR   0x4003F000

static bool s_IsAudioInOn;
static bool s_IsAudioOutOn;

void MyTPM08KHz_DRV_Init(void);
void MyTPM08KHz_DRV_Start(void);
void MyTPM08KHz_DRV_Stop(void);

void MyAudioDAC_DRV_Init(void);

void Audio_Init(void)
{
	s_IsAudioInOn  = true;
	s_IsAudioOutOn = true;
	
	MyTPM08KHz_DRV_Init();
	MyTPM08KHz_DRV_Start();
	MyAudioDAC_DRV_Init();
	//MyAudioADC_DRV_Init();
    
    LED2_EN;
}

void MyAudioADC_DRV_SWtrigger(void)
{
	;
}

void MyAudioADC_DRV_Init(void)
{
	CLOCK_SYS_EnableAdcClock(0);
	
	ADC16_HAL_Init(ADC0_BASE);
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
	TPM_HAL_SetMod(TPM0_BASE, 48000000 / 8000);

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


void TPM0_IRQHandler(void)
{
	static int i = 0;
    if (TPM_HAL_GetTimerOverflowStatus(TPM0_BASE))
    {
		TPM_HAL_ClearTimerOverflowFlag(TPM0_BASE);

        LED2_TGL;
        
		if (s_IsAudioInOn)
		{
			;
		}

		if (s_IsAudioOutOn)
		{
			MyAudioDAC_DRV_Output(i++);
			if (i == 4096) i = 0;
		}
    }
}

