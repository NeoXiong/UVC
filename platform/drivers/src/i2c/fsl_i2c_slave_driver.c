/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include <assert.h>
#include <string.h>
#include "fsl_i2c_hal.h"
#include "fsl_i2c_slave_driver.h"
#include "fsl_i2c_shared_function.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveInit
 * Description   : initializes the I2C module.
 * This function will save the application callback info, turn on the clock to the
 * module, enable the device and enable interrupts. Set the I2C to slave mode.
 *
 *END**************************************************************************/
void I2C_DRV_SlaveInit(uint32_t instance, uint8_t address, i2c_slave_state_t * slave)
{
    assert(slave);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    uint32_t baseAddr = g_i2cBaseAddr[instance];

    /* Exit if current instance is already initilized. */
    if (g_i2cStatePtr[instance])
    {
        return;
    }

    /* Init driver instance struct.*/
    memset(slave, 0, sizeof(i2c_slave_state_t));

    /* Enable clock for I2C.*/
    CLOCK_SYS_EnableI2cClock(instance);

    /* Init instance to known state. */
    I2C_HAL_Init(baseAddr);

    /* Set slave address.*/
    I2C_HAL_SetAddress7bit(baseAddr, address);

    /* Save runtime structure poniter.*/
    g_i2cStatePtr[instance] = slave;

    /* Create Event for irqSync */
    OSA_EventCreate(&slave->irqEvent, kEventAutoClear);

    /* Enable I2C interrupt in the peripheral.*/
    I2C_HAL_SetIntCmd(baseAddr, true);

    /* Enable I2C interrupt.*/
    INT_SYS_EnableIRQ(g_i2cIrqId[instance]);

    /* Enable the peripheral operation.*/
    I2C_HAL_Enable(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveDeinit
 * Description   : Shuts down the I2C slave driver.
 * This function will clear the control register and turn off the clock to the module.
 *
 *END**************************************************************************/
void I2C_DRV_SlaveDeinit(uint32_t instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    uint32_t baseAddr = g_i2cBaseAddr[instance];
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    /* Disable I2C interrupt. */
    I2C_HAL_SetIntCmd(baseAddr, false);

    /* Turn off I2C.*/
    I2C_HAL_Disable(baseAddr);

    /* Disable clock for I2C.*/
    CLOCK_SYS_DisableI2cClock(instance);

    /* Disable I2C NVIC interrupt */
    INT_SYS_DisableIRQ(g_i2cIrqId[instance]);

    /* Destroy sema. */
    OSA_EventDestroy(&i2cSlaveState->irqEvent);

    /* Clear runtime structure poniter.*/
    g_i2cStatePtr[instance] = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveReceiveDataBlocking
 * Description   : Receive the data using a blocking method.
 * This function set buffer pointer and length to Rx buffer &Rx Size. Then wait until the
 * transmission is end (all data are received or STOP signal is detected)
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveReceiveDataBlocking(uint32_t instance,
                                    uint8_t * rxBuff,
                                    uint32_t rxSize,
                                    uint32_t timeout_ms)
{
    assert(rxBuff);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    event_flags_t i2cIrqSetFlags;
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    osa_status_t syncStatus;

    if (i2cSlaveState->isRxBusy)
    {
        return kStatus_I2C_Busy;
    }

    i2cSlaveState->rxBuff = rxBuff;
    i2cSlaveState->rxSize = rxSize;
    i2cSlaveState->isRxBusy = true;
    i2cSlaveState->isRxBlocking = true;

    /* Wait until the transmit is complete. */
    do
    {
        syncStatus = OSA_EventWait(&i2cSlaveState->irqEvent,
                                    kI2CSlaveRxDone,
                                    false,
                                    timeout_ms,
                                    &i2cIrqSetFlags);

    } while(syncStatus == kStatus_OSA_Idle);

    if (syncStatus != kStatus_OSA_Success)
    {
        i2cSlaveState->status = kStatus_I2C_Timeout;
    }

    i2cSlaveState->isRxBlocking = false;
    return i2cSlaveState->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveReceiveData
 * Description   : Receive the data using a non-blocking method.
 * This function set buffer pointer and length to Rx buffer & Rx Size
 * A non-blocking (also known as synchronous) function means that the function
 * returns immediately after initiating the receive function. The application
 * has to get the receive status to see when the receive is complete.
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveReceiveData(uint32_t instance,
                                    uint8_t * rxBuff,
                                    uint32_t rxSize)
{
    assert(rxBuff);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    if (i2cSlaveState->isRxBusy)
    {
        return kStatus_I2C_Busy;
    }

    i2cSlaveState->rxBuff = rxBuff;
    i2cSlaveState->rxSize = rxSize;
    i2cSlaveState->isRxBusy = true;

    return kStatus_I2C_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSendDataBlocking
 * Description   : Send the data using a blocking method.
 * This function set buffer pointer and length to Tx buffer & Tx Size.
 * Then wait until the transmission is end (all data are received or
 * STOP signal is detected)
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveSendDataBlocking(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize,
                                    uint32_t timeout_ms)
{
    assert(txBuff);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    event_flags_t       i2cIrqSetFlags;
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    osa_status_t syncStatus;

    if (i2cSlaveState->isTxBusy)
    {
        return kStatus_I2C_Busy;
    }

    /* Initialize the module driver state structure. */
    i2cSlaveState->txBuff = txBuff;
    i2cSlaveState->txSize = txSize;
    i2cSlaveState->isTxBusy = true;
    i2cSlaveState->isTxBlocking = true;

    /* Wait until the transmit is complete. */
    do
    {
        syncStatus = OSA_EventWait(&i2cSlaveState->irqEvent,
                                    kI2CSlaveTxDone,
                                    false,
                                    timeout_ms,
                                    &i2cIrqSetFlags);

    } while(syncStatus == kStatus_OSA_Idle);

    if (syncStatus != kStatus_OSA_Success)
    {
        i2cSlaveState->status = kStatus_I2C_Timeout;
    }

    i2cSlaveState->isTxBlocking = false;

    return i2cSlaveState->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSendData
 * Description   : Send the data using a non-blocking method.
 * This function set buffer pointer and length to Tx buffer & Tx Size
 * A non-blocking (also known as synchronous) function means that the function
 * returns immediately after initiating the receive function. The application
 * has to get the receive status to see when the receive is complete.
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveSendData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint32_t txSize)
{
    assert(txBuff);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    if (i2cSlaveState->isTxBusy)
    {
        return kStatus_I2C_Busy;
    }

    /* Initialize the module driver state structure. */
    i2cSlaveState->txBuff = txBuff;
    i2cSlaveState->txSize = txSize;
    i2cSlaveState->isTxBusy = true;

    return kStatus_I2C_Success;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveGetTransmitStatus
 * Description   : Gets current status of I2C slave driver. This function returns
 * whether the previous I2C Slave Transmit has finished
 * When performing a non-blocking transmit, the user can call this function to ascertain
 * the state of the current transmission: in progress (or busy) or complete (finished).
 * The user can obtain the number of bytes that have been currently transferred
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveGetTransmitStatus(uint32_t instance,
                                            uint32_t *bytesRemaining)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    /* Get current runtime structure */
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    if (bytesRemaining)
    {
        *bytesRemaining = i2cSlaveState->txSize;
    }

    return (i2cSlaveState->isTxBusy ? kStatus_I2C_Busy : kStatus_I2C_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveGetReceiveStatus
 * Description   : Gets current status of I2C slave driver. This function returns
 * whether the previous I2C Slave Receive has finished
 * When performing a non-blocking receiving, the user can call this function to ascertain
 * the state of the current receiving: in progress (or busy) or complete (finished).
 * The user can obtain the number of bytes that have been currently transferred
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveGetReceiveStatus(uint32_t instance,
                                            uint32_t *bytesRemaining)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    /* Get current runtime structure */
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    if (bytesRemaining)
    {
        *bytesRemaining = i2cSlaveState->rxSize;
    }

    return (i2cSlaveState->isRxBusy ? kStatus_I2C_Busy : kStatus_I2C_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveAbortReceiveData
 * Description   : This function is used to abort receiving of I2C slave
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveAbortReceiveData(uint32_t instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    /* Check if a transfer is running. */
    if (!i2cSlaveState->isRxBusy)
    {
        return kStatus_I2C_NoReceiveInProgress;
    }

    /* Stop the running transfer. */
    i2cSlaveState->isRxBusy = false;
    i2cSlaveState->rxBuff = NULL;
    i2cSlaveState->rxSize = 0;
    if (i2cSlaveState->isRxBlocking)
    {
        /* Set kI2CSlaveRxDone event to notify that the receiving is done */
        OSA_EventSet(&i2cSlaveState->irqEvent, kI2CSlaveRxDone);
    }

    return kStatus_I2C_Success;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveAbortSendData
 * Description   : This function is used to abort sending of I2C slave
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_SlaveAbortSendData(uint32_t instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    /* Check if a transfer is running. */
    if (!i2cSlaveState->isTxBusy)
    {
        return kStatus_I2C_NoSendInProgress;
    }

    /* Stop the running transfer. */
    i2cSlaveState->isTxBusy = false;
    i2cSlaveState->txBuff = NULL;
    i2cSlaveState->txSize = 0;

    if (i2cSlaveState->isTxBlocking)
    {
        /* Set kI2CSlaveRxDone event to notify that the sending is done */
        OSA_EventSet(&i2cSlaveState->irqEvent, kI2CSlaveTxDone);
    }

    return kStatus_I2C_Success;
}


/*!
 * @brief I2C Slave Generic IRQ handler.
 *
 * This handler implements the flow chart at the end of the I2C chapter in the Kinetis
 * KL25 Sub-Family Reference Manual. It puts data from/to the application as well as
 * alert the application of an error condition.
 *
 * @param instance Instance number of the I2C module.
 */
void I2C_DRV_SlaveIRQHandler(uint32_t instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    uint32_t baseAddr = g_i2cBaseAddr[instance];
    uint8_t  i2cData  = 0x00;
    bool     doTransmit = false;
    bool     wasArbLost = I2C_HAL_GetStatusFlag(baseAddr, kI2CArbitrationLost);
    bool     addressed = I2C_HAL_GetStatusFlag(baseAddr, kI2CAddressAsSlave);


    /* Get current runtime structure */
    i2c_slave_state_t * i2cSlaveState = (i2c_slave_state_t *)g_i2cStatePtr[instance];

    /* Get current slave transfer direction */
    i2c_direction_t direction = I2C_HAL_GetDirMode(baseAddr);

    /* Clear I2C IRQ.*/
    I2C_HAL_ClearInt(baseAddr);

    if (wasArbLost)
    {
        I2C_HAL_ClearArbitrationLost(baseAddr);
        if (!addressed)
        {
            i2cSlaveState->status = kStatus_I2C_AribtrationLost;
            return;
        }
    }

    /* Addressed only happens when receiving address. */
    if (addressed) /* Slave is addressed. */
    {
        /* Master read from Slave. Slave transmit.*/
        if (I2C_HAL_GetStatusFlag(baseAddr, kI2CSlaveTransmit))
        {
            /* Switch to TX mode*/
            I2C_HAL_SetDirMode(baseAddr, kI2CSend);

            doTransmit = true;
        }
        else /* Master write to Slave. Slave receive.*/
        {
            /* Switch to RX mode.*/
            I2C_HAL_SetDirMode(baseAddr, kI2CReceive);

            /* Read dummy character.*/
            I2C_HAL_ReadByte(baseAddr);
        }
    }
    else
    {
        if (direction == kI2CSend)
        {
            if (I2C_HAL_GetStatusFlag(baseAddr, kI2CReceivedNak))
            {
                /* Record that we got a NAK */
                i2cSlaveState->status = kStatus_I2C_ReceivedNak;

                /* Got a NAK, so we're done with this transfer */
                if (i2cSlaveState->isTxBlocking)
                {
                    OSA_EventSet(&i2cSlaveState->irqEvent, kI2CSlaveTxDone);
                }

                i2cSlaveState->txSize = 0;
                i2cSlaveState->txBuff = NULL;
                i2cSlaveState->isTxBusy = false;

                /* Switch to RX mode.*/
                I2C_HAL_SetDirMode(baseAddr, kI2CReceive);

                /* Read dummy character.*/
                I2C_HAL_ReadByte(baseAddr);
            }
            else /* ACK from receiver.*/
            {
                doTransmit = true;
            }
        }
        else
        {
            /* Get byte from data register */
            i2cData = I2C_HAL_ReadByte(baseAddr);
            if (i2cSlaveState->rxSize)
            {
                *(i2cSlaveState->rxBuff) = i2cData;
                ++ i2cSlaveState->rxBuff;
                -- i2cSlaveState->rxSize;

                if (!i2cSlaveState->rxSize)
                {
                    /* All bytes are received, so we're done with this transfer */
                    if (i2cSlaveState->isRxBlocking)
                    {
                        OSA_EventSet(&i2cSlaveState->irqEvent, kI2CSlaveRxDone);
                    }

                    i2cSlaveState->isRxBusy = false;
                    i2cSlaveState->rxBuff = NULL;
                }
            }
            else
            {
                /* The Rxbuff is full --> Set kStatus_I2C_SlaveRxOverrun*/
                i2cSlaveState->status = kStatus_I2C_SlaveRxOverrun;
            }
        }
    }

    /* DO TRANSMIT*/
    if (doTransmit)
    {
        /* Send byte to data register */
        if (i2cSlaveState->txSize)
        {
            i2cData = *(i2cSlaveState->txBuff);
            I2C_HAL_WriteByte(baseAddr, i2cData);
            ++ i2cSlaveState->txBuff;
            -- i2cSlaveState->txSize;
            if (!i2cSlaveState->txSize)
            {
                /* All bytes are received, so we're done with this transfer */
                if (i2cSlaveState->isTxBlocking)
                {
                    OSA_EventSet(&i2cSlaveState->irqEvent, kI2CSlaveTxDone);
                }

                i2cSlaveState->txBuff = NULL;
                i2cSlaveState->isTxBusy = false;
            }
        }
        else
        {
            /* The Txbuff is empty --> set kStatus_I2C_SlaveTxUnderrun*/
            i2cSlaveState->status = kStatus_I2C_SlaveTxUnderrun ;
        }
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
