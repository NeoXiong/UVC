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
#ifndef __FSL_I2C_SLAVE_H__
#define __FSL_I2C_SLAVE_H__

#include <stdint.h>
#include "fsl_i2c_hal.h"
#include "fsl_os_abstraction.h"


/*!
 * @addtogroup i2c_slave
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for I2C instances. */
extern const uint32_t g_i2cBaseAddr[HW_I2C_INSTANCE_COUNT];
extern void * g_i2cStatePtr[HW_I2C_INSTANCE_COUNT];
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 */
typedef struct I2CSlaveStateStructure
{
    i2c_status_t status;                 /*!< The slave I2C status. */
    volatile uint32_t txSize;            /*!< Size of the TX buffer.*/
    volatile uint32_t rxSize;            /*!< Size of the RX buffer.*/
    const uint8_t *txBuff;               /*!< Pointer to Tx Buffer.*/
    uint8_t *rxBuff;                     /*!< Pointer to Rx Buffer.*/
    bool isTxBusy;                       /*!< True if the driver is sending data.*/
    bool isRxBusy;                       /*!< True if the driver is receiving data.*/
    bool isTxBlocking;                   /*!< True if transmit is blocking transaction. */
    bool isRxBlocking;                   /*!< True if receive is blocking transaction. */
    event_t irqEvent;                    /*!< Use to wait for ISR to complete its Tx, Rx business */
} i2c_slave_state_t;

typedef enum _i2c_slave_event_flag
{
    kI2CSlaveTxDone = 0x01u,             /*!< The slave I2C Transmitting done flag.*/
    kI2CSlaveRxDone = 0x02u,             /*!< The slave I2C Receiving done flag.*/
} i2c_slave_event_flag_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name I2C Slave
 * @{
 */

/*!
 * @brief Initializes the I2C module.
 *
 * Saves the application callback info, turns on the clock to the module,
 * enables the device, and enables interrupts. Sets the I2C to slave mode.
 * IOMUX should be handled in the init_hardware() function.
 *
 * @param instance   Instance number of the I2C module.
 * @param address    7-bit address for slave.
 * @param slave      Pointer of the slave run-time structure.
 */
void I2C_DRV_SlaveInit(uint32_t instance, uint8_t address, i2c_slave_state_t * slave);

/*!
 * @brief Shuts down the I2C slave driver.
 *
 * Clears the control register and turns off the clock to the module.
 *
 * @param instance  Instance number of the I2C module.
 */
void I2C_DRV_SlaveDeinit(uint32_t instance);

/*!
 * @brief Sends (transmits) data using a non-blocking method
 *
 * This function returns immediately when set buffer pointer and length to Tx buffer and
 * Tx Size. The user must check the status of I2C slave to know the whether transmission
 * is finished or not. User can also wait kI2CSlaveStop or kI2CSlaveTxDone to ensure that
 * the transmission is end.
 *
 * @param instance  Instance number of the I2C module.
 * @param txBuff    pointer to sending data buffer.
 * @param txSize    the number bytes which user wants to send.
 *
 * @return          success (if I2C slave status is not error) or error code in others.
 */
i2c_status_t I2C_DRV_SlaveSendData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint32_t txSize);


/*!
 * @brief Sends (transmits) data using a blocking method
 *
 * This function set buffer pointer and length to Tx buffer and Tx Size and wait until
 * transmission is end (all data are sent out or STOP signal is detected)
 *
 * @param instance  Instance number of the I2C module.
 * @param txBuff    pointer to sending data buffer.
 * @param txSize    the number bytes which user wants to send.
 * @param timeout_ms   The maximum number of milliseconds to wait for transmit completed
 *
 * @return success (if I2C slave status is not error) or error code in others.
 */

i2c_status_t I2C_DRV_SlaveSendDataBlocking(uint32_t    instance,
                                           const uint8_t *  txBuff,
                                           uint32_t   txSize,
                                           uint32_t   timeout_ms);

/*!
 * @brief Receive the data using a non-blocking method.
 *
 * This function returns immediately when set buffer pointer and length to Rx buffer and
 * Rx Size. The user must check the status of I2C slave to know the whether transmission
 * is finished or not. User can also wait kI2CSlaveStop or kI2CSlaveRxDone to ensure that
 * the transmission is end.
 *
 * @param instance  Instance number of the I2C module.
 * @param rxBuff    pointer to received data buffer.
 * @param rxSize    the number bytes which user wants to receive.
 *
 * @return          success (if I2C slave status is not error) or error code in others.
 */
i2c_status_t I2C_DRV_SlaveReceiveData(uint32_t   instance,
                                       uint8_t * rxBuff,
                                       uint32_t  rxSize);

/*!
 * @brief Receive the data using a blocking method.
 *
 * This function set buffer pointer and length to Rx buffer &Rx Size. Then wait until the
 * transmission is end (all data are received or STOP signal is detected)
 *
 * @param instance  Instance number of the I2C module.
 * @param rxBuff    pointer to received data buffer.
 * @param rxSize    the number bytes which user wants to receive.
 * @param timeout_ms   The maximum number of milliseconds to wait for receive completed
 *
 * @return          success (if I2C slave status is not error) or error code in others.
 */
i2c_status_t I2C_DRV_SlaveReceiveDataBlocking(uint32_t instance,
                                       uint8_t  * rxBuff,
                                       uint32_t   rxSize,
                                       uint32_t   timeout_ms);

/*!
 * @brief Gets current status of I2C slave driver .
 *
 * @param instance        Instance number of the I2C module.
 * @param bytesRemaining  The number of remaining bytes that I2C transmits.
 * @return                The current status of I2C instance: in progress (busy),
 *                        complete (success) or idle (i2c bus is idle).
 */
i2c_status_t I2C_DRV_SlaveGetReceiveStatus(uint32_t instance,
                                            uint32_t *bytesRemaining);

/*!
 * @brief Gets current status of I2C slave driver.
 *
 * @param instance        Instance number of the I2C module.
 * @param bytesRemaining  The number of remaining bytes that I2C transmits.
 * @return                The current status of I2C instance: in progress (busy),
 *                        complete (success) or idle (i2c bus is idle).
 */
i2c_status_t I2C_DRV_SlaveGetTransmitStatus(uint32_t instance,
                                            uint32_t *bytesRemaining);

/*!
 * @brief Terminates a non-blocking receiving I2C Slave early.
 *
 * During an non-blocking receiving
 *
 * @param instance        Instance number of the I2C module.
 * @return                kStatus_I2C_Success if success
 *                        kStatus_I2C_NoReceiveInProgress if none receiving is available.
 *
 */
i2c_status_t I2C_DRV_SlaveAbortReceiveData(uint32_t instance);

/*!
 * @brief Terminates a non-blocking sending I2C Slave early.
 *
 * During an non-blocking receiving
 *
 * @param instance        Instance number of the I2C module.
 * @return                kStatus_I2C_Success if success
 *                        kStatus_I2C_NoReceiveInProgress if none receiving is available.
 *
 */
i2c_status_t I2C_DRV_SlaveAbortSendData(uint32_t instance);

/*!
 * @brief Get current status bus of I2C Slave.
 *
 * @param instance        Instance number of the I2C module.
 * @return                true if bus is busy
 *                        false if bus is idle
 *
 */
static inline bool I2C_DRV_SlaveIsBusBusy(uint32_t instance)
{
  return I2C_HAL_GetStatusFlag(g_i2cBaseAddr[instance], kI2CBusBusy);
}

/*!
* @brief Send out multiple bytes of data using polling method.
*
* @param  instance Instance number of the I2C module.
* @param  txBuff The buffer pointer which saves the data to be sent.
* @param  txSize Size of data to be sent in unit of byte.
* @return Error or success status returned by API.
*/
static inline i2c_status_t I2C_DRV_SlaveSendDataPolling(uint32_t instance,
                                                        const uint8_t* txBuff,
                                                        uint32_t txSize)
{
    return I2C_HAL_SlaveSendDataPolling(g_i2cBaseAddr[instance], txBuff, txSize);
}

/*!
* @brief Receive multiple bytes of data using polling method.
*
* @param  instance Instance number of the I2C module.
* @param  rxBuff The buffer pointer which saves the data to be received.
* @param  rxSize Size of data need to be received in unit of byte.
* @return Error or success status returned by API.
*/
static inline i2c_status_t I2C_DRV_SlaveReceiveDataPolling(uint32_t instance,
                                                           uint8_t *rxBuff,
                                                           uint32_t rxSize)
{
    return I2C_HAL_SlaveReceiveDataPolling(g_i2cBaseAddr[instance], rxBuff, rxSize);
}


/*@}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_I2C_SLAVE_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
