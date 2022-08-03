/***************************************************************************//**
* \file mUART_UART.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mUART_PVT.h"
#include "mUART_SPI_UART_PVT.h"
#include "cyapicallbacks.h"

#if (mUART_UART_WAKE_ENABLE_CONST && mUART_UART_RX_WAKEUP_IRQ)
    /**
    * \addtogroup group_globals
    * \{
    */
    /** This global variable determines whether to enable Skip Start
    * functionality when mUART_Sleep() function is called:
    * 0 – disable, other values – enable. Default value is 1.
    * It is only available when Enable wakeup from Deep Sleep Mode is enabled.
    */
    uint8 mUART_skipStart = 1u;
    /** \} globals */
#endif /* (mUART_UART_WAKE_ENABLE_CONST && mUART_UART_RX_WAKEUP_IRQ) */

#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const mUART_UART_INIT_STRUCT mUART_configUart =
    {
        mUART_UART_SUB_MODE,
        mUART_UART_DIRECTION,
        mUART_UART_DATA_BITS_NUM,
        mUART_UART_PARITY_TYPE,
        mUART_UART_STOP_BITS_NUM,
        mUART_UART_OVS_FACTOR,
        mUART_UART_IRDA_LOW_POWER,
        mUART_UART_MEDIAN_FILTER_ENABLE,
        mUART_UART_RETRY_ON_NACK,
        mUART_UART_IRDA_POLARITY,
        mUART_UART_DROP_ON_PARITY_ERR,
        mUART_UART_DROP_ON_FRAME_ERR,
        mUART_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        mUART_UART_MP_MODE_ENABLE,
        mUART_UART_MP_ACCEPT_ADDRESS,
        mUART_UART_MP_RX_ADDRESS,
        mUART_UART_MP_RX_ADDRESS_MASK,
        (uint32) mUART_SCB_IRQ_INTERNAL,
        mUART_UART_INTR_RX_MASK,
        mUART_UART_RX_TRIGGER_LEVEL,
        mUART_UART_INTR_TX_MASK,
        mUART_UART_TX_TRIGGER_LEVEL,
        (uint8) mUART_UART_BYTE_MODE_ENABLE,
        (uint8) mUART_UART_CTS_ENABLE,
        (uint8) mUART_UART_CTS_POLARITY,
        (uint8) mUART_UART_RTS_POLARITY,
        (uint8) mUART_UART_RTS_FIFO_LEVEL,
        (uint8) mUART_UART_RX_BREAK_WIDTH
    };


    /*******************************************************************************
    * Function Name: mUART_UartInit
    ****************************************************************************//**
    *
    *  Configures the mUART for UART operation.
    *
    *  This function is intended specifically to be used when the mUART
    *  configuration is set to “Unconfigured mUART” in the customizer.
    *  After initializing the mUART in UART mode using this function,
    *  the component can be enabled using the mUART_Start() or
    * mUART_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration
    *  settings. This structure contains the same information that would otherwise
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of
    *   fields. These fields match the selections available in the customizer.
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void mUART_UartInit(const mUART_UART_INIT_STRUCT *config)
    {
        uint32 pinsConfig;

        if (NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Get direction to configure UART pins: TX, RX or TX+RX */
            pinsConfig  = config->direction;

        #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (mUART_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)      ? (mUART_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

            /* Configure pins */
            mUART_SetPins(mUART_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            mUART_scbMode       = (uint8) mUART_SCB_MODE_UART;
            mUART_scbEnableWake = (uint8) config->enableWake;
            mUART_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            mUART_rxBuffer      =         config->rxBuffer;
            mUART_rxDataBits    = (uint8) config->dataBits;
            mUART_rxBufferSize  =         config->rxBufferSize;

            /* Set TX direction internal variables */
            mUART_txBuffer      =         config->txBuffer;
            mUART_txDataBits    = (uint8) config->dataBits;
            mUART_txBufferSize  =         config->txBufferSize;

            /* Configure UART interface */
            if(mUART_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                mUART_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (mUART_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (mUART_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                mUART_CTRL_REG  = mUART_GET_CTRL_OVS(config->oversample);
            }

            mUART_CTRL_REG     |= mUART_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             mUART_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             mUART_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            mUART_UART_CTRL_REG = mUART_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            mUART_UART_RX_CTRL_REG = mUART_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        mUART_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        mUART_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        mUART_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        mUART_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr)   |
                                        mUART_GET_UART_RX_CTRL_BREAK_WIDTH(config->breakWidth);

            if(mUART_UART_PARITY_NONE != config->parity)
            {
               mUART_UART_RX_CTRL_REG |= mUART_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    mUART_UART_RX_CTRL_PARITY_ENABLED;
            }

            mUART_RX_CTRL_REG      = mUART_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                mUART_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                mUART_GET_UART_RX_CTRL_ENABLED(config->direction);

            mUART_RX_FIFO_CTRL_REG = mUART_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            mUART_RX_MATCH_REG     = mUART_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                mUART_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            mUART_UART_TX_CTRL_REG = mUART_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                mUART_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(mUART_UART_PARITY_NONE != config->parity)
            {
               mUART_UART_TX_CTRL_REG |= mUART_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    mUART_UART_TX_CTRL_PARITY_ENABLED;
            }

            mUART_TX_CTRL_REG      = mUART_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                mUART_GET_UART_TX_CTRL_ENABLED(config->direction);

            mUART_TX_FIFO_CTRL_REG = mUART_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
            mUART_UART_FLOW_CTRL_REG = mUART_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            mUART_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            mUART_GET_UART_FLOW_CTRL_RTS_POLARITY (config->rtsPolarity)  | \
                                            mUART_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (mUART_ISR_NUMBER);
            CyIntSetPriority(mUART_ISR_NUMBER, mUART_ISR_PRIORITY);
            (void) CyIntSetVector(mUART_ISR_NUMBER, &mUART_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(mUART_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (mUART_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(mUART_RX_WAKE_ISR_NUMBER, mUART_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(mUART_RX_WAKE_ISR_NUMBER, &mUART_UART_WAKEUP_ISR);
        #endif /* (mUART_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            mUART_INTR_I2C_EC_MASK_REG = mUART_NO_INTR_SOURCES;
            mUART_INTR_SPI_EC_MASK_REG = mUART_NO_INTR_SOURCES;
            mUART_INTR_SLAVE_MASK_REG  = mUART_NO_INTR_SOURCES;
            mUART_INTR_MASTER_MASK_REG = mUART_NO_INTR_SOURCES;
            mUART_INTR_RX_MASK_REG     = config->rxInterruptMask;
            mUART_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Configure TX interrupt sources to restore. */
            mUART_IntrTxMask = LO16(mUART_INTR_TX_MASK_REG);

            /* Clear RX buffer indexes */
            mUART_rxBufferHead     = 0u;
            mUART_rxBufferTail     = 0u;
            mUART_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            mUART_txBufferHead = 0u;
            mUART_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: mUART_UartInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the UART operation.
    *
    *******************************************************************************/
    void mUART_UartInit(void)
    {
        /* Configure UART interface */
        mUART_CTRL_REG = mUART_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        mUART_UART_CTRL_REG = mUART_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        mUART_UART_RX_CTRL_REG = mUART_UART_DEFAULT_UART_RX_CTRL;
        mUART_RX_CTRL_REG      = mUART_UART_DEFAULT_RX_CTRL;
        mUART_RX_FIFO_CTRL_REG = mUART_UART_DEFAULT_RX_FIFO_CTRL;
        mUART_RX_MATCH_REG     = mUART_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        mUART_UART_TX_CTRL_REG = mUART_UART_DEFAULT_UART_TX_CTRL;
        mUART_TX_CTRL_REG      = mUART_UART_DEFAULT_TX_CTRL;
        mUART_TX_FIFO_CTRL_REG = mUART_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
        mUART_UART_FLOW_CTRL_REG = mUART_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(mUART_SCB_IRQ_INTERNAL)
        CyIntDisable    (mUART_ISR_NUMBER);
        CyIntSetPriority(mUART_ISR_NUMBER, mUART_ISR_PRIORITY);
        (void) CyIntSetVector(mUART_ISR_NUMBER, &mUART_SPI_UART_ISR);
    #endif /* (mUART_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(mUART_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (mUART_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(mUART_RX_WAKE_ISR_NUMBER, mUART_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(mUART_RX_WAKE_ISR_NUMBER, &mUART_UART_WAKEUP_ISR);
    #endif /* (mUART_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        mUART_INTR_I2C_EC_MASK_REG = mUART_UART_DEFAULT_INTR_I2C_EC_MASK;
        mUART_INTR_SPI_EC_MASK_REG = mUART_UART_DEFAULT_INTR_SPI_EC_MASK;
        mUART_INTR_SLAVE_MASK_REG  = mUART_UART_DEFAULT_INTR_SLAVE_MASK;
        mUART_INTR_MASTER_MASK_REG = mUART_UART_DEFAULT_INTR_MASTER_MASK;
        mUART_INTR_RX_MASK_REG     = mUART_UART_DEFAULT_INTR_RX_MASK;
        mUART_INTR_TX_MASK_REG     = mUART_UART_DEFAULT_INTR_TX_MASK;

        /* Configure TX interrupt sources to restore. */
        mUART_IntrTxMask = LO16(mUART_INTR_TX_MASK_REG);

    #if(mUART_INTERNAL_RX_SW_BUFFER_CONST)
        mUART_rxBufferHead     = 0u;
        mUART_rxBufferTail     = 0u;
        mUART_rxBufferOverflow = 0u;
    #endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(mUART_INTERNAL_TX_SW_BUFFER_CONST)
        mUART_txBufferHead = 0u;
        mUART_txBufferTail = 0u;
    #endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: mUART_UartPostEnable
****************************************************************************//**
*
*  Restores HSIOM settings for the UART output pins (TX and/or RTS) to be
*  controlled by the SCB UART.
*
*******************************************************************************/
void mUART_UartPostEnable(void)
{
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (mUART_TX_SDA_MISO_PIN)
        if (mUART_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set SCB UART to drive the output pin */
            mUART_SET_HSIOM_SEL(mUART_TX_SDA_MISO_HSIOM_REG, mUART_TX_SDA_MISO_HSIOM_MASK,
                                           mUART_TX_SDA_MISO_HSIOM_POS, mUART_TX_SDA_MISO_HSIOM_SEL_UART);
        }
    #endif /* (mUART_TX_SDA_MISO_PIN_PIN) */

    #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
        #if (mUART_RTS_SS0_PIN)
            if (mUART_CHECK_RTS_SS0_PIN_USED)
            {
                /* Set SCB UART to drive the output pin */
                mUART_SET_HSIOM_SEL(mUART_RTS_SS0_HSIOM_REG, mUART_RTS_SS0_HSIOM_MASK,
                                               mUART_RTS_SS0_HSIOM_POS, mUART_RTS_SS0_HSIOM_SEL_UART);
            }
        #endif /* (mUART_RTS_SS0_PIN) */
    #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

#else
    #if (mUART_UART_TX_PIN)
         /* Set SCB UART to drive the output pin */
        mUART_SET_HSIOM_SEL(mUART_TX_HSIOM_REG, mUART_TX_HSIOM_MASK,
                                       mUART_TX_HSIOM_POS, mUART_TX_HSIOM_SEL_UART);
    #endif /* (mUART_UART_TX_PIN) */

    #if (mUART_UART_RTS_PIN)
        /* Set SCB UART to drive the output pin */
        mUART_SET_HSIOM_SEL(mUART_RTS_HSIOM_REG, mUART_RTS_HSIOM_MASK,
                                       mUART_RTS_HSIOM_POS, mUART_RTS_HSIOM_SEL_UART);
    #endif /* (mUART_UART_RTS_PIN) */
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Restore TX interrupt sources. */
    mUART_SetTxInterruptMode(mUART_IntrTxMask);
}


/*******************************************************************************
* Function Name: mUART_UartStop
****************************************************************************//**
*
*  Changes the HSIOM settings for the UART output pins (TX and/or RTS) to keep
*  them inactive after the block is disabled. The output pins are controlled by
*  the GPIO data register. Also, the function disables the skip start feature
*  to not cause it to trigger after the component is enabled.
*
*******************************************************************************/
void mUART_UartStop(void)
{
#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (mUART_TX_SDA_MISO_PIN)
        if (mUART_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set GPIO to drive output pin */
            mUART_SET_HSIOM_SEL(mUART_TX_SDA_MISO_HSIOM_REG, mUART_TX_SDA_MISO_HSIOM_MASK,
                                           mUART_TX_SDA_MISO_HSIOM_POS, mUART_TX_SDA_MISO_HSIOM_SEL_GPIO);
        }
    #endif /* (mUART_TX_SDA_MISO_PIN_PIN) */

    #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
        #if (mUART_RTS_SS0_PIN)
            if (mUART_CHECK_RTS_SS0_PIN_USED)
            {
                /* Set output pin state after block is disabled */
                mUART_uart_rts_spi_ss0_Write(mUART_GET_UART_RTS_INACTIVE);

                /* Set GPIO to drive output pin */
                mUART_SET_HSIOM_SEL(mUART_RTS_SS0_HSIOM_REG, mUART_RTS_SS0_HSIOM_MASK,
                                               mUART_RTS_SS0_HSIOM_POS, mUART_RTS_SS0_HSIOM_SEL_GPIO);
            }
        #endif /* (mUART_RTS_SS0_PIN) */
    #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

#else
    #if (mUART_UART_TX_PIN)
        /* Set GPIO to drive output pin */
        mUART_SET_HSIOM_SEL(mUART_TX_HSIOM_REG, mUART_TX_HSIOM_MASK,
                                       mUART_TX_HSIOM_POS, mUART_TX_HSIOM_SEL_GPIO);
    #endif /* (mUART_UART_TX_PIN) */

    #if (mUART_UART_RTS_PIN)
        /* Set output pin state after block is disabled */
        mUART_rts_Write(mUART_GET_UART_RTS_INACTIVE);

        /* Set GPIO to drive output pin */
        mUART_SET_HSIOM_SEL(mUART_RTS_HSIOM_REG, mUART_RTS_HSIOM_MASK,
                                       mUART_RTS_HSIOM_POS, mUART_RTS_HSIOM_SEL_GPIO);
    #endif /* (mUART_UART_RTS_PIN) */

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (mUART_UART_WAKE_ENABLE_CONST)
    /* Disable skip start feature used for wakeup */
    mUART_UART_RX_CTRL_REG &= (uint32) ~mUART_UART_RX_CTRL_SKIP_START;
#endif /* (mUART_UART_WAKE_ENABLE_CONST) */

    /* Store TX interrupt sources (exclude level triggered). */
    mUART_IntrTxMask = LO16(mUART_GetTxInterruptMode() & mUART_INTR_UART_TX_RESTORE);
}


/*******************************************************************************
* Function Name: mUART_UartSetRxAddress
****************************************************************************//**
*
*  Sets the hardware detectable receiver address for the UART in the
*  Multiprocessor mode.
*
*  \param address: Address for hardware address detection.
*
*******************************************************************************/
void mUART_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = mUART_RX_MATCH_REG;

    matchReg &= ((uint32) ~mUART_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & mUART_RX_MATCH_ADDR_MASK)); /* Set address  */

    mUART_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: mUART_UartSetRxAddressMask
****************************************************************************//**
*
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
*  \param addressMask: Address mask.
*   - Bit value 0 – excludes bit from address comparison.
*   - Bit value 1 – the bit needs to match with the corresponding bit
*     of the address.
*
*******************************************************************************/
void mUART_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = mUART_RX_MATCH_REG;

    matchReg &= ((uint32) ~mUART_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << mUART_RX_MATCH_MASK_POS));

    mUART_RX_MATCH_REG = matchReg;
}


#if(mUART_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: mUART_UartGetChar
    ****************************************************************************//**
    *
    *  Retrieves next data element from receive buffer.
    *  This function is designed for ASCII characters and returns a char where
    *  1 to 255 are valid characters and 0 indicates an error occurred or no data
    *  is present.
    *  - RX software buffer is disabled: Returns data element retrieved from RX
    *    FIFO.
    *  - RX software buffer is enabled: Returns data element from the software
    *    receive buffer.
    *
    *  \return
    *   Next data element from the receive buffer. ASCII character values from
    *   1 to 255 are valid. A returned zero signifies an error condition or no
    *   data available.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check mUART_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 mUART_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if (0u != mUART_SpiUartGetRxBufferSize())
        {
            rxData = mUART_SpiUartReadRxData();
        }

        if (mUART_CHECK_INTR_RX(mUART_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            mUART_ClearRxInterruptSource(mUART_INTR_RX_ERR);
        }

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: mUART_UartGetByte
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer, returns the
    *  received byte and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved
    *     from the RX FIFO. Undefined data will be returned if the RX FIFO is
    *     empty.
    *   - The RX software buffer is enabled: returns data element from the
    *     software receive buffer.
    *
    *  \return
    *   Bits 7-0 contain the next data element from the receive buffer and
    *   other bits contain the error condition.
    *   - mUART_UART_RX_OVERFLOW - Attempt to write to a full
    *     receiver FIFO.
    *   - mUART_UART_RX_UNDERFLOW    Attempt to read from an empty
    *     receiver FIFO.
    *   - mUART_UART_RX_FRAME_ERROR - UART framing error detected.
    *   - mUART_UART_RX_PARITY_ERROR - UART parity error detected.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check mUART_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 mUART_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;

        #if (mUART_CHECK_RX_SW_BUFFER)
        {
            mUART_DisableInt();
        }
        #endif

        if (0u != mUART_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            #if (mUART_CHECK_RX_SW_BUFFER)
            {
                mUART_EnableInt();
            }
            #endif

            /* Get received byte */
            rxData = mUART_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the
            * case of empty. Otherwise the first received byte will be read.
            */
            rxData = mUART_RX_FIFO_RD_REG;


            /* Enables interrupt to receive more bytes. */
            #if (mUART_CHECK_RX_SW_BUFFER)
            {

                /* The byte has been read from RX FIFO. Clear RX interrupt to
                * not involve interrupt handler when RX FIFO is empty.
                */
                mUART_ClearRxInterruptSource(mUART_INTR_RX_NOT_EMPTY);

                mUART_EnableInt();
            }
            #endif
        }

        /* Get and clear RX error mask */
        tmpStatus = (mUART_GetRxInterruptSource() & mUART_INTR_RX_ERR);
        mUART_ClearRxInterruptSource(mUART_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return (rxData);
    }


    #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: mUART_UartSetRtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of RTS output signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of RTS output signal.
        *   - mUART_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   - mUART_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        *******************************************************************************/
        void mUART_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                mUART_UART_FLOW_CTRL_REG |= (uint32)  mUART_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                mUART_UART_FLOW_CTRL_REG &= (uint32) ~mUART_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: mUART_UartSetRtsFifoLevel
        ****************************************************************************//**
        *
        *  Sets level in the RX FIFO for RTS signal activation.
        *  While the RX FIFO has fewer entries than the RX FIFO level the RTS signal
        *  remains active, otherwise the RTS signal becomes inactive.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param level: Level in the RX FIFO for RTS signal activation.
        *   The range of valid level values is between 0 and RX FIFO depth - 1.
        *   Setting level value to 0 disables RTS signal activation.
        *
        *******************************************************************************/
        void mUART_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = mUART_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~mUART_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (mUART_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            mUART_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */

#endif /* (mUART_UART_RX_DIRECTION) */


#if(mUART_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: mUART_UartPutString
    ****************************************************************************//**
    *
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is a space available to put
    *  requested data in transmit buffer.
    *
    *  \param string: pointer to the null terminated string array to be placed in the
    *   transmit buffer.
    *
    *******************************************************************************/
    void mUART_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            mUART_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: mUART_UartPutCRLF
    ****************************************************************************//**
    *
    *  Places byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) in the transmit buffer.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *
    *******************************************************************************/
    void mUART_UartPutCRLF(uint32 txDataByte)
    {
        mUART_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        mUART_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        mUART_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: mUARTSCB_UartEnableCts
        ****************************************************************************//**
        *
        *  Enables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void mUART_UartEnableCts(void)
        {
            mUART_UART_FLOW_CTRL_REG |= (uint32)  mUART_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: mUART_UartDisableCts
        ****************************************************************************//**
        *
        *  Disables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void mUART_UartDisableCts(void)
        {
            mUART_UART_FLOW_CTRL_REG &= (uint32) ~mUART_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: mUART_UartSetCtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of CTS input signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        * \param
        * polarity: Active polarity of CTS output signal.
        *   - mUART_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   - mUART_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        *******************************************************************************/
        void mUART_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                mUART_UART_FLOW_CTRL_REG |= (uint32)  mUART_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                mUART_UART_FLOW_CTRL_REG &= (uint32) ~mUART_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */


    /*******************************************************************************
    * Function Name: mUART_UartSendBreakBlocking
    ****************************************************************************//**
    *
    * Sends a break condition (logic low) of specified width on UART TX line.
    * Blocks until break is completed. Only call this function when UART TX FIFO
    * and shifter are empty.
    *
    * \param breakWidth
    * Width of break condition. Valid range is 4 to 16 bits.
    *
    * \note
    * Before sending break all UART TX interrupt sources are disabled. The state
    * of UART TX interrupt sources is restored before function returns.
    *
    * \sideeffect
    * If this function is called while there is data in the TX FIFO or shifter that
    * data will be shifted out in packets the size of breakWidth.
    *
    *******************************************************************************/
    void mUART_UartSendBreakBlocking(uint32 breakWidth)
    {
        uint32 txCtrlReg;
        uint32 txIntrReg;

        /* Disable all UART TX interrupt source and clear UART TX Done history */
        txIntrReg = mUART_GetTxInterruptMode();
        mUART_SetTxInterruptMode(0u);
        mUART_ClearTxInterruptSource(mUART_INTR_TX_UART_DONE);

        /* Store TX CTRL configuration */
        txCtrlReg = mUART_TX_CTRL_REG;

        /* Set break width */
        mUART_TX_CTRL_REG = (mUART_TX_CTRL_REG & (uint32) ~mUART_TX_CTRL_DATA_WIDTH_MASK) |
                                        mUART_GET_TX_CTRL_DATA_WIDTH(breakWidth);

        /* Generate break */
        mUART_TX_FIFO_WR_REG = 0u;

        /* Wait for break completion */
        while (0u == (mUART_GetTxInterruptSource() & mUART_INTR_TX_UART_DONE))
        {
        }

        /* Clear all UART TX interrupt sources to  */
        mUART_ClearTxInterruptSource(mUART_INTR_TX_ALL);

        /* Restore TX interrupt sources and data width */
        mUART_TX_CTRL_REG = txCtrlReg;
        mUART_SetTxInterruptMode(txIntrReg);
    }
#endif /* (mUART_UART_TX_DIRECTION) */


#if (mUART_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: mUART_UartSaveConfig
    ****************************************************************************//**
    *
    *  Clears and enables an interrupt on a falling edge of the Rx input. The GPIO
    *  interrupt does not track in the active mode, therefore requires to be
    *  cleared by this API.
    *
    *******************************************************************************/
    void mUART_UartSaveConfig(void)
    {
    #if (mUART_UART_RX_WAKEUP_IRQ)
        /* Set SKIP_START if requested (set by default). */
        if (0u != mUART_skipStart)
        {
            mUART_UART_RX_CTRL_REG |= (uint32)  mUART_UART_RX_CTRL_SKIP_START;
        }
        else
        {
            mUART_UART_RX_CTRL_REG &= (uint32) ~mUART_UART_RX_CTRL_SKIP_START;
        }

        /* Clear RX GPIO interrupt status and pending interrupt in NVIC because
        * falling edge on RX line occurs while UART communication in active mode.
        * Enable interrupt: next interrupt trigger should wakeup device.
        */
        mUART_CLEAR_UART_RX_WAKE_INTR;
        mUART_RxWakeClearPendingInt();
        mUART_RxWakeEnableInt();
    #endif /* (mUART_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: mUART_UartRestoreConfig
    ****************************************************************************//**
    *
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt
    *  remains active and triggers on every falling edge of the UART RX line.
    *
    *******************************************************************************/
    void mUART_UartRestoreConfig(void)
    {
    #if (mUART_UART_RX_WAKEUP_IRQ)
        /* Disable interrupt: no more triggers in active mode */
        mUART_RxWakeDisableInt();
    #endif /* (mUART_UART_RX_WAKEUP_IRQ) */
    }


    #if (mUART_UART_RX_WAKEUP_IRQ)
        /*******************************************************************************
        * Function Name: mUART_UART_WAKEUP_ISR
        ****************************************************************************//**
        *
        *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup
        *  event. This event is configured to trigger on a falling edge of the RX line.
        *
        *******************************************************************************/
        CY_ISR(mUART_UART_WAKEUP_ISR)
        {
        #ifdef mUART_UART_WAKEUP_ISR_ENTRY_CALLBACK
            mUART_UART_WAKEUP_ISR_EntryCallback();
        #endif /* mUART_UART_WAKEUP_ISR_ENTRY_CALLBACK */

            mUART_CLEAR_UART_RX_WAKE_INTR;

        #ifdef mUART_UART_WAKEUP_ISR_EXIT_CALLBACK
            mUART_UART_WAKEUP_ISR_ExitCallback();
        #endif /* mUART_UART_WAKEUP_ISR_EXIT_CALLBACK */
        }
    #endif /* (mUART_UART_RX_WAKEUP_IRQ) */
#endif /* (mUART_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
