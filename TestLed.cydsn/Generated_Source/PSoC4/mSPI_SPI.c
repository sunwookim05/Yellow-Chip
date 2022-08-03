/***************************************************************************//**
* \file mSPI_SPI.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  SPI mode.
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

#include "mSPI_PVT.h"
#include "mSPI_SPI_UART_PVT.h"

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const mSPI_SPI_INIT_STRUCT mSPI_configSpi =
    {
        mSPI_SPI_MODE,
        mSPI_SPI_SUB_MODE,
        mSPI_SPI_CLOCK_MODE,
        mSPI_SPI_OVS_FACTOR,
        mSPI_SPI_MEDIAN_FILTER_ENABLE,
        mSPI_SPI_LATE_MISO_SAMPLE_ENABLE,
        mSPI_SPI_WAKE_ENABLE,
        mSPI_SPI_RX_DATA_BITS_NUM,
        mSPI_SPI_TX_DATA_BITS_NUM,
        mSPI_SPI_BITS_ORDER,
        mSPI_SPI_TRANSFER_SEPARATION,
        0u,
        NULL,
        0u,
        NULL,
        (uint32) mSPI_SCB_IRQ_INTERNAL,
        mSPI_SPI_INTR_RX_MASK,
        mSPI_SPI_RX_TRIGGER_LEVEL,
        mSPI_SPI_INTR_TX_MASK,
        mSPI_SPI_TX_TRIGGER_LEVEL,
        (uint8) mSPI_SPI_BYTE_MODE_ENABLE,
        (uint8) mSPI_SPI_FREE_RUN_SCLK_ENABLE,
        (uint8) mSPI_SPI_SS_POLARITY
    };


    /*******************************************************************************
    * Function Name: mSPI_SpiInit
    ****************************************************************************//**
    *
    *  Configures the mSPI for SPI operation.
    *
    *  This function is intended specifically to be used when the mSPI 
    *  configuration is set to “Unconfigured mSPI” in the customizer. 
    *  After initializing the mSPI in SPI mode using this function, 
    *  the component can be enabled using the mSPI_Start() or 
    * mSPI_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration 
    *  settings. This structure contains the same information that would otherwise 
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of 
    *   fields. These fields match the selections available in the customizer. 
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void mSPI_SpiInit(const mSPI_SPI_INIT_STRUCT *config)
    {
        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Configure pins */
            mSPI_SetPins(mSPI_SCB_MODE_SPI, config->mode, mSPI_DUMMY_PARAM);

            /* Store internal configuration */
            mSPI_scbMode       = (uint8) mSPI_SCB_MODE_SPI;
            mSPI_scbEnableWake = (uint8) config->enableWake;
            mSPI_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            mSPI_rxBuffer      =         config->rxBuffer;
            mSPI_rxDataBits    = (uint8) config->rxDataBits;
            mSPI_rxBufferSize  =         config->rxBufferSize;

            /* Set TX direction internal variables */
            mSPI_txBuffer      =         config->txBuffer;
            mSPI_txDataBits    = (uint8) config->txDataBits;
            mSPI_txBufferSize  =         config->txBufferSize;

            /* Configure SPI interface */
            mSPI_CTRL_REG     = mSPI_GET_CTRL_OVS(config->oversample)           |
                                            mSPI_GET_CTRL_BYTE_MODE(config->enableByteMode) |
                                            mSPI_GET_CTRL_EC_AM_MODE(config->enableWake)    |
                                            mSPI_CTRL_SPI;

            mSPI_SPI_CTRL_REG = mSPI_GET_SPI_CTRL_CONTINUOUS    (config->transferSeperation)  |
                                            mSPI_GET_SPI_CTRL_SELECT_PRECEDE(config->submode &
                                                                          mSPI_SPI_MODE_TI_PRECEDES_MASK) |
                                            mSPI_GET_SPI_CTRL_SCLK_MODE     (config->sclkMode)            |
                                            mSPI_GET_SPI_CTRL_LATE_MISO_SAMPLE(config->enableLateSampling)|
                                            mSPI_GET_SPI_CTRL_SCLK_CONTINUOUS(config->enableFreeRunSclk)  |
                                            mSPI_GET_SPI_CTRL_SSEL_POLARITY (config->polaritySs)          |
                                            mSPI_GET_SPI_CTRL_SUB_MODE      (config->submode)             |
                                            mSPI_GET_SPI_CTRL_MASTER_MODE   (config->mode);

            /* Configure RX direction */
            mSPI_RX_CTRL_REG     =  mSPI_GET_RX_CTRL_DATA_WIDTH(config->rxDataBits)         |
                                                mSPI_GET_RX_CTRL_BIT_ORDER (config->bitOrder)           |
                                                mSPI_GET_RX_CTRL_MEDIAN    (config->enableMedianFilter) |
                                                mSPI_SPI_RX_CTRL;

            mSPI_RX_FIFO_CTRL_REG = mSPI_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure TX direction */
            mSPI_TX_CTRL_REG      = mSPI_GET_TX_CTRL_DATA_WIDTH(config->txDataBits) |
                                                mSPI_GET_TX_CTRL_BIT_ORDER (config->bitOrder)   |
                                                mSPI_SPI_TX_CTRL;

            mSPI_TX_FIFO_CTRL_REG = mSPI_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

            /* Configure interrupt with SPI handler but do not enable it */
            CyIntDisable    (mSPI_ISR_NUMBER);
            CyIntSetPriority(mSPI_ISR_NUMBER, mSPI_ISR_PRIORITY);
            (void) CyIntSetVector(mSPI_ISR_NUMBER, &mSPI_SPI_UART_ISR);

            /* Configure interrupt sources */
            mSPI_INTR_I2C_EC_MASK_REG = mSPI_NO_INTR_SOURCES;
            mSPI_INTR_SPI_EC_MASK_REG = mSPI_NO_INTR_SOURCES;
            mSPI_INTR_SLAVE_MASK_REG  = mSPI_GET_SPI_INTR_SLAVE_MASK(config->rxInterruptMask);
            mSPI_INTR_MASTER_MASK_REG = mSPI_GET_SPI_INTR_MASTER_MASK(config->txInterruptMask);
            mSPI_INTR_RX_MASK_REG     = mSPI_GET_SPI_INTR_RX_MASK(config->rxInterruptMask);
            mSPI_INTR_TX_MASK_REG     = mSPI_GET_SPI_INTR_TX_MASK(config->txInterruptMask);
            
            /* Configure TX interrupt sources to restore. */
            mSPI_IntrTxMask = LO16(mSPI_INTR_TX_MASK_REG);

            /* Set active SS0 */
            mSPI_SpiSetActiveSlaveSelect(mSPI_SPI_SLAVE_SELECT0);

            /* Clear RX buffer indexes */
            mSPI_rxBufferHead     = 0u;
            mSPI_rxBufferTail     = 0u;
            mSPI_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            mSPI_txBufferHead = 0u;
            mSPI_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: mSPI_SpiInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the SPI operation.
    *
    *******************************************************************************/
    void mSPI_SpiInit(void)
    {
        /* Configure SPI interface */
        mSPI_CTRL_REG     = mSPI_SPI_DEFAULT_CTRL;
        mSPI_SPI_CTRL_REG = mSPI_SPI_DEFAULT_SPI_CTRL;

        /* Configure TX and RX direction */
        mSPI_RX_CTRL_REG      = mSPI_SPI_DEFAULT_RX_CTRL;
        mSPI_RX_FIFO_CTRL_REG = mSPI_SPI_DEFAULT_RX_FIFO_CTRL;

        /* Configure TX and RX direction */
        mSPI_TX_CTRL_REG      = mSPI_SPI_DEFAULT_TX_CTRL;
        mSPI_TX_FIFO_CTRL_REG = mSPI_SPI_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with SPI handler but do not enable it */
    #if(mSPI_SCB_IRQ_INTERNAL)
            CyIntDisable    (mSPI_ISR_NUMBER);
            CyIntSetPriority(mSPI_ISR_NUMBER, mSPI_ISR_PRIORITY);
            (void) CyIntSetVector(mSPI_ISR_NUMBER, &mSPI_SPI_UART_ISR);
    #endif /* (mSPI_SCB_IRQ_INTERNAL) */

        /* Configure interrupt sources */
        mSPI_INTR_I2C_EC_MASK_REG = mSPI_SPI_DEFAULT_INTR_I2C_EC_MASK;
        mSPI_INTR_SPI_EC_MASK_REG = mSPI_SPI_DEFAULT_INTR_SPI_EC_MASK;
        mSPI_INTR_SLAVE_MASK_REG  = mSPI_SPI_DEFAULT_INTR_SLAVE_MASK;
        mSPI_INTR_MASTER_MASK_REG = mSPI_SPI_DEFAULT_INTR_MASTER_MASK;
        mSPI_INTR_RX_MASK_REG     = mSPI_SPI_DEFAULT_INTR_RX_MASK;
        mSPI_INTR_TX_MASK_REG     = mSPI_SPI_DEFAULT_INTR_TX_MASK;

        /* Configure TX interrupt sources to restore. */
        mSPI_IntrTxMask = LO16(mSPI_INTR_TX_MASK_REG);
            
        /* Set active SS0 for master */
    #if (mSPI_SPI_MASTER_CONST)
        mSPI_SpiSetActiveSlaveSelect(mSPI_SPI_SLAVE_SELECT0);
    #endif /* (mSPI_SPI_MASTER_CONST) */

    #if(mSPI_INTERNAL_RX_SW_BUFFER_CONST)
        mSPI_rxBufferHead     = 0u;
        mSPI_rxBufferTail     = 0u;
        mSPI_rxBufferOverflow = 0u;
    #endif /* (mSPI_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(mSPI_INTERNAL_TX_SW_BUFFER_CONST)
        mSPI_txBufferHead = 0u;
        mSPI_txBufferTail = 0u;
    #endif /* (mSPI_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: mSPI_SpiPostEnable
****************************************************************************//**
*
*  Restores HSIOM settings for the SPI master output pins (SCLK and/or SS0-SS3) 
*  to be controlled by the SCB SPI.
*
*******************************************************************************/
void mSPI_SpiPostEnable(void)
{
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    if (mSPI_CHECK_SPI_MASTER)
    {
    #if (mSPI_CTS_SCLK_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_CTS_SCLK_HSIOM_REG, mSPI_CTS_SCLK_HSIOM_MASK,
                                       mSPI_CTS_SCLK_HSIOM_POS, mSPI_CTS_SCLK_HSIOM_SEL_SPI);
    #endif /* (mSPI_CTS_SCLK_PIN) */

    #if (mSPI_RTS_SS0_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_RTS_SS0_HSIOM_REG, mSPI_RTS_SS0_HSIOM_MASK,
                                       mSPI_RTS_SS0_HSIOM_POS, mSPI_RTS_SS0_HSIOM_SEL_SPI);
    #endif /* (mSPI_RTS_SS0_PIN) */

    #if (mSPI_SS1_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS1_HSIOM_REG, mSPI_SS1_HSIOM_MASK,
                                       mSPI_SS1_HSIOM_POS, mSPI_SS1_HSIOM_SEL_SPI);
    #endif /* (mSPI_SS1_PIN) */

    #if (mSPI_SS2_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS2_HSIOM_REG, mSPI_SS2_HSIOM_MASK,
                                       mSPI_SS2_HSIOM_POS, mSPI_SS2_HSIOM_SEL_SPI);
    #endif /* (mSPI_SS2_PIN) */

    #if (mSPI_SS3_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS3_HSIOM_REG, mSPI_SS3_HSIOM_MASK,
                                       mSPI_SS3_HSIOM_POS, mSPI_SS3_HSIOM_SEL_SPI);
    #endif /* (mSPI_SS3_PIN) */
    }

#else

    #if (mSPI_SPI_MASTER_SCLK_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SCLK_M_HSIOM_REG, mSPI_SCLK_M_HSIOM_MASK,
                                       mSPI_SCLK_M_HSIOM_POS, mSPI_SCLK_M_HSIOM_SEL_SPI);
    #endif /* (mSPI_MISO_SDA_TX_PIN_PIN) */

    #if (mSPI_SPI_MASTER_SS0_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS0_M_HSIOM_REG, mSPI_SS0_M_HSIOM_MASK,
                                       mSPI_SS0_M_HSIOM_POS, mSPI_SS0_M_HSIOM_SEL_SPI);
    #endif /* (mSPI_SPI_MASTER_SS0_PIN) */

    #if (mSPI_SPI_MASTER_SS1_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS1_M_HSIOM_REG, mSPI_SS1_M_HSIOM_MASK,
                                       mSPI_SS1_M_HSIOM_POS, mSPI_SS1_M_HSIOM_SEL_SPI);
    #endif /* (mSPI_SPI_MASTER_SS1_PIN) */

    #if (mSPI_SPI_MASTER_SS2_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS2_M_HSIOM_REG, mSPI_SS2_M_HSIOM_MASK,
                                       mSPI_SS2_M_HSIOM_POS, mSPI_SS2_M_HSIOM_SEL_SPI);
    #endif /* (mSPI_SPI_MASTER_SS2_PIN) */

    #if (mSPI_SPI_MASTER_SS3_PIN)
        /* Set SCB SPI to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS3_M_HSIOM_REG, mSPI_SS3_M_HSIOM_MASK,
                                       mSPI_SS3_M_HSIOM_POS, mSPI_SS3_M_HSIOM_SEL_SPI);
    #endif /* (mSPI_SPI_MASTER_SS3_PIN) */

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Restore TX interrupt sources. */
    mSPI_SetTxInterruptMode(mSPI_IntrTxMask);
}


/*******************************************************************************
* Function Name: mSPI_SpiStop
****************************************************************************//**
*
*  Changes the HSIOM settings for the SPI master output pins 
*  (SCLK and/or SS0-SS3) to keep them inactive after the block is disabled. 
*  The output pins are controlled by the GPIO data register.
*
*******************************************************************************/
void mSPI_SpiStop(void)
{
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    if (mSPI_CHECK_SPI_MASTER)
    {
    #if (mSPI_CTS_SCLK_PIN)
        /* Set output pin state after block is disabled */
        mSPI_uart_cts_spi_sclk_Write(mSPI_GET_SPI_SCLK_INACTIVE);

        /* Set GPIO to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_CTS_SCLK_HSIOM_REG, mSPI_CTS_SCLK_HSIOM_MASK,
                                       mSPI_CTS_SCLK_HSIOM_POS, mSPI_CTS_SCLK_HSIOM_SEL_GPIO);
    #endif /* (mSPI_uart_cts_spi_sclk_PIN) */

    #if (mSPI_RTS_SS0_PIN)
        /* Set output pin state after block is disabled */
        mSPI_uart_rts_spi_ss0_Write(mSPI_GET_SPI_SS0_INACTIVE);

        /* Set GPIO to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_RTS_SS0_HSIOM_REG, mSPI_RTS_SS0_HSIOM_MASK,
                                       mSPI_RTS_SS0_HSIOM_POS, mSPI_RTS_SS0_HSIOM_SEL_GPIO);
    #endif /* (mSPI_uart_rts_spi_ss0_PIN) */

    #if (mSPI_SS1_PIN)
        /* Set output pin state after block is disabled */
        mSPI_spi_ss1_Write(mSPI_GET_SPI_SS1_INACTIVE);

        /* Set GPIO to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS1_HSIOM_REG, mSPI_SS1_HSIOM_MASK,
                                       mSPI_SS1_HSIOM_POS, mSPI_SS1_HSIOM_SEL_GPIO);
    #endif /* (mSPI_SS1_PIN) */

    #if (mSPI_SS2_PIN)
        /* Set output pin state after block is disabled */
        mSPI_spi_ss2_Write(mSPI_GET_SPI_SS2_INACTIVE);

        /* Set GPIO to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS2_HSIOM_REG, mSPI_SS2_HSIOM_MASK,
                                       mSPI_SS2_HSIOM_POS, mSPI_SS2_HSIOM_SEL_GPIO);
    #endif /* (mSPI_SS2_PIN) */

    #if (mSPI_SS3_PIN)
        /* Set output pin state after block is disabled */
        mSPI_spi_ss3_Write(mSPI_GET_SPI_SS3_INACTIVE);

        /* Set GPIO to drive output pin */
        mSPI_SET_HSIOM_SEL(mSPI_SS3_HSIOM_REG, mSPI_SS3_HSIOM_MASK,
                                       mSPI_SS3_HSIOM_POS, mSPI_SS3_HSIOM_SEL_GPIO);
    #endif /* (mSPI_SS3_PIN) */
    
        /* Store TX interrupt sources (exclude level triggered) for master. */
        mSPI_IntrTxMask = LO16(mSPI_GetTxInterruptMode() & mSPI_INTR_SPIM_TX_RESTORE);
    }
    else
    {
        /* Store TX interrupt sources (exclude level triggered) for slave. */
        mSPI_IntrTxMask = LO16(mSPI_GetTxInterruptMode() & mSPI_INTR_SPIS_TX_RESTORE);
    }

#else

#if (mSPI_SPI_MASTER_SCLK_PIN)
    /* Set output pin state after block is disabled */
    mSPI_sclk_m_Write(mSPI_GET_SPI_SCLK_INACTIVE);

    /* Set GPIO to drive output pin */
    mSPI_SET_HSIOM_SEL(mSPI_SCLK_M_HSIOM_REG, mSPI_SCLK_M_HSIOM_MASK,
                                   mSPI_SCLK_M_HSIOM_POS, mSPI_SCLK_M_HSIOM_SEL_GPIO);
#endif /* (mSPI_MISO_SDA_TX_PIN_PIN) */

#if (mSPI_SPI_MASTER_SS0_PIN)
    /* Set output pin state after block is disabled */
    mSPI_ss0_m_Write(mSPI_GET_SPI_SS0_INACTIVE);

    /* Set GPIO to drive output pin */
    mSPI_SET_HSIOM_SEL(mSPI_SS0_M_HSIOM_REG, mSPI_SS0_M_HSIOM_MASK,
                                   mSPI_SS0_M_HSIOM_POS, mSPI_SS0_M_HSIOM_SEL_GPIO);
#endif /* (mSPI_SPI_MASTER_SS0_PIN) */

#if (mSPI_SPI_MASTER_SS1_PIN)
    /* Set output pin state after block is disabled */
    mSPI_ss1_m_Write(mSPI_GET_SPI_SS1_INACTIVE);

    /* Set GPIO to drive output pin */
    mSPI_SET_HSIOM_SEL(mSPI_SS1_M_HSIOM_REG, mSPI_SS1_M_HSIOM_MASK,
                                   mSPI_SS1_M_HSIOM_POS, mSPI_SS1_M_HSIOM_SEL_GPIO);
#endif /* (mSPI_SPI_MASTER_SS1_PIN) */

#if (mSPI_SPI_MASTER_SS2_PIN)
    /* Set output pin state after block is disabled */
    mSPI_ss2_m_Write(mSPI_GET_SPI_SS2_INACTIVE);

    /* Set GPIO to drive output pin */
    mSPI_SET_HSIOM_SEL(mSPI_SS2_M_HSIOM_REG, mSPI_SS2_M_HSIOM_MASK,
                                   mSPI_SS2_M_HSIOM_POS, mSPI_SS2_M_HSIOM_SEL_GPIO);
#endif /* (mSPI_SPI_MASTER_SS2_PIN) */

#if (mSPI_SPI_MASTER_SS3_PIN)
    /* Set output pin state after block is disabled */
    mSPI_ss3_m_Write(mSPI_GET_SPI_SS3_INACTIVE);

    /* Set GPIO to drive output pin */
    mSPI_SET_HSIOM_SEL(mSPI_SS3_M_HSIOM_REG, mSPI_SS3_M_HSIOM_MASK,
                                   mSPI_SS3_M_HSIOM_POS, mSPI_SS3_M_HSIOM_SEL_GPIO);
#endif /* (mSPI_SPI_MASTER_SS3_PIN) */

    #if (mSPI_SPI_MASTER_CONST)
        /* Store TX interrupt sources (exclude level triggered). */
        mSPI_IntrTxMask = LO16(mSPI_GetTxInterruptMode() & mSPI_INTR_SPIM_TX_RESTORE);
    #else
        /* Store TX interrupt sources (exclude level triggered). */
        mSPI_IntrTxMask = LO16(mSPI_GetTxInterruptMode() & mSPI_INTR_SPIS_TX_RESTORE);
    #endif /* (mSPI_SPI_MASTER_CONST) */

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (mSPI_SPI_MASTER_CONST)
    /*******************************************************************************
    * Function Name: mSPI_SetActiveSlaveSelect
    ****************************************************************************//**
    *
    *  Selects one of the four slave select lines to be active during the transfer.
    *  After initialization the active slave select line is 0.
    *  The component should be in one of the following states to change the active
    *  slave select signal source correctly:
    *   - The component is disabled.
    *   - The component has completed transfer (TX FIFO is empty and the
    *     SCB_INTR_MASTER_SPI_DONE status is set).
    *
    *  This function does not check that these conditions are met.
    *  This function is only applicable to SPI Master mode of operation.
    *
    *  \param slaveSelect: slave select line which will be active while the following
    *   transfer.
    *   - mSPI_SPI_SLAVE_SELECT0 - Slave select 0.
    *   - mSPI_SPI_SLAVE_SELECT1 - Slave select 1.
    *   - mSPI_SPI_SLAVE_SELECT2 - Slave select 2.
    *   - mSPI_SPI_SLAVE_SELECT3 - Slave select 3.
    *
    *******************************************************************************/
    void mSPI_SpiSetActiveSlaveSelect(uint32 slaveSelect)
    {
        uint32 spiCtrl;

        spiCtrl = mSPI_SPI_CTRL_REG;

        spiCtrl &= (uint32) ~mSPI_SPI_CTRL_SLAVE_SELECT_MASK;
        spiCtrl |= (uint32)  mSPI_GET_SPI_CTRL_SS(slaveSelect);

        mSPI_SPI_CTRL_REG = spiCtrl;
    }
#endif /* (mSPI_SPI_MASTER_CONST) */


#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: mSPI_SpiSetSlaveSelectPolarity
    ****************************************************************************//**
    *
    *  Sets active polarity for slave select line.
    *  The component should be in one of the following states to change the active
    *  slave select signal source correctly:
    *   - The component is disabled.
    *   - The component has completed transfer.
    *  
    *  This function does not check that these conditions are met.
    *
    *  \param slaveSelect: slave select line to change active polarity.
    *   - mSPI_SPI_SLAVE_SELECT0 - Slave select 0.
    *   - mSPI_SPI_SLAVE_SELECT1 - Slave select 1.
    *   - mSPI_SPI_SLAVE_SELECT2 - Slave select 2.
    *   - mSPI_SPI_SLAVE_SELECT3 - Slave select 3.
    *
    *  \param polarity: active polarity of slave select line.
    *   - mSPI_SPI_SS_ACTIVE_LOW  - Slave select is active low.
    *   - mSPI_SPI_SS_ACTIVE_HIGH - Slave select is active high.
    *
    *******************************************************************************/
    void mSPI_SpiSetSlaveSelectPolarity(uint32 slaveSelect, uint32 polarity)
    {
        uint32 ssPolarity;

        /* Get position of the polarity bit associated with slave select line */
        ssPolarity = mSPI_GET_SPI_CTRL_SSEL_POLARITY((uint32) 1u << slaveSelect);

        if (0u != polarity)
        {
            mSPI_SPI_CTRL_REG |= (uint32)  ssPolarity;
        }
        else
        {
            mSPI_SPI_CTRL_REG &= (uint32) ~ssPolarity;
        }
    }
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */


#if(mSPI_SPI_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: mSPI_SpiSaveConfig
    ****************************************************************************//**
    *
    *  Clears INTR_SPI_EC.WAKE_UP and enables it. This interrupt
    *  source triggers when the master assigns the SS line and wakes up the device.
    *
    *******************************************************************************/
    void mSPI_SpiSaveConfig(void)
    {
        /* Clear and enable SPI wakeup interrupt source */
        mSPI_ClearSpiExtClkInterruptSource(mSPI_INTR_SPI_EC_WAKE_UP);
        mSPI_SetSpiExtClkInterruptMode(mSPI_INTR_SPI_EC_WAKE_UP);
    }


    /*******************************************************************************
    * Function Name: mSPI_SpiRestoreConfig
    ****************************************************************************//**
    *
    *  Disables the INTR_SPI_EC.WAKE_UP interrupt source. After wakeup
    *  slave does not drive the MISO line and the master receives 0xFF.
    *
    *******************************************************************************/
    void mSPI_SpiRestoreConfig(void)
    {
        /* Disable SPI wakeup interrupt source */
        mSPI_SetSpiExtClkInterruptMode(mSPI_NO_INTR_SOURCES);
    }
#endif /* (mSPI_SPI_WAKE_ENABLE_CONST) */


/* [] END OF FILE */
