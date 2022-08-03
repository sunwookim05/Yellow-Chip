/***************************************************************************//**
* \file mI2C_I2C.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  I2C mode.
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

#include "mI2C_PVT.h"
#include "mI2C_I2C_PVT.h"


/***************************************
*      I2C Private Vars
***************************************/

volatile uint8 mI2C_state;  /* Current state of I2C FSM */

#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    /* Constant configuration of I2C */
    const mI2C_I2C_INIT_STRUCT mI2C_configI2C =
    {
        mI2C_I2C_MODE,
        mI2C_I2C_OVS_FACTOR_LOW,
        mI2C_I2C_OVS_FACTOR_HIGH,
        mI2C_I2C_MEDIAN_FILTER_ENABLE,
        mI2C_I2C_SLAVE_ADDRESS,
        mI2C_I2C_SLAVE_ADDRESS_MASK,
        mI2C_I2C_ACCEPT_ADDRESS,
        mI2C_I2C_WAKE_ENABLE,
        mI2C_I2C_BYTE_MODE_ENABLE,
        mI2C_I2C_DATA_RATE,
        mI2C_I2C_ACCEPT_GENERAL_CALL,
    };

    /*******************************************************************************
    * Function Name: mI2C_I2CInit
    ****************************************************************************//**
    *
    *
    *  Configures the mI2C for I2C operation.
    *
    *  This function is intended specifically to be used when the mI2C 
    *  configuration is set to “Unconfigured mI2C” in the customizer. 
    *  After initializing the mI2C in I2C mode using this function, 
    *  the component can be enabled using the mI2C_Start() or 
    * mI2C_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration 
    *  settings. This structure contains the same information that would otherwise 
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of 
    *   fields. These fields match the selections available in the customizer. 
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void mI2C_I2CInit(const mI2C_I2C_INIT_STRUCT *config)
    {
        uint32 medianFilter;
        uint32 locEnableWake;

        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Configure pins */
            mI2C_SetPins(mI2C_SCB_MODE_I2C, mI2C_DUMMY_PARAM,
                                     mI2C_DUMMY_PARAM);

            /* Store internal configuration */
            mI2C_scbMode       = (uint8) mI2C_SCB_MODE_I2C;
            mI2C_scbEnableWake = (uint8) config->enableWake;
            mI2C_scbEnableIntr = (uint8) mI2C_SCB_IRQ_INTERNAL;

            mI2C_mode          = (uint8) config->mode;
            mI2C_acceptAddr    = (uint8) config->acceptAddr;

        #if (mI2C_CY_SCBIP_V0)
            /* Adjust SDA filter settings. Ticket ID#150521 */
            mI2C_SET_I2C_CFG_SDA_FILT_TRIM(mI2C_EC_AM_I2C_CFG_SDA_FILT_TRIM);
        #endif /* (mI2C_CY_SCBIP_V0) */

            /* Adjust AF and DF filter settings. Ticket ID#176179 */
            if (((mI2C_I2C_MODE_SLAVE != config->mode) &&
                 (config->dataRate <= mI2C_I2C_DATA_RATE_FS_MODE_MAX)) ||
                 (mI2C_I2C_MODE_SLAVE == config->mode))
            {
                /* AF = 1, DF = 0 */
                mI2C_I2C_CFG_ANALOG_FITER_ENABLE;
                medianFilter = mI2C_DIGITAL_FILTER_DISABLE;
            }
            else
            {
                /* AF = 0, DF = 1 */
                mI2C_I2C_CFG_ANALOG_FITER_DISABLE;
                medianFilter = mI2C_DIGITAL_FILTER_ENABLE;
            }

        #if (!mI2C_CY_SCBIP_V0)
            locEnableWake = (mI2C_I2C_MULTI_MASTER_SLAVE) ? (0u) : (config->enableWake);
        #else
            locEnableWake = config->enableWake;
        #endif /* (!mI2C_CY_SCBIP_V0) */

            /* Configure I2C interface */
            mI2C_CTRL_REG     = mI2C_GET_CTRL_BYTE_MODE  (config->enableByteMode) |
                                            mI2C_GET_CTRL_ADDR_ACCEPT(config->acceptAddr)     |
                                            mI2C_GET_CTRL_EC_AM_MODE (locEnableWake);

            mI2C_I2C_CTRL_REG = mI2C_GET_I2C_CTRL_HIGH_PHASE_OVS(config->oversampleHigh) |
                    mI2C_GET_I2C_CTRL_LOW_PHASE_OVS (config->oversampleLow)                          |
                    mI2C_GET_I2C_CTRL_S_GENERAL_IGNORE((uint32)(0u == config->acceptGeneralAddr))    |
                    mI2C_GET_I2C_CTRL_SL_MSTR_MODE  (config->mode);

            /* Configure RX direction */
            mI2C_RX_CTRL_REG      = mI2C_GET_RX_CTRL_MEDIAN(medianFilter) |
                                                mI2C_I2C_RX_CTRL;
            mI2C_RX_FIFO_CTRL_REG = mI2C_CLEAR_REG;

            /* Set default address and mask */
            mI2C_RX_MATCH_REG    = ((mI2C_I2C_SLAVE) ?
                                                (mI2C_GET_I2C_8BIT_ADDRESS(config->slaveAddr) |
                                                 mI2C_GET_RX_MATCH_MASK(config->slaveAddrMask)) :
                                                (mI2C_CLEAR_REG));


            /* Configure TX direction */
            mI2C_TX_CTRL_REG      = mI2C_I2C_TX_CTRL;
            mI2C_TX_FIFO_CTRL_REG = mI2C_CLEAR_REG;

            /* Configure interrupt with I2C handler but do not enable it */
            CyIntDisable    (mI2C_ISR_NUMBER);
            CyIntSetPriority(mI2C_ISR_NUMBER, mI2C_ISR_PRIORITY);
            (void) CyIntSetVector(mI2C_ISR_NUMBER, &mI2C_I2C_ISR);

            /* Configure interrupt sources */
        #if(!mI2C_CY_SCBIP_V1)
            mI2C_INTR_SPI_EC_MASK_REG = mI2C_NO_INTR_SOURCES;
        #endif /* (!mI2C_CY_SCBIP_V1) */

            mI2C_INTR_I2C_EC_MASK_REG = mI2C_NO_INTR_SOURCES;
            mI2C_INTR_RX_MASK_REG     = mI2C_NO_INTR_SOURCES;
            mI2C_INTR_TX_MASK_REG     = mI2C_NO_INTR_SOURCES;

            mI2C_INTR_SLAVE_MASK_REG  = ((mI2C_I2C_SLAVE) ?
                            (mI2C_GET_INTR_SLAVE_I2C_GENERAL(config->acceptGeneralAddr) |
                             mI2C_I2C_INTR_SLAVE_MASK) : (mI2C_CLEAR_REG));

            mI2C_INTR_MASTER_MASK_REG = mI2C_NO_INTR_SOURCES;

            /* Configure global variables */
            mI2C_state = mI2C_I2C_FSM_IDLE;

            /* Internal slave variables */
            mI2C_slStatus        = 0u;
            mI2C_slRdBufIndex    = 0u;
            mI2C_slWrBufIndex    = 0u;
            mI2C_slOverFlowCount = 0u;

            /* Internal master variables */
            mI2C_mstrStatus     = 0u;
            mI2C_mstrRdBufIndex = 0u;
            mI2C_mstrWrBufIndex = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: mI2C_I2CInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the I2C operation.
    *
    *******************************************************************************/
    void mI2C_I2CInit(void)
    {
    #if(mI2C_CY_SCBIP_V0)
        /* Adjust SDA filter settings. Ticket ID#150521 */
        mI2C_SET_I2C_CFG_SDA_FILT_TRIM(mI2C_EC_AM_I2C_CFG_SDA_FILT_TRIM);
    #endif /* (mI2C_CY_SCBIP_V0) */

        /* Adjust AF and DF filter settings. Ticket ID#176179 */
        mI2C_I2C_CFG_ANALOG_FITER_ENABLE_ADJ;

        /* Configure I2C interface */
        mI2C_CTRL_REG     = mI2C_I2C_DEFAULT_CTRL;
        mI2C_I2C_CTRL_REG = mI2C_I2C_DEFAULT_I2C_CTRL;

        /* Configure RX direction */
        mI2C_RX_CTRL_REG      = mI2C_I2C_DEFAULT_RX_CTRL;
        mI2C_RX_FIFO_CTRL_REG = mI2C_I2C_DEFAULT_RX_FIFO_CTRL;

        /* Set default address and mask */
        mI2C_RX_MATCH_REG     = mI2C_I2C_DEFAULT_RX_MATCH;

        /* Configure TX direction */
        mI2C_TX_CTRL_REG      = mI2C_I2C_DEFAULT_TX_CTRL;
        mI2C_TX_FIFO_CTRL_REG = mI2C_I2C_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with I2C handler but do not enable it */
        CyIntDisable    (mI2C_ISR_NUMBER);
        CyIntSetPriority(mI2C_ISR_NUMBER, mI2C_ISR_PRIORITY);
    #if(!mI2C_I2C_EXTERN_INTR_HANDLER)
        (void) CyIntSetVector(mI2C_ISR_NUMBER, &mI2C_I2C_ISR);
    #endif /* (mI2C_I2C_EXTERN_INTR_HANDLER) */

        /* Configure interrupt sources */
    #if(!mI2C_CY_SCBIP_V1)
        mI2C_INTR_SPI_EC_MASK_REG = mI2C_I2C_DEFAULT_INTR_SPI_EC_MASK;
    #endif /* (!mI2C_CY_SCBIP_V1) */

        mI2C_INTR_I2C_EC_MASK_REG = mI2C_I2C_DEFAULT_INTR_I2C_EC_MASK;
        mI2C_INTR_SLAVE_MASK_REG  = mI2C_I2C_DEFAULT_INTR_SLAVE_MASK;
        mI2C_INTR_MASTER_MASK_REG = mI2C_I2C_DEFAULT_INTR_MASTER_MASK;
        mI2C_INTR_RX_MASK_REG     = mI2C_I2C_DEFAULT_INTR_RX_MASK;
        mI2C_INTR_TX_MASK_REG     = mI2C_I2C_DEFAULT_INTR_TX_MASK;

        /* Configure global variables */
        mI2C_state = mI2C_I2C_FSM_IDLE;

    #if(mI2C_I2C_SLAVE)
        /* Internal slave variable */
        mI2C_slStatus        = 0u;
        mI2C_slRdBufIndex    = 0u;
        mI2C_slWrBufIndex    = 0u;
        mI2C_slOverFlowCount = 0u;
    #endif /* (mI2C_I2C_SLAVE) */

    #if(mI2C_I2C_MASTER)
    /* Internal master variable */
        mI2C_mstrStatus     = 0u;
        mI2C_mstrRdBufIndex = 0u;
        mI2C_mstrWrBufIndex = 0u;
    #endif /* (mI2C_I2C_MASTER) */
    }
#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: mI2C_I2CStop
****************************************************************************//**
*
*  Resets the I2C FSM into the default state.
*
*******************************************************************************/
void mI2C_I2CStop(void)
{
    /* Clear command registers because they keep assigned value after IP block was disabled */
    mI2C_I2C_MASTER_CMD_REG = 0u;
    mI2C_I2C_SLAVE_CMD_REG  = 0u;
    
    mI2C_state = mI2C_I2C_FSM_IDLE;
}


/*******************************************************************************
* Function Name: mI2C_I2CFwBlockReset
****************************************************************************//**
*
* Resets the scb IP block and I2C into the known state.
*
*******************************************************************************/
void mI2C_I2CFwBlockReset(void)
{
    /* Disable scb IP: stop respond to I2C traffic */
    mI2C_CTRL_REG &= (uint32) ~mI2C_CTRL_ENABLED;

    /* Clear command registers they are not cleared after scb IP is disabled */
    mI2C_I2C_MASTER_CMD_REG = 0u;
    mI2C_I2C_SLAVE_CMD_REG  = 0u;

    mI2C_DISABLE_AUTO_DATA;

    mI2C_SetTxInterruptMode(mI2C_NO_INTR_SOURCES);
    mI2C_SetRxInterruptMode(mI2C_NO_INTR_SOURCES);
    
#if(mI2C_CY_SCBIP_V0)
    /* Clear interrupt sources as they are not cleared after scb IP is disabled */
    mI2C_ClearTxInterruptSource    (mI2C_INTR_TX_ALL);
    mI2C_ClearRxInterruptSource    (mI2C_INTR_RX_ALL);
    mI2C_ClearSlaveInterruptSource (mI2C_INTR_SLAVE_ALL);
    mI2C_ClearMasterInterruptSource(mI2C_INTR_MASTER_ALL);
#endif /* (mI2C_CY_SCBIP_V0) */

    mI2C_state = mI2C_I2C_FSM_IDLE;

    /* Enable scb IP: start respond to I2C traffic */
    mI2C_CTRL_REG |= (uint32) mI2C_CTRL_ENABLED;
}


#if(mI2C_I2C_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: mI2C_I2CSaveConfig
    ****************************************************************************//**
    *
    *  Enables mI2C_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    *******************************************************************************/
    void mI2C_I2CSaveConfig(void)
    {
    #if (!mI2C_CY_SCBIP_V0)
        #if (mI2C_I2C_MULTI_MASTER_SLAVE_CONST && mI2C_I2C_WAKE_ENABLE_CONST)
            /* Enable externally clocked address match if it was not enabled before.
            * This applicable only for Multi-Master-Slave. Ticket ID#192742 */
            if (0u == (mI2C_CTRL_REG & mI2C_CTRL_EC_AM_MODE))
            {
                /* Enable external address match logic */
                mI2C_Stop();
                mI2C_CTRL_REG |= mI2C_CTRL_EC_AM_MODE;
                mI2C_Enable();
            }
        #endif /* (mI2C_I2C_MULTI_MASTER_SLAVE_CONST) */

        #if (mI2C_SCB_CLK_INTERNAL)
            /* Disable clock to internal address match logic. Ticket ID#187931 */
            mI2C_SCBCLK_Stop();
        #endif /* (mI2C_SCB_CLK_INTERNAL) */
    #endif /* (!mI2C_CY_SCBIP_V0) */

        mI2C_SetI2CExtClkInterruptMode(mI2C_INTR_I2C_EC_WAKE_UP);
    }


    /*******************************************************************************
    * Function Name: mI2C_I2CRestoreConfig
    ****************************************************************************//**
    *
    *  Disables mI2C_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    *******************************************************************************/
    void mI2C_I2CRestoreConfig(void)
    {
        /* Disable wakeup interrupt on address match */
        mI2C_SetI2CExtClkInterruptMode(mI2C_NO_INTR_SOURCES);

    #if (!mI2C_CY_SCBIP_V0)
        #if (mI2C_SCB_CLK_INTERNAL)
            /* Enable clock to internal address match logic. Ticket ID#187931 */
            mI2C_SCBCLK_Start();
        #endif /* (mI2C_SCB_CLK_INTERNAL) */
    #endif /* (!mI2C_CY_SCBIP_V0) */
    }
#endif /* (mI2C_I2C_WAKE_ENABLE_CONST) */


/* [] END OF FILE */
