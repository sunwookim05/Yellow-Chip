/***************************************************************************//**
* \file mSPI.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component.
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

#if (mSPI_SCB_MODE_I2C_INC)
    #include "mSPI_I2C_PVT.h"
#endif /* (mSPI_SCB_MODE_I2C_INC) */

#if (mSPI_SCB_MODE_EZI2C_INC)
    #include "mSPI_EZI2C_PVT.h"
#endif /* (mSPI_SCB_MODE_EZI2C_INC) */

#if (mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC)
    #include "mSPI_SPI_UART_PVT.h"
#endif /* (mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 mSPI_scbMode = mSPI_SCB_MODE_UNCONFIG;
    uint8 mSPI_scbEnableWake;
    uint8 mSPI_scbEnableIntr;

    /* I2C configuration variables */
    uint8 mSPI_mode;
    uint8 mSPI_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * mSPI_rxBuffer;
    uint8  mSPI_rxDataBits;
    uint32 mSPI_rxBufferSize;

    volatile uint8 * mSPI_txBuffer;
    uint8  mSPI_txDataBits;
    uint32 mSPI_txBufferSize;

    /* EZI2C configuration variables */
    uint8 mSPI_numberOfAddr;
    uint8 mSPI_subAddrSize;
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** mSPI_initVar indicates whether the mSPI 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the mSPI_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  mSPI_Init() function can be called before the 
*  mSPI_Start() or mSPI_Enable() function.
*/
uint8 mSPI_initVar = 0u;


#if (! (mSPI_SCB_MODE_I2C_CONST_CFG || \
        mSPI_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * mSPI_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent mSPI_Enable() call.
    */
    uint16 mSPI_IntrTxMask = 0u;
#endif /* (! (mSPI_SCB_MODE_I2C_CONST_CFG || \
              mSPI_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (mSPI_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER)
    void (*mSPI_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER) */
#endif /* (mSPI_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void mSPI_ScbEnableIntr(void);
static void mSPI_ScbModeStop(void);
static void mSPI_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: mSPI_Init
****************************************************************************//**
*
*  Initializes the mSPI component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  mSPI_I2CInit, mSPI_SpiInit, 
*  mSPI_UartInit or mSPI_EzI2CInit.
*
*******************************************************************************/
void mSPI_Init(void)
{
#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    if (mSPI_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        mSPI_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (mSPI_SCB_MODE_I2C_CONST_CFG)
    mSPI_I2CInit();

#elif (mSPI_SCB_MODE_SPI_CONST_CFG)
    mSPI_SpiInit();

#elif (mSPI_SCB_MODE_UART_CONST_CFG)
    mSPI_UartInit();

#elif (mSPI_SCB_MODE_EZI2C_CONST_CFG)
    mSPI_EzI2CInit();

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mSPI_Enable
****************************************************************************//**
*
*  Enables mSPI component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  mSPI_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The mSPI configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured mSPI”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void mSPI_Enable(void)
{
#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!mSPI_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        mSPI_CTRL_REG |= mSPI_CTRL_ENABLED;

        mSPI_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        mSPI_ScbModePostEnable();
    }
#else
    mSPI_CTRL_REG |= mSPI_CTRL_ENABLED;

    mSPI_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    mSPI_ScbModePostEnable();
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mSPI_Start
****************************************************************************//**
*
*  Invokes mSPI_Init() and mSPI_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  mSPI_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void mSPI_Start(void)
{
    if (0u == mSPI_initVar)
    {
        mSPI_Init();
        mSPI_initVar = 1u; /* Component was initialized */
    }

    mSPI_Enable();
}


/*******************************************************************************
* Function Name: mSPI_Stop
****************************************************************************//**
*
*  Disables the mSPI component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function mSPI_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void mSPI_Stop(void)
{
#if (mSPI_SCB_IRQ_INTERNAL)
    mSPI_DisableInt();
#endif /* (mSPI_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    mSPI_ScbModeStop();

    /* Disable SCB IP */
    mSPI_CTRL_REG &= (uint32) ~mSPI_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    mSPI_SetTxInterruptMode(mSPI_NO_INTR_SOURCES);

#if (mSPI_SCB_IRQ_INTERNAL)
    mSPI_ClearPendingInt();
#endif /* (mSPI_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: mSPI_SetRxFifoLevel
****************************************************************************//**
*
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
*  \param level: Level in the RX FIFO to generate RX level interrupt.
*   The range of valid level values is between 0 and RX FIFO depth - 1.
*
*******************************************************************************/
void mSPI_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = mSPI_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~mSPI_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (mSPI_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    mSPI_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: mSPI_SetTxFifoLevel
****************************************************************************//**
*
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has less entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
*  \param level: Level in the TX FIFO to generate TX level interrupt.
*   The range of valid level values is between 0 and TX FIFO depth - 1.
*
*******************************************************************************/
void mSPI_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = mSPI_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~mSPI_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (mSPI_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    mSPI_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (mSPI_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: mSPI_SetCustomInterruptHandler
    ****************************************************************************//**
    *
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    *  \param func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    *******************************************************************************/
    void mSPI_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER)
        mSPI_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER) */
    }
#endif /* (mSPI_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: mSPI_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void mSPI_ScbEnableIntr(void)
{
#if (mSPI_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != mSPI_scbEnableIntr)
        {
            mSPI_EnableInt();
        }

    #else
        mSPI_EnableInt();

    #endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (mSPI_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: mSPI_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void mSPI_ScbModePostEnable(void)
{
#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!mSPI_CY_SCBIP_V1)
    if (mSPI_SCB_MODE_SPI_RUNTM_CFG)
    {
        mSPI_SpiPostEnable();
    }
    else if (mSPI_SCB_MODE_UART_RUNTM_CFG)
    {
        mSPI_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!mSPI_CY_SCBIP_V1) */

#elif (mSPI_SCB_MODE_SPI_CONST_CFG)
    mSPI_SpiPostEnable();

#elif (mSPI_SCB_MODE_UART_CONST_CFG)
    mSPI_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mSPI_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void mSPI_ScbModeStop(void)
{
#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    if (mSPI_SCB_MODE_I2C_RUNTM_CFG)
    {
        mSPI_I2CStop();
    }
    else if (mSPI_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        mSPI_EzI2CStop();
    }
#if (!mSPI_CY_SCBIP_V1)
    else if (mSPI_SCB_MODE_SPI_RUNTM_CFG)
    {
        mSPI_SpiStop();
    }
    else if (mSPI_SCB_MODE_UART_RUNTM_CFG)
    {
        mSPI_UartStop();
    }
#endif /* (!mSPI_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (mSPI_SCB_MODE_I2C_CONST_CFG)
    mSPI_I2CStop();

#elif (mSPI_SCB_MODE_EZI2C_CONST_CFG)
    mSPI_EzI2CStop();

#elif (mSPI_SCB_MODE_SPI_CONST_CFG)
    mSPI_SpiStop();

#elif (mSPI_SCB_MODE_UART_CONST_CFG)
    mSPI_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: mSPI_SetPins
    ****************************************************************************//**
    *
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    *  \param mode:      Mode of SCB operation.
    *  \param subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  \param uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    *******************************************************************************/
    void mSPI_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[mSPI_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!mSPI_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!mSPI_CY_SCBIP_V1) */
        
        uint32 hsiomSel[mSPI_SCB_PINS_NUMBER] = 
        {
            mSPI_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            mSPI_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (mSPI_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (mSPI_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < mSPI_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = mSPI_PIN_DM_ALG_HIZ;
        }

        if ((mSPI_SCB_MODE_I2C   == mode) ||
            (mSPI_SCB_MODE_EZI2C == mode))
        {
        #if (mSPI_RX_SCL_MOSI_PIN)
            hsiomSel[mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_OD_LO;
        #elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX] = mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_OD_LO;
        #else
        #endif /* (mSPI_RX_SCL_MOSI_PIN) */
        
        #if (mSPI_TX_SDA_MISO_PIN)
            hsiomSel[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_PIN_DM_OD_LO;
        #endif /* (mSPI_TX_SDA_MISO_PIN) */
        }
    #if (!mSPI_CY_SCBIP_V1)
        else if (mSPI_SCB_MODE_SPI == mode)
        {
        #if (mSPI_RX_SCL_MOSI_PIN)
            hsiomSel[mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX] = mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (mSPI_RX_SCL_MOSI_PIN) */
        
        #if (mSPI_TX_SDA_MISO_PIN)
            hsiomSel[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (mSPI_TX_SDA_MISO_PIN) */
        
        #if (mSPI_CTS_SCLK_PIN)
            hsiomSel[mSPI_CTS_SCLK_PIN_INDEX] = mSPI_CTS_SCLK_HSIOM_SEL_SPI;
        #endif /* (mSPI_CTS_SCLK_PIN) */

            if (mSPI_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
                pinsDm[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsDm[mSPI_CTS_SCLK_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;

            #if (mSPI_RTS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[mSPI_RTS_SS0_PIN_INDEX] = mSPI_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm  [mSPI_RTS_SS0_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
            #endif /* (mSPI_RTS_SS0_PIN) */

            #if (mSPI_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= mSPI_TX_SDA_MISO_PIN_MASK;
            #endif /* (mSPI_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsDm[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
                pinsDm[mSPI_CTS_SCLK_PIN_INDEX] = mSPI_PIN_DM_STRONG;

            #if (mSPI_RTS_SS0_PIN)
                hsiomSel [mSPI_RTS_SS0_PIN_INDEX] = mSPI_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm   [mSPI_RTS_SS0_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsInBuf |= mSPI_RTS_SS0_PIN_MASK;
            #endif /* (mSPI_RTS_SS0_PIN) */

            #if (mSPI_SS1_PIN)
                hsiomSel [mSPI_SS1_PIN_INDEX] = mSPI_SS1_HSIOM_SEL_SPI;
                pinsDm   [mSPI_SS1_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsInBuf |= mSPI_SS1_PIN_MASK;
            #endif /* (mSPI_SS1_PIN) */

            #if (mSPI_SS2_PIN)
                hsiomSel [mSPI_SS2_PIN_INDEX] = mSPI_SS2_HSIOM_SEL_SPI;
                pinsDm   [mSPI_SS2_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsInBuf |= mSPI_SS2_PIN_MASK;
            #endif /* (mSPI_SS2_PIN) */

            #if (mSPI_SS3_PIN)
                hsiomSel [mSPI_SS3_PIN_INDEX] = mSPI_SS3_HSIOM_SEL_SPI;
                pinsDm   [mSPI_SS3_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                pinsInBuf |= mSPI_SS3_PIN_MASK;
            #endif /* (mSPI_SS3_PIN) */

                /* Disable input buffers */
            #if (mSPI_RX_SCL_MOSI_PIN)
                pinsInBuf |= mSPI_RX_SCL_MOSI_PIN_MASK;
            #elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= mSPI_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (mSPI_RX_SCL_MOSI_PIN) */

            #if (mSPI_CTS_SCLK_PIN)
                pinsInBuf |= mSPI_CTS_SCLK_PIN_MASK;
            #endif /* (mSPI_CTS_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (mSPI_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (mSPI_TX_SDA_MISO_PIN)
                hsiomSel[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_PIN_DM_OD_LO;
            #endif /* (mSPI_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (mSPI_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (mSPI_RX_SCL_MOSI_PIN)
                    hsiomSel[mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [mSPI_RX_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
                #elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX] = mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
                #else
                #endif /* (mSPI_RX_SCL_MOSI_PIN) */
                }

                if (0u != (mSPI_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (mSPI_TX_SDA_MISO_PIN)
                    hsiomSel[mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [mSPI_TX_SDA_MISO_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= mSPI_TX_SDA_MISO_PIN_MASK;
                #endif /* (mSPI_TX_SDA_MISO_PIN) */
                }

            #if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
                if (mSPI_UART_MODE_STD == subMode)
                {
                    if (0u != (mSPI_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (mSPI_CTS_SCLK_PIN)
                        hsiomSel[mSPI_CTS_SCLK_PIN_INDEX] = mSPI_CTS_SCLK_HSIOM_SEL_UART;
                        pinsDm  [mSPI_CTS_SCLK_PIN_INDEX] = mSPI_PIN_DM_DIG_HIZ;
                    #endif /* (mSPI_CTS_SCLK_PIN) */
                    }

                    if (0u != (mSPI_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (mSPI_RTS_SS0_PIN)
                        hsiomSel[mSPI_RTS_SS0_PIN_INDEX] = mSPI_RTS_SS0_HSIOM_SEL_UART;
                        pinsDm  [mSPI_RTS_SS0_PIN_INDEX] = mSPI_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= mSPI_RTS_SS0_PIN_MASK;
                    #endif /* (mSPI_RTS_SS0_PIN) */
                    }
                }
            #endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
            }
        }
    #endif /* (!mSPI_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (mSPI_RX_SCL_MOSI_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_RX_SCL_MOSI_HSIOM_REG,
                                       mSPI_RX_SCL_MOSI_HSIOM_MASK,
                                       mSPI_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[mSPI_RX_SCL_MOSI_PIN_INDEX]);

        mSPI_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[mSPI_RX_SCL_MOSI_PIN_INDEX]);

        #if (!mSPI_CY_SCBIP_V1)
            mSPI_SET_INP_DIS(mSPI_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         mSPI_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & mSPI_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!mSPI_CY_SCBIP_V1) */
    
    #elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       mSPI_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       mSPI_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        mSPI_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     mSPI_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & mSPI_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        mSPI_SET_INCFG_TYPE(mSPI_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        mSPI_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        mSPI_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        mSPI_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (mSPI_RX_WAKE_SCL_MOSI_PIN) */

    #if (mSPI_TX_SDA_MISO_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_TX_SDA_MISO_HSIOM_REG,
                                       mSPI_TX_SDA_MISO_HSIOM_MASK,
                                       mSPI_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[mSPI_TX_SDA_MISO_PIN_INDEX]);

        mSPI_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[mSPI_TX_SDA_MISO_PIN_INDEX]);

    #if (!mSPI_CY_SCBIP_V1)
        mSPI_SET_INP_DIS(mSPI_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     mSPI_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & mSPI_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!mSPI_CY_SCBIP_V1) */
    #endif /* (mSPI_RX_SCL_MOSI_PIN) */

    #if (mSPI_CTS_SCLK_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_CTS_SCLK_HSIOM_REG,
                                       mSPI_CTS_SCLK_HSIOM_MASK,
                                       mSPI_CTS_SCLK_HSIOM_POS,
                                       hsiomSel[mSPI_CTS_SCLK_PIN_INDEX]);

        mSPI_uart_cts_spi_sclk_SetDriveMode((uint8) pinsDm[mSPI_CTS_SCLK_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_uart_cts_spi_sclk_INP_DIS,
                                     mSPI_uart_cts_spi_sclk_MASK,
                                     (0u != (pinsInBuf & mSPI_CTS_SCLK_PIN_MASK)));
    #endif /* (mSPI_CTS_SCLK_PIN) */

    #if (mSPI_RTS_SS0_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_RTS_SS0_HSIOM_REG,
                                       mSPI_RTS_SS0_HSIOM_MASK,
                                       mSPI_RTS_SS0_HSIOM_POS,
                                       hsiomSel[mSPI_RTS_SS0_PIN_INDEX]);

        mSPI_uart_rts_spi_ss0_SetDriveMode((uint8) pinsDm[mSPI_RTS_SS0_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_uart_rts_spi_ss0_INP_DIS,
                                     mSPI_uart_rts_spi_ss0_MASK,
                                     (0u != (pinsInBuf & mSPI_RTS_SS0_PIN_MASK)));
    #endif /* (mSPI_RTS_SS0_PIN) */

    #if (mSPI_SS1_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_SS1_HSIOM_REG,
                                       mSPI_SS1_HSIOM_MASK,
                                       mSPI_SS1_HSIOM_POS,
                                       hsiomSel[mSPI_SS1_PIN_INDEX]);

        mSPI_spi_ss1_SetDriveMode((uint8) pinsDm[mSPI_SS1_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_spi_ss1_INP_DIS,
                                     mSPI_spi_ss1_MASK,
                                     (0u != (pinsInBuf & mSPI_SS1_PIN_MASK)));
    #endif /* (mSPI_SS1_PIN) */

    #if (mSPI_SS2_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_SS2_HSIOM_REG,
                                       mSPI_SS2_HSIOM_MASK,
                                       mSPI_SS2_HSIOM_POS,
                                       hsiomSel[mSPI_SS2_PIN_INDEX]);

        mSPI_spi_ss2_SetDriveMode((uint8) pinsDm[mSPI_SS2_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_spi_ss2_INP_DIS,
                                     mSPI_spi_ss2_MASK,
                                     (0u != (pinsInBuf & mSPI_SS2_PIN_MASK)));
    #endif /* (mSPI_SS2_PIN) */

    #if (mSPI_SS3_PIN)
        mSPI_SET_HSIOM_SEL(mSPI_SS3_HSIOM_REG,
                                       mSPI_SS3_HSIOM_MASK,
                                       mSPI_SS3_HSIOM_POS,
                                       hsiomSel[mSPI_SS3_PIN_INDEX]);

        mSPI_spi_ss3_SetDriveMode((uint8) pinsDm[mSPI_SS3_PIN_INDEX]);

        mSPI_SET_INP_DIS(mSPI_spi_ss3_INP_DIS,
                                     mSPI_spi_ss3_MASK,
                                     (0u != (pinsInBuf & mSPI_SS3_PIN_MASK)));
    #endif /* (mSPI_SS3_PIN) */
    }

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: mSPI_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void mSPI_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (mSPI_CTRL_REG & mSPI_CTRL_EC_AM_MODE)) &&
            (0u == (mSPI_I2C_CTRL_REG & mSPI_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            mSPI_CTRL_REG &= ~mSPI_CTRL_EC_AM_MODE;
            mSPI_CTRL_REG |=  mSPI_CTRL_EC_AM_MODE;
        }

        mSPI_I2C_SLAVE_CMD_REG = mSPI_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */


/* [] END OF FILE */
