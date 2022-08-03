/***************************************************************************//**
* \file mUART.c
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

#include "mUART_PVT.h"

#if (mUART_SCB_MODE_I2C_INC)
    #include "mUART_I2C_PVT.h"
#endif /* (mUART_SCB_MODE_I2C_INC) */

#if (mUART_SCB_MODE_EZI2C_INC)
    #include "mUART_EZI2C_PVT.h"
#endif /* (mUART_SCB_MODE_EZI2C_INC) */

#if (mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC)
    #include "mUART_SPI_UART_PVT.h"
#endif /* (mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 mUART_scbMode = mUART_SCB_MODE_UNCONFIG;
    uint8 mUART_scbEnableWake;
    uint8 mUART_scbEnableIntr;

    /* I2C configuration variables */
    uint8 mUART_mode;
    uint8 mUART_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * mUART_rxBuffer;
    uint8  mUART_rxDataBits;
    uint32 mUART_rxBufferSize;

    volatile uint8 * mUART_txBuffer;
    uint8  mUART_txDataBits;
    uint32 mUART_txBufferSize;

    /* EZI2C configuration variables */
    uint8 mUART_numberOfAddr;
    uint8 mUART_subAddrSize;
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** mUART_initVar indicates whether the mUART 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the mUART_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  mUART_Init() function can be called before the 
*  mUART_Start() or mUART_Enable() function.
*/
uint8 mUART_initVar = 0u;


#if (! (mUART_SCB_MODE_I2C_CONST_CFG || \
        mUART_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * mUART_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent mUART_Enable() call.
    */
    uint16 mUART_IntrTxMask = 0u;
#endif /* (! (mUART_SCB_MODE_I2C_CONST_CFG || \
              mUART_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (mUART_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER)
    void (*mUART_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER) */
#endif /* (mUART_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void mUART_ScbEnableIntr(void);
static void mUART_ScbModeStop(void);
static void mUART_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: mUART_Init
****************************************************************************//**
*
*  Initializes the mUART component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  mUART_I2CInit, mUART_SpiInit, 
*  mUART_UartInit or mUART_EzI2CInit.
*
*******************************************************************************/
void mUART_Init(void)
{
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    if (mUART_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        mUART_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (mUART_SCB_MODE_I2C_CONST_CFG)
    mUART_I2CInit();

#elif (mUART_SCB_MODE_SPI_CONST_CFG)
    mUART_SpiInit();

#elif (mUART_SCB_MODE_UART_CONST_CFG)
    mUART_UartInit();

#elif (mUART_SCB_MODE_EZI2C_CONST_CFG)
    mUART_EzI2CInit();

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mUART_Enable
****************************************************************************//**
*
*  Enables mUART component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  mUART_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The mUART configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured mUART”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void mUART_Enable(void)
{
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!mUART_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        mUART_CTRL_REG |= mUART_CTRL_ENABLED;

        mUART_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        mUART_ScbModePostEnable();
    }
#else
    mUART_CTRL_REG |= mUART_CTRL_ENABLED;

    mUART_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    mUART_ScbModePostEnable();
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mUART_Start
****************************************************************************//**
*
*  Invokes mUART_Init() and mUART_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  mUART_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void mUART_Start(void)
{
    if (0u == mUART_initVar)
    {
        mUART_Init();
        mUART_initVar = 1u; /* Component was initialized */
    }

    mUART_Enable();
}


/*******************************************************************************
* Function Name: mUART_Stop
****************************************************************************//**
*
*  Disables the mUART component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function mUART_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void mUART_Stop(void)
{
#if (mUART_SCB_IRQ_INTERNAL)
    mUART_DisableInt();
#endif /* (mUART_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    mUART_ScbModeStop();

    /* Disable SCB IP */
    mUART_CTRL_REG &= (uint32) ~mUART_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    mUART_SetTxInterruptMode(mUART_NO_INTR_SOURCES);

#if (mUART_SCB_IRQ_INTERNAL)
    mUART_ClearPendingInt();
#endif /* (mUART_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: mUART_SetRxFifoLevel
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
void mUART_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = mUART_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~mUART_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (mUART_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    mUART_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: mUART_SetTxFifoLevel
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
void mUART_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = mUART_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~mUART_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (mUART_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    mUART_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (mUART_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: mUART_SetCustomInterruptHandler
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
    void mUART_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER)
        mUART_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER) */
    }
#endif /* (mUART_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: mUART_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void mUART_ScbEnableIntr(void)
{
#if (mUART_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != mUART_scbEnableIntr)
        {
            mUART_EnableInt();
        }

    #else
        mUART_EnableInt();

    #endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (mUART_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: mUART_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void mUART_ScbModePostEnable(void)
{
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!mUART_CY_SCBIP_V1)
    if (mUART_SCB_MODE_SPI_RUNTM_CFG)
    {
        mUART_SpiPostEnable();
    }
    else if (mUART_SCB_MODE_UART_RUNTM_CFG)
    {
        mUART_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!mUART_CY_SCBIP_V1) */

#elif (mUART_SCB_MODE_SPI_CONST_CFG)
    mUART_SpiPostEnable();

#elif (mUART_SCB_MODE_UART_CONST_CFG)
    mUART_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mUART_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void mUART_ScbModeStop(void)
{
#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    if (mUART_SCB_MODE_I2C_RUNTM_CFG)
    {
        mUART_I2CStop();
    }
    else if (mUART_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        mUART_EzI2CStop();
    }
#if (!mUART_CY_SCBIP_V1)
    else if (mUART_SCB_MODE_SPI_RUNTM_CFG)
    {
        mUART_SpiStop();
    }
    else if (mUART_SCB_MODE_UART_RUNTM_CFG)
    {
        mUART_UartStop();
    }
#endif /* (!mUART_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (mUART_SCB_MODE_I2C_CONST_CFG)
    mUART_I2CStop();

#elif (mUART_SCB_MODE_EZI2C_CONST_CFG)
    mUART_EzI2CStop();

#elif (mUART_SCB_MODE_SPI_CONST_CFG)
    mUART_SpiStop();

#elif (mUART_SCB_MODE_UART_CONST_CFG)
    mUART_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: mUART_SetPins
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
    void mUART_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[mUART_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!mUART_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!mUART_CY_SCBIP_V1) */
        
        uint32 hsiomSel[mUART_SCB_PINS_NUMBER] = 
        {
            mUART_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            mUART_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (mUART_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (mUART_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < mUART_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = mUART_PIN_DM_ALG_HIZ;
        }

        if ((mUART_SCB_MODE_I2C   == mode) ||
            (mUART_SCB_MODE_EZI2C == mode))
        {
        #if (mUART_RX_SCL_MOSI_PIN)
            hsiomSel[mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_OD_LO;
        #elif (mUART_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[mUART_RX_WAKE_SCL_MOSI_PIN_INDEX] = mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [mUART_RX_WAKE_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_OD_LO;
        #else
        #endif /* (mUART_RX_SCL_MOSI_PIN) */
        
        #if (mUART_TX_SDA_MISO_PIN)
            hsiomSel[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [mUART_TX_SDA_MISO_PIN_INDEX] = mUART_PIN_DM_OD_LO;
        #endif /* (mUART_TX_SDA_MISO_PIN) */
        }
    #if (!mUART_CY_SCBIP_V1)
        else if (mUART_SCB_MODE_SPI == mode)
        {
        #if (mUART_RX_SCL_MOSI_PIN)
            hsiomSel[mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (mUART_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[mUART_RX_WAKE_SCL_MOSI_PIN_INDEX] = mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (mUART_RX_SCL_MOSI_PIN) */
        
        #if (mUART_TX_SDA_MISO_PIN)
            hsiomSel[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (mUART_TX_SDA_MISO_PIN) */
        
        #if (mUART_CTS_SCLK_PIN)
            hsiomSel[mUART_CTS_SCLK_PIN_INDEX] = mUART_CTS_SCLK_HSIOM_SEL_SPI;
        #endif /* (mUART_CTS_SCLK_PIN) */

            if (mUART_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
                pinsDm[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsDm[mUART_CTS_SCLK_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;

            #if (mUART_RTS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[mUART_RTS_SS0_PIN_INDEX] = mUART_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm  [mUART_RTS_SS0_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
            #endif /* (mUART_RTS_SS0_PIN) */

            #if (mUART_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= mUART_TX_SDA_MISO_PIN_MASK;
            #endif /* (mUART_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsDm[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
                pinsDm[mUART_CTS_SCLK_PIN_INDEX] = mUART_PIN_DM_STRONG;

            #if (mUART_RTS_SS0_PIN)
                hsiomSel [mUART_RTS_SS0_PIN_INDEX] = mUART_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm   [mUART_RTS_SS0_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsInBuf |= mUART_RTS_SS0_PIN_MASK;
            #endif /* (mUART_RTS_SS0_PIN) */

            #if (mUART_SS1_PIN)
                hsiomSel [mUART_SS1_PIN_INDEX] = mUART_SS1_HSIOM_SEL_SPI;
                pinsDm   [mUART_SS1_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsInBuf |= mUART_SS1_PIN_MASK;
            #endif /* (mUART_SS1_PIN) */

            #if (mUART_SS2_PIN)
                hsiomSel [mUART_SS2_PIN_INDEX] = mUART_SS2_HSIOM_SEL_SPI;
                pinsDm   [mUART_SS2_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsInBuf |= mUART_SS2_PIN_MASK;
            #endif /* (mUART_SS2_PIN) */

            #if (mUART_SS3_PIN)
                hsiomSel [mUART_SS3_PIN_INDEX] = mUART_SS3_HSIOM_SEL_SPI;
                pinsDm   [mUART_SS3_PIN_INDEX] = mUART_PIN_DM_STRONG;
                pinsInBuf |= mUART_SS3_PIN_MASK;
            #endif /* (mUART_SS3_PIN) */

                /* Disable input buffers */
            #if (mUART_RX_SCL_MOSI_PIN)
                pinsInBuf |= mUART_RX_SCL_MOSI_PIN_MASK;
            #elif (mUART_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= mUART_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (mUART_RX_SCL_MOSI_PIN) */

            #if (mUART_CTS_SCLK_PIN)
                pinsInBuf |= mUART_CTS_SCLK_PIN_MASK;
            #endif /* (mUART_CTS_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (mUART_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (mUART_TX_SDA_MISO_PIN)
                hsiomSel[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [mUART_TX_SDA_MISO_PIN_INDEX] = mUART_PIN_DM_OD_LO;
            #endif /* (mUART_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (mUART_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (mUART_RX_SCL_MOSI_PIN)
                    hsiomSel[mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [mUART_RX_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
                #elif (mUART_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[mUART_RX_WAKE_SCL_MOSI_PIN_INDEX] = mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [mUART_RX_WAKE_SCL_MOSI_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
                #else
                #endif /* (mUART_RX_SCL_MOSI_PIN) */
                }

                if (0u != (mUART_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (mUART_TX_SDA_MISO_PIN)
                    hsiomSel[mUART_TX_SDA_MISO_PIN_INDEX] = mUART_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [mUART_TX_SDA_MISO_PIN_INDEX] = mUART_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= mUART_TX_SDA_MISO_PIN_MASK;
                #endif /* (mUART_TX_SDA_MISO_PIN) */
                }

            #if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
                if (mUART_UART_MODE_STD == subMode)
                {
                    if (0u != (mUART_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (mUART_CTS_SCLK_PIN)
                        hsiomSel[mUART_CTS_SCLK_PIN_INDEX] = mUART_CTS_SCLK_HSIOM_SEL_UART;
                        pinsDm  [mUART_CTS_SCLK_PIN_INDEX] = mUART_PIN_DM_DIG_HIZ;
                    #endif /* (mUART_CTS_SCLK_PIN) */
                    }

                    if (0u != (mUART_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (mUART_RTS_SS0_PIN)
                        hsiomSel[mUART_RTS_SS0_PIN_INDEX] = mUART_RTS_SS0_HSIOM_SEL_UART;
                        pinsDm  [mUART_RTS_SS0_PIN_INDEX] = mUART_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= mUART_RTS_SS0_PIN_MASK;
                    #endif /* (mUART_RTS_SS0_PIN) */
                    }
                }
            #endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */
            }
        }
    #endif /* (!mUART_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (mUART_RX_SCL_MOSI_PIN)
        mUART_SET_HSIOM_SEL(mUART_RX_SCL_MOSI_HSIOM_REG,
                                       mUART_RX_SCL_MOSI_HSIOM_MASK,
                                       mUART_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[mUART_RX_SCL_MOSI_PIN_INDEX]);

        mUART_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[mUART_RX_SCL_MOSI_PIN_INDEX]);

        #if (!mUART_CY_SCBIP_V1)
            mUART_SET_INP_DIS(mUART_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         mUART_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & mUART_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!mUART_CY_SCBIP_V1) */
    
    #elif (mUART_RX_WAKE_SCL_MOSI_PIN)
        mUART_SET_HSIOM_SEL(mUART_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       mUART_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       mUART_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[mUART_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        mUART_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[mUART_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     mUART_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & mUART_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        mUART_SET_INCFG_TYPE(mUART_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        mUART_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        mUART_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        mUART_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (mUART_RX_WAKE_SCL_MOSI_PIN) */

    #if (mUART_TX_SDA_MISO_PIN)
        mUART_SET_HSIOM_SEL(mUART_TX_SDA_MISO_HSIOM_REG,
                                       mUART_TX_SDA_MISO_HSIOM_MASK,
                                       mUART_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[mUART_TX_SDA_MISO_PIN_INDEX]);

        mUART_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[mUART_TX_SDA_MISO_PIN_INDEX]);

    #if (!mUART_CY_SCBIP_V1)
        mUART_SET_INP_DIS(mUART_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     mUART_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & mUART_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!mUART_CY_SCBIP_V1) */
    #endif /* (mUART_RX_SCL_MOSI_PIN) */

    #if (mUART_CTS_SCLK_PIN)
        mUART_SET_HSIOM_SEL(mUART_CTS_SCLK_HSIOM_REG,
                                       mUART_CTS_SCLK_HSIOM_MASK,
                                       mUART_CTS_SCLK_HSIOM_POS,
                                       hsiomSel[mUART_CTS_SCLK_PIN_INDEX]);

        mUART_uart_cts_spi_sclk_SetDriveMode((uint8) pinsDm[mUART_CTS_SCLK_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_uart_cts_spi_sclk_INP_DIS,
                                     mUART_uart_cts_spi_sclk_MASK,
                                     (0u != (pinsInBuf & mUART_CTS_SCLK_PIN_MASK)));
    #endif /* (mUART_CTS_SCLK_PIN) */

    #if (mUART_RTS_SS0_PIN)
        mUART_SET_HSIOM_SEL(mUART_RTS_SS0_HSIOM_REG,
                                       mUART_RTS_SS0_HSIOM_MASK,
                                       mUART_RTS_SS0_HSIOM_POS,
                                       hsiomSel[mUART_RTS_SS0_PIN_INDEX]);

        mUART_uart_rts_spi_ss0_SetDriveMode((uint8) pinsDm[mUART_RTS_SS0_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_uart_rts_spi_ss0_INP_DIS,
                                     mUART_uart_rts_spi_ss0_MASK,
                                     (0u != (pinsInBuf & mUART_RTS_SS0_PIN_MASK)));
    #endif /* (mUART_RTS_SS0_PIN) */

    #if (mUART_SS1_PIN)
        mUART_SET_HSIOM_SEL(mUART_SS1_HSIOM_REG,
                                       mUART_SS1_HSIOM_MASK,
                                       mUART_SS1_HSIOM_POS,
                                       hsiomSel[mUART_SS1_PIN_INDEX]);

        mUART_spi_ss1_SetDriveMode((uint8) pinsDm[mUART_SS1_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_spi_ss1_INP_DIS,
                                     mUART_spi_ss1_MASK,
                                     (0u != (pinsInBuf & mUART_SS1_PIN_MASK)));
    #endif /* (mUART_SS1_PIN) */

    #if (mUART_SS2_PIN)
        mUART_SET_HSIOM_SEL(mUART_SS2_HSIOM_REG,
                                       mUART_SS2_HSIOM_MASK,
                                       mUART_SS2_HSIOM_POS,
                                       hsiomSel[mUART_SS2_PIN_INDEX]);

        mUART_spi_ss2_SetDriveMode((uint8) pinsDm[mUART_SS2_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_spi_ss2_INP_DIS,
                                     mUART_spi_ss2_MASK,
                                     (0u != (pinsInBuf & mUART_SS2_PIN_MASK)));
    #endif /* (mUART_SS2_PIN) */

    #if (mUART_SS3_PIN)
        mUART_SET_HSIOM_SEL(mUART_SS3_HSIOM_REG,
                                       mUART_SS3_HSIOM_MASK,
                                       mUART_SS3_HSIOM_POS,
                                       hsiomSel[mUART_SS3_PIN_INDEX]);

        mUART_spi_ss3_SetDriveMode((uint8) pinsDm[mUART_SS3_PIN_INDEX]);

        mUART_SET_INP_DIS(mUART_spi_ss3_INP_DIS,
                                     mUART_spi_ss3_MASK,
                                     (0u != (pinsInBuf & mUART_SS3_PIN_MASK)));
    #endif /* (mUART_SS3_PIN) */
    }

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: mUART_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void mUART_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (mUART_CTRL_REG & mUART_CTRL_EC_AM_MODE)) &&
            (0u == (mUART_I2C_CTRL_REG & mUART_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            mUART_CTRL_REG &= ~mUART_CTRL_EC_AM_MODE;
            mUART_CTRL_REG |=  mUART_CTRL_EC_AM_MODE;
        }

        mUART_I2C_SLAVE_CMD_REG = mUART_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */


/* [] END OF FILE */
