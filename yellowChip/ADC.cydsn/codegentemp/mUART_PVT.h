/***************************************************************************//**
* \file .h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_mUART_H)
#define CY_SCB_PVT_mUART_H

#include "mUART.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define mUART_SetI2CExtClkInterruptMode(interruptMask) mUART_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define mUART_ClearI2CExtClkInterruptSource(interruptMask) mUART_CLEAR_INTR_I2C_EC(interruptMask)
#define mUART_GetI2CExtClkInterruptSource()                (mUART_INTR_I2C_EC_REG)
#define mUART_GetI2CExtClkInterruptMode()                  (mUART_INTR_I2C_EC_MASK_REG)
#define mUART_GetI2CExtClkInterruptSourceMasked()          (mUART_INTR_I2C_EC_MASKED_REG)

#if (!mUART_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define mUART_SetSpiExtClkInterruptMode(interruptMask) \
                                                                mUART_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define mUART_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                mUART_CLEAR_INTR_SPI_EC(interruptMask)
    #define mUART_GetExtSpiClkInterruptSource()                 (mUART_INTR_SPI_EC_REG)
    #define mUART_GetExtSpiClkInterruptMode()                   (mUART_INTR_SPI_EC_MASK_REG)
    #define mUART_GetExtSpiClkInterruptSourceMasked()           (mUART_INTR_SPI_EC_MASKED_REG)
#endif /* (!mUART_CY_SCBIP_V1) */

#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void mUART_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (mUART_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER)
    extern cyisraddress mUART_customIntrHandler;
#endif /* !defined (CY_REMOVE_mUART_CUSTOM_INTR_HANDLER) */
#endif /* (mUART_SCB_IRQ_INTERNAL) */

extern mUART_BACKUP_STRUCT mUART_backup;

#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 mUART_scbMode;
    extern uint8 mUART_scbEnableWake;
    extern uint8 mUART_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 mUART_mode;
    extern uint8 mUART_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * mUART_rxBuffer;
    extern uint8   mUART_rxDataBits;
    extern uint32  mUART_rxBufferSize;

    extern volatile uint8 * mUART_txBuffer;
    extern uint8   mUART_txDataBits;
    extern uint32  mUART_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 mUART_numberOfAddr;
    extern uint8 mUART_subAddrSize;
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (mUART_SCB_MODE_I2C_CONST_CFG || \
        mUART_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 mUART_IntrTxMask;
#endif /* (! (mUART_SCB_MODE_I2C_CONST_CFG || \
              mUART_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define mUART_SCB_MODE_I2C_RUNTM_CFG     (mUART_SCB_MODE_I2C      == mUART_scbMode)
    #define mUART_SCB_MODE_SPI_RUNTM_CFG     (mUART_SCB_MODE_SPI      == mUART_scbMode)
    #define mUART_SCB_MODE_UART_RUNTM_CFG    (mUART_SCB_MODE_UART     == mUART_scbMode)
    #define mUART_SCB_MODE_EZI2C_RUNTM_CFG   (mUART_SCB_MODE_EZI2C    == mUART_scbMode)
    #define mUART_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (mUART_SCB_MODE_UNCONFIG == mUART_scbMode)

    /* Defines wakeup enable */
    #define mUART_SCB_WAKE_ENABLE_CHECK       (0u != mUART_scbEnableWake)
#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!mUART_CY_SCBIP_V1)
    #define mUART_SCB_PINS_NUMBER    (7u)
#else
    #define mUART_SCB_PINS_NUMBER    (2u)
#endif /* (!mUART_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_mUART_H) */


/* [] END OF FILE */
