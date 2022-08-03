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

#if !defined(CY_SCB_PVT_mI2C_H)
#define CY_SCB_PVT_mI2C_H

#include "mI2C.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define mI2C_SetI2CExtClkInterruptMode(interruptMask) mI2C_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define mI2C_ClearI2CExtClkInterruptSource(interruptMask) mI2C_CLEAR_INTR_I2C_EC(interruptMask)
#define mI2C_GetI2CExtClkInterruptSource()                (mI2C_INTR_I2C_EC_REG)
#define mI2C_GetI2CExtClkInterruptMode()                  (mI2C_INTR_I2C_EC_MASK_REG)
#define mI2C_GetI2CExtClkInterruptSourceMasked()          (mI2C_INTR_I2C_EC_MASKED_REG)

#if (!mI2C_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define mI2C_SetSpiExtClkInterruptMode(interruptMask) \
                                                                mI2C_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define mI2C_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                mI2C_CLEAR_INTR_SPI_EC(interruptMask)
    #define mI2C_GetExtSpiClkInterruptSource()                 (mI2C_INTR_SPI_EC_REG)
    #define mI2C_GetExtSpiClkInterruptMode()                   (mI2C_INTR_SPI_EC_MASK_REG)
    #define mI2C_GetExtSpiClkInterruptSourceMasked()           (mI2C_INTR_SPI_EC_MASKED_REG)
#endif /* (!mI2C_CY_SCBIP_V1) */

#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void mI2C_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (mI2C_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_mI2C_CUSTOM_INTR_HANDLER)
    extern cyisraddress mI2C_customIntrHandler;
#endif /* !defined (CY_REMOVE_mI2C_CUSTOM_INTR_HANDLER) */
#endif /* (mI2C_SCB_IRQ_INTERNAL) */

extern mI2C_BACKUP_STRUCT mI2C_backup;

#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 mI2C_scbMode;
    extern uint8 mI2C_scbEnableWake;
    extern uint8 mI2C_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 mI2C_mode;
    extern uint8 mI2C_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * mI2C_rxBuffer;
    extern uint8   mI2C_rxDataBits;
    extern uint32  mI2C_rxBufferSize;

    extern volatile uint8 * mI2C_txBuffer;
    extern uint8   mI2C_txDataBits;
    extern uint32  mI2C_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 mI2C_numberOfAddr;
    extern uint8 mI2C_subAddrSize;
#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (mI2C_SCB_MODE_I2C_CONST_CFG || \
        mI2C_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 mI2C_IntrTxMask;
#endif /* (! (mI2C_SCB_MODE_I2C_CONST_CFG || \
              mI2C_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define mI2C_SCB_MODE_I2C_RUNTM_CFG     (mI2C_SCB_MODE_I2C      == mI2C_scbMode)
    #define mI2C_SCB_MODE_SPI_RUNTM_CFG     (mI2C_SCB_MODE_SPI      == mI2C_scbMode)
    #define mI2C_SCB_MODE_UART_RUNTM_CFG    (mI2C_SCB_MODE_UART     == mI2C_scbMode)
    #define mI2C_SCB_MODE_EZI2C_RUNTM_CFG   (mI2C_SCB_MODE_EZI2C    == mI2C_scbMode)
    #define mI2C_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (mI2C_SCB_MODE_UNCONFIG == mI2C_scbMode)

    /* Defines wakeup enable */
    #define mI2C_SCB_WAKE_ENABLE_CHECK       (0u != mI2C_scbEnableWake)
#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!mI2C_CY_SCBIP_V1)
    #define mI2C_SCB_PINS_NUMBER    (7u)
#else
    #define mI2C_SCB_PINS_NUMBER    (2u)
#endif /* (!mI2C_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_mI2C_H) */


/* [] END OF FILE */
