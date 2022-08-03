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

#if !defined(CY_SCB_PVT_mSPI_H)
#define CY_SCB_PVT_mSPI_H

#include "mSPI.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define mSPI_SetI2CExtClkInterruptMode(interruptMask) mSPI_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define mSPI_ClearI2CExtClkInterruptSource(interruptMask) mSPI_CLEAR_INTR_I2C_EC(interruptMask)
#define mSPI_GetI2CExtClkInterruptSource()                (mSPI_INTR_I2C_EC_REG)
#define mSPI_GetI2CExtClkInterruptMode()                  (mSPI_INTR_I2C_EC_MASK_REG)
#define mSPI_GetI2CExtClkInterruptSourceMasked()          (mSPI_INTR_I2C_EC_MASKED_REG)

#if (!mSPI_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define mSPI_SetSpiExtClkInterruptMode(interruptMask) \
                                                                mSPI_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define mSPI_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                mSPI_CLEAR_INTR_SPI_EC(interruptMask)
    #define mSPI_GetExtSpiClkInterruptSource()                 (mSPI_INTR_SPI_EC_REG)
    #define mSPI_GetExtSpiClkInterruptMode()                   (mSPI_INTR_SPI_EC_MASK_REG)
    #define mSPI_GetExtSpiClkInterruptSourceMasked()           (mSPI_INTR_SPI_EC_MASKED_REG)
#endif /* (!mSPI_CY_SCBIP_V1) */

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void mSPI_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (mSPI_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER)
    extern cyisraddress mSPI_customIntrHandler;
#endif /* !defined (CY_REMOVE_mSPI_CUSTOM_INTR_HANDLER) */
#endif /* (mSPI_SCB_IRQ_INTERNAL) */

extern mSPI_BACKUP_STRUCT mSPI_backup;

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 mSPI_scbMode;
    extern uint8 mSPI_scbEnableWake;
    extern uint8 mSPI_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 mSPI_mode;
    extern uint8 mSPI_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * mSPI_rxBuffer;
    extern uint8   mSPI_rxDataBits;
    extern uint32  mSPI_rxBufferSize;

    extern volatile uint8 * mSPI_txBuffer;
    extern uint8   mSPI_txDataBits;
    extern uint32  mSPI_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 mSPI_numberOfAddr;
    extern uint8 mSPI_subAddrSize;
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (mSPI_SCB_MODE_I2C_CONST_CFG || \
        mSPI_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 mSPI_IntrTxMask;
#endif /* (! (mSPI_SCB_MODE_I2C_CONST_CFG || \
              mSPI_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define mSPI_SCB_MODE_I2C_RUNTM_CFG     (mSPI_SCB_MODE_I2C      == mSPI_scbMode)
    #define mSPI_SCB_MODE_SPI_RUNTM_CFG     (mSPI_SCB_MODE_SPI      == mSPI_scbMode)
    #define mSPI_SCB_MODE_UART_RUNTM_CFG    (mSPI_SCB_MODE_UART     == mSPI_scbMode)
    #define mSPI_SCB_MODE_EZI2C_RUNTM_CFG   (mSPI_SCB_MODE_EZI2C    == mSPI_scbMode)
    #define mSPI_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (mSPI_SCB_MODE_UNCONFIG == mSPI_scbMode)

    /* Defines wakeup enable */
    #define mSPI_SCB_WAKE_ENABLE_CHECK       (0u != mSPI_scbEnableWake)
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_SCB_PINS_NUMBER    (7u)
#else
    #define mSPI_SCB_PINS_NUMBER    (2u)
#endif /* (!mSPI_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_mSPI_H) */


/* [] END OF FILE */
