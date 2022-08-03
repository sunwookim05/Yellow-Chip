/***************************************************************************//**
* \file mSPI_PM.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mSPI.h"
#include "mSPI_PVT.h"

#if(mSPI_SCB_MODE_I2C_INC)
    #include "mSPI_I2C_PVT.h"
#endif /* (mSPI_SCB_MODE_I2C_INC) */

#if(mSPI_SCB_MODE_EZI2C_INC)
    #include "mSPI_EZI2C_PVT.h"
#endif /* (mSPI_SCB_MODE_EZI2C_INC) */

#if(mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC)
    #include "mSPI_SPI_UART_PVT.h"
#endif /* (mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG || \
   (mSPI_SCB_MODE_I2C_CONST_CFG   && (!mSPI_I2C_WAKE_ENABLE_CONST))   || \
   (mSPI_SCB_MODE_EZI2C_CONST_CFG && (!mSPI_EZI2C_WAKE_ENABLE_CONST)) || \
   (mSPI_SCB_MODE_SPI_CONST_CFG   && (!mSPI_SPI_WAKE_ENABLE_CONST))   || \
   (mSPI_SCB_MODE_UART_CONST_CFG  && (!mSPI_UART_WAKE_ENABLE_CONST)))

    mSPI_BACKUP_STRUCT mSPI_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: mSPI_Sleep
****************************************************************************//**
*
*  Prepares the mSPI component to enter Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the mSPI_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void mSPI_Sleep(void)
{
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mSPI_SCB_WAKE_ENABLE_CHECK)
    {
        if(mSPI_SCB_MODE_I2C_RUNTM_CFG)
        {
            mSPI_I2CSaveConfig();
        }
        else if(mSPI_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mSPI_EzI2CSaveConfig();
        }
    #if(!mSPI_CY_SCBIP_V1)
        else if(mSPI_SCB_MODE_SPI_RUNTM_CFG)
        {
            mSPI_SpiSaveConfig();
        }
        else if(mSPI_SCB_MODE_UART_RUNTM_CFG)
        {
            mSPI_UartSaveConfig();
        }
    #endif /* (!mSPI_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        mSPI_backup.enableState = (uint8) mSPI_GET_CTRL_ENABLED;

        if(0u != mSPI_backup.enableState)
        {
            mSPI_Stop();
        }
    }

#else

    #if (mSPI_SCB_MODE_I2C_CONST_CFG && mSPI_I2C_WAKE_ENABLE_CONST)
        mSPI_I2CSaveConfig();

    #elif (mSPI_SCB_MODE_EZI2C_CONST_CFG && mSPI_EZI2C_WAKE_ENABLE_CONST)
        mSPI_EzI2CSaveConfig();

    #elif (mSPI_SCB_MODE_SPI_CONST_CFG && mSPI_SPI_WAKE_ENABLE_CONST)
        mSPI_SpiSaveConfig();

    #elif (mSPI_SCB_MODE_UART_CONST_CFG && mSPI_UART_WAKE_ENABLE_CONST)
        mSPI_UartSaveConfig();

    #else

        mSPI_backup.enableState = (uint8) mSPI_GET_CTRL_ENABLED;

        if(0u != mSPI_backup.enableState)
        {
            mSPI_Stop();
        }

    #endif /* defined (mSPI_SCB_MODE_I2C_CONST_CFG) && (mSPI_I2C_WAKE_ENABLE_CONST) */

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mSPI_Wakeup
****************************************************************************//**
*
*  Prepares the mSPI component for Active mode operation after 
*  Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the mSPI_Wakeup() function without first calling the 
*   mSPI_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void mSPI_Wakeup(void)
{
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mSPI_SCB_WAKE_ENABLE_CHECK)
    {
        if(mSPI_SCB_MODE_I2C_RUNTM_CFG)
        {
            mSPI_I2CRestoreConfig();
        }
        else if(mSPI_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mSPI_EzI2CRestoreConfig();
        }
    #if(!mSPI_CY_SCBIP_V1)
        else if(mSPI_SCB_MODE_SPI_RUNTM_CFG)
        {
            mSPI_SpiRestoreConfig();
        }
        else if(mSPI_SCB_MODE_UART_RUNTM_CFG)
        {
            mSPI_UartRestoreConfig();
        }
    #endif /* (!mSPI_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != mSPI_backup.enableState)
        {
            mSPI_Enable();
        }
    }

#else

    #if (mSPI_SCB_MODE_I2C_CONST_CFG  && mSPI_I2C_WAKE_ENABLE_CONST)
        mSPI_I2CRestoreConfig();

    #elif (mSPI_SCB_MODE_EZI2C_CONST_CFG && mSPI_EZI2C_WAKE_ENABLE_CONST)
        mSPI_EzI2CRestoreConfig();

    #elif (mSPI_SCB_MODE_SPI_CONST_CFG && mSPI_SPI_WAKE_ENABLE_CONST)
        mSPI_SpiRestoreConfig();

    #elif (mSPI_SCB_MODE_UART_CONST_CFG && mSPI_UART_WAKE_ENABLE_CONST)
        mSPI_UartRestoreConfig();

    #else

        if(0u != mSPI_backup.enableState)
        {
            mSPI_Enable();
        }

    #endif /* (mSPI_I2C_WAKE_ENABLE_CONST) */

#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
