/***************************************************************************//**
* \file mI2C_PM.c
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

#include "mI2C.h"
#include "mI2C_PVT.h"

#if(mI2C_SCB_MODE_I2C_INC)
    #include "mI2C_I2C_PVT.h"
#endif /* (mI2C_SCB_MODE_I2C_INC) */

#if(mI2C_SCB_MODE_EZI2C_INC)
    #include "mI2C_EZI2C_PVT.h"
#endif /* (mI2C_SCB_MODE_EZI2C_INC) */

#if(mI2C_SCB_MODE_SPI_INC || mI2C_SCB_MODE_UART_INC)
    #include "mI2C_SPI_UART_PVT.h"
#endif /* (mI2C_SCB_MODE_SPI_INC || mI2C_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG || \
   (mI2C_SCB_MODE_I2C_CONST_CFG   && (!mI2C_I2C_WAKE_ENABLE_CONST))   || \
   (mI2C_SCB_MODE_EZI2C_CONST_CFG && (!mI2C_EZI2C_WAKE_ENABLE_CONST)) || \
   (mI2C_SCB_MODE_SPI_CONST_CFG   && (!mI2C_SPI_WAKE_ENABLE_CONST))   || \
   (mI2C_SCB_MODE_UART_CONST_CFG  && (!mI2C_UART_WAKE_ENABLE_CONST)))

    mI2C_BACKUP_STRUCT mI2C_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: mI2C_Sleep
****************************************************************************//**
*
*  Prepares the mI2C component to enter Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the mI2C_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void mI2C_Sleep(void)
{
#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mI2C_SCB_WAKE_ENABLE_CHECK)
    {
        if(mI2C_SCB_MODE_I2C_RUNTM_CFG)
        {
            mI2C_I2CSaveConfig();
        }
        else if(mI2C_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mI2C_EzI2CSaveConfig();
        }
    #if(!mI2C_CY_SCBIP_V1)
        else if(mI2C_SCB_MODE_SPI_RUNTM_CFG)
        {
            mI2C_SpiSaveConfig();
        }
        else if(mI2C_SCB_MODE_UART_RUNTM_CFG)
        {
            mI2C_UartSaveConfig();
        }
    #endif /* (!mI2C_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        mI2C_backup.enableState = (uint8) mI2C_GET_CTRL_ENABLED;

        if(0u != mI2C_backup.enableState)
        {
            mI2C_Stop();
        }
    }

#else

    #if (mI2C_SCB_MODE_I2C_CONST_CFG && mI2C_I2C_WAKE_ENABLE_CONST)
        mI2C_I2CSaveConfig();

    #elif (mI2C_SCB_MODE_EZI2C_CONST_CFG && mI2C_EZI2C_WAKE_ENABLE_CONST)
        mI2C_EzI2CSaveConfig();

    #elif (mI2C_SCB_MODE_SPI_CONST_CFG && mI2C_SPI_WAKE_ENABLE_CONST)
        mI2C_SpiSaveConfig();

    #elif (mI2C_SCB_MODE_UART_CONST_CFG && mI2C_UART_WAKE_ENABLE_CONST)
        mI2C_UartSaveConfig();

    #else

        mI2C_backup.enableState = (uint8) mI2C_GET_CTRL_ENABLED;

        if(0u != mI2C_backup.enableState)
        {
            mI2C_Stop();
        }

    #endif /* defined (mI2C_SCB_MODE_I2C_CONST_CFG) && (mI2C_I2C_WAKE_ENABLE_CONST) */

#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mI2C_Wakeup
****************************************************************************//**
*
*  Prepares the mI2C component for Active mode operation after 
*  Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the mI2C_Wakeup() function without first calling the 
*   mI2C_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void mI2C_Wakeup(void)
{
#if(mI2C_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mI2C_SCB_WAKE_ENABLE_CHECK)
    {
        if(mI2C_SCB_MODE_I2C_RUNTM_CFG)
        {
            mI2C_I2CRestoreConfig();
        }
        else if(mI2C_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mI2C_EzI2CRestoreConfig();
        }
    #if(!mI2C_CY_SCBIP_V1)
        else if(mI2C_SCB_MODE_SPI_RUNTM_CFG)
        {
            mI2C_SpiRestoreConfig();
        }
        else if(mI2C_SCB_MODE_UART_RUNTM_CFG)
        {
            mI2C_UartRestoreConfig();
        }
    #endif /* (!mI2C_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != mI2C_backup.enableState)
        {
            mI2C_Enable();
        }
    }

#else

    #if (mI2C_SCB_MODE_I2C_CONST_CFG  && mI2C_I2C_WAKE_ENABLE_CONST)
        mI2C_I2CRestoreConfig();

    #elif (mI2C_SCB_MODE_EZI2C_CONST_CFG && mI2C_EZI2C_WAKE_ENABLE_CONST)
        mI2C_EzI2CRestoreConfig();

    #elif (mI2C_SCB_MODE_SPI_CONST_CFG && mI2C_SPI_WAKE_ENABLE_CONST)
        mI2C_SpiRestoreConfig();

    #elif (mI2C_SCB_MODE_UART_CONST_CFG && mI2C_UART_WAKE_ENABLE_CONST)
        mI2C_UartRestoreConfig();

    #else

        if(0u != mI2C_backup.enableState)
        {
            mI2C_Enable();
        }

    #endif /* (mI2C_I2C_WAKE_ENABLE_CONST) */

#endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
