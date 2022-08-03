/***************************************************************************//**
* \file mUART_PM.c
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

#include "mUART.h"
#include "mUART_PVT.h"

#if(mUART_SCB_MODE_I2C_INC)
    #include "mUART_I2C_PVT.h"
#endif /* (mUART_SCB_MODE_I2C_INC) */

#if(mUART_SCB_MODE_EZI2C_INC)
    #include "mUART_EZI2C_PVT.h"
#endif /* (mUART_SCB_MODE_EZI2C_INC) */

#if(mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC)
    #include "mUART_SPI_UART_PVT.h"
#endif /* (mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG || \
   (mUART_SCB_MODE_I2C_CONST_CFG   && (!mUART_I2C_WAKE_ENABLE_CONST))   || \
   (mUART_SCB_MODE_EZI2C_CONST_CFG && (!mUART_EZI2C_WAKE_ENABLE_CONST)) || \
   (mUART_SCB_MODE_SPI_CONST_CFG   && (!mUART_SPI_WAKE_ENABLE_CONST))   || \
   (mUART_SCB_MODE_UART_CONST_CFG  && (!mUART_UART_WAKE_ENABLE_CONST)))

    mUART_BACKUP_STRUCT mUART_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: mUART_Sleep
****************************************************************************//**
*
*  Prepares the mUART component to enter Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the mUART_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void mUART_Sleep(void)
{
#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mUART_SCB_WAKE_ENABLE_CHECK)
    {
        if(mUART_SCB_MODE_I2C_RUNTM_CFG)
        {
            mUART_I2CSaveConfig();
        }
        else if(mUART_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mUART_EzI2CSaveConfig();
        }
    #if(!mUART_CY_SCBIP_V1)
        else if(mUART_SCB_MODE_SPI_RUNTM_CFG)
        {
            mUART_SpiSaveConfig();
        }
        else if(mUART_SCB_MODE_UART_RUNTM_CFG)
        {
            mUART_UartSaveConfig();
        }
    #endif /* (!mUART_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        mUART_backup.enableState = (uint8) mUART_GET_CTRL_ENABLED;

        if(0u != mUART_backup.enableState)
        {
            mUART_Stop();
        }
    }

#else

    #if (mUART_SCB_MODE_I2C_CONST_CFG && mUART_I2C_WAKE_ENABLE_CONST)
        mUART_I2CSaveConfig();

    #elif (mUART_SCB_MODE_EZI2C_CONST_CFG && mUART_EZI2C_WAKE_ENABLE_CONST)
        mUART_EzI2CSaveConfig();

    #elif (mUART_SCB_MODE_SPI_CONST_CFG && mUART_SPI_WAKE_ENABLE_CONST)
        mUART_SpiSaveConfig();

    #elif (mUART_SCB_MODE_UART_CONST_CFG && mUART_UART_WAKE_ENABLE_CONST)
        mUART_UartSaveConfig();

    #else

        mUART_backup.enableState = (uint8) mUART_GET_CTRL_ENABLED;

        if(0u != mUART_backup.enableState)
        {
            mUART_Stop();
        }

    #endif /* defined (mUART_SCB_MODE_I2C_CONST_CFG) && (mUART_I2C_WAKE_ENABLE_CONST) */

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: mUART_Wakeup
****************************************************************************//**
*
*  Prepares the mUART component for Active mode operation after 
*  Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the mUART_Wakeup() function without first calling the 
*   mUART_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void mUART_Wakeup(void)
{
#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)

    if(mUART_SCB_WAKE_ENABLE_CHECK)
    {
        if(mUART_SCB_MODE_I2C_RUNTM_CFG)
        {
            mUART_I2CRestoreConfig();
        }
        else if(mUART_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            mUART_EzI2CRestoreConfig();
        }
    #if(!mUART_CY_SCBIP_V1)
        else if(mUART_SCB_MODE_SPI_RUNTM_CFG)
        {
            mUART_SpiRestoreConfig();
        }
        else if(mUART_SCB_MODE_UART_RUNTM_CFG)
        {
            mUART_UartRestoreConfig();
        }
    #endif /* (!mUART_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != mUART_backup.enableState)
        {
            mUART_Enable();
        }
    }

#else

    #if (mUART_SCB_MODE_I2C_CONST_CFG  && mUART_I2C_WAKE_ENABLE_CONST)
        mUART_I2CRestoreConfig();

    #elif (mUART_SCB_MODE_EZI2C_CONST_CFG && mUART_EZI2C_WAKE_ENABLE_CONST)
        mUART_EzI2CRestoreConfig();

    #elif (mUART_SCB_MODE_SPI_CONST_CFG && mUART_SPI_WAKE_ENABLE_CONST)
        mUART_SpiRestoreConfig();

    #elif (mUART_SCB_MODE_UART_CONST_CFG && mUART_UART_WAKE_ENABLE_CONST)
        mUART_UartRestoreConfig();

    #else

        if(0u != mUART_backup.enableState)
        {
            mUART_Enable();
        }

    #endif /* (mUART_I2C_WAKE_ENABLE_CONST) */

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
