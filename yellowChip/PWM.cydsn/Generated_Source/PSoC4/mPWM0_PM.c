/*******************************************************************************
* File Name: mPWM0_PM.c
* Version 2.10
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mPWM0.h"

static mPWM0_BACKUP_STRUCT mPWM0_backup;


/*******************************************************************************
* Function Name: mPWM0_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mPWM0_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM0_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mPWM0_Sleep(void)
{
    if(0u != (mPWM0_BLOCK_CONTROL_REG & mPWM0_MASK))
    {
        mPWM0_backup.enableState = 1u;
    }
    else
    {
        mPWM0_backup.enableState = 0u;
    }

    mPWM0_Stop();
    mPWM0_SaveConfig();
}


/*******************************************************************************
* Function Name: mPWM0_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mPWM0_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM0_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mPWM0_Wakeup(void)
{
    mPWM0_RestoreConfig();

    if(0u != mPWM0_backup.enableState)
    {
        mPWM0_Enable();
    }
}


/* [] END OF FILE */
