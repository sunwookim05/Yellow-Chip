/*******************************************************************************
* File Name: mPWM_PM.c
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

#include "mPWM.h"

static mPWM_BACKUP_STRUCT mPWM_backup;


/*******************************************************************************
* Function Name: mPWM_SaveConfig
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
void mPWM_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM_Sleep
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
void mPWM_Sleep(void)
{
    if(0u != (mPWM_BLOCK_CONTROL_REG & mPWM_MASK))
    {
        mPWM_backup.enableState = 1u;
    }
    else
    {
        mPWM_backup.enableState = 0u;
    }

    mPWM_Stop();
    mPWM_SaveConfig();
}


/*******************************************************************************
* Function Name: mPWM_RestoreConfig
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
void mPWM_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM_Wakeup
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
void mPWM_Wakeup(void)
{
    mPWM_RestoreConfig();

    if(0u != mPWM_backup.enableState)
    {
        mPWM_Enable();
    }
}


/* [] END OF FILE */
