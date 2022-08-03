/*******************************************************************************
* File Name: mPWM2_PM.c
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

#include "mPWM2.h"

static mPWM2_BACKUP_STRUCT mPWM2_backup;


/*******************************************************************************
* Function Name: mPWM2_SaveConfig
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
void mPWM2_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM2_Sleep
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
void mPWM2_Sleep(void)
{
    if(0u != (mPWM2_BLOCK_CONTROL_REG & mPWM2_MASK))
    {
        mPWM2_backup.enableState = 1u;
    }
    else
    {
        mPWM2_backup.enableState = 0u;
    }

    mPWM2_Stop();
    mPWM2_SaveConfig();
}


/*******************************************************************************
* Function Name: mPWM2_RestoreConfig
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
void mPWM2_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM2_Wakeup
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
void mPWM2_Wakeup(void)
{
    mPWM2_RestoreConfig();

    if(0u != mPWM2_backup.enableState)
    {
        mPWM2_Enable();
    }
}


/* [] END OF FILE */
