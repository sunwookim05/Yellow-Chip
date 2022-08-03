/*******************************************************************************
* File Name: mPWM1_PM.c
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

#include "mPWM1.h"

static mPWM1_BACKUP_STRUCT mPWM1_backup;


/*******************************************************************************
* Function Name: mPWM1_SaveConfig
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
void mPWM1_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM1_Sleep
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
void mPWM1_Sleep(void)
{
    if(0u != (mPWM1_BLOCK_CONTROL_REG & mPWM1_MASK))
    {
        mPWM1_backup.enableState = 1u;
    }
    else
    {
        mPWM1_backup.enableState = 0u;
    }

    mPWM1_Stop();
    mPWM1_SaveConfig();
}


/*******************************************************************************
* Function Name: mPWM1_RestoreConfig
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
void mPWM1_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: mPWM1_Wakeup
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
void mPWM1_Wakeup(void)
{
    mPWM1_RestoreConfig();

    if(0u != mPWM1_backup.enableState)
    {
        mPWM1_Enable();
    }
}


/* [] END OF FILE */
