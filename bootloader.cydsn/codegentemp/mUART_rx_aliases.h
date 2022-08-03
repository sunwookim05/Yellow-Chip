/*******************************************************************************
* File Name: mUART_rx.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_mUART_rx_ALIASES_H) /* Pins mUART_rx_ALIASES_H */
#define CY_PINS_mUART_rx_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define mUART_rx_0			(mUART_rx__0__PC)
#define mUART_rx_0_PS		(mUART_rx__0__PS)
#define mUART_rx_0_PC		(mUART_rx__0__PC)
#define mUART_rx_0_DR		(mUART_rx__0__DR)
#define mUART_rx_0_SHIFT	(mUART_rx__0__SHIFT)
#define mUART_rx_0_INTR	((uint16)((uint16)0x0003u << (mUART_rx__0__SHIFT*2u)))

#define mUART_rx_INTR_ALL	 ((uint16)(mUART_rx_0_INTR))


#endif /* End Pins mUART_rx_ALIASES_H */


/* [] END OF FILE */
