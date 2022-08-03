/*******************************************************************************
* File Name: pReset.h  
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

#if !defined(CY_PINS_pReset_ALIASES_H) /* Pins pReset_ALIASES_H */
#define CY_PINS_pReset_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define pReset_0			(pReset__0__PC)
#define pReset_0_PS		(pReset__0__PS)
#define pReset_0_PC		(pReset__0__PC)
#define pReset_0_DR		(pReset__0__DR)
#define pReset_0_SHIFT	(pReset__0__SHIFT)
#define pReset_0_INTR	((uint16)((uint16)0x0003u << (pReset__0__SHIFT*2u)))

#define pReset_INTR_ALL	 ((uint16)(pReset_0_INTR))


#endif /* End Pins pReset_ALIASES_H */


/* [] END OF FILE */
