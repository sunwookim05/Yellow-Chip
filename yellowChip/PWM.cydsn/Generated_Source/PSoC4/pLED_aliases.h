/*******************************************************************************
* File Name: pLED.h  
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

#if !defined(CY_PINS_pLED_ALIASES_H) /* Pins pLED_ALIASES_H */
#define CY_PINS_pLED_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define pLED_0			(pLED__0__PC)
#define pLED_0_PS		(pLED__0__PS)
#define pLED_0_PC		(pLED__0__PC)
#define pLED_0_DR		(pLED__0__DR)
#define pLED_0_SHIFT	(pLED__0__SHIFT)
#define pLED_0_INTR	((uint16)((uint16)0x0003u << (pLED__0__SHIFT*2u)))

#define pLED_INTR_ALL	 ((uint16)(pLED_0_INTR))


#endif /* End Pins pLED_ALIASES_H */


/* [] END OF FILE */
