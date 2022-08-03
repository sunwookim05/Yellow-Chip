/*******************************************************************************
* File Name: pLed.h  
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

#if !defined(CY_PINS_pLed_ALIASES_H) /* Pins pLed_ALIASES_H */
#define CY_PINS_pLed_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define pLed_0			(pLed__0__PC)
#define pLed_0_PS		(pLed__0__PS)
#define pLed_0_PC		(pLed__0__PC)
#define pLed_0_DR		(pLed__0__DR)
#define pLed_0_SHIFT	(pLed__0__SHIFT)
#define pLed_0_INTR	((uint16)((uint16)0x0003u << (pLed__0__SHIFT*2u)))

#define pLed_INTR_ALL	 ((uint16)(pLed_0_INTR))


#endif /* End Pins pLed_ALIASES_H */


/* [] END OF FILE */
