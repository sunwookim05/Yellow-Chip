/*******************************************************************************
* File Name: pOledReset.h  
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

#if !defined(CY_PINS_pOledReset_ALIASES_H) /* Pins pOledReset_ALIASES_H */
#define CY_PINS_pOledReset_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define pOledReset_0			(pOledReset__0__PC)
#define pOledReset_0_PS		(pOledReset__0__PS)
#define pOledReset_0_PC		(pOledReset__0__PC)
#define pOledReset_0_DR		(pOledReset__0__DR)
#define pOledReset_0_SHIFT	(pOledReset__0__SHIFT)
#define pOledReset_0_INTR	((uint16)((uint16)0x0003u << (pOledReset__0__SHIFT*2u)))

#define pOledReset_INTR_ALL	 ((uint16)(pOledReset_0_INTR))


#endif /* End Pins pOledReset_ALIASES_H */


/* [] END OF FILE */
