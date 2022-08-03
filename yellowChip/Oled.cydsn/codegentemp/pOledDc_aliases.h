/*******************************************************************************
* File Name: pOledDc.h  
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

#if !defined(CY_PINS_pOledDc_ALIASES_H) /* Pins pOledDc_ALIASES_H */
#define CY_PINS_pOledDc_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define pOledDc_0			(pOledDc__0__PC)
#define pOledDc_0_PS		(pOledDc__0__PS)
#define pOledDc_0_PC		(pOledDc__0__PC)
#define pOledDc_0_DR		(pOledDc__0__DR)
#define pOledDc_0_SHIFT	(pOledDc__0__SHIFT)
#define pOledDc_0_INTR	((uint16)((uint16)0x0003u << (pOledDc__0__SHIFT*2u)))

#define pOledDc_INTR_ALL	 ((uint16)(pOledDc_0_INTR))


#endif /* End Pins pOledDc_ALIASES_H */


/* [] END OF FILE */
