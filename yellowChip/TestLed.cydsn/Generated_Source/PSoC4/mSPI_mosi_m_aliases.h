/*******************************************************************************
* File Name: mSPI_mosi_m.h  
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

#if !defined(CY_PINS_mSPI_mosi_m_ALIASES_H) /* Pins mSPI_mosi_m_ALIASES_H */
#define CY_PINS_mSPI_mosi_m_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define mSPI_mosi_m_0			(mSPI_mosi_m__0__PC)
#define mSPI_mosi_m_0_PS		(mSPI_mosi_m__0__PS)
#define mSPI_mosi_m_0_PC		(mSPI_mosi_m__0__PC)
#define mSPI_mosi_m_0_DR		(mSPI_mosi_m__0__DR)
#define mSPI_mosi_m_0_SHIFT	(mSPI_mosi_m__0__SHIFT)
#define mSPI_mosi_m_0_INTR	((uint16)((uint16)0x0003u << (mSPI_mosi_m__0__SHIFT*2u)))

#define mSPI_mosi_m_INTR_ALL	 ((uint16)(mSPI_mosi_m_0_INTR))


#endif /* End Pins mSPI_mosi_m_ALIASES_H */


/* [] END OF FILE */
