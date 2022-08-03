/*******************************************************************************
* File Name: C12M.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_C12M_H)
#define CY_CLOCK_C12M_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void C12M_StartEx(uint32 alignClkDiv);
#define C12M_Start() \
    C12M_StartEx(C12M__PA_DIV_ID)

#else

void C12M_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void C12M_Stop(void);

void C12M_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 C12M_GetDividerRegister(void);
uint8  C12M_GetFractionalDividerRegister(void);

#define C12M_Enable()                         C12M_Start()
#define C12M_Disable()                        C12M_Stop()
#define C12M_SetDividerRegister(clkDivider, reset)  \
    C12M_SetFractionalDividerRegister((clkDivider), 0u)
#define C12M_SetDivider(clkDivider)           C12M_SetDividerRegister((clkDivider), 1u)
#define C12M_SetDividerValue(clkDivider)      C12M_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define C12M_DIV_ID     C12M__DIV_ID

#define C12M_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define C12M_CTRL_REG   (*(reg32 *)C12M__CTRL_REGISTER)
#define C12M_DIV_REG    (*(reg32 *)C12M__DIV_REGISTER)

#define C12M_CMD_DIV_SHIFT          (0u)
#define C12M_CMD_PA_DIV_SHIFT       (8u)
#define C12M_CMD_DISABLE_SHIFT      (30u)
#define C12M_CMD_ENABLE_SHIFT       (31u)

#define C12M_CMD_DISABLE_MASK       ((uint32)((uint32)1u << C12M_CMD_DISABLE_SHIFT))
#define C12M_CMD_ENABLE_MASK        ((uint32)((uint32)1u << C12M_CMD_ENABLE_SHIFT))

#define C12M_DIV_FRAC_MASK  (0x000000F8u)
#define C12M_DIV_FRAC_SHIFT (3u)
#define C12M_DIV_INT_MASK   (0xFFFFFF00u)
#define C12M_DIV_INT_SHIFT  (8u)

#else 

#define C12M_DIV_REG        (*(reg32 *)C12M__REGISTER)
#define C12M_ENABLE_REG     C12M_DIV_REG
#define C12M_DIV_FRAC_MASK  C12M__FRAC_MASK
#define C12M_DIV_FRAC_SHIFT (16u)
#define C12M_DIV_INT_MASK   C12M__DIVIDER_MASK
#define C12M_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_C12M_H) */

/* [] END OF FILE */
