/*******************************************************************************
* File Name: mUART_SCBCLK.h
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

#if !defined(CY_CLOCK_mUART_SCBCLK_H)
#define CY_CLOCK_mUART_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void mUART_SCBCLK_StartEx(uint32 alignClkDiv);
#define mUART_SCBCLK_Start() \
    mUART_SCBCLK_StartEx(mUART_SCBCLK__PA_DIV_ID)

#else

void mUART_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void mUART_SCBCLK_Stop(void);

void mUART_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 mUART_SCBCLK_GetDividerRegister(void);
uint8  mUART_SCBCLK_GetFractionalDividerRegister(void);

#define mUART_SCBCLK_Enable()                         mUART_SCBCLK_Start()
#define mUART_SCBCLK_Disable()                        mUART_SCBCLK_Stop()
#define mUART_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    mUART_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define mUART_SCBCLK_SetDivider(clkDivider)           mUART_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define mUART_SCBCLK_SetDividerValue(clkDivider)      mUART_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define mUART_SCBCLK_DIV_ID     mUART_SCBCLK__DIV_ID

#define mUART_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define mUART_SCBCLK_CTRL_REG   (*(reg32 *)mUART_SCBCLK__CTRL_REGISTER)
#define mUART_SCBCLK_DIV_REG    (*(reg32 *)mUART_SCBCLK__DIV_REGISTER)

#define mUART_SCBCLK_CMD_DIV_SHIFT          (0u)
#define mUART_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define mUART_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define mUART_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define mUART_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << mUART_SCBCLK_CMD_DISABLE_SHIFT))
#define mUART_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << mUART_SCBCLK_CMD_ENABLE_SHIFT))

#define mUART_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define mUART_SCBCLK_DIV_FRAC_SHIFT (3u)
#define mUART_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define mUART_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define mUART_SCBCLK_DIV_REG        (*(reg32 *)mUART_SCBCLK__REGISTER)
#define mUART_SCBCLK_ENABLE_REG     mUART_SCBCLK_DIV_REG
#define mUART_SCBCLK_DIV_FRAC_MASK  mUART_SCBCLK__FRAC_MASK
#define mUART_SCBCLK_DIV_FRAC_SHIFT (16u)
#define mUART_SCBCLK_DIV_INT_MASK   mUART_SCBCLK__DIVIDER_MASK
#define mUART_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_mUART_SCBCLK_H) */

/* [] END OF FILE */
