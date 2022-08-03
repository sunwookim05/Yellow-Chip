/*******************************************************************************
* File Name: mI2C_SCBCLK.h
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

#if !defined(CY_CLOCK_mI2C_SCBCLK_H)
#define CY_CLOCK_mI2C_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void mI2C_SCBCLK_StartEx(uint32 alignClkDiv);
#define mI2C_SCBCLK_Start() \
    mI2C_SCBCLK_StartEx(mI2C_SCBCLK__PA_DIV_ID)

#else

void mI2C_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void mI2C_SCBCLK_Stop(void);

void mI2C_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 mI2C_SCBCLK_GetDividerRegister(void);
uint8  mI2C_SCBCLK_GetFractionalDividerRegister(void);

#define mI2C_SCBCLK_Enable()                         mI2C_SCBCLK_Start()
#define mI2C_SCBCLK_Disable()                        mI2C_SCBCLK_Stop()
#define mI2C_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    mI2C_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define mI2C_SCBCLK_SetDivider(clkDivider)           mI2C_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define mI2C_SCBCLK_SetDividerValue(clkDivider)      mI2C_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define mI2C_SCBCLK_DIV_ID     mI2C_SCBCLK__DIV_ID

#define mI2C_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define mI2C_SCBCLK_CTRL_REG   (*(reg32 *)mI2C_SCBCLK__CTRL_REGISTER)
#define mI2C_SCBCLK_DIV_REG    (*(reg32 *)mI2C_SCBCLK__DIV_REGISTER)

#define mI2C_SCBCLK_CMD_DIV_SHIFT          (0u)
#define mI2C_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define mI2C_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define mI2C_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define mI2C_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << mI2C_SCBCLK_CMD_DISABLE_SHIFT))
#define mI2C_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << mI2C_SCBCLK_CMD_ENABLE_SHIFT))

#define mI2C_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define mI2C_SCBCLK_DIV_FRAC_SHIFT (3u)
#define mI2C_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define mI2C_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define mI2C_SCBCLK_DIV_REG        (*(reg32 *)mI2C_SCBCLK__REGISTER)
#define mI2C_SCBCLK_ENABLE_REG     mI2C_SCBCLK_DIV_REG
#define mI2C_SCBCLK_DIV_FRAC_MASK  mI2C_SCBCLK__FRAC_MASK
#define mI2C_SCBCLK_DIV_FRAC_SHIFT (16u)
#define mI2C_SCBCLK_DIV_INT_MASK   mI2C_SCBCLK__DIVIDER_MASK
#define mI2C_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_mI2C_SCBCLK_H) */

/* [] END OF FILE */
