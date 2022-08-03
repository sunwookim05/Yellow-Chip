/*******************************************************************************
* File Name: mSPI_SCBCLK.h
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

#if !defined(CY_CLOCK_mSPI_SCBCLK_H)
#define CY_CLOCK_mSPI_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void mSPI_SCBCLK_StartEx(uint32 alignClkDiv);
#define mSPI_SCBCLK_Start() \
    mSPI_SCBCLK_StartEx(mSPI_SCBCLK__PA_DIV_ID)

#else

void mSPI_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void mSPI_SCBCLK_Stop(void);

void mSPI_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 mSPI_SCBCLK_GetDividerRegister(void);
uint8  mSPI_SCBCLK_GetFractionalDividerRegister(void);

#define mSPI_SCBCLK_Enable()                         mSPI_SCBCLK_Start()
#define mSPI_SCBCLK_Disable()                        mSPI_SCBCLK_Stop()
#define mSPI_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    mSPI_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define mSPI_SCBCLK_SetDivider(clkDivider)           mSPI_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define mSPI_SCBCLK_SetDividerValue(clkDivider)      mSPI_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define mSPI_SCBCLK_DIV_ID     mSPI_SCBCLK__DIV_ID

#define mSPI_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define mSPI_SCBCLK_CTRL_REG   (*(reg32 *)mSPI_SCBCLK__CTRL_REGISTER)
#define mSPI_SCBCLK_DIV_REG    (*(reg32 *)mSPI_SCBCLK__DIV_REGISTER)

#define mSPI_SCBCLK_CMD_DIV_SHIFT          (0u)
#define mSPI_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define mSPI_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define mSPI_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define mSPI_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << mSPI_SCBCLK_CMD_DISABLE_SHIFT))
#define mSPI_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << mSPI_SCBCLK_CMD_ENABLE_SHIFT))

#define mSPI_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define mSPI_SCBCLK_DIV_FRAC_SHIFT (3u)
#define mSPI_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define mSPI_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define mSPI_SCBCLK_DIV_REG        (*(reg32 *)mSPI_SCBCLK__REGISTER)
#define mSPI_SCBCLK_ENABLE_REG     mSPI_SCBCLK_DIV_REG
#define mSPI_SCBCLK_DIV_FRAC_MASK  mSPI_SCBCLK__FRAC_MASK
#define mSPI_SCBCLK_DIV_FRAC_SHIFT (16u)
#define mSPI_SCBCLK_DIV_INT_MASK   mSPI_SCBCLK__DIVIDER_MASK
#define mSPI_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_mSPI_SCBCLK_H) */

/* [] END OF FILE */
