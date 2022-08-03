/*******************************************************************************
* File Name: mPWM2.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the mPWM2
*  component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_TCPWM_mPWM2_H)
#define CY_TCPWM_mPWM2_H


#include "CyLib.h"
#include "cytypes.h"
#include "cyfitter.h"


/*******************************************************************************
* Internal Type defines
*******************************************************************************/

/* Structure to save state before go to sleep */
typedef struct
{
    uint8  enableState;
} mPWM2_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  mPWM2_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define mPWM2_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define mPWM2_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define mPWM2_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define mPWM2_QUAD_ENCODING_MODES            (0lu)
#define mPWM2_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define mPWM2_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define mPWM2_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define mPWM2_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define mPWM2_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define mPWM2_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define mPWM2_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define mPWM2_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define mPWM2_TC_RUN_MODE                    (0lu)
#define mPWM2_TC_COUNTER_MODE                (0lu)
#define mPWM2_TC_COMP_CAP_MODE               (2lu)
#define mPWM2_TC_PRESCALER                   (0lu)

/* Signal modes */
#define mPWM2_TC_RELOAD_SIGNAL_MODE          (0lu)
#define mPWM2_TC_COUNT_SIGNAL_MODE           (3lu)
#define mPWM2_TC_START_SIGNAL_MODE           (0lu)
#define mPWM2_TC_STOP_SIGNAL_MODE            (0lu)
#define mPWM2_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM2_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define mPWM2_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define mPWM2_TC_START_SIGNAL_PRESENT        (0lu)
#define mPWM2_TC_STOP_SIGNAL_PRESENT         (0lu)
#define mPWM2_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM2_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define mPWM2_PWM_KILL_EVENT                 (0lu)
#define mPWM2_PWM_STOP_EVENT                 (0lu)
#define mPWM2_PWM_MODE                       (4lu)
#define mPWM2_PWM_OUT_N_INVERT               (0lu)
#define mPWM2_PWM_OUT_INVERT                 (0lu)
#define mPWM2_PWM_ALIGN                      (0lu)
#define mPWM2_PWM_RUN_MODE                   (0lu)
#define mPWM2_PWM_DEAD_TIME_CYCLE            (0lu)
#define mPWM2_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define mPWM2_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define mPWM2_PWM_COUNT_SIGNAL_MODE          (3lu)
#define mPWM2_PWM_START_SIGNAL_MODE          (0lu)
#define mPWM2_PWM_STOP_SIGNAL_MODE           (0lu)
#define mPWM2_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM2_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define mPWM2_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define mPWM2_PWM_START_SIGNAL_PRESENT       (0lu)
#define mPWM2_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define mPWM2_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM2_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define mPWM2_TC_PERIOD_VALUE                (65535lu)
#define mPWM2_TC_COMPARE_VALUE               (65535lu)
#define mPWM2_TC_COMPARE_BUF_VALUE           (65535lu)
#define mPWM2_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define mPWM2_PWM_PERIOD_VALUE               (1000lu)
#define mPWM2_PWM_PERIOD_BUF_VALUE           (65535lu)
#define mPWM2_PWM_PERIOD_SWAP                (0lu)
#define mPWM2_PWM_COMPARE_VALUE              (500lu)
#define mPWM2_PWM_COMPARE_BUF_VALUE          (65535lu)
#define mPWM2_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define mPWM2__LEFT 0
#define mPWM2__RIGHT 1
#define mPWM2__CENTER 2
#define mPWM2__ASYMMETRIC 3

#define mPWM2__X1 0
#define mPWM2__X2 1
#define mPWM2__X4 2

#define mPWM2__PWM 4
#define mPWM2__PWM_DT 5
#define mPWM2__PWM_PR 6

#define mPWM2__INVERSE 1
#define mPWM2__DIRECT 0

#define mPWM2__CAPTURE 2
#define mPWM2__COMPARE 0

#define mPWM2__TRIG_LEVEL 3
#define mPWM2__TRIG_RISING 0
#define mPWM2__TRIG_FALLING 1
#define mPWM2__TRIG_BOTH 2

#define mPWM2__INTR_MASK_TC 1
#define mPWM2__INTR_MASK_CC_MATCH 2
#define mPWM2__INTR_MASK_NONE 0
#define mPWM2__INTR_MASK_TC_CC 3

#define mPWM2__UNCONFIG 8
#define mPWM2__TIMER 1
#define mPWM2__QUAD 3
#define mPWM2__PWM_SEL 7

#define mPWM2__COUNT_UP 0
#define mPWM2__COUNT_DOWN 1
#define mPWM2__COUNT_UPDOWN0 2
#define mPWM2__COUNT_UPDOWN1 3


/* Prescaler */
#define mPWM2_PRESCALE_DIVBY1                ((uint32)(0u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY2                ((uint32)(1u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY4                ((uint32)(2u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY8                ((uint32)(3u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY16               ((uint32)(4u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY32               ((uint32)(5u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY64               ((uint32)(6u << mPWM2_PRESCALER_SHIFT))
#define mPWM2_PRESCALE_DIVBY128              ((uint32)(7u << mPWM2_PRESCALER_SHIFT))

/* TCPWM set modes */
#define mPWM2_MODE_TIMER_COMPARE             ((uint32)(mPWM2__COMPARE         <<  \
                                                                  mPWM2_MODE_SHIFT))
#define mPWM2_MODE_TIMER_CAPTURE             ((uint32)(mPWM2__CAPTURE         <<  \
                                                                  mPWM2_MODE_SHIFT))
#define mPWM2_MODE_QUAD                      ((uint32)(mPWM2__QUAD            <<  \
                                                                  mPWM2_MODE_SHIFT))
#define mPWM2_MODE_PWM                       ((uint32)(mPWM2__PWM             <<  \
                                                                  mPWM2_MODE_SHIFT))
#define mPWM2_MODE_PWM_DT                    ((uint32)(mPWM2__PWM_DT          <<  \
                                                                  mPWM2_MODE_SHIFT))
#define mPWM2_MODE_PWM_PR                    ((uint32)(mPWM2__PWM_PR          <<  \
                                                                  mPWM2_MODE_SHIFT))

/* Quad Modes */
#define mPWM2_MODE_X1                        ((uint32)(mPWM2__X1              <<  \
                                                                  mPWM2_QUAD_MODE_SHIFT))
#define mPWM2_MODE_X2                        ((uint32)(mPWM2__X2              <<  \
                                                                  mPWM2_QUAD_MODE_SHIFT))
#define mPWM2_MODE_X4                        ((uint32)(mPWM2__X4              <<  \
                                                                  mPWM2_QUAD_MODE_SHIFT))

/* Counter modes */
#define mPWM2_COUNT_UP                       ((uint32)(mPWM2__COUNT_UP        <<  \
                                                                  mPWM2_UPDOWN_SHIFT))
#define mPWM2_COUNT_DOWN                     ((uint32)(mPWM2__COUNT_DOWN      <<  \
                                                                  mPWM2_UPDOWN_SHIFT))
#define mPWM2_COUNT_UPDOWN0                  ((uint32)(mPWM2__COUNT_UPDOWN0   <<  \
                                                                  mPWM2_UPDOWN_SHIFT))
#define mPWM2_COUNT_UPDOWN1                  ((uint32)(mPWM2__COUNT_UPDOWN1   <<  \
                                                                  mPWM2_UPDOWN_SHIFT))

/* PWM output invert */
#define mPWM2_INVERT_LINE                    ((uint32)(mPWM2__INVERSE         <<  \
                                                                  mPWM2_INV_OUT_SHIFT))
#define mPWM2_INVERT_LINE_N                  ((uint32)(mPWM2__INVERSE         <<  \
                                                                  mPWM2_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define mPWM2_TRIG_RISING                    ((uint32)mPWM2__TRIG_RISING)
#define mPWM2_TRIG_FALLING                   ((uint32)mPWM2__TRIG_FALLING)
#define mPWM2_TRIG_BOTH                      ((uint32)mPWM2__TRIG_BOTH)
#define mPWM2_TRIG_LEVEL                     ((uint32)mPWM2__TRIG_LEVEL)

/* Interrupt mask */
#define mPWM2_INTR_MASK_TC                   ((uint32)mPWM2__INTR_MASK_TC)
#define mPWM2_INTR_MASK_CC_MATCH             ((uint32)mPWM2__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define mPWM2_CC_MATCH_SET                   (0x00u)
#define mPWM2_CC_MATCH_CLEAR                 (0x01u)
#define mPWM2_CC_MATCH_INVERT                (0x02u)
#define mPWM2_CC_MATCH_NO_CHANGE             (0x03u)
#define mPWM2_OVERLOW_SET                    (0x00u)
#define mPWM2_OVERLOW_CLEAR                  (0x04u)
#define mPWM2_OVERLOW_INVERT                 (0x08u)
#define mPWM2_OVERLOW_NO_CHANGE              (0x0Cu)
#define mPWM2_UNDERFLOW_SET                  (0x00u)
#define mPWM2_UNDERFLOW_CLEAR                (0x10u)
#define mPWM2_UNDERFLOW_INVERT               (0x20u)
#define mPWM2_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define mPWM2_PWM_MODE_LEFT                  (mPWM2_CC_MATCH_CLEAR        |   \
                                                         mPWM2_OVERLOW_SET           |   \
                                                         mPWM2_UNDERFLOW_NO_CHANGE)
#define mPWM2_PWM_MODE_RIGHT                 (mPWM2_CC_MATCH_SET          |   \
                                                         mPWM2_OVERLOW_NO_CHANGE     |   \
                                                         mPWM2_UNDERFLOW_CLEAR)
#define mPWM2_PWM_MODE_ASYM                  (mPWM2_CC_MATCH_INVERT       |   \
                                                         mPWM2_OVERLOW_SET           |   \
                                                         mPWM2_UNDERFLOW_CLEAR)

#if (mPWM2_CY_TCPWM_V2)
    #if(mPWM2_CY_TCPWM_4000)
        #define mPWM2_PWM_MODE_CENTER                (mPWM2_CC_MATCH_INVERT       |   \
                                                                 mPWM2_OVERLOW_NO_CHANGE     |   \
                                                                 mPWM2_UNDERFLOW_CLEAR)
    #else
        #define mPWM2_PWM_MODE_CENTER                (mPWM2_CC_MATCH_INVERT       |   \
                                                                 mPWM2_OVERLOW_SET           |   \
                                                                 mPWM2_UNDERFLOW_CLEAR)
    #endif /* (mPWM2_CY_TCPWM_4000) */
#else
    #define mPWM2_PWM_MODE_CENTER                (mPWM2_CC_MATCH_INVERT       |   \
                                                             mPWM2_OVERLOW_NO_CHANGE     |   \
                                                             mPWM2_UNDERFLOW_CLEAR)
#endif /* (mPWM2_CY_TCPWM_NEW) */

/* Command operations without condition */
#define mPWM2_CMD_CAPTURE                    (0u)
#define mPWM2_CMD_RELOAD                     (8u)
#define mPWM2_CMD_STOP                       (16u)
#define mPWM2_CMD_START                      (24u)

/* Status */
#define mPWM2_STATUS_DOWN                    (1u)
#define mPWM2_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   mPWM2_Init(void);
void   mPWM2_Enable(void);
void   mPWM2_Start(void);
void   mPWM2_Stop(void);

void   mPWM2_SetMode(uint32 mode);
void   mPWM2_SetCounterMode(uint32 counterMode);
void   mPWM2_SetPWMMode(uint32 modeMask);
void   mPWM2_SetQDMode(uint32 qdMode);

void   mPWM2_SetPrescaler(uint32 prescaler);
void   mPWM2_TriggerCommand(uint32 mask, uint32 command);
void   mPWM2_SetOneShot(uint32 oneShotEnable);
uint32 mPWM2_ReadStatus(void);

void   mPWM2_SetPWMSyncKill(uint32 syncKillEnable);
void   mPWM2_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   mPWM2_SetPWMDeadTime(uint32 deadTime);
void   mPWM2_SetPWMInvert(uint32 mask);

void   mPWM2_SetInterruptMode(uint32 interruptMask);
uint32 mPWM2_GetInterruptSourceMasked(void);
uint32 mPWM2_GetInterruptSource(void);
void   mPWM2_ClearInterrupt(uint32 interruptMask);
void   mPWM2_SetInterrupt(uint32 interruptMask);

void   mPWM2_WriteCounter(uint32 count);
uint32 mPWM2_ReadCounter(void);

uint32 mPWM2_ReadCapture(void);
uint32 mPWM2_ReadCaptureBuf(void);

void   mPWM2_WritePeriod(uint32 period);
uint32 mPWM2_ReadPeriod(void);
void   mPWM2_WritePeriodBuf(uint32 periodBuf);
uint32 mPWM2_ReadPeriodBuf(void);

void   mPWM2_WriteCompare(uint32 compare);
uint32 mPWM2_ReadCompare(void);
void   mPWM2_WriteCompareBuf(uint32 compareBuf);
uint32 mPWM2_ReadCompareBuf(void);

void   mPWM2_SetPeriodSwap(uint32 swapEnable);
void   mPWM2_SetCompareSwap(uint32 swapEnable);

void   mPWM2_SetCaptureMode(uint32 triggerMode);
void   mPWM2_SetReloadMode(uint32 triggerMode);
void   mPWM2_SetStartMode(uint32 triggerMode);
void   mPWM2_SetStopMode(uint32 triggerMode);
void   mPWM2_SetCountMode(uint32 triggerMode);

void   mPWM2_SaveConfig(void);
void   mPWM2_RestoreConfig(void);
void   mPWM2_Sleep(void);
void   mPWM2_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define mPWM2_BLOCK_CONTROL_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM2_BLOCK_CONTROL_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM2_COMMAND_REG                    (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM2_COMMAND_PTR                    ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM2_INTRRUPT_CAUSE_REG             (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM2_INTRRUPT_CAUSE_PTR             ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM2_CONTROL_REG                    (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__CTRL )
#define mPWM2_CONTROL_PTR                    ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__CTRL )
#define mPWM2_STATUS_REG                     (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__STATUS )
#define mPWM2_STATUS_PTR                     ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__STATUS )
#define mPWM2_COUNTER_REG                    (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM2_COUNTER_PTR                    ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM2_COMP_CAP_REG                   (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__CC )
#define mPWM2_COMP_CAP_PTR                   ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__CC )
#define mPWM2_COMP_CAP_BUF_REG               (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM2_COMP_CAP_BUF_PTR               ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM2_PERIOD_REG                     (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM2_PERIOD_PTR                     ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM2_PERIOD_BUF_REG                 (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM2_PERIOD_BUF_PTR                 ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM2_TRIG_CONTROL0_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM2_TRIG_CONTROL0_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM2_TRIG_CONTROL1_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM2_TRIG_CONTROL1_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM2_TRIG_CONTROL2_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM2_TRIG_CONTROL2_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM2_INTERRUPT_REQ_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR )
#define mPWM2_INTERRUPT_REQ_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR )
#define mPWM2_INTERRUPT_SET_REG              (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM2_INTERRUPT_SET_PTR              ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM2_INTERRUPT_MASK_REG             (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM2_INTERRUPT_MASK_PTR             ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM2_INTERRUPT_MASKED_REG           (*(reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_MASKED )
#define mPWM2_INTERRUPT_MASKED_PTR           ( (reg32 *) mPWM2_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define mPWM2_MASK                           ((uint32)mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define mPWM2_RELOAD_CC_SHIFT                (0u)
#define mPWM2_RELOAD_PERIOD_SHIFT            (1u)
#define mPWM2_PWM_SYNC_KILL_SHIFT            (2u)
#define mPWM2_PWM_STOP_KILL_SHIFT            (3u)
#define mPWM2_PRESCALER_SHIFT                (8u)
#define mPWM2_UPDOWN_SHIFT                   (16u)
#define mPWM2_ONESHOT_SHIFT                  (18u)
#define mPWM2_QUAD_MODE_SHIFT                (20u)
#define mPWM2_INV_OUT_SHIFT                  (20u)
#define mPWM2_INV_COMPL_OUT_SHIFT            (21u)
#define mPWM2_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define mPWM2_RELOAD_CC_MASK                 ((uint32)(mPWM2_1BIT_MASK        <<  \
                                                                            mPWM2_RELOAD_CC_SHIFT))
#define mPWM2_RELOAD_PERIOD_MASK             ((uint32)(mPWM2_1BIT_MASK        <<  \
                                                                            mPWM2_RELOAD_PERIOD_SHIFT))
#define mPWM2_PWM_SYNC_KILL_MASK             ((uint32)(mPWM2_1BIT_MASK        <<  \
                                                                            mPWM2_PWM_SYNC_KILL_SHIFT))
#define mPWM2_PWM_STOP_KILL_MASK             ((uint32)(mPWM2_1BIT_MASK        <<  \
                                                                            mPWM2_PWM_STOP_KILL_SHIFT))
#define mPWM2_PRESCALER_MASK                 ((uint32)(mPWM2_8BIT_MASK        <<  \
                                                                            mPWM2_PRESCALER_SHIFT))
#define mPWM2_UPDOWN_MASK                    ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                            mPWM2_UPDOWN_SHIFT))
#define mPWM2_ONESHOT_MASK                   ((uint32)(mPWM2_1BIT_MASK        <<  \
                                                                            mPWM2_ONESHOT_SHIFT))
#define mPWM2_QUAD_MODE_MASK                 ((uint32)(mPWM2_3BIT_MASK        <<  \
                                                                            mPWM2_QUAD_MODE_SHIFT))
#define mPWM2_INV_OUT_MASK                   ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                            mPWM2_INV_OUT_SHIFT))
#define mPWM2_MODE_MASK                      ((uint32)(mPWM2_3BIT_MASK        <<  \
                                                                            mPWM2_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define mPWM2_CAPTURE_SHIFT                  (0u)
#define mPWM2_COUNT_SHIFT                    (2u)
#define mPWM2_RELOAD_SHIFT                   (4u)
#define mPWM2_STOP_SHIFT                     (6u)
#define mPWM2_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define mPWM2_CAPTURE_MASK                   ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                  mPWM2_CAPTURE_SHIFT))
#define mPWM2_COUNT_MASK                     ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                  mPWM2_COUNT_SHIFT))
#define mPWM2_RELOAD_MASK                    ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                  mPWM2_RELOAD_SHIFT))
#define mPWM2_STOP_MASK                      ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                  mPWM2_STOP_SHIFT))
#define mPWM2_START_MASK                     ((uint32)(mPWM2_2BIT_MASK        <<  \
                                                                  mPWM2_START_SHIFT))

/* MASK */
#define mPWM2_1BIT_MASK                      ((uint32)0x01u)
#define mPWM2_2BIT_MASK                      ((uint32)0x03u)
#define mPWM2_3BIT_MASK                      ((uint32)0x07u)
#define mPWM2_6BIT_MASK                      ((uint32)0x3Fu)
#define mPWM2_8BIT_MASK                      ((uint32)0xFFu)
#define mPWM2_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define mPWM2_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define mPWM2_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(mPWM2_QUAD_ENCODING_MODES     << mPWM2_QUAD_MODE_SHIFT))       |\
         ((uint32)(mPWM2_CONFIG                  << mPWM2_MODE_SHIFT)))

#define mPWM2_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(mPWM2_PWM_STOP_EVENT          << mPWM2_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(mPWM2_PWM_OUT_INVERT          << mPWM2_INV_OUT_SHIFT))         |\
         ((uint32)(mPWM2_PWM_OUT_N_INVERT        << mPWM2_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(mPWM2_PWM_MODE                << mPWM2_MODE_SHIFT)))

#define mPWM2_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(mPWM2_PWM_RUN_MODE         << mPWM2_ONESHOT_SHIFT))
            
#define mPWM2_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(mPWM2_PWM_ALIGN            << mPWM2_UPDOWN_SHIFT))

#define mPWM2_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(mPWM2_PWM_KILL_EVENT      << mPWM2_PWM_SYNC_KILL_SHIFT))

#define mPWM2_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(mPWM2_PWM_DEAD_TIME_CYCLE  << mPWM2_PRESCALER_SHIFT))

#define mPWM2_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(mPWM2_PWM_PRESCALER        << mPWM2_PRESCALER_SHIFT))

#define mPWM2_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(mPWM2_TC_PRESCALER            << mPWM2_PRESCALER_SHIFT))       |\
         ((uint32)(mPWM2_TC_COUNTER_MODE         << mPWM2_UPDOWN_SHIFT))          |\
         ((uint32)(mPWM2_TC_RUN_MODE             << mPWM2_ONESHOT_SHIFT))         |\
         ((uint32)(mPWM2_TC_COMP_CAP_MODE        << mPWM2_MODE_SHIFT)))
        
#define mPWM2_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(mPWM2_QUAD_PHIA_SIGNAL_MODE   << mPWM2_COUNT_SHIFT))           |\
         ((uint32)(mPWM2_QUAD_INDEX_SIGNAL_MODE  << mPWM2_RELOAD_SHIFT))          |\
         ((uint32)(mPWM2_QUAD_STOP_SIGNAL_MODE   << mPWM2_STOP_SHIFT))            |\
         ((uint32)(mPWM2_QUAD_PHIB_SIGNAL_MODE   << mPWM2_START_SHIFT)))

#define mPWM2_PWM_SIGNALS_MODES                                                              \
        (((uint32)(mPWM2_PWM_SWITCH_SIGNAL_MODE  << mPWM2_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM2_PWM_COUNT_SIGNAL_MODE   << mPWM2_COUNT_SHIFT))           |\
         ((uint32)(mPWM2_PWM_RELOAD_SIGNAL_MODE  << mPWM2_RELOAD_SHIFT))          |\
         ((uint32)(mPWM2_PWM_STOP_SIGNAL_MODE    << mPWM2_STOP_SHIFT))            |\
         ((uint32)(mPWM2_PWM_START_SIGNAL_MODE   << mPWM2_START_SHIFT)))

#define mPWM2_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(mPWM2_TC_CAPTURE_SIGNAL_MODE  << mPWM2_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM2_TC_COUNT_SIGNAL_MODE    << mPWM2_COUNT_SHIFT))           |\
         ((uint32)(mPWM2_TC_RELOAD_SIGNAL_MODE   << mPWM2_RELOAD_SHIFT))          |\
         ((uint32)(mPWM2_TC_STOP_SIGNAL_MODE     << mPWM2_STOP_SHIFT))            |\
         ((uint32)(mPWM2_TC_START_SIGNAL_MODE    << mPWM2_START_SHIFT)))
        
#define mPWM2_TIMER_UPDOWN_CNT_USED                                                          \
                ((mPWM2__COUNT_UPDOWN0 == mPWM2_TC_COUNTER_MODE)                  ||\
                 (mPWM2__COUNT_UPDOWN1 == mPWM2_TC_COUNTER_MODE))

#define mPWM2_PWM_UPDOWN_CNT_USED                                                            \
                ((mPWM2__CENTER == mPWM2_PWM_ALIGN)                               ||\
                 (mPWM2__ASYMMETRIC == mPWM2_PWM_ALIGN))               
        
#define mPWM2_PWM_PR_INIT_VALUE              (1u)
#define mPWM2_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_mPWM2_H */

/* [] END OF FILE */
