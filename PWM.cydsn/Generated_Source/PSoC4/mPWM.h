/*******************************************************************************
* File Name: mPWM.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the mPWM
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

#if !defined(CY_TCPWM_mPWM_H)
#define CY_TCPWM_mPWM_H


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
} mPWM_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  mPWM_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define mPWM_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define mPWM_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define mPWM_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define mPWM_QUAD_ENCODING_MODES            (0lu)
#define mPWM_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define mPWM_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define mPWM_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define mPWM_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define mPWM_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define mPWM_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define mPWM_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define mPWM_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define mPWM_TC_RUN_MODE                    (0lu)
#define mPWM_TC_COUNTER_MODE                (0lu)
#define mPWM_TC_COMP_CAP_MODE               (2lu)
#define mPWM_TC_PRESCALER                   (0lu)

/* Signal modes */
#define mPWM_TC_RELOAD_SIGNAL_MODE          (0lu)
#define mPWM_TC_COUNT_SIGNAL_MODE           (3lu)
#define mPWM_TC_START_SIGNAL_MODE           (0lu)
#define mPWM_TC_STOP_SIGNAL_MODE            (0lu)
#define mPWM_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define mPWM_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define mPWM_TC_START_SIGNAL_PRESENT        (0lu)
#define mPWM_TC_STOP_SIGNAL_PRESENT         (0lu)
#define mPWM_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define mPWM_PWM_KILL_EVENT                 (0lu)
#define mPWM_PWM_STOP_EVENT                 (0lu)
#define mPWM_PWM_MODE                       (4lu)
#define mPWM_PWM_OUT_N_INVERT               (0lu)
#define mPWM_PWM_OUT_INVERT                 (0lu)
#define mPWM_PWM_ALIGN                      (0lu)
#define mPWM_PWM_RUN_MODE                   (0lu)
#define mPWM_PWM_DEAD_TIME_CYCLE            (0lu)
#define mPWM_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define mPWM_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define mPWM_PWM_COUNT_SIGNAL_MODE          (3lu)
#define mPWM_PWM_START_SIGNAL_MODE          (0lu)
#define mPWM_PWM_STOP_SIGNAL_MODE           (0lu)
#define mPWM_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define mPWM_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define mPWM_PWM_START_SIGNAL_PRESENT       (0lu)
#define mPWM_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define mPWM_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define mPWM_TC_PERIOD_VALUE                (65535lu)
#define mPWM_TC_COMPARE_VALUE               (65535lu)
#define mPWM_TC_COMPARE_BUF_VALUE           (65535lu)
#define mPWM_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define mPWM_PWM_PERIOD_VALUE               (1000lu)
#define mPWM_PWM_PERIOD_BUF_VALUE           (65535lu)
#define mPWM_PWM_PERIOD_SWAP                (0lu)
#define mPWM_PWM_COMPARE_VALUE              (500lu)
#define mPWM_PWM_COMPARE_BUF_VALUE          (65535lu)
#define mPWM_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define mPWM__LEFT 0
#define mPWM__RIGHT 1
#define mPWM__CENTER 2
#define mPWM__ASYMMETRIC 3

#define mPWM__X1 0
#define mPWM__X2 1
#define mPWM__X4 2

#define mPWM__PWM 4
#define mPWM__PWM_DT 5
#define mPWM__PWM_PR 6

#define mPWM__INVERSE 1
#define mPWM__DIRECT 0

#define mPWM__CAPTURE 2
#define mPWM__COMPARE 0

#define mPWM__TRIG_LEVEL 3
#define mPWM__TRIG_RISING 0
#define mPWM__TRIG_FALLING 1
#define mPWM__TRIG_BOTH 2

#define mPWM__INTR_MASK_TC 1
#define mPWM__INTR_MASK_CC_MATCH 2
#define mPWM__INTR_MASK_NONE 0
#define mPWM__INTR_MASK_TC_CC 3

#define mPWM__UNCONFIG 8
#define mPWM__TIMER 1
#define mPWM__QUAD 3
#define mPWM__PWM_SEL 7

#define mPWM__COUNT_UP 0
#define mPWM__COUNT_DOWN 1
#define mPWM__COUNT_UPDOWN0 2
#define mPWM__COUNT_UPDOWN1 3


/* Prescaler */
#define mPWM_PRESCALE_DIVBY1                ((uint32)(0u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY2                ((uint32)(1u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY4                ((uint32)(2u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY8                ((uint32)(3u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY16               ((uint32)(4u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY32               ((uint32)(5u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY64               ((uint32)(6u << mPWM_PRESCALER_SHIFT))
#define mPWM_PRESCALE_DIVBY128              ((uint32)(7u << mPWM_PRESCALER_SHIFT))

/* TCPWM set modes */
#define mPWM_MODE_TIMER_COMPARE             ((uint32)(mPWM__COMPARE         <<  \
                                                                  mPWM_MODE_SHIFT))
#define mPWM_MODE_TIMER_CAPTURE             ((uint32)(mPWM__CAPTURE         <<  \
                                                                  mPWM_MODE_SHIFT))
#define mPWM_MODE_QUAD                      ((uint32)(mPWM__QUAD            <<  \
                                                                  mPWM_MODE_SHIFT))
#define mPWM_MODE_PWM                       ((uint32)(mPWM__PWM             <<  \
                                                                  mPWM_MODE_SHIFT))
#define mPWM_MODE_PWM_DT                    ((uint32)(mPWM__PWM_DT          <<  \
                                                                  mPWM_MODE_SHIFT))
#define mPWM_MODE_PWM_PR                    ((uint32)(mPWM__PWM_PR          <<  \
                                                                  mPWM_MODE_SHIFT))

/* Quad Modes */
#define mPWM_MODE_X1                        ((uint32)(mPWM__X1              <<  \
                                                                  mPWM_QUAD_MODE_SHIFT))
#define mPWM_MODE_X2                        ((uint32)(mPWM__X2              <<  \
                                                                  mPWM_QUAD_MODE_SHIFT))
#define mPWM_MODE_X4                        ((uint32)(mPWM__X4              <<  \
                                                                  mPWM_QUAD_MODE_SHIFT))

/* Counter modes */
#define mPWM_COUNT_UP                       ((uint32)(mPWM__COUNT_UP        <<  \
                                                                  mPWM_UPDOWN_SHIFT))
#define mPWM_COUNT_DOWN                     ((uint32)(mPWM__COUNT_DOWN      <<  \
                                                                  mPWM_UPDOWN_SHIFT))
#define mPWM_COUNT_UPDOWN0                  ((uint32)(mPWM__COUNT_UPDOWN0   <<  \
                                                                  mPWM_UPDOWN_SHIFT))
#define mPWM_COUNT_UPDOWN1                  ((uint32)(mPWM__COUNT_UPDOWN1   <<  \
                                                                  mPWM_UPDOWN_SHIFT))

/* PWM output invert */
#define mPWM_INVERT_LINE                    ((uint32)(mPWM__INVERSE         <<  \
                                                                  mPWM_INV_OUT_SHIFT))
#define mPWM_INVERT_LINE_N                  ((uint32)(mPWM__INVERSE         <<  \
                                                                  mPWM_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define mPWM_TRIG_RISING                    ((uint32)mPWM__TRIG_RISING)
#define mPWM_TRIG_FALLING                   ((uint32)mPWM__TRIG_FALLING)
#define mPWM_TRIG_BOTH                      ((uint32)mPWM__TRIG_BOTH)
#define mPWM_TRIG_LEVEL                     ((uint32)mPWM__TRIG_LEVEL)

/* Interrupt mask */
#define mPWM_INTR_MASK_TC                   ((uint32)mPWM__INTR_MASK_TC)
#define mPWM_INTR_MASK_CC_MATCH             ((uint32)mPWM__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define mPWM_CC_MATCH_SET                   (0x00u)
#define mPWM_CC_MATCH_CLEAR                 (0x01u)
#define mPWM_CC_MATCH_INVERT                (0x02u)
#define mPWM_CC_MATCH_NO_CHANGE             (0x03u)
#define mPWM_OVERLOW_SET                    (0x00u)
#define mPWM_OVERLOW_CLEAR                  (0x04u)
#define mPWM_OVERLOW_INVERT                 (0x08u)
#define mPWM_OVERLOW_NO_CHANGE              (0x0Cu)
#define mPWM_UNDERFLOW_SET                  (0x00u)
#define mPWM_UNDERFLOW_CLEAR                (0x10u)
#define mPWM_UNDERFLOW_INVERT               (0x20u)
#define mPWM_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define mPWM_PWM_MODE_LEFT                  (mPWM_CC_MATCH_CLEAR        |   \
                                                         mPWM_OVERLOW_SET           |   \
                                                         mPWM_UNDERFLOW_NO_CHANGE)
#define mPWM_PWM_MODE_RIGHT                 (mPWM_CC_MATCH_SET          |   \
                                                         mPWM_OVERLOW_NO_CHANGE     |   \
                                                         mPWM_UNDERFLOW_CLEAR)
#define mPWM_PWM_MODE_ASYM                  (mPWM_CC_MATCH_INVERT       |   \
                                                         mPWM_OVERLOW_SET           |   \
                                                         mPWM_UNDERFLOW_CLEAR)

#if (mPWM_CY_TCPWM_V2)
    #if(mPWM_CY_TCPWM_4000)
        #define mPWM_PWM_MODE_CENTER                (mPWM_CC_MATCH_INVERT       |   \
                                                                 mPWM_OVERLOW_NO_CHANGE     |   \
                                                                 mPWM_UNDERFLOW_CLEAR)
    #else
        #define mPWM_PWM_MODE_CENTER                (mPWM_CC_MATCH_INVERT       |   \
                                                                 mPWM_OVERLOW_SET           |   \
                                                                 mPWM_UNDERFLOW_CLEAR)
    #endif /* (mPWM_CY_TCPWM_4000) */
#else
    #define mPWM_PWM_MODE_CENTER                (mPWM_CC_MATCH_INVERT       |   \
                                                             mPWM_OVERLOW_NO_CHANGE     |   \
                                                             mPWM_UNDERFLOW_CLEAR)
#endif /* (mPWM_CY_TCPWM_NEW) */

/* Command operations without condition */
#define mPWM_CMD_CAPTURE                    (0u)
#define mPWM_CMD_RELOAD                     (8u)
#define mPWM_CMD_STOP                       (16u)
#define mPWM_CMD_START                      (24u)

/* Status */
#define mPWM_STATUS_DOWN                    (1u)
#define mPWM_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   mPWM_Init(void);
void   mPWM_Enable(void);
void   mPWM_Start(void);
void   mPWM_Stop(void);

void   mPWM_SetMode(uint32 mode);
void   mPWM_SetCounterMode(uint32 counterMode);
void   mPWM_SetPWMMode(uint32 modeMask);
void   mPWM_SetQDMode(uint32 qdMode);

void   mPWM_SetPrescaler(uint32 prescaler);
void   mPWM_TriggerCommand(uint32 mask, uint32 command);
void   mPWM_SetOneShot(uint32 oneShotEnable);
uint32 mPWM_ReadStatus(void);

void   mPWM_SetPWMSyncKill(uint32 syncKillEnable);
void   mPWM_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   mPWM_SetPWMDeadTime(uint32 deadTime);
void   mPWM_SetPWMInvert(uint32 mask);

void   mPWM_SetInterruptMode(uint32 interruptMask);
uint32 mPWM_GetInterruptSourceMasked(void);
uint32 mPWM_GetInterruptSource(void);
void   mPWM_ClearInterrupt(uint32 interruptMask);
void   mPWM_SetInterrupt(uint32 interruptMask);

void   mPWM_WriteCounter(uint32 count);
uint32 mPWM_ReadCounter(void);

uint32 mPWM_ReadCapture(void);
uint32 mPWM_ReadCaptureBuf(void);

void   mPWM_WritePeriod(uint32 period);
uint32 mPWM_ReadPeriod(void);
void   mPWM_WritePeriodBuf(uint32 periodBuf);
uint32 mPWM_ReadPeriodBuf(void);

void   mPWM_WriteCompare(uint32 compare);
uint32 mPWM_ReadCompare(void);
void   mPWM_WriteCompareBuf(uint32 compareBuf);
uint32 mPWM_ReadCompareBuf(void);

void   mPWM_SetPeriodSwap(uint32 swapEnable);
void   mPWM_SetCompareSwap(uint32 swapEnable);

void   mPWM_SetCaptureMode(uint32 triggerMode);
void   mPWM_SetReloadMode(uint32 triggerMode);
void   mPWM_SetStartMode(uint32 triggerMode);
void   mPWM_SetStopMode(uint32 triggerMode);
void   mPWM_SetCountMode(uint32 triggerMode);

void   mPWM_SaveConfig(void);
void   mPWM_RestoreConfig(void);
void   mPWM_Sleep(void);
void   mPWM_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define mPWM_BLOCK_CONTROL_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM_BLOCK_CONTROL_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM_COMMAND_REG                    (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM_COMMAND_PTR                    ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM_INTRRUPT_CAUSE_REG             (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM_INTRRUPT_CAUSE_PTR             ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM_CONTROL_REG                    (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__CTRL )
#define mPWM_CONTROL_PTR                    ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__CTRL )
#define mPWM_STATUS_REG                     (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__STATUS )
#define mPWM_STATUS_PTR                     ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__STATUS )
#define mPWM_COUNTER_REG                    (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM_COUNTER_PTR                    ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM_COMP_CAP_REG                   (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__CC )
#define mPWM_COMP_CAP_PTR                   ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__CC )
#define mPWM_COMP_CAP_BUF_REG               (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM_COMP_CAP_BUF_PTR               ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM_PERIOD_REG                     (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM_PERIOD_PTR                     ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM_PERIOD_BUF_REG                 (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM_PERIOD_BUF_PTR                 ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM_TRIG_CONTROL0_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM_TRIG_CONTROL0_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM_TRIG_CONTROL1_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM_TRIG_CONTROL1_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM_TRIG_CONTROL2_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM_TRIG_CONTROL2_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM_INTERRUPT_REQ_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR )
#define mPWM_INTERRUPT_REQ_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR )
#define mPWM_INTERRUPT_SET_REG              (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM_INTERRUPT_SET_PTR              ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM_INTERRUPT_MASK_REG             (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM_INTERRUPT_MASK_PTR             ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM_INTERRUPT_MASKED_REG           (*(reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_MASKED )
#define mPWM_INTERRUPT_MASKED_PTR           ( (reg32 *) mPWM_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define mPWM_MASK                           ((uint32)mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define mPWM_RELOAD_CC_SHIFT                (0u)
#define mPWM_RELOAD_PERIOD_SHIFT            (1u)
#define mPWM_PWM_SYNC_KILL_SHIFT            (2u)
#define mPWM_PWM_STOP_KILL_SHIFT            (3u)
#define mPWM_PRESCALER_SHIFT                (8u)
#define mPWM_UPDOWN_SHIFT                   (16u)
#define mPWM_ONESHOT_SHIFT                  (18u)
#define mPWM_QUAD_MODE_SHIFT                (20u)
#define mPWM_INV_OUT_SHIFT                  (20u)
#define mPWM_INV_COMPL_OUT_SHIFT            (21u)
#define mPWM_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define mPWM_RELOAD_CC_MASK                 ((uint32)(mPWM_1BIT_MASK        <<  \
                                                                            mPWM_RELOAD_CC_SHIFT))
#define mPWM_RELOAD_PERIOD_MASK             ((uint32)(mPWM_1BIT_MASK        <<  \
                                                                            mPWM_RELOAD_PERIOD_SHIFT))
#define mPWM_PWM_SYNC_KILL_MASK             ((uint32)(mPWM_1BIT_MASK        <<  \
                                                                            mPWM_PWM_SYNC_KILL_SHIFT))
#define mPWM_PWM_STOP_KILL_MASK             ((uint32)(mPWM_1BIT_MASK        <<  \
                                                                            mPWM_PWM_STOP_KILL_SHIFT))
#define mPWM_PRESCALER_MASK                 ((uint32)(mPWM_8BIT_MASK        <<  \
                                                                            mPWM_PRESCALER_SHIFT))
#define mPWM_UPDOWN_MASK                    ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                            mPWM_UPDOWN_SHIFT))
#define mPWM_ONESHOT_MASK                   ((uint32)(mPWM_1BIT_MASK        <<  \
                                                                            mPWM_ONESHOT_SHIFT))
#define mPWM_QUAD_MODE_MASK                 ((uint32)(mPWM_3BIT_MASK        <<  \
                                                                            mPWM_QUAD_MODE_SHIFT))
#define mPWM_INV_OUT_MASK                   ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                            mPWM_INV_OUT_SHIFT))
#define mPWM_MODE_MASK                      ((uint32)(mPWM_3BIT_MASK        <<  \
                                                                            mPWM_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define mPWM_CAPTURE_SHIFT                  (0u)
#define mPWM_COUNT_SHIFT                    (2u)
#define mPWM_RELOAD_SHIFT                   (4u)
#define mPWM_STOP_SHIFT                     (6u)
#define mPWM_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define mPWM_CAPTURE_MASK                   ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                  mPWM_CAPTURE_SHIFT))
#define mPWM_COUNT_MASK                     ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                  mPWM_COUNT_SHIFT))
#define mPWM_RELOAD_MASK                    ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                  mPWM_RELOAD_SHIFT))
#define mPWM_STOP_MASK                      ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                  mPWM_STOP_SHIFT))
#define mPWM_START_MASK                     ((uint32)(mPWM_2BIT_MASK        <<  \
                                                                  mPWM_START_SHIFT))

/* MASK */
#define mPWM_1BIT_MASK                      ((uint32)0x01u)
#define mPWM_2BIT_MASK                      ((uint32)0x03u)
#define mPWM_3BIT_MASK                      ((uint32)0x07u)
#define mPWM_6BIT_MASK                      ((uint32)0x3Fu)
#define mPWM_8BIT_MASK                      ((uint32)0xFFu)
#define mPWM_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define mPWM_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define mPWM_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(mPWM_QUAD_ENCODING_MODES     << mPWM_QUAD_MODE_SHIFT))       |\
         ((uint32)(mPWM_CONFIG                  << mPWM_MODE_SHIFT)))

#define mPWM_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(mPWM_PWM_STOP_EVENT          << mPWM_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(mPWM_PWM_OUT_INVERT          << mPWM_INV_OUT_SHIFT))         |\
         ((uint32)(mPWM_PWM_OUT_N_INVERT        << mPWM_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(mPWM_PWM_MODE                << mPWM_MODE_SHIFT)))

#define mPWM_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(mPWM_PWM_RUN_MODE         << mPWM_ONESHOT_SHIFT))
            
#define mPWM_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(mPWM_PWM_ALIGN            << mPWM_UPDOWN_SHIFT))

#define mPWM_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(mPWM_PWM_KILL_EVENT      << mPWM_PWM_SYNC_KILL_SHIFT))

#define mPWM_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(mPWM_PWM_DEAD_TIME_CYCLE  << mPWM_PRESCALER_SHIFT))

#define mPWM_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(mPWM_PWM_PRESCALER        << mPWM_PRESCALER_SHIFT))

#define mPWM_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(mPWM_TC_PRESCALER            << mPWM_PRESCALER_SHIFT))       |\
         ((uint32)(mPWM_TC_COUNTER_MODE         << mPWM_UPDOWN_SHIFT))          |\
         ((uint32)(mPWM_TC_RUN_MODE             << mPWM_ONESHOT_SHIFT))         |\
         ((uint32)(mPWM_TC_COMP_CAP_MODE        << mPWM_MODE_SHIFT)))
        
#define mPWM_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(mPWM_QUAD_PHIA_SIGNAL_MODE   << mPWM_COUNT_SHIFT))           |\
         ((uint32)(mPWM_QUAD_INDEX_SIGNAL_MODE  << mPWM_RELOAD_SHIFT))          |\
         ((uint32)(mPWM_QUAD_STOP_SIGNAL_MODE   << mPWM_STOP_SHIFT))            |\
         ((uint32)(mPWM_QUAD_PHIB_SIGNAL_MODE   << mPWM_START_SHIFT)))

#define mPWM_PWM_SIGNALS_MODES                                                              \
        (((uint32)(mPWM_PWM_SWITCH_SIGNAL_MODE  << mPWM_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM_PWM_COUNT_SIGNAL_MODE   << mPWM_COUNT_SHIFT))           |\
         ((uint32)(mPWM_PWM_RELOAD_SIGNAL_MODE  << mPWM_RELOAD_SHIFT))          |\
         ((uint32)(mPWM_PWM_STOP_SIGNAL_MODE    << mPWM_STOP_SHIFT))            |\
         ((uint32)(mPWM_PWM_START_SIGNAL_MODE   << mPWM_START_SHIFT)))

#define mPWM_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(mPWM_TC_CAPTURE_SIGNAL_MODE  << mPWM_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM_TC_COUNT_SIGNAL_MODE    << mPWM_COUNT_SHIFT))           |\
         ((uint32)(mPWM_TC_RELOAD_SIGNAL_MODE   << mPWM_RELOAD_SHIFT))          |\
         ((uint32)(mPWM_TC_STOP_SIGNAL_MODE     << mPWM_STOP_SHIFT))            |\
         ((uint32)(mPWM_TC_START_SIGNAL_MODE    << mPWM_START_SHIFT)))
        
#define mPWM_TIMER_UPDOWN_CNT_USED                                                          \
                ((mPWM__COUNT_UPDOWN0 == mPWM_TC_COUNTER_MODE)                  ||\
                 (mPWM__COUNT_UPDOWN1 == mPWM_TC_COUNTER_MODE))

#define mPWM_PWM_UPDOWN_CNT_USED                                                            \
                ((mPWM__CENTER == mPWM_PWM_ALIGN)                               ||\
                 (mPWM__ASYMMETRIC == mPWM_PWM_ALIGN))               
        
#define mPWM_PWM_PR_INIT_VALUE              (1u)
#define mPWM_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_mPWM_H */

/* [] END OF FILE */
