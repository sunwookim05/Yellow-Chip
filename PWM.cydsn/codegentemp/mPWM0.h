/*******************************************************************************
* File Name: mPWM0.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the mPWM0
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

#if !defined(CY_TCPWM_mPWM0_H)
#define CY_TCPWM_mPWM0_H


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
} mPWM0_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  mPWM0_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define mPWM0_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define mPWM0_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define mPWM0_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define mPWM0_QUAD_ENCODING_MODES            (0lu)
#define mPWM0_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define mPWM0_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define mPWM0_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define mPWM0_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define mPWM0_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define mPWM0_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define mPWM0_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define mPWM0_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define mPWM0_TC_RUN_MODE                    (0lu)
#define mPWM0_TC_COUNTER_MODE                (0lu)
#define mPWM0_TC_COMP_CAP_MODE               (2lu)
#define mPWM0_TC_PRESCALER                   (0lu)

/* Signal modes */
#define mPWM0_TC_RELOAD_SIGNAL_MODE          (0lu)
#define mPWM0_TC_COUNT_SIGNAL_MODE           (3lu)
#define mPWM0_TC_START_SIGNAL_MODE           (0lu)
#define mPWM0_TC_STOP_SIGNAL_MODE            (0lu)
#define mPWM0_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM0_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define mPWM0_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define mPWM0_TC_START_SIGNAL_PRESENT        (0lu)
#define mPWM0_TC_STOP_SIGNAL_PRESENT         (0lu)
#define mPWM0_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM0_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define mPWM0_PWM_KILL_EVENT                 (0lu)
#define mPWM0_PWM_STOP_EVENT                 (0lu)
#define mPWM0_PWM_MODE                       (4lu)
#define mPWM0_PWM_OUT_N_INVERT               (0lu)
#define mPWM0_PWM_OUT_INVERT                 (0lu)
#define mPWM0_PWM_ALIGN                      (0lu)
#define mPWM0_PWM_RUN_MODE                   (0lu)
#define mPWM0_PWM_DEAD_TIME_CYCLE            (0lu)
#define mPWM0_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define mPWM0_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define mPWM0_PWM_COUNT_SIGNAL_MODE          (3lu)
#define mPWM0_PWM_START_SIGNAL_MODE          (0lu)
#define mPWM0_PWM_STOP_SIGNAL_MODE           (0lu)
#define mPWM0_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define mPWM0_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define mPWM0_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define mPWM0_PWM_START_SIGNAL_PRESENT       (0lu)
#define mPWM0_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define mPWM0_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define mPWM0_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define mPWM0_TC_PERIOD_VALUE                (65535lu)
#define mPWM0_TC_COMPARE_VALUE               (65535lu)
#define mPWM0_TC_COMPARE_BUF_VALUE           (65535lu)
#define mPWM0_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define mPWM0_PWM_PERIOD_VALUE               (1000lu)
#define mPWM0_PWM_PERIOD_BUF_VALUE           (65535lu)
#define mPWM0_PWM_PERIOD_SWAP                (0lu)
#define mPWM0_PWM_COMPARE_VALUE              (500lu)
#define mPWM0_PWM_COMPARE_BUF_VALUE          (65535lu)
#define mPWM0_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define mPWM0__LEFT 0
#define mPWM0__RIGHT 1
#define mPWM0__CENTER 2
#define mPWM0__ASYMMETRIC 3

#define mPWM0__X1 0
#define mPWM0__X2 1
#define mPWM0__X4 2

#define mPWM0__PWM 4
#define mPWM0__PWM_DT 5
#define mPWM0__PWM_PR 6

#define mPWM0__INVERSE 1
#define mPWM0__DIRECT 0

#define mPWM0__CAPTURE 2
#define mPWM0__COMPARE 0

#define mPWM0__TRIG_LEVEL 3
#define mPWM0__TRIG_RISING 0
#define mPWM0__TRIG_FALLING 1
#define mPWM0__TRIG_BOTH 2

#define mPWM0__INTR_MASK_TC 1
#define mPWM0__INTR_MASK_CC_MATCH 2
#define mPWM0__INTR_MASK_NONE 0
#define mPWM0__INTR_MASK_TC_CC 3

#define mPWM0__UNCONFIG 8
#define mPWM0__TIMER 1
#define mPWM0__QUAD 3
#define mPWM0__PWM_SEL 7

#define mPWM0__COUNT_UP 0
#define mPWM0__COUNT_DOWN 1
#define mPWM0__COUNT_UPDOWN0 2
#define mPWM0__COUNT_UPDOWN1 3


/* Prescaler */
#define mPWM0_PRESCALE_DIVBY1                ((uint32)(0u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY2                ((uint32)(1u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY4                ((uint32)(2u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY8                ((uint32)(3u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY16               ((uint32)(4u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY32               ((uint32)(5u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY64               ((uint32)(6u << mPWM0_PRESCALER_SHIFT))
#define mPWM0_PRESCALE_DIVBY128              ((uint32)(7u << mPWM0_PRESCALER_SHIFT))

/* TCPWM set modes */
#define mPWM0_MODE_TIMER_COMPARE             ((uint32)(mPWM0__COMPARE         <<  \
                                                                  mPWM0_MODE_SHIFT))
#define mPWM0_MODE_TIMER_CAPTURE             ((uint32)(mPWM0__CAPTURE         <<  \
                                                                  mPWM0_MODE_SHIFT))
#define mPWM0_MODE_QUAD                      ((uint32)(mPWM0__QUAD            <<  \
                                                                  mPWM0_MODE_SHIFT))
#define mPWM0_MODE_PWM                       ((uint32)(mPWM0__PWM             <<  \
                                                                  mPWM0_MODE_SHIFT))
#define mPWM0_MODE_PWM_DT                    ((uint32)(mPWM0__PWM_DT          <<  \
                                                                  mPWM0_MODE_SHIFT))
#define mPWM0_MODE_PWM_PR                    ((uint32)(mPWM0__PWM_PR          <<  \
                                                                  mPWM0_MODE_SHIFT))

/* Quad Modes */
#define mPWM0_MODE_X1                        ((uint32)(mPWM0__X1              <<  \
                                                                  mPWM0_QUAD_MODE_SHIFT))
#define mPWM0_MODE_X2                        ((uint32)(mPWM0__X2              <<  \
                                                                  mPWM0_QUAD_MODE_SHIFT))
#define mPWM0_MODE_X4                        ((uint32)(mPWM0__X4              <<  \
                                                                  mPWM0_QUAD_MODE_SHIFT))

/* Counter modes */
#define mPWM0_COUNT_UP                       ((uint32)(mPWM0__COUNT_UP        <<  \
                                                                  mPWM0_UPDOWN_SHIFT))
#define mPWM0_COUNT_DOWN                     ((uint32)(mPWM0__COUNT_DOWN      <<  \
                                                                  mPWM0_UPDOWN_SHIFT))
#define mPWM0_COUNT_UPDOWN0                  ((uint32)(mPWM0__COUNT_UPDOWN0   <<  \
                                                                  mPWM0_UPDOWN_SHIFT))
#define mPWM0_COUNT_UPDOWN1                  ((uint32)(mPWM0__COUNT_UPDOWN1   <<  \
                                                                  mPWM0_UPDOWN_SHIFT))

/* PWM output invert */
#define mPWM0_INVERT_LINE                    ((uint32)(mPWM0__INVERSE         <<  \
                                                                  mPWM0_INV_OUT_SHIFT))
#define mPWM0_INVERT_LINE_N                  ((uint32)(mPWM0__INVERSE         <<  \
                                                                  mPWM0_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define mPWM0_TRIG_RISING                    ((uint32)mPWM0__TRIG_RISING)
#define mPWM0_TRIG_FALLING                   ((uint32)mPWM0__TRIG_FALLING)
#define mPWM0_TRIG_BOTH                      ((uint32)mPWM0__TRIG_BOTH)
#define mPWM0_TRIG_LEVEL                     ((uint32)mPWM0__TRIG_LEVEL)

/* Interrupt mask */
#define mPWM0_INTR_MASK_TC                   ((uint32)mPWM0__INTR_MASK_TC)
#define mPWM0_INTR_MASK_CC_MATCH             ((uint32)mPWM0__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define mPWM0_CC_MATCH_SET                   (0x00u)
#define mPWM0_CC_MATCH_CLEAR                 (0x01u)
#define mPWM0_CC_MATCH_INVERT                (0x02u)
#define mPWM0_CC_MATCH_NO_CHANGE             (0x03u)
#define mPWM0_OVERLOW_SET                    (0x00u)
#define mPWM0_OVERLOW_CLEAR                  (0x04u)
#define mPWM0_OVERLOW_INVERT                 (0x08u)
#define mPWM0_OVERLOW_NO_CHANGE              (0x0Cu)
#define mPWM0_UNDERFLOW_SET                  (0x00u)
#define mPWM0_UNDERFLOW_CLEAR                (0x10u)
#define mPWM0_UNDERFLOW_INVERT               (0x20u)
#define mPWM0_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define mPWM0_PWM_MODE_LEFT                  (mPWM0_CC_MATCH_CLEAR        |   \
                                                         mPWM0_OVERLOW_SET           |   \
                                                         mPWM0_UNDERFLOW_NO_CHANGE)
#define mPWM0_PWM_MODE_RIGHT                 (mPWM0_CC_MATCH_SET          |   \
                                                         mPWM0_OVERLOW_NO_CHANGE     |   \
                                                         mPWM0_UNDERFLOW_CLEAR)
#define mPWM0_PWM_MODE_ASYM                  (mPWM0_CC_MATCH_INVERT       |   \
                                                         mPWM0_OVERLOW_SET           |   \
                                                         mPWM0_UNDERFLOW_CLEAR)

#if (mPWM0_CY_TCPWM_V2)
    #if(mPWM0_CY_TCPWM_4000)
        #define mPWM0_PWM_MODE_CENTER                (mPWM0_CC_MATCH_INVERT       |   \
                                                                 mPWM0_OVERLOW_NO_CHANGE     |   \
                                                                 mPWM0_UNDERFLOW_CLEAR)
    #else
        #define mPWM0_PWM_MODE_CENTER                (mPWM0_CC_MATCH_INVERT       |   \
                                                                 mPWM0_OVERLOW_SET           |   \
                                                                 mPWM0_UNDERFLOW_CLEAR)
    #endif /* (mPWM0_CY_TCPWM_4000) */
#else
    #define mPWM0_PWM_MODE_CENTER                (mPWM0_CC_MATCH_INVERT       |   \
                                                             mPWM0_OVERLOW_NO_CHANGE     |   \
                                                             mPWM0_UNDERFLOW_CLEAR)
#endif /* (mPWM0_CY_TCPWM_NEW) */

/* Command operations without condition */
#define mPWM0_CMD_CAPTURE                    (0u)
#define mPWM0_CMD_RELOAD                     (8u)
#define mPWM0_CMD_STOP                       (16u)
#define mPWM0_CMD_START                      (24u)

/* Status */
#define mPWM0_STATUS_DOWN                    (1u)
#define mPWM0_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   mPWM0_Init(void);
void   mPWM0_Enable(void);
void   mPWM0_Start(void);
void   mPWM0_Stop(void);

void   mPWM0_SetMode(uint32 mode);
void   mPWM0_SetCounterMode(uint32 counterMode);
void   mPWM0_SetPWMMode(uint32 modeMask);
void   mPWM0_SetQDMode(uint32 qdMode);

void   mPWM0_SetPrescaler(uint32 prescaler);
void   mPWM0_TriggerCommand(uint32 mask, uint32 command);
void   mPWM0_SetOneShot(uint32 oneShotEnable);
uint32 mPWM0_ReadStatus(void);

void   mPWM0_SetPWMSyncKill(uint32 syncKillEnable);
void   mPWM0_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   mPWM0_SetPWMDeadTime(uint32 deadTime);
void   mPWM0_SetPWMInvert(uint32 mask);

void   mPWM0_SetInterruptMode(uint32 interruptMask);
uint32 mPWM0_GetInterruptSourceMasked(void);
uint32 mPWM0_GetInterruptSource(void);
void   mPWM0_ClearInterrupt(uint32 interruptMask);
void   mPWM0_SetInterrupt(uint32 interruptMask);

void   mPWM0_WriteCounter(uint32 count);
uint32 mPWM0_ReadCounter(void);

uint32 mPWM0_ReadCapture(void);
uint32 mPWM0_ReadCaptureBuf(void);

void   mPWM0_WritePeriod(uint32 period);
uint32 mPWM0_ReadPeriod(void);
void   mPWM0_WritePeriodBuf(uint32 periodBuf);
uint32 mPWM0_ReadPeriodBuf(void);

void   mPWM0_WriteCompare(uint32 compare);
uint32 mPWM0_ReadCompare(void);
void   mPWM0_WriteCompareBuf(uint32 compareBuf);
uint32 mPWM0_ReadCompareBuf(void);

void   mPWM0_SetPeriodSwap(uint32 swapEnable);
void   mPWM0_SetCompareSwap(uint32 swapEnable);

void   mPWM0_SetCaptureMode(uint32 triggerMode);
void   mPWM0_SetReloadMode(uint32 triggerMode);
void   mPWM0_SetStartMode(uint32 triggerMode);
void   mPWM0_SetStopMode(uint32 triggerMode);
void   mPWM0_SetCountMode(uint32 triggerMode);

void   mPWM0_SaveConfig(void);
void   mPWM0_RestoreConfig(void);
void   mPWM0_Sleep(void);
void   mPWM0_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define mPWM0_BLOCK_CONTROL_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM0_BLOCK_CONTROL_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define mPWM0_COMMAND_REG                    (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM0_COMMAND_PTR                    ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define mPWM0_INTRRUPT_CAUSE_REG             (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM0_INTRRUPT_CAUSE_PTR             ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define mPWM0_CONTROL_REG                    (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__CTRL )
#define mPWM0_CONTROL_PTR                    ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__CTRL )
#define mPWM0_STATUS_REG                     (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__STATUS )
#define mPWM0_STATUS_PTR                     ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__STATUS )
#define mPWM0_COUNTER_REG                    (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM0_COUNTER_PTR                    ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__COUNTER )
#define mPWM0_COMP_CAP_REG                   (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__CC )
#define mPWM0_COMP_CAP_PTR                   ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__CC )
#define mPWM0_COMP_CAP_BUF_REG               (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM0_COMP_CAP_BUF_PTR               ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__CC_BUFF )
#define mPWM0_PERIOD_REG                     (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM0_PERIOD_PTR                     ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__PERIOD )
#define mPWM0_PERIOD_BUF_REG                 (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM0_PERIOD_BUF_PTR                 ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define mPWM0_TRIG_CONTROL0_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM0_TRIG_CONTROL0_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define mPWM0_TRIG_CONTROL1_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM0_TRIG_CONTROL1_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define mPWM0_TRIG_CONTROL2_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM0_TRIG_CONTROL2_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define mPWM0_INTERRUPT_REQ_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR )
#define mPWM0_INTERRUPT_REQ_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR )
#define mPWM0_INTERRUPT_SET_REG              (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM0_INTERRUPT_SET_PTR              ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_SET )
#define mPWM0_INTERRUPT_MASK_REG             (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM0_INTERRUPT_MASK_PTR             ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_MASK )
#define mPWM0_INTERRUPT_MASKED_REG           (*(reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_MASKED )
#define mPWM0_INTERRUPT_MASKED_PTR           ( (reg32 *) mPWM0_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define mPWM0_MASK                           ((uint32)mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define mPWM0_RELOAD_CC_SHIFT                (0u)
#define mPWM0_RELOAD_PERIOD_SHIFT            (1u)
#define mPWM0_PWM_SYNC_KILL_SHIFT            (2u)
#define mPWM0_PWM_STOP_KILL_SHIFT            (3u)
#define mPWM0_PRESCALER_SHIFT                (8u)
#define mPWM0_UPDOWN_SHIFT                   (16u)
#define mPWM0_ONESHOT_SHIFT                  (18u)
#define mPWM0_QUAD_MODE_SHIFT                (20u)
#define mPWM0_INV_OUT_SHIFT                  (20u)
#define mPWM0_INV_COMPL_OUT_SHIFT            (21u)
#define mPWM0_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define mPWM0_RELOAD_CC_MASK                 ((uint32)(mPWM0_1BIT_MASK        <<  \
                                                                            mPWM0_RELOAD_CC_SHIFT))
#define mPWM0_RELOAD_PERIOD_MASK             ((uint32)(mPWM0_1BIT_MASK        <<  \
                                                                            mPWM0_RELOAD_PERIOD_SHIFT))
#define mPWM0_PWM_SYNC_KILL_MASK             ((uint32)(mPWM0_1BIT_MASK        <<  \
                                                                            mPWM0_PWM_SYNC_KILL_SHIFT))
#define mPWM0_PWM_STOP_KILL_MASK             ((uint32)(mPWM0_1BIT_MASK        <<  \
                                                                            mPWM0_PWM_STOP_KILL_SHIFT))
#define mPWM0_PRESCALER_MASK                 ((uint32)(mPWM0_8BIT_MASK        <<  \
                                                                            mPWM0_PRESCALER_SHIFT))
#define mPWM0_UPDOWN_MASK                    ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                            mPWM0_UPDOWN_SHIFT))
#define mPWM0_ONESHOT_MASK                   ((uint32)(mPWM0_1BIT_MASK        <<  \
                                                                            mPWM0_ONESHOT_SHIFT))
#define mPWM0_QUAD_MODE_MASK                 ((uint32)(mPWM0_3BIT_MASK        <<  \
                                                                            mPWM0_QUAD_MODE_SHIFT))
#define mPWM0_INV_OUT_MASK                   ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                            mPWM0_INV_OUT_SHIFT))
#define mPWM0_MODE_MASK                      ((uint32)(mPWM0_3BIT_MASK        <<  \
                                                                            mPWM0_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define mPWM0_CAPTURE_SHIFT                  (0u)
#define mPWM0_COUNT_SHIFT                    (2u)
#define mPWM0_RELOAD_SHIFT                   (4u)
#define mPWM0_STOP_SHIFT                     (6u)
#define mPWM0_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define mPWM0_CAPTURE_MASK                   ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                  mPWM0_CAPTURE_SHIFT))
#define mPWM0_COUNT_MASK                     ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                  mPWM0_COUNT_SHIFT))
#define mPWM0_RELOAD_MASK                    ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                  mPWM0_RELOAD_SHIFT))
#define mPWM0_STOP_MASK                      ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                  mPWM0_STOP_SHIFT))
#define mPWM0_START_MASK                     ((uint32)(mPWM0_2BIT_MASK        <<  \
                                                                  mPWM0_START_SHIFT))

/* MASK */
#define mPWM0_1BIT_MASK                      ((uint32)0x01u)
#define mPWM0_2BIT_MASK                      ((uint32)0x03u)
#define mPWM0_3BIT_MASK                      ((uint32)0x07u)
#define mPWM0_6BIT_MASK                      ((uint32)0x3Fu)
#define mPWM0_8BIT_MASK                      ((uint32)0xFFu)
#define mPWM0_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define mPWM0_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define mPWM0_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(mPWM0_QUAD_ENCODING_MODES     << mPWM0_QUAD_MODE_SHIFT))       |\
         ((uint32)(mPWM0_CONFIG                  << mPWM0_MODE_SHIFT)))

#define mPWM0_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(mPWM0_PWM_STOP_EVENT          << mPWM0_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(mPWM0_PWM_OUT_INVERT          << mPWM0_INV_OUT_SHIFT))         |\
         ((uint32)(mPWM0_PWM_OUT_N_INVERT        << mPWM0_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(mPWM0_PWM_MODE                << mPWM0_MODE_SHIFT)))

#define mPWM0_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(mPWM0_PWM_RUN_MODE         << mPWM0_ONESHOT_SHIFT))
            
#define mPWM0_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(mPWM0_PWM_ALIGN            << mPWM0_UPDOWN_SHIFT))

#define mPWM0_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(mPWM0_PWM_KILL_EVENT      << mPWM0_PWM_SYNC_KILL_SHIFT))

#define mPWM0_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(mPWM0_PWM_DEAD_TIME_CYCLE  << mPWM0_PRESCALER_SHIFT))

#define mPWM0_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(mPWM0_PWM_PRESCALER        << mPWM0_PRESCALER_SHIFT))

#define mPWM0_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(mPWM0_TC_PRESCALER            << mPWM0_PRESCALER_SHIFT))       |\
         ((uint32)(mPWM0_TC_COUNTER_MODE         << mPWM0_UPDOWN_SHIFT))          |\
         ((uint32)(mPWM0_TC_RUN_MODE             << mPWM0_ONESHOT_SHIFT))         |\
         ((uint32)(mPWM0_TC_COMP_CAP_MODE        << mPWM0_MODE_SHIFT)))
        
#define mPWM0_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(mPWM0_QUAD_PHIA_SIGNAL_MODE   << mPWM0_COUNT_SHIFT))           |\
         ((uint32)(mPWM0_QUAD_INDEX_SIGNAL_MODE  << mPWM0_RELOAD_SHIFT))          |\
         ((uint32)(mPWM0_QUAD_STOP_SIGNAL_MODE   << mPWM0_STOP_SHIFT))            |\
         ((uint32)(mPWM0_QUAD_PHIB_SIGNAL_MODE   << mPWM0_START_SHIFT)))

#define mPWM0_PWM_SIGNALS_MODES                                                              \
        (((uint32)(mPWM0_PWM_SWITCH_SIGNAL_MODE  << mPWM0_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM0_PWM_COUNT_SIGNAL_MODE   << mPWM0_COUNT_SHIFT))           |\
         ((uint32)(mPWM0_PWM_RELOAD_SIGNAL_MODE  << mPWM0_RELOAD_SHIFT))          |\
         ((uint32)(mPWM0_PWM_STOP_SIGNAL_MODE    << mPWM0_STOP_SHIFT))            |\
         ((uint32)(mPWM0_PWM_START_SIGNAL_MODE   << mPWM0_START_SHIFT)))

#define mPWM0_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(mPWM0_TC_CAPTURE_SIGNAL_MODE  << mPWM0_CAPTURE_SHIFT))         |\
         ((uint32)(mPWM0_TC_COUNT_SIGNAL_MODE    << mPWM0_COUNT_SHIFT))           |\
         ((uint32)(mPWM0_TC_RELOAD_SIGNAL_MODE   << mPWM0_RELOAD_SHIFT))          |\
         ((uint32)(mPWM0_TC_STOP_SIGNAL_MODE     << mPWM0_STOP_SHIFT))            |\
         ((uint32)(mPWM0_TC_START_SIGNAL_MODE    << mPWM0_START_SHIFT)))
        
#define mPWM0_TIMER_UPDOWN_CNT_USED                                                          \
                ((mPWM0__COUNT_UPDOWN0 == mPWM0_TC_COUNTER_MODE)                  ||\
                 (mPWM0__COUNT_UPDOWN1 == mPWM0_TC_COUNTER_MODE))

#define mPWM0_PWM_UPDOWN_CNT_USED                                                            \
                ((mPWM0__CENTER == mPWM0_PWM_ALIGN)                               ||\
                 (mPWM0__ASYMMETRIC == mPWM0_PWM_ALIGN))               
        
#define mPWM0_PWM_PR_INIT_VALUE              (1u)
#define mPWM0_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_mPWM0_H */

/* [] END OF FILE */
