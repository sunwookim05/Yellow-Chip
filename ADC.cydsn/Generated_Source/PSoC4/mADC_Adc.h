/***************************************************************************//**
* \file mADC_Adc.h
* \version 7.0
*
* \brief
*   This file provides the sources of APIs specific to the ADC implementation.
*
* \see mADC v7.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016-2019), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_SENSE_mADC_ADC_H)
#define CY_SENSE_mADC_ADC_H

#include "cytypes.h"
#include "mADC_Configuration.h"
#include "mADC_Structure.h"

#if (mADC_ADC_EN)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/**
* \cond SECTION_STANDALONE_ADC
* \addtogroup group_adc_public
* \{
*/
#if (mADC_ENABLE == mADC_ADC_STANDALONE_EN)
    void mADC_Start(void);
    void mADC_Sleep(void);
    void mADC_Wakeup(void);
#endif  /* (mADC_ENABLE == mADC_ADC_STANDALONE_EN) */

/** \}
* \endcond */

/**
* \cond SECTION_ADC_PUBLIC
* \addtogroup group_adc_public
* \{
*/
cystatus mADC_StartConvert(uint8 chId);
uint8 mADC_IsBusy(void);
uint16 mADC_ReadResult_mVolts(uint8 chId);
uint16 mADC_GetResult_mVolts(uint8 chId);
cystatus mADC_Calibrate(void);

void mADC_Stop(void);
void mADC_Resume(void);

/** \}
* \endcond */

CY_ISR_PROTO(mADC_IntrHandler);

/**
* \cond SECTION_ADC_INTERNAL
* \addtogroup group_adc_internal
* \{
*/

void mADC_Initialize(void);
void mADC_SetAdcChannel(uint8 chId, uint32 state);
void mADC_ConfigAdcResources(void);
void mADC_StartAdcFSM(uint32 measureMode);
cystatus mADC_AdcCaptureResources(void);
cystatus mADC_AdcReleaseResources(void);
void mADC_ClearAdcChannels(void);
void mADC_SetNonDedicatedAdcChannel(uint8 chId, uint32 state);

/** \}
* \endcond */

/**************************************
* Global software/external variables
**************************************/

extern uint16 mADC_adcVrefMv;

/**************************************
*           API Constants
**************************************/

/* Error value if given bad channel ID. */
#define mADC_VALUE_BAD_CHAN_ID            (0x0000FFFFuL)
#define mADC_VALUE_BAD_RESULT             (0x0000FFFFuL)

/* Statuses defined for use with IsBusy */
#define mADC_STATUS_LASTCHAN_MASK         (0x0000000FuL)
#define mADC_STATUS_FSM_MASK              (0x000000F0uL)
#define mADC_STATUS_IDLE                  (0x00u)
#define mADC_STATUS_CALIBPH1              (0x10u)
#define mADC_STATUS_CALIBPH2              (0x20u)
#define mADC_STATUS_CALIBPH3              (0x30u)
#define mADC_STATUS_CONVERTING            (0x40u)
#define mADC_STATUS_OVERFLOW              (0x80u)

/* Potential channel states */
#define mADC_CHAN_CONNECT                 (1uL)
#define mADC_CHAN_DISCONNECT              (0uL)

/* Active channel when ADC is not configured */
#define mADC_NO_CHANNEL                   (0xFFu)

#define mADC_GPIO_PC_INPUT                (0x1uL)

/* Default filter delay */
#define mADC_FILTER_DELAY_DEFAULT         (2uL)

/* Adc Config */
#define mADC_CONFIG_DEFAULT               (mADC_CONFIG_ENABLE_MASK | \
                                                                            mADC_CONFIG_SAMPLE_SYNC_MASK | \
                                                                            mADC_CONFIG_SENSE_EN_MASK | \
                                                                            mADC_CONFIG_DSI_COUNT_SEL_MASK | \
                                                                            mADC_FILTER_DELAY_DEFAULT)

/* Measurement modes */
#define mADC_MEASMODE_OFF                 (0x0uL << CYFLD_CSD_ADC_MODE__OFFSET)
#define mADC_MEASMODE_VREF                (0x1uL << CYFLD_CSD_ADC_MODE__OFFSET)
#define mADC_MEASMODE_VREFBY2             (0x2uL << CYFLD_CSD_ADC_MODE__OFFSET)
#define mADC_MEASMODE_VIN                 (0x3uL << CYFLD_CSD_ADC_MODE__OFFSET)

/* Clock defines */
#define mADC_SENSE_DIV_DEFAULT            (0x4uL)
#define mADC_TOTAL_CLOCK_DIV              (mADC_ADC_MODCLK_DIV_DEFAULT * \
                                                                         mADC_SENSE_DIV_DEFAULT)

/* The MAX possible value of the AZ time in CSD_SENSE cycles. The value is limited by the width of the SEQ_TIME register */
#define mADC_ADC_SEQ_TIME_MAX                               (0x100u)

#define mADC_ADC_SEQ_TIME_CYCLES                            (((CYDEV_BCLK__HFCLK__HZ * mADC_ADC_AZ_TIME) / \
                                                                         mADC_TOTAL_CLOCK_DIV) / 1000000uL)

#if (mADC_ADC_SEQ_TIME_MAX < mADC_ADC_SEQ_TIME_CYCLES)
    #define mADC_SEQ_TIME_BASE            (mADC_ADC_SEQ_TIME_MAX)
#else
    #define mADC_SEQ_TIME_BASE            (mADC_ADC_SEQ_TIME_CYCLES)
#endif

#if (0 == mADC_SEQ_TIME_BASE)
    #define mADC_SEQ_TIME_DEFAUL          (1u)
#else
    #define mADC_SEQ_TIME_DEFAUL          (mADC_SEQ_TIME_BASE)
#endif

/* Acquisition time definitions: ADC_CTL */
#define mADC_ACQUISITION_BASE             ((mADC_ADC_ACQUISITION_TIME_US * \
                                                                        (CYDEV_BCLK__HFCLK__MHZ)) / \
                                                                         mADC_TOTAL_CLOCK_DIV)

/* SEQ_START field definitions */
#define mADC_FSMSETTING_ABORT             (0x1uL << CYFLD_CSD_ABORT__OFFSET)
#define mADC_FSMSETTING_DSI_START_EN      (0x1uL << CYFLD_CSD_DSI_START_EN__OFFSET)
#define mADC_FSMSETTING_AZ0SKIP           (0x1uL << CYFLD_CSD_AZ0_SKIP__OFFSET)
#define mADC_FSMSETTING_AZ1SKIP           (0x1uL << CYFLD_CSD_AZ1_SKIP__OFFSET)


#define mADC_FSMSETTING_NOAZSKIP          (0uL)
#define mADC_FSMSETTING_AZSKIP_DEFAULT    (mADC_FSMSETTING_AZ0SKIP \
                                                                            | ((0u != mADC_ADC_AZ_EN) \
                                                                            ? 0u \
                                                                            : mADC_FSMSETTING_AZ1SKIP))
#define mADC_FSMSETTING_DSIIGNORE         (0x00000000uL)
#define mADC_FSMSETTING_NOABORT           (0x00000000uL)
#define mADC_FSMSETTING_SEQMODE           (0x00000000uL)
#define mADC_FSMSETTING_START             (0x00000001uL)

/* IDACB definitions
*  The idac configuration for ADC use is mostly static, with only the VAL field varying.
*   Dynamic Polarity = 1 << 7
*   Polarity (normal) = 0 << 8
*   Balance, Leg1, Leg2 modes = don't care.
*   DSI Control Enable (no mix) = 0 << 21
*   Range (low) = 0 << 22
*   Leg1, Leg2 enable = 0
*   Leg3 enable = 1 << 26
*/
#define mADC_IDACB_CONFIG                 (0x04000080uL)
#define mADC_CLK16_MASK                   (0x0000FFFFuL)
#define mADC_CLK16VAL_SHIFT               (8uL)

/* Switch definitions */
#define mADC_SW_HSP_DEFAULT               (0x10000000uL)
#define mADC_SW_HSN_DEFAULT               (0x00100000uL)
#define mADC_SW_HSP_GETINPOL              (0x00010000uL)
#define mADC_SW_HSN_GETINPOL              (0x01000000uL)
#define mADC_SW_SHIELD_DEFAULT            (0x00000000uL)
#define mADC_SW_SHIELD_VDDA2CSDBUSB       (0x00000100uL)
#define mADC_SW_BYP_DEFAULT               (0x00110000uL)
#define mADC_SW_CMPP_DEFAULT              (0x00000000uL)
#define mADC_SW_CMPN_DEFAULT              (0x00000000uL)
#define mADC_SW_REFGEN_DEFAULT            (0x10000000uL)

#define mADC_SW_FWMOD_DEFAULT             (0x01100000uL)
#define mADC_SW_FWTANK_DEFAULT            (0x01100000uL)

/* The reference voltage macros */
#define mADC_SW_CTANK_PINSHIFT            (9uL)
#define mADC_SW_CMOD_PINSHIFT             (6uL)
#define mADC_SW_CMOD_PORT_MASK            (0x400uL)

#define mADC_LVTHRESH                     (2700uL)

/* The reference voltage is measured at nominal 2400 mV. Measured amount is stored in CYREG_SFLASH_CSDV2_CSD0_ADC_TRIM1 */
#define mADC_VREFCALIB_BASE               (2400uL)

/* RefGen settings */
#define mADC_REFGEN_GAIN_SHIFT            (CYFLD_CSD_GAIN__OFFSET)

/* At low voltage, REFGEN is enabled and bypassed. */
#define mADC_SW_AMUBUF_LV                 (0x01000100uL)
#define mADC_AMBUF_LV                     (0x00000002uL)
/* At normal voltage, REFGEN uses customizer-defined gain */
#define mADC_REFGEN_NORM                  (0x00000041uL | (mADC_ADC_GAIN << \
                                                                            mADC_REFGEN_GAIN_SHIFT))
#define mADC_SW_AMUBUF_NORM               (0x00000000uL)

/* HSCOMP definitions */
#define mADC_HSCMP_AZ_DEFAULT             (mADC_HSCMP_EN_MASK | \
                                                                            (mADC_ADC_AZ_EN << CYFLD_CSD_AZ_EN__OFFSET))

/* ADC_RES definitions */
#define mADC_RES_MAX                      ((1uL << mADC_ADC_RESOLUTION) - 1uL)
#define mADC_ADC_RES_OVERFLOW_MASK        (0x40000000uL)
#define mADC_ADC_RES_ABORT_MASK           (0x80000000uL)
#define mADC_ADC_RES_HSCMPPOL_MASK        (0x00010000uL)
#define mADC_ADC_RES_VALUE_MASK           (0x0000FFFFuL)

#endif  /* mADC_ADC_EN */

#endif  /* CY_SENSE_mADC_ADC_H */


/* [] END OF FILE */
