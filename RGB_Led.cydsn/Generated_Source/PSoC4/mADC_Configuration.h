/*******************************************************************************
* \file mADC_Configuration.h
* \version 7.0
*
* \brief
*   This file provides the customizer parameters definitions.
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

#if !defined(CY_SENSE_mADC_CONFIGURATION_H)
#define CY_SENSE_mADC_CONFIGURATION_H

#include <cytypes.h>
#include <cyfitter.h>

/*******************************************************************************
* Customizer-generated defines
*******************************************************************************/
#define mADC_ENABLE                             (1u)
#define mADC_DISABLE                            (0u)

#define mADC_THIRD_GENERATION_BLOCK             (1u)
#define mADC_FOURTH_GENERATION_BLOCK            (2u)

#define mADC_GENERATION_BLOCK_VERSION           (2u)

/*******************************************************************************
* HW IP block global defines
*******************************************************************************/

#if (mADC_GENERATION_BLOCK_VERSION == mADC_THIRD_GENERATION_BLOCK)
    #define mADC_CSDV1                          (mADC_ENABLE)

    #ifdef CYIPBLOCK_m0s8csd_VERSION
        #if (0u == CYIPBLOCK_m0s8csd_VERSION)
            #define mADC_CSDV1_VER1             (mADC_ENABLE)
            #define mADC_CSDV1_VER2             (mADC_DISABLE)
        #else
            #define mADC_CSDV1_VER1             (mADC_DISABLE)
            #define mADC_CSDV1_VER2             (mADC_ENABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define mADC_CSDV1                          (mADC_DISABLE)
    #define mADC_CSDV1_VER1                     (mADC_DISABLE)
    #define mADC_CSDV1_VER2                     (mADC_DISABLE)
#endif

#if (mADC_GENERATION_BLOCK_VERSION == mADC_FOURTH_GENERATION_BLOCK)
    #define mADC_CSDV2                          (mADC_ENABLE)

    #ifdef CYIPBLOCK_m0s8csdv2_VERSION
        #if (1u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define mADC_CSDV2_VER1             (mADC_ENABLE)
        #else
            #define mADC_CSDV2_VER1             (mADC_DISABLE)
        #endif
        #if (2u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define mADC_CSDV2_VER2             (mADC_ENABLE)
        #else
            #define mADC_CSDV2_VER2             (mADC_DISABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define mADC_CSDV2                          (mADC_DISABLE)
    #define mADC_CSDV2_VER1                     (mADC_DISABLE)
    #define mADC_CSDV2_VER2                     (mADC_DISABLE)
#endif

/*******************************************************************************
* Project-global defines
*******************************************************************************/

#define mADC_2000_MV                            (2000u)

#ifdef CYDEV_VDDA_MV
    #define mADC_CYDEV_VDDA_MV                  (CYDEV_VDDA_MV)
#else
    #ifdef CYDEV_VDD_MV
        #define mADC_CYDEV_VDDA_MV              (CYDEV_VDD_MV)
    #endif
#endif

#define mADC_BAD_CONVERSIONS_NUM                (1u)
#define mADC_RESAMPLING_CYCLES_MAX_NUMBER       (1u)


/*******************************************************************************
* Enabled Scan Methods
*******************************************************************************/
#define mADC_CSD_EN                             (0u)
#define mADC_CSX_EN                             (0u)
#define mADC_ISX_EN                             (0u)
#define mADC_CSD_CSX_EN                         (mADC_CSD_EN && mADC_CSX_EN)

#define mADC_MANY_SENSE_MODES_EN                ((mADC_CSD_EN && mADC_CSX_EN) || \
                                                             (mADC_CSD_EN && mADC_ISX_EN) || \
                                                             (mADC_CSX_EN && mADC_ISX_EN) || \
                                                             (mADC_SELF_TEST_EN))

#define mADC_MANY_WIDGET_METHODS_EN             ((mADC_CSD_EN && mADC_CSX_EN) || \
                                                             (mADC_CSD_EN && mADC_ISX_EN) || \
                                                             (mADC_CSX_EN && mADC_ISX_EN))

#define mADC_CSD2X_EN                           (0u)
#define mADC_CSX2X_EN                           (0u)

/*******************************************************************************
* Definitions for number of widgets and sensors
*******************************************************************************/
#define mADC_TOTAL_WIDGETS                      (0u)
#define mADC_TOTAL_CSD_WIDGETS                  (0u)
#define mADC_TOTAL_CSD_SENSORS                  (0u)
#define mADC_TOTAL_CSX_WIDGETS                  (0u)
#define mADC_TOTAL_ISX_WIDGETS                  (0u)
#define mADC_TOTAL_CSX_NODES                    (0u)
#define mADC_TOTAL_ISX_NODES                    (0u)

/*******************************************************************************
* Total number of CSD sensors + CSX nodes
*******************************************************************************/
#define mADC_TOTAL_SENSORS            (mADC_TOTAL_CSD_SENSORS + \
                                                   mADC_TOTAL_CSX_NODES+ \
                                                   mADC_TOTAL_ISX_NODES)

/*******************************************************************************
* Total number of scan slots (used only when dual-channel scan is enabled)
*******************************************************************************/
#define mADC_TOTAL_SCAN_SLOTS         (0u)

/*******************************************************************************
* Defines widget IDs
*******************************************************************************/

/*******************************************************************************
* Defines sensor IDs
*******************************************************************************/


/*******************************************************************************
* Defines ADC channel IDs
*******************************************************************************/
#define mADC_CHANNEL_0                          (0u)
#define mADC_CHANNEL_1                          (1u)


/*******************************************************************************
* Enabled widget types
*******************************************************************************/
#define mADC_BUTTON_WIDGET_EN         (0u)
#define mADC_SLIDER_WIDGET_EN         (0u)
#define mADC_MATRIX_WIDGET_EN         (0u)
#define mADC_PROXIMITY_WIDGET_EN      (0u)
#define mADC_TOUCHPAD_WIDGET_EN       (0u)
#define mADC_ENCODERDIAL_WIDGET_EN    (0u)

#define mADC_CSD_MATRIX_WIDGET_EN     (0u)
#define mADC_CSD_TOUCHPAD_WIDGET_EN   (0u)

#define mADC_CSX_MATRIX_WIDGET_EN     (0u)
#define mADC_CSX_TOUCHPAD_WIDGET_EN   (0u)

/*******************************************************************************
* Centroid APIs
*******************************************************************************/
#define mADC_CENTROID_EN              (0u)
#define mADC_TOTAL_DIPLEXED_SLIDERS   (0u)
#define mADC_TOTAL_LINEAR_SLIDERS     (0u)
#define mADC_TOTAL_RADIAL_SLIDERS     (0u)
#define mADC_TOTAL_TOUCHPADS          (0u)
#define mADC_MAX_CENTROID_LENGTH      (0u)
#define mADC_SLIDER_MULT_METHOD       (0u)
#define mADC_TOUCHPAD_MULT_METHOD     (0u)

/*******************************************************************************
* Enabled sensor types
*******************************************************************************/
#define mADC_REGULAR_SENSOR_EN        (0u)
#define mADC_PROXIMITY_SENSOR_EN      (0u)

/*******************************************************************************
* Sensor ganging
*******************************************************************************/
#define mADC_GANGED_SNS_EN            (0u)
#define mADC_CSD_GANGED_SNS_EN        (0u)
#define mADC_CSX_GANGED_SNS_EN        (0u)

/*******************************************************************************
* Max number of sensors used among all the widgets
*******************************************************************************/
#define mADC_MAX_SENSORS_PER_WIDGET   (0u)
#define mADC_MAX_SENSORS_PER_5X5_TOUCHPAD (1u)

/*******************************************************************************
* Total number of all used electrodes (NOT unique)
*******************************************************************************/
#define mADC_TOTAL_ELECTRODES         (0u)
/* Obsolete */
#define mADC_TOTAL_SENSOR_IOS         mADC_TOTAL_ELECTRODES

/*******************************************************************************
* Total number of used physical IOs (unique)
*******************************************************************************/
#define mADC_TOTAL_IO_CNT             (0u)

/*******************************************************************************
* Array length for widget status registers
*******************************************************************************/
#define mADC_WDGT_STATUS_WORDS        \
                        (((uint8)((mADC_TOTAL_WIDGETS - 1u) / 32u)) + 1u)


/*******************************************************************************
* Auto-tuning mode selection
*******************************************************************************/
#define mADC_CSD_SS_DIS         (0x00ul)
#define mADC_CSD_SS_HW_EN       (0x01ul)
#define mADC_CSD_SS_TH_EN       (0x02ul)
#define mADC_CSD_SS_HWTH_EN     (mADC_CSD_SS_HW_EN | \
                                             mADC_CSD_SS_TH_EN)

#define mADC_CSD_AUTOTUNE       mADC_CSD_SS_DIS


/*******************************************************************************
* General settings
*******************************************************************************/

#define mADC_AUTO_RESET_METHOD_LEGACY (0u)
#define mADC_AUTO_RESET_METHOD_SAMPLE (1u)

#define mADC_MULTI_FREQ_SCAN_EN       (0u)
#define mADC_SENSOR_AUTO_RESET_EN     (0u)
#define mADC_SENSOR_AUTO_RESET_METHOD (0u)
#define mADC_NUM_CENTROIDS            (1u)
#define mADC_4PTS_LOCAL_MAX_EN        (0u)
#define mADC_OFF_DEBOUNCE_EN          (0u)
#define mADC_CUSTOM_DS_RAM_SIZE       (0u)

/* Defines power status of HW block after scanning */
#define mADC_BLOCK_OFF_AFTER_SCAN_EN  (0u)

/* Defines number of scan frequencies */
#if (mADC_DISABLE != mADC_MULTI_FREQ_SCAN_EN)
    #define mADC_NUM_SCAN_FREQS       (3u)
#else
    #define mADC_NUM_SCAN_FREQS       (1u)
#endif /* #if (mADC_DISABLE != mADC_MULTI_FREQ_SCAN_EN) */

/* Data size for thresholds / low baseline reset */
#define mADC_SIZE_8BITS               (8u)
#define mADC_SIZE_16BITS              (16u)

#define mADC_THRESHOLD_SIZE           mADC_SIZE_16BITS
typedef uint16 mADC_THRESHOLD_TYPE;

#if (mADC_AUTO_RESET_METHOD_LEGACY == mADC_SENSOR_AUTO_RESET_METHOD)
    #define mADC_LOW_BSLN_RST_SIZE        mADC_SIZE_8BITS
    typedef uint8 mADC_LOW_BSLN_RST_TYPE;
#else
    #define mADC_LOW_BSLN_RST_SIZE    (16u)
    typedef uint16 mADC_LOW_BSLN_RST_TYPE;
#endif /* #if (mADC_AUTO_RESET_METHOD_LEGACY == mADC_SENSOR_AUTO_RESET_METHOD) */

/* Coefficient to define touch threshold for proximity sensors */
#define mADC_PROX_TOUCH_COEFF         (300u)

/*******************************************************************************
* General Filter Constants
*******************************************************************************/

/* Baseline algorithm options */
#define mADC_IIR_BASELINE                 (0u)
#define mADC_BUCKET_BASELINE              (1u)

#define mADC_BASELINE_TYPE                mADC_IIR_BASELINE

/* IIR baseline filter algorithm for regular sensors*/
#define mADC_REGULAR_IIR_BL_TYPE          mADC_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for regular sensors */
#define mADC_REGULAR_IIR_BL_N             (1u)
#define mADC_REGULAR_IIR_BL_SHIFT         (8u)

/* IIR baseline filter algorithm for proximity sensors*/
#define mADC_PROX_IIR_BL_TYPE             mADC_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for proximity sensors */
#define mADC_PROX_IIR_BL_N                (1u)
#define mADC_PROX_IIR_BL_SHIFT            (8u)


/* IIR filter constants */
#define mADC_IIR_COEFFICIENT_K            (256u)

/* IIR filter type */
#define mADC_IIR_FILTER_STANDARD          (1u)
#define mADC_IIR_FILTER_PERFORMANCE       (2u)
#define mADC_IIR_FILTER_MEMORY            (3u)

/* Regular sensor raw count filters */
#define mADC_REGULAR_RC_FILTER_EN         (0u)
#define mADC_REGULAR_RC_IIR_FILTER_EN     (0u)
#define mADC_REGULAR_RC_MEDIAN_FILTER_EN  (0u)
#define mADC_REGULAR_RC_AVERAGE_FILTER_EN (0u)
#define mADC_REGULAR_RC_CUSTOM_FILTER_EN  (0u)
#define mADC_REGULAR_RC_ALP_FILTER_EN     (0u)

/* Proximity sensor raw count filters */
#define mADC_PROX_RC_FILTER_EN            (0u)
#define mADC_PROX_RC_IIR_FILTER_EN        (0u)
#define mADC_PROX_RC_MEDIAN_FILTER_EN     (0u)
#define mADC_PROX_RC_AVERAGE_FILTER_EN    (0u)
#define mADC_PROX_RC_CUSTOM_FILTER_EN     (0u)
#define mADC_PROX_RC_ALP_FILTER_EN        (0u)

#define mADC_ALP_FILTER_EN                (0u)
#define mADC_REGULAR_RC_ALP_FILTER_COEFF  (2u)
#define mADC_PROX_RC_ALP_FILTER_COEFF     (2u)

/* Raw count filters */
#define mADC_RC_FILTER_EN                 (mADC_REGULAR_RC_FILTER_EN || mADC_PROX_RC_FILTER_EN)

/* IIR raw count filter algorithm for regular sensors */
#define mADC_REGULAR_IIR_RC_TYPE          (mADC_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for regular sensors */
#define mADC_REGULAR_IIR_RC_N             (128u)
#define mADC_REGULAR_IIR_RC_SHIFT         (0u)

/* IIR raw count filter algorithm for proximity sensors*/
#define mADC_PROX_IIR_RC_TYPE             (mADC_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for proximity sensors */
#define mADC_PROX_IIR_RC_N                (128u)
#define mADC_PROX_IIR_RC_SHIFT            (0u)

/* Median filter constants */

/* Order of regular sensor median filter */
#define mADC_REGULAR_MEDIAN_LEN           (2u)

/* Order of proximity sensor median filter */
#define mADC_PROX_MEDIAN_LEN              (2u)

/* Average filter constants*/
#define mADC_AVERAGE_FILTER_LEN_2         (1u)
#define mADC_AVERAGE_FILTER_LEN_4         (3u)

/* Order of regular sensor average filter */
#define mADC_REGULAR_AVERAGE_LEN          (mADC_AVERAGE_FILTER_LEN_4)

/* Order of proximity sensor average filter */
#define mADC_PROX_AVERAGE_LEN             (mADC_AVERAGE_FILTER_LEN_4)

/* Widget baseline coefficient enable */
#define mADC_WD_BSLN_COEFF_EN             (0u)

/* Centroid position filters */
#define mADC_POSITION_FILTER_EN           (0u)
#define mADC_POS_MEDIAN_FILTER_EN         (0u)
#define mADC_POS_IIR_FILTER_EN            (0u)
#define mADC_POS_ADAPTIVE_IIR_FILTER_EN   (0u)
#define mADC_POS_AVERAGE_FILTER_EN        (0u)
#define mADC_POS_JITTER_FILTER_EN         (0u)
#define mADC_BALLISTIC_MULTIPLIER_EN      (0u)
#define mADC_CENTROID_3X3_CSD_EN          (0u)
#define mADC_CENTROID_5X5_CSD_EN          (0u)
#define mADC_CSD_5X5_MAX_FINGERS          (1u)

#define mADC_POS_IIR_COEFF                (128u)
#define mADC_POS_IIR_RESET_RADIAL_SLIDER  (35u)

#define mADC_CSX_TOUCHPAD_UNDEFINED       (40u)

/* IDAC options */

/* Third-generation HW block IDAC gain */
#define mADC_IDAC_GAIN_4X                 (4u)
#define mADC_IDAC_GAIN_8X                 (8u)

/* Fourth-generation HW block IDAC gain */
#define mADC_IDAC_GAIN_LOW                (0uL)
#define mADC_IDAC_GAIN_MEDIUM             (1uL)
#define mADC_IDAC_GAIN_HIGH               (2uL)

#define mADC_IDAC_SOURCING                (0u)
#define mADC_IDAC_SINKING                 (1u)

/* Shield tank capacitor precharge source */
#define mADC_CSH_PRECHARGE_VREF           (0u)
#define mADC_CSH_PRECHARGE_IO_BUF         (1u)

/* Shield electrode delay */
#define mADC_NO_DELAY                     (0u)

#if(mADC_ENABLE == mADC_CSDV2)
    #define mADC_SH_DELAY_5NS             (1u)
    #define mADC_SH_DELAY_10NS            (2u)
    #define mADC_SH_DELAY_20NS            (3u)
#else
    #if(mADC_ENABLE == mADC_CSDV1_VER2)
        #define mADC_SH_DELAY_10NS        (3u)
        #define mADC_SH_DELAY_50NS        (2u)
    #else
        #define mADC_SH_DELAY_1CYCLES     (1u)
        #define mADC_SH_DELAY_2CYCLES     (2u)
    #endif /* (mADC_ENABLE == mADC_CSDV1_VER2) */
#endif /* (mADC_ENABLE == mADC_CSDV2) */

/* Inactive sensor connection options */
#define mADC_SNS_CONNECTION_GROUND        (0x00000000u)
#define mADC_SNS_CONNECTION_HIGHZ         (0x00000001u)
#define mADC_SNS_CONNECTION_SHIELD        (0x00000002u)
#define mADC_SNS_CONNECTION_UNDEFINED     (0x00000003u)

/* Sense clock selection options */
#if defined(mADC_TAPEOUT_STAR_USED)
    #define mADC_CSDV2_REF9P6UA_EN            (0u)
#else
    #define mADC_CSDV2_REF9P6UA_EN            (1u)
#endif /* defined(mADC_TAPEOUT_STAR_USED) */

#define mADC_CLK_SOURCE_DIRECT            (0x00000000Lu)

#define mADC_CLK_SOURCE_SSC1              (0x01u)
#define mADC_CLK_SOURCE_SSC2              (0x02u)
#define mADC_CLK_SOURCE_SSC3              (0x03u)
#define mADC_CLK_SOURCE_SSC4              (0x04u)

#define mADC_CLK_SOURCE_PRS8              (0x05u)
#define mADC_CLK_SOURCE_PRS12             (0x06u)
#define mADC_CLK_SOURCE_PRSAUTO           (0xFFu)

#define mADC_MFS_IMO                      (0u)
#define mADC_MFS_SNS_CLK                  (1u)

/* Defines scan resolutions */
#define mADC_RES6BIT                      (6u)
#define mADC_RES7BIT                      (7u)
#define mADC_RES8BIT                      (8u)
#define mADC_RES9BIT                      (9u)
#define mADC_RES10BIT                     (10u)
#define mADC_RES11BIT                     (11u)
#define mADC_RES12BIT                     (12u)
#define mADC_RES13BIT                     (13u)
#define mADC_RES14BIT                     (14u)
#define mADC_RES15BIT                     (15u)
#define mADC_RES16BIT                     (16u)

/* Fourth-generation HW block: Initialization switch resistance */
#define mADC_INIT_SW_RES_LOW              (0x00000000Lu)
#define mADC_INIT_SW_RES_MEDIUM           (0x00000001Lu)
#define mADC_INIT_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: Initialization switch resistance */
#define mADC_SCAN_SW_RES_LOW              (0x00000000Lu)
#define mADC_SCAN_SW_RES_MEDIUM           (0x00000001Lu)
#define mADC_SCAN_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define mADC_SHIELD_SW_RES_LOW            (0x00000000Lu)
#define mADC_SHIELD_SW_RES_MEDIUM         (0x00000001Lu)
#define mADC_SHIELD_SW_RES_HIGH           (0x00000002Lu)
#define mADC_SHIELD_SW_RES_LOW_EMI        (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define mADC_INIT_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define mADC_INIT_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define mADC_INIT_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define mADC_INIT_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define mADC_SCAN_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define mADC_SCAN_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define mADC_SCAN_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define mADC_SCAN_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Sensing method */
#define mADC_SENSING_LEGACY               (0x00000000Lu)
#define mADC_SENSING_LOW_EMI              (0x00000001Lu)
#define mADC_SENSING_FULL_WAVE            (0x00000002Lu)


/*******************************************************************************
* CSD/CSX Common Settings
*******************************************************************************/

#define mADC_BLOCK_ANALOG_WAKEUP_DELAY_US (0u)

#define mADC_MFS_METHOD                   (0u)
#define mADC_IMO_FREQUENCY_OFFSET_F1      (20u)
#define mADC_IMO_FREQUENCY_OFFSET_F2      (20u)

/*******************************************************************************
* CSD Specific Settings
*******************************************************************************/

/* CSD scan method settings */
#define mADC_CSD_IDAC_AUTOCAL_EN          (0u)
#define mADC_CSD_IDAC_GAIN                (mADC_IDAC_GAIN_HIGH)
#define mADC_CSD_SHIELD_EN                (0u)
#define mADC_CSD_SHIELD_TANK_EN           (0u)
#define mADC_CSD_CSH_PRECHARGE_SRC        (mADC_CSH_PRECHARGE_VREF)
#define mADC_CSD_SHIELD_DELAY             (mADC_NO_DELAY)
#define mADC_CSD_TOTAL_SHIELD_COUNT       (0u)
#define mADC_CSD_SCANSPEED_DIVIDER        (2u)
#define mADC_CSD_COMMON_SNS_CLK_EN        (0u)
#define mADC_CSD_SNS_CLK_SOURCE           (mADC_CLK_SOURCE_PRSAUTO)
#define mADC_CSD_SNS_CLK_DIVIDER          (4u)
#define mADC_CSD_INACTIVE_SNS_CONNECTION  (mADC_SNS_CONNECTION_GROUND)
#define mADC_CSD_IDAC_COMP_EN             (0u)
#define mADC_CSD_IDAC_CONFIG              (mADC_IDAC_SOURCING)
#define mADC_CSD_RAWCOUNT_CAL_LEVEL       (85u)
#define mADC_CSD_DUALIDAC_LEVEL           (50u)
#define mADC_CSD_PRESCAN_SETTLING_TIME    (5u)
#define mADC_CSD_SNSCLK_R_CONST           (1000u)
#define mADC_CSD_VREF_MV                  (2021u)

#define mADC_CSD_CALIBRATION_ERROR        (10u)
#define mADC_CSD_IDAC_AUTO_GAIN_EN        (1u)
#define mADC_CSD_IDAC_GAIN_INDEX_DEFAULT  (4u)
#define mADC_CSD_IDAC_MIN                 (20u)
#define mADC_CSD_COL_ROW_IDAC_ALIGNMENT_EN      (1u)

/* The macro is obsolete and should not be used */
#define mADC_CSD_DEDICATED_IDAC_COMP_EN   (1u)
/* CSD settings - Fourth-generation HW block */
#define mADC_CSD_ANALOG_STARTUP_DELAY_US  (10u)
#define mADC_CSD_FINE_INIT_TIME           (10u)
#define mADC_CSD_AUTO_ZERO_EN             (0u)
#define mADC_CSD_AUTO_ZERO_TIME           (15Lu)
#define mADC_CSD_NOISE_METRIC_EN          (0u)
#define mADC_CSD_NOISE_METRIC_TH          (1Lu)
#define mADC_CSD_INIT_SWITCH_RES          (mADC_INIT_SW_RES_MEDIUM)
#define mADC_CSD_SENSING_METHOD           (0)
#define mADC_CSD_SHIELD_SWITCH_RES        (mADC_SHIELD_SW_RES_MEDIUM)
#define mADC_CSD_GAIN                     (18Lu)

#define mADC_CSD_MFS_DIVIDER_OFFSET_F1    (1u)
#define mADC_CSD_MFS_DIVIDER_OFFSET_F2    (2u)

/*******************************************************************************
* CSX Specific Settings
*******************************************************************************/

/* CSX scan method settings */
#define mADC_CSX_SCANSPEED_DIVIDER        (2u)
#define mADC_CSX_COMMON_TX_CLK_EN         (0u)
#define mADC_CSX_TX_CLK_SOURCE            (mADC_CLK_SOURCE_PRSAUTO)
#define mADC_CSX_TX_CLK_DIVIDER           (40u)
#define mADC_CSX_INACTIVE_SNS_CONNECTION  (mADC_SNS_CONNECTION_GROUND)
#define mADC_CSX_MAX_FINGERS              (1u)
#define mADC_CSX_MAX_LOCAL_PEAKS          (5u)
#define mADC_CSX_IDAC_AUTOCAL_EN          (0u)
#define mADC_CSX_IDAC_BITS_USED           (7u)
#define mADC_CSX_RAWCOUNT_CAL_LEVEL       (40u)
#define mADC_CSX_IDAC_GAIN                (mADC_IDAC_GAIN_MEDIUM)
#define mADC_CSX_CALIBRATION_ERROR        (20u)
#define mADC_CSX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define mADC_CSX_MULTIPHASE_TX_EN         (0u)
#define mADC_CSX_MAX_TX_PHASE_LENGTH      (0u)

/* CSX settings - Fourth-generation HW block */
#define mADC_CSX_ANALOG_STARTUP_DELAY_US  (10u)
#define mADC_CSX_AUTO_ZERO_EN             (0u)
#define mADC_CSX_AUTO_ZERO_TIME           (15u)
#define mADC_CSX_FINE_INIT_TIME           (4u)
#define mADC_CSX_NOISE_METRIC_EN          (0u)
#define mADC_CSX_NOISE_METRIC_TH          (1u)
#define mADC_CSX_INIT_SWITCH_RES          (mADC_INIT_SW_RES_MEDIUM)
#define mADC_CSX_SCAN_SWITCH_RES          (mADC_SCAN_SW_RES_LOW)
#define mADC_CSX_INIT_SHIELD_SWITCH_RES   (mADC_INIT_SHIELD_SW_RES_HIGH)
#define mADC_CSX_SCAN_SHIELD_SWITCH_RES   (mADC_SCAN_SHIELD_SW_RES_LOW)

#define mADC_CSX_MFS_DIVIDER_OFFSET_F1    (1u)
#define mADC_CSX_MFS_DIVIDER_OFFSET_F2    (2u)

/* Gesture parameters */
#define mADC_GES_GLOBAL_EN                (0u)

/*******************************************************************************
* ISX Specific Settings
*******************************************************************************/

/* ISX scan method settings */
#define mADC_ISX_SCANSPEED_DIVIDER        (1u)
#define mADC_ISX_LX_CLK_DIVIDER           (80u)
#define mADC_ISX_IDAC_AUTOCAL_EN          (0u)
#define mADC_ISX_IDAC_BITS_USED           (7u)
#define mADC_ISX_RAWCOUNT_CAL_LEVEL       (30u)
#define mADC_ISX_IDAC_GAIN                (mADC_IDAC_GAIN_MEDIUM)
#define mADC_ISX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define mADC_ISX_MAX_TX_PHASE_LENGTH      (0u)
#define mADC_ISX_PIN_COUNT_LX             (u)
/* ISX settings - Fourth-generation HW block */
#define mADC_ISX_AUTO_ZERO_EN             (0u)
#define mADC_ISX_AUTO_ZERO_TIME           (15u)
#define mADC_ISX_FINE_INIT_TIME           (20u)
#define mADC_ISX_NOISE_METRIC_EN          (0u)
#define mADC_ISX_NOISE_METRIC_TH          (1u)
#define mADC_ISX_INIT_SWITCH_RES          (mADC_INIT_SW_RES_MEDIUM)
#define mADC_ISX_SCAN_SWITCH_RES          (mADC_SCAN_SW_RES_LOW)
#define mADC_ISX_INIT_SHIELD_SWITCH_RES   (mADC_INIT_SHIELD_SW_RES_HIGH)
#define mADC_ISX_SCAN_SHIELD_SWITCH_RES   (mADC_SCAN_SHIELD_SW_RES_LOW)
#define mADC_ISX_SAMPLE_PHASE_DEG         (30u)

/*******************************************************************************
* Global Parameter Definitions
*******************************************************************************/

/* Compound section definitions */
#define mADC_ANY_NONSS_AUTOCAL ((0u != mADC_CSX_IDAC_AUTOCAL_EN) || \
                                       (0u != mADC_ISX_IDAC_AUTOCAL_EN) || \
                                      ((mADC_CSD_AUTOTUNE == mADC_CSD_SS_DIS) && (0u != mADC_CSD_IDAC_AUTOCAL_EN)))
#define mADC_ANYMODE_AUTOCAL (((0u != mADC_CSX_IDAC_AUTOCAL_EN) \
                                       || (0u != mADC_ISX_IDAC_AUTOCAL_EN)) \
                                       || (0u != mADC_CSD_IDAC_AUTOCAL_EN))
/* RAM Global Parameters Definitions */


/* RAM Sensor Parameters Definitions */


/*******************************************************************************
* ADC Specific Macros
*******************************************************************************/
#define mADC_ADC_RES8BIT                  (8u)
#define mADC_ADC_RES10BIT                 (10u)

#define mADC_ADC_FULLRANGE_MODE           (0u)
#define mADC_ADC_VREF_MODE                (1u)

#define mADC_ADC_MIN_CHANNELS             (1u)
#define mADC_ADC_EN                       (1u)
#define mADC_ADC_STANDALONE_EN            (1u)
#define mADC_ADC_TOTAL_CHANNELS           (2u)
#define mADC_ADC_RESOLUTION               (mADC_ADC_RES10BIT)
#define mADC_ADC_AMUXB_INPUT_EN           (0u)
#define mADC_ADC_SELECT_AMUXB_CH          (0u)
#define mADC_ADC_AZ_EN                    (1Lu)
#define mADC_ADC_AZ_TIME                  (5u)
#define mADC_ADC_VREF_MV                  (2133u)
#define mADC_ADC_GAIN                     (17Lu)
#define mADC_ADC_IDAC_DEFAULT             (15u)
#define mADC_ADC_MODCLK_DIV_DEFAULT       (1u)
#define mADC_ADC_MEASURE_MODE             (mADC_ADC_FULLRANGE_MODE)
#define mADC_ADC_ANALOG_STARTUP_DELAY_US  (5u)
#define mADC_ADC_ACQUISITION_TIME_US      (10u)

/*******************************************************************************
* Built-In Self-Test Configuration
*******************************************************************************/
#define mADC_SELF_TEST_EN                   (0Lu)
#define mADC_TST_GLOBAL_CRC_EN              (0Lu)
#define mADC_TST_WDGT_CRC_EN                (0Lu)
#define mADC_TST_BSLN_DUPLICATION_EN        (0Lu)
#define mADC_TST_BSLN_RAW_OUT_RANGE_EN      (0Lu)
#define mADC_TST_SNS_SHORT_EN               (0Lu)
#define mADC_TST_SNS_CAP_EN                 (0Lu)
#define mADC_TST_SH_CAP_EN                  (0Lu)
#define mADC_TST_EXTERNAL_CAP_EN            (0Lu)
#define mADC_TST_VDDA_EN                    (0Lu)


#define mADC_GLOBAL_CRC_AREA_START          (0u)
#define mADC_GLOBAL_CRC_AREA_SIZE           (0u)
#define mADC_WIDGET_CRC_AREA_START          (0u)
#define mADC_WIDGET_CRC_AREA_SIZE           (0u)

/* The resolution for sensor capacity measurement */
#define mADC_TST_SNS_CAP_RESOLUTION         (12u)

/* VDDA measurement test configuration */
/* The resolution for VDDA measurement */
#define mADC_TST_VDDA_RESOLUTION            (10u)
/* The ModClk divider for external capacitor capacity measurement */
#define mADC_TST_VDDA_MOD_CLK_DIVIDER       (1u)

#define mADC_TST_VDDA_VREF_MV               (0u)
#define mADC_TST_VDDA_VREF_GAIN             (0u)
#define mADC_TST_VDDA_IDAC_DEFAULT          (0u)

#define mADC_TST_FINE_INIT_TIME             (10u)
#define mADC_TST_ANALOG_STARTUP_DELAY_US    (23u)

#define mADC_TST_IDAC_AUTO_GAIN_EN          (1u)
#define mADC_TST_SNS_CAP_RAW_ERROR          (10u)
#define mADC_TST_IDAC_GAIN_INDEX            (0xFFu)
#define mADC_TST_RAW_TARGET                 (85u)

#define mADC_TST_SNS_SHORT_TIME             (2u)

#define mADC_SNS_CAP_CSD_INACTIVE_CONNECTION        (mADC_SNS_CONNECTION_GROUND)
#define mADC_SNS_CAP_CSX_INACTIVE_CONNECTION        (mADC_SNS_CONNECTION_GROUND)
#define mADC_SHIELD_CAP_INACTIVE_CONNECTION         (mADC_SNS_CONNECTION_GROUND)


/*******************************************************************************
* Gesture Configuration
*******************************************************************************/
#define mADC_TIMESTAMP_INTERVAL             (1Lu)
#define mADC_GESTURE_EN_WIDGET_ID           (0Lu)
#define mADC_BALLISTIC_EN_WIDGET_ID         (0Lu)


#endif /* CY_SENSE_mADC_CONFIGURATION_H */


/* [] END OF FILE */
