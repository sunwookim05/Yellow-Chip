/***************************************************************************//**
* \file mADC_Structure.c
* \version 7.0
*
* \brief
*   This file defines the data structure global variables and provides implementation
*   for the high-level and low-level APIs of the Data Structure module.
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

#include <string.h>
#include <cytypes.h>
#include "CyLib.h"
#include "mADC_Structure.h"
#include "mADC_Configuration.h"

#if (mADC_ENABLE == mADC_ADC_EN)
    #include "mADC_Adc.h"
#endif /* (mADC_ENABLE == mADC_ADC_EN) */

/*******************************************************************************
* Defines the RAM Data Structure variables and their init data in flash
*******************************************************************************/

/**
* \cond SECTION_GLOBAL_VARIABLES
* \addtogroup group_global_variables
* \{
*/

/**
* The variable that contains the mADC configuration, settings and
* scanning results. mADC_dsRam represents RAM Data Structure.
*/
mADC_RAM_STRUCT mADC_dsRam;

/** \}
* \endcond */

/**
* \cond SECTION_API_CONSTANTS
* \addtogroup group_api_constants
* \{
*/


/**
 *  The array of the pointers to the ADC input channels specific register.
 */
const mADC_FLASH_IO_STRUCT mADC_adcIoList[mADC_ADC_TOTAL_CHANNELS] =
{
    {
        (reg32 *)mADC_AdcInput__0__HSIOM,
        (reg32 *)mADC_AdcInput__0__PC,
        (reg32 *)mADC_AdcInput__0__DR,
        (reg32 *)mADC_AdcInput__0__PS,
        mADC_AdcInput__0__HSIOM_MASK,
        mADC_AdcInput__0__MASK,
        mADC_AdcInput__0__HSIOM_SHIFT,
        (uint8)mADC_AdcInput__0__SHIFT,
        (uint8)mADC_AdcInput__0__SHIFT * 3u,
    },
    {
        (reg32 *)mADC_AdcInput__1__HSIOM,
        (reg32 *)mADC_AdcInput__1__PC,
        (reg32 *)mADC_AdcInput__1__DR,
        (reg32 *)mADC_AdcInput__1__PS,
        mADC_AdcInput__1__HSIOM_MASK,
        mADC_AdcInput__1__MASK,
        mADC_AdcInput__1__HSIOM_SHIFT,
        (uint8)mADC_AdcInput__1__SHIFT,
        (uint8)mADC_AdcInput__1__SHIFT * 3u,
    },
};



/** \}
* \endcond */


/*******************************************************************************
* Function Name: mADC_DsInitialize
****************************************************************************//**
*
* \brief
*   This function initializes the Data Structure storage.
*
* \details
*   Configures the initial Adc datastructure members.
*
*******************************************************************************/
void mADC_DsInitialize(void)
{
    /* Reset RAM data structure content */
    (void)memset(&mADC_dsRam, 0, sizeof(mADC_dsRam));

    mADC_dsRam.adcResolution = mADC_ADC_RESOLUTION;
    mADC_dsRam.adcIdac = (uint8)(mADC_ADC_IDAC_DEFAULT);
    mADC_dsRam.adcActiveCh = mADC_NO_CHANNEL;
}

/* [] END OF FILE */
