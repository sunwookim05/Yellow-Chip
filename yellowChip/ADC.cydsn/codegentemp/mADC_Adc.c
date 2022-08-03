/***************************************************************************//**
* \file mADC_Adc.c
* \version 7.0
*
* \brief
*   This file provides implementation for the ADC module of the mADC
*   Component.
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
#include <cytypes.h>
#include "CyLib.h"
#include "mADC_ISR.h"
#include "mADC_Configuration.h"
#include "mADC_Structure.h"
#include "mADC_Adc.h"
#include "mADC_Sensing.h"
#if (mADC_ENABLE == mADC_SELF_TEST_EN)
    #include "mADC_SelfTest.h"
#endif

#if (mADC_ADC_EN)

static uint8 mADC_initVar;
uint16 mADC_adcVrefMv = mADC_ADC_VREF_MV;


/*******************************************************************************
* Module local function declarations
*******************************************************************************/
/**
* \cond SECTION_ADC_INTERNAL
* \addtogroup group_adc_internal
* \{
*/

void mADC_SetModClkClockDivider(uint32 modClk);

/** \}
* \endcond */

/*******************************************************************************
* Local definition
*******************************************************************************/
#define mADC_INIT_DONE                    (1u)
#define mADC_INIT_NEEDED                  (0u)
#define mADC_CAL_WATCHDOG_CYCLES_NUM      (0x0000FFFFLu)

#if (mADC_ENABLE == mADC_ADC_STANDALONE_EN)
    /*******************************************************************************
    * Function Name: mADC_Start
    ****************************************************************************//**
    * \cond SECTION_STANDALONE_ADC
    * \brief
    *   Configures the hardware and performs calibration.
    *
    * \details
    *   Configures the hardware and performs calibration.
    * \endcond
    *
    *******************************************************************************/
    void mADC_Start(void)
    {
        mADC_ClearAdcChannels();

        if (mADC_INIT_DONE != mADC_initVar)
        {
            mADC_DsInitialize();
        }
        mADC_ConfigAdcResources();
        (void) mADC_Calibrate();
    }

    /*******************************************************************************
    * Function Name: mADC_Sleep
    ****************************************************************************//**
    * \cond SECTION_STANDALONE_ADC
    * \brief
    *  Prepares the Component for deep sleep.
    *
    * \details
    *  Currently this function is empty and exists as a place for future updates,
    *  this function will be used to prepare the Component to enter deep sleep.
    * \endcond
    *
    *******************************************************************************/
    void mADC_Sleep(void)
    {
    }


    /*******************************************************************************
    * Function Name: mADC_Wakeup
    ****************************************************************************//**
    * \cond SECTION_STANDALONE_ADC
    * \brief
    *  This function resumes the Component after sleep.
    *
    * \details
    *  Currently this function is empty and exists as a place for future updates,
    *  this function will be used to resume the Component after exiting deep sleep.
    * \endcond
    *
    *******************************************************************************/
    void mADC_Wakeup(void)
    {
    }
#endif  /* (mADC_ENABLE != mADC_ADC_STANDALONE_EN) */

/*******************************************************************************
* Function Name: mADC_SetNonDedicatedAdcChannel
****************************************************************************//**
*
* \brief
*   Sets the non dedicated channel to the given state.
*
* \details
*   Connects/disconnects the pin and the analog muxbus B. Sets the drive mode
*   of the pin as well.
*
* \param chId  The ID of the non dedicated channel to be set.
* \param state The state in which the channel is to be put:
*         - (0) mADC_CHAN_DISCONNECT
*         - (1) mADC_CHAN_CONNECT
*
*******************************************************************************/
void mADC_SetNonDedicatedAdcChannel(uint8 chId, uint32 state)
{
    mADC_FLASH_IO_STRUCT const *ptr2adcIO;
    uint32 newRegisterValue;
    uint8  interruptState;
    uint32 pinHSIOMShift;
    uint32 pinModeShift;
    uint32 tmpVal;

    ptr2adcIO = &mADC_adcIoList[chId];
    pinHSIOMShift = (uint32)ptr2adcIO->hsiomShift;
    pinModeShift = (uint32)ptr2adcIO->shift;

    /* Clear port connections. */
    tmpVal = CY_GET_REG32(ptr2adcIO->hsiomPtr);
    tmpVal &= ~(mADC_HSIOM_SEL_MASK << pinHSIOMShift);

    interruptState = CyEnterCriticalSection();

    switch (state)
    {
    case mADC_CHAN_CONNECT:
        /* Connect AMuxBusB to the selected port */
        CY_SET_REG32(ptr2adcIO->hsiomPtr, (tmpVal | (mADC_HSIOM_SEL_AMUXB << pinHSIOMShift)));
        /* Update port configuration register (drive mode) to HiZ Analog */
        newRegisterValue = CY_GET_REG32(ptr2adcIO->pcPtr);
        newRegisterValue &= ~(mADC_GPIO_PC_MASK << pinModeShift);
        newRegisterValue |= (mADC_GPIO_PC_INPUT << pinModeShift);
        CY_SET_REG32(ptr2adcIO->pcPtr, newRegisterValue);

        /* Cmod and Ctank are not typical GPIO, require CSD setting. */
        if (0u != ((uint32)(ptr2adcIO->hsiomPtr) & mADC_SW_CMOD_PORT_MASK))
        {
            if (mADC_SW_CTANK_PINSHIFT == pinModeShift)
            {
                CY_SET_REG32(mADC_SW_DSI_SEL_PTR, mADC_SW_DSI_CTANK);
            }
            else if (mADC_SW_CMOD_PINSHIFT == pinModeShift)
            {
                CY_SET_REG32(mADC_SW_DSI_SEL_PTR, mADC_SW_DSI_CMOD);
            }
            else { /* No action */ }
        }
        break;

    /* Disconnection is a safe default state. Fall-through is intentional. */
    case mADC_CHAN_DISCONNECT:
    default:
        /* Disconnec AMuxBusB from the selected port */
        CY_SET_REG32(ptr2adcIO->hsiomPtr, tmpVal);

        /* Update port configuration register (drive mode) to HiZ input/output by clearing PC */
        newRegisterValue = CY_GET_REG32(ptr2adcIO->pcPtr);
        newRegisterValue &= ~(mADC_GPIO_PC_MASK << pinModeShift);
        CY_SET_REG32(ptr2adcIO->pcPtr, newRegisterValue);

        /* Cmod and Ctank are not typical GPIO, require CSD setting. */
        if (0u != ((uint32)(ptr2adcIO->hsiomPtr) & mADC_SW_CMOD_PORT_MASK))
        {
            if ((mADC_SW_CTANK_PINSHIFT == pinModeShift) ||
                (mADC_SW_CMOD_PINSHIFT == pinModeShift ))
            {
                CY_SET_REG32(mADC_SW_DSI_SEL_PTR, 0u);
            }
        }
        break;
    }

    /* Set logic 0 to port data register */
    tmpVal = CY_GET_REG32(ptr2adcIO->drPtr);
    tmpVal &= (uint32)~(uint32)((uint32)1u << ptr2adcIO->drShift);
    CY_SET_REG32(ptr2adcIO->drPtr, tmpVal);
    
    CyExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: mADC_SetAdcChannel
****************************************************************************//**
*
* \brief
*   Sets the given channel to the given state.
*
* \details
*   Connects/disconnects the pin and the analog muxbus B.  Sets the drive mode
*   of the pin as well.
*
* \param chId  The ID of the channel to be set.
* \param state The state in which the channel is to be put:
*         - (0) mADC_CHAN_DISCONNECT
*         - (1) mADC_CHAN_CONNECT
*
*******************************************************************************/
void mADC_SetAdcChannel(uint8 chId, uint32 state)
{
    #if (0u != mADC_ADC_AMUXB_INPUT_EN)
        #if (mADC_ADC_MIN_CHANNELS < mADC_ADC_TOTAL_CHANNELS)
            if(mADC_ADC_SELECT_AMUXB_CH != chId)
            {
                mADC_SetNonDedicatedAdcChannel(chId, state);
            }
        #endif /* (0u != mADC_ADC_TOTAL_CHANNELS) */
    #else
        mADC_SetNonDedicatedAdcChannel(chId, state);
    #endif /* (0u != mADC_ADC_AMUXB_INPUT_EN) */
}


/*******************************************************************************
* Function Name: mADC_ConfigAdcResources
****************************************************************************//**
*
* \brief
*   Configures the CSD block to be used as an ADC.
*
* \details
*   Configures the IDACB, internal switches, REFGEN, HSCOMP, enables CSD
*   block interrupt and set interrupt vector to ADC sensing method.
*
*******************************************************************************/
void mADC_ConfigAdcResources(void)
{
    uint8  interruptState;
    uint32 newRegValue;

    /* Configure clocks */
    mADC_SetModClkClockDivider(mADC_ADC_MODCLK_DIV_DEFAULT);
    CY_SET_REG32(mADC_SENSE_PERIOD_PTR, (mADC_SENSE_DIV_DEFAULT - 1uL));

    /* Configure the IDAC */
    CY_SET_REG32(mADC_CONFIG_PTR, mADC_CONFIG_DEFAULT);
    CY_SET_REG32(mADC_CSD_IDACB_PTR, mADC_IDACB_CONFIG | mADC_dsRam.adcIdac);

    /* Configure AZ Time */
    CY_SET_REG32(mADC_SEQ_TIME_PTR, (mADC_SEQ_TIME_DEFAUL - 1uL));

    CY_SET_REG32(mADC_CSDCMP_PTR, 0uL);
    CY_SET_REG32(mADC_SW_DSI_SEL_PTR, 0uL);

    CY_SET_REG32(mADC_SENSE_DUTY_PTR, 0uL);
    CY_SET_REG32(mADC_SEQ_INIT_CNT_PTR, 1uL);
    CY_SET_REG32(mADC_SEQ_NORM_CNT_PTR, 2uL);

    /* Configure the block-level routing */
    CY_SET_REG32(mADC_SW_HS_P_SEL_PTR, mADC_SW_HSP_DEFAULT);
    CY_SET_REG32(mADC_SW_HS_N_SEL_PTR, mADC_SW_HSN_DEFAULT);
    CY_SET_REG32(mADC_SW_SHIELD_SEL_PTR, mADC_SW_SHIELD_DEFAULT);
    CY_SET_REG32(mADC_SW_CMP_P_SEL_PTR, mADC_SW_CMPP_DEFAULT);
    CY_SET_REG32(mADC_SW_CMP_N_SEL_PTR, mADC_SW_CMPN_DEFAULT);
    CY_SET_REG32(mADC_SW_FW_MOD_SEL_PTR, mADC_SW_FWMOD_DEFAULT);
    CY_SET_REG32(mADC_SW_FW_TANK_SEL_PTR, mADC_SW_FWTANK_DEFAULT);
    CY_SET_REG32(mADC_SW_REFGEN_SEL_PTR, mADC_SW_REFGEN_DEFAULT);

    interruptState = CyEnterCriticalSection();
    newRegValue = CY_GET_REG32(mADC_SW_BYP_SEL_PTR);
    newRegValue |= mADC_SW_BYP_DEFAULT;
    CY_SET_REG32(mADC_SW_BYP_SEL_PTR, newRegValue);
    CyExitCriticalSection(interruptState);

    /* Config RefGen */
    #if (mADC_CYDEV_VDDA_MV < mADC_LVTHRESH)
        /* The routing of the HS_N and AMUXBUF switches depend on RefGen */
        CY_SET_REG32(mADC_REFGEN_PTR, mADC_REFGEN_LV);
        CY_SET_REG32(mADC_SW_AMUXBUF_SEL_PTR, mADC_SW_AMUBUF_LV);
        CY_SET_REG32(mADC_AMBUF_PTR, mADC_AMBUF_LV);
    #else
        CY_SET_REG32(mADC_REFGEN_PTR, mADC_REFGEN_NORM);
        CY_SET_REG32(mADC_SW_AMUXBUF_SEL_PTR, mADC_SW_AMUBUF_NORM);
    #endif /* mADC__CYDEV_VDDA_MV < mADC_LVTHRESH */

    /* Configure HSCOMP */
    CY_SET_REG32(mADC_HSCMP_PTR, mADC_HSCMP_AZ_DEFAULT);

    /* Clear all pending interrupts of CSD block */
    CY_SET_REG32(mADC_INTR_PTR, mADC_INTR_ALL_MASK);

    /* Mask all CSD block interrupts (disable all interrupts) */
    CY_SET_REG32(mADC_INTR_MASK_PTR, mADC_INTR_MASK_CLEAR_MASK);

    /* Set the ISR vector */
    mADC_ISR_StartEx(&mADC_IntrHandler);

    /* Component is initialized */
    mADC_initVar = mADC_INIT_DONE;
}


/*******************************************************************************
* Function Name: mADC_StartAdcFSM
****************************************************************************//**
*
* \brief
*   Starts the CSD state machine with correct parameters to initialize an ADC
*   conversion.
*
* \details
*   Starts the CSD state machine with correct parameters to initialize an ADC
*   conversion.
*
* \param measureMode The FSM mode:
*        - (0) mADC_MEASMODE_OFF
*        - (1) mADC_MEASMODE_VREF
*        - (2) mADC_MEASMODE_VREFBY2
*        - (3) mADC_MEASMODE_VIN
*
*******************************************************************************/
void mADC_StartAdcFSM(uint32 measureMode)
{
    uint32 tmpStartVal;

    /* Set the mode and acquisition time */
    CY_SET_REG32(mADC_ADC_CTL_PTR, (measureMode | (mADC_ACQUISITION_BASE - 1u)));

    if(mADC_MEASMODE_VREF == measureMode)
    {
        tmpStartVal =
            mADC_FSMSETTING_AZSKIP_DEFAULT    |
            mADC_FSMSETTING_DSIIGNORE   |
            mADC_FSMSETTING_NOABORT     |
            mADC_FSMSETTING_SEQMODE     |
            mADC_FSMSETTING_START;
    }
    else if (mADC_MEASMODE_OFF == measureMode)
    {
        tmpStartVal = mADC_FSMSETTING_ABORT;
    }
    /* This setting is used for both MEASMODE_VREFBY2 and MEASMODE_VIN */
    else
    {
        tmpStartVal =
            mADC_FSMSETTING_AZSKIP_DEFAULT    |
            mADC_FSMSETTING_DSIIGNORE   |
            mADC_FSMSETTING_NOABORT     |
            mADC_FSMSETTING_SEQMODE     |
            mADC_FSMSETTING_START;
    }

    /* Enable HSComp */
    CY_SET_REG32(mADC_SEQ_START_PTR, tmpStartVal);
}

/*******************************************************************************
* Function Name: mADC_AdcCaptureResources
****************************************************************************//**
*
* \brief
*   Releases CSD resources from sensing mode, and sets it into ADC mode.
*
* \details
*   Releases CSD resources from sensing mode, and sets it into ADC mode.
*
* \return     The function returns cystatus of its operation.
*   - CYRET_LOCKED  - The sensing hardware is in-use and could not be released.
*   - CYRET_SUCCESS - Block is configured for ADC use.
*
*******************************************************************************/
cystatus mADC_AdcCaptureResources(void)
{
    cystatus tmpStatus = CYRET_SUCCESS;

    #if !(mADC_ENABLE == mADC_ADC_STANDALONE_EN)
        tmpStatus = mADC_SsReleaseResources();

        if (mADC_INIT_NEEDED == mADC_initVar)
        {
            if(CYRET_SUCCESS == tmpStatus)
            {
                mADC_ConfigAdcResources();
            }
            else
            {
                tmpStatus = CYRET_LOCKED;
            }
        }
    #else
        if (mADC_INIT_NEEDED == mADC_initVar)
        {
            mADC_ConfigAdcResources();
        }
    #endif /* !(mADC_ENABLE == mADC_ADC_STANDALONE_EN) */

    return tmpStatus;
}


/*******************************************************************************
* Function Name: mADC_AdcReleaseResources
****************************************************************************//**
*
* \brief
*   Releases CSD resources from ADC mode.
*
* \details
*   Releases CSD resources from ADC mode.
*
* \return     The function returns cystatus of its operation.
*   CYRET_SUCCESS   Block resources no longer in use.
*
*******************************************************************************/
cystatus mADC_AdcReleaseResources(void)
{
    uint8  interruptState;
    uint32 newRegValue;

    if (mADC_INIT_DONE == mADC_initVar)
    {
        /* If the FSM is running, shut it down. */
        if(mADC_STATUS_IDLE != (mADC_dsRam.adcStatus
                            & mADC_STATUS_FSM_MASK))
        {
            mADC_StartAdcFSM(mADC_MEASMODE_OFF);
            mADC_SetAdcChannel((mADC_dsRam.adcStatus
                            & (uint8)(mADC_STATUS_LASTCHAN_MASK)),
                            mADC_CHAN_DISCONNECT);
            mADC_dsRam.adcStatus = mADC_STATUS_IDLE;
        }

        /* Disable the subblocks. */
        CY_SET_REG32(mADC_CSD_IDACB_PTR, 0u);
        CY_SET_REG32(mADC_REFGEN_PTR, 0u);
        CY_SET_REG32(mADC_AMBUF_PTR, 0u);
        CY_SET_REG32(mADC_HSCMP_PTR, 0u);

        /* Reset the block-level routing */
        CY_SET_REG32(mADC_SW_HS_P_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_HS_N_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_SHIELD_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_CMP_P_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_CMP_N_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_FW_MOD_SEL_PTR, 0u);
        CY_SET_REG32(mADC_SW_FW_TANK_SEL_PTR, 0u);

        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(mADC_SW_BYP_SEL_PTR);
        newRegValue &= (uint32)(~mADC_SW_BYP_DEFAULT);
        CY_SET_REG32(mADC_SW_BYP_SEL_PTR, newRegValue);
        CyExitCriticalSection(interruptState);

         /* Disconnect all ADC channels */
        mADC_ClearAdcChannels();

        mADC_initVar = mADC_INIT_NEEDED;
    }

    return CYRET_SUCCESS;
}


/*******************************************************************************
* Function Name: mADC_StartConvert
****************************************************************************//**
*
* \brief
*  Initializes the hardware and initiates an analog-to-digital conversion on the
*  selected input channel.
*
* \details
*  Initializes the hardware and initiates an analog-to-digital conversion on the
*  selected input channel. This API only initiates a conversion and does not
*  wait for the conversion to be completed, therefore the
*  mADC_IsBusy() API must be used to check the
*  status and ensure that the conversion is complete prior to reading the result,
*  starting a new conversion with the same or a different channel, or reconfiguring
*  the hardware for different functionality.
*
* \param chId
*  The ID of the channel to be converted.
*
* \return
*  The function returns cystatus of its operation.
*    - CYRET_SUCCESS - A conversion has started.
*    - CYRET_LOCKED - The hardware is already in-use by a previously initialized
*      conversion or other functionality. No new conversion is started by this API.
*    - CYRET_BAD_PARAM - An invalid channel Id. No conversion is started.
*
*******************************************************************************/
cystatus mADC_StartConvert(uint8 chId)
{
    uint32 tmpStatus = CYRET_SUCCESS;

    /* If non-standalone ADC, try to Capture resources */
    #if (mADC_ENABLE != mADC_ADC_STANDALONE_EN)
        tmpStatus = mADC_AdcCaptureResources();

        if (CYRET_SUCCESS == tmpStatus)
        {
    #else /* Otherwise, configure resources if needed. */
        if (mADC_INIT_NEEDED == mADC_initVar)
        {
            mADC_ConfigAdcResources();
        }
    #endif /* (mADC_ENABLE != mADC_ADC_STANDALONE_EN) */

    if(chId >= mADC_ADC_TOTAL_CHANNELS)
    {
        tmpStatus = CYRET_BAD_PARAM;
    }

    if (CYRET_SUCCESS == tmpStatus)
    {
        if(mADC_STATUS_IDLE != (mADC_dsRam.adcStatus & mADC_STATUS_FSM_MASK))
        {
            tmpStatus = CYRET_LOCKED;
        }

        if(CYRET_SUCCESS == tmpStatus)
        {
            #if(mADC_ADC_ANALOG_STARTUP_DELAY_US > 0uL)
                CyDelayUs(mADC_ADC_ANALOG_STARTUP_DELAY_US);
            #endif /* (mADC_ADC_ANALOG_STARTUP_DELAY_US > 0uL) */

            /* Set Component Status */
            mADC_dsRam.adcStatus = (mADC_STATUS_CONVERTING | chId);

            /* Configure a desired channel if needed */
            if (chId != mADC_dsRam.adcActiveCh)
            {
                if (mADC_NO_CHANNEL != mADC_dsRam.adcActiveCh)
                {
                    /* Disconnect existing channel */
                    mADC_SetAdcChannel(mADC_dsRam.adcActiveCh, mADC_CHAN_DISCONNECT);
                }
                /* Connect desired input */
                mADC_SetAdcChannel(chId, mADC_CHAN_CONNECT);
                mADC_dsRam.adcActiveCh = chId;
            }

            /* Un-mask ADC_RES interrupt (enable interrupt) */
            CY_SET_REG32(mADC_INTR_MASK_PTR, mADC_INTR_MASK_ADC_RES_MASK);
            mADC_StartAdcFSM(mADC_MEASMODE_VIN);
        }
    }

    #if (mADC_ENABLE != mADC_ADC_STANDALONE_EN)
        }
    #endif /* (mADC_ENABLE != mADC_ADC_STANDALONE_EN) */

    return tmpStatus;
}


/*******************************************************************************
* Function Name: mADC_IsBusy
****************************************************************************//**
*
* \brief
*   The function returns the status of the ADC's operation.
*
* \details
*   The function returns the status of the ADC's operation. A new conversion or
*   calibration must not be started unless the ADC is in the IDLE state.
*
* \return
*  The function returns the status of the ADC's operation.
*    - mADC_STATUS_IDLE - The ADC is not busy,
*       a new conversion can be initiated.
*    - mADC_STATUS_CONVERTING - A previously
*       initiated conversion is in progress.
*    - mADC_STATUS_CALIBPH1 - The ADC is in the
*      first phase (of 3) of calibration.
*    - mADC_STATUS_CALIBPH2 - The ADC is in the
*      second phase (of 3) of calibration.
*    - mADC_STATUS_CALIBPH3 - The ADC is in the
*      third phase (of 3) of calibration.
*    - mADC_STATUS_OVERFLOW - The most recent
*      measurement caused an overflow. The root cause of the overflow may be
*      the previous calibration values being invalid or the VDDA setting in cydwr
*      and hardware do not match. Perform re-calibration or set the
*      appropriate VDDA value in cydwr to avoid this error condition.
*
*******************************************************************************/
uint8 mADC_IsBusy(void)
{
    uint8 tmpStatus;

    if (0u != (mADC_ADC_RES_REG & mADC_ADC_RES_OVERFLOW_MASK))
    {
        tmpStatus = mADC_STATUS_OVERFLOW;
    }
    else
    {
        tmpStatus = (uint8)((*(volatile uint8 *)&mADC_dsRam.adcStatus) & mADC_STATUS_FSM_MASK);
    }

    return tmpStatus;
}


/*******************************************************************************
* Function Name: mADC_ReadResult_mVolts
****************************************************************************//**
*
* \brief
*  This is a blocking API. It initiates a conversion, waits for completion and
*  returns the result.
*
* \details
*  This is a blocking API. Internally, it starts a conversion using
*  mADC_StartConvert(), checks the status using
*  mADC_IsBusy(), waits until the conversion is
*  completed and returns the result.
*
* \param chId
*  The ID of the channel to be measured
*
* \return
*  The function returns voltage in millivolts or
*  mADC_VALUE_BAD_RESULT if:
*   - chId is invalid
*   - The ADC conversion is not started
*   - The ADC conversion watch-dog triggered.
*
*******************************************************************************/
uint16 mADC_ReadResult_mVolts(uint8 chId)
{
    cystatus convertStatus;
    uint16 tmpValue;
    uint32 watchdogAdcCounter;

    convertStatus = mADC_StartConvert(chId);
    if (CYRET_SUCCESS == convertStatus)
    {
        /* Initialize Watchdog Counter with time interval which is enough to ADC conversion is completed */
        watchdogAdcCounter = mADC_CAL_WATCHDOG_CYCLES_NUM;
        while ((mADC_IsBusy() != mADC_STATUS_IDLE) &&
               (0u != watchdogAdcCounter))
        {
            /* Wait until conversion complete and decrement Watchdog Counter to prevent unending loop */
            watchdogAdcCounter--;
        }
        if (0u != watchdogAdcCounter)
        {
            tmpValue = mADC_GetResult_mVolts(chId);
        }
        else
        {
            tmpValue = (uint16) mADC_VALUE_BAD_RESULT;
        }
    }
    else
    {
        tmpValue = (uint16) mADC_VALUE_BAD_RESULT;
    }

    return tmpValue;
}


/*******************************************************************************
* Function Name: mADC_GetResult_mVolts
****************************************************************************//**
*
* \brief
*  This API does not perform an ADC conversion and returns the last valid result
*  for the specified channel.
*
* \details
*  Returns the last valid result from the data structure for the specified channel.
*  This function can be used to read a previous result of any channel even if the
*  ADC is busy or a conversion is in progress. However, it is highly recommended
*  not to use this function with a channel that is in an active conversion.
*
* \param chId
*  The ID of the channel to be measured
*
* \return
*  The function returns a voltage in millivolts or
*  mADC_VALUE_BAD_CHAN_ID if chId is invalid.
*
*******************************************************************************/
uint16 mADC_GetResult_mVolts(uint8 chId)
{
    uint32 tmpRetVal = mADC_VALUE_BAD_CHAN_ID;

    if(chId < mADC_ADC_TOTAL_CHANNELS)
    {
        tmpRetVal = mADC_dsRam.adcResult[chId];
    }
    return (uint16)tmpRetVal;
}


/*******************************************************************************
* Function Name: mADC_Calibrate
****************************************************************************//**
*
* \brief
*  Performs calibration of the ADC module.
*
* \details
*  Performs calibration for the ADC to identify the appropriate hardware configuration
*  to produce accurate results. It is recommended to run the calibration
*  periodically (for example every 10 seconds) for accuracy and compensations.
*
* \return
*  The function returns cystatus of its operation.
*    - CYRET_SUCCESS - The block is configured for the ADC use.
*    - CYRET_LOCKED - The hardware is already in-use by a previously initialized
*      conversion or other functionality. No new conversion is started by this API.
*
*******************************************************************************/
cystatus mADC_Calibrate(void)
{
    uint32 tmpStatus;
    uint32 watchdogAdcCounter;
    uint32 tmpVrefCal;

    tmpStatus = mADC_AdcCaptureResources();

    if(mADC_STATUS_IDLE != (mADC_dsRam.adcStatus
        & mADC_STATUS_FSM_MASK))
    {
        tmpStatus = CYRET_LOCKED;
    }

    if (CYRET_SUCCESS == tmpStatus)
    {
        #if(mADC_ADC_ANALOG_STARTUP_DELAY_US > 0uL)
            CyDelayUs(mADC_ADC_ANALOG_STARTUP_DELAY_US);
        #endif /* (mADC_ADC_ANALOG_STARTUP_DELAY_US > 0uL) */

        /* Disconnect a channel if connected */
        if (mADC_NO_CHANNEL != mADC_dsRam.adcActiveCh)
        {
            mADC_SetAdcChannel(mADC_dsRam.adcActiveCh, mADC_CHAN_DISCONNECT);
            mADC_dsRam.adcActiveCh = mADC_NO_CHANNEL;
        }

        /* Get Vref trim-value */
        tmpVrefCal = (uint32)CY_GET_REG8(CYREG_SFLASH_CSDV2_CSD0_ADC_TRIM1) |
            (((uint32)CY_GET_REG8(CYREG_SFLASH_CSDV2_CSD0_ADC_TRIM2)) << 8u);

        /* Update nominal Vref to real Vref */
        tmpVrefCal *= mADC_ADC_VREF_MV;
        tmpVrefCal /= mADC_VREFCALIB_BASE;

        mADC_adcVrefMv = (uint16)tmpVrefCal;

        mADC_dsRam.adcIdac = (uint8)mADC_ADC_IDAC_DEFAULT;
        CY_SET_REG32(mADC_CSD_IDACB_PTR, mADC_IDACB_CONFIG |
                                                     mADC_dsRam.adcIdac);

        mADC_dsRam.adcStatus = (mADC_STATUS_CALIBPH1);

        /* Un-mask ADC_RES interrupt (enable interrupt) */
        CY_SET_REG32(mADC_INTR_MASK_PTR, mADC_INTR_MASK_ADC_RES_MASK);
        mADC_StartAdcFSM(mADC_MEASMODE_VREF);

        /* Global CRC update */
        #if (mADC_ENABLE == mADC_SELF_TEST_EN)
            #if (mADC_ENABLE ==mADC_TST_GLOBAL_CRC_EN)
                mADC_DsUpdateGlobalCrc();
            #endif /* (mADC_ENABLE == mADC_TST_GLOBAL_CRC_EN) */
        #endif /* (mADC_ENABLE == mADC_SELF_TEST_EN) */

        /* Initialize Watchdog Counter with time interval which is enough to ADC calibration is completed */
        watchdogAdcCounter = mADC_CAL_WATCHDOG_CYCLES_NUM;
        while (((*(volatile uint8 *)&mADC_dsRam.adcStatus & mADC_STATUS_FSM_MASK)
                != 0u) &&  (0u != watchdogAdcCounter))
        {
            /* Wait until scan complete and decrement Watchdog Counter to prevent unending loop */
            watchdogAdcCounter--;
        }
    }

    return tmpStatus;
}


/*******************************************************************************
* Function Name: mADC_Initialize
****************************************************************************//**
*
* \brief
*   Configures the hardware to ADC mode and begins a calibration.
*
* \details
*   Configures the hardware to ADC mode and begins a calibration.
*
*******************************************************************************/
void mADC_Initialize(void)
{
    mADC_ConfigAdcResources();
    (void)mADC_Calibrate();
}


/*******************************************************************************
* Function Name: mADC_Stop
****************************************************************************//**
*
* \brief
*   Disables the hardware sub-blocks that are in use while in the ADC mode,
*   and frees the routing.
*
* \details
*   This function stops the Component operation. No ADC conversion can be
*   initiated when the Component is stopped. Once stopped, the hardware block
*   may be reconfigured by the application program for any other special usage.
*   The ADC operation can be resumed by calling the
*   mADC_Resume() function or the Component can
*   be reset by calling the mADC_Start() function.
*   This function is called when no ADC conversion is in progress.
*
*******************************************************************************/
void mADC_Stop(void)
{
    #if (mADC_ENABLE != mADC_ADC_STANDALONE_EN)
        (void)mADC_AdcReleaseResources();
    #endif /* (mADC_ENABLE != mADC_ADC_STANDALONE_EN) */
    mADC_initVar = mADC_INIT_NEEDED;
}


/*******************************************************************************
* Function Name: mADC_Resume
****************************************************************************//**
*
* \brief
*   Resumes the ADC operation after a stop call.
*
* \details
*   Resumes the ADC operation if the operation is stopped
*   previously by the mADC_Stop() API.
*
*******************************************************************************/
void mADC_Resume(void)
{
    mADC_Initialize();
}


/*******************************************************************************
* Function Name: mADC_ClearAdcChannels
****************************************************************************//**
*
* \brief
*  Resets all the ADC channels to disconnected state.
*
* \details
*   The function goes through all the ADC channels and disconnects the pin
*   and the analog muxbus B.  Sets the drive mode of the pin as well.
*
*******************************************************************************/
void mADC_ClearAdcChannels(void)
{
    uint32 chId;

    for (chId = 0u; chId < mADC_ADC_TOTAL_CHANNELS; chId++)
    {
        mADC_SetAdcChannel((uint8)chId, mADC_CHAN_DISCONNECT);
    }
    mADC_dsRam.adcActiveCh = mADC_NO_CHANNEL;
}

/*******************************************************************************
* Function Name: mADC_SetModClkClockDivider
****************************************************************************//**
*
* \brief
*   Sets the divider values for the modulator clock and then starts
*   the modulator clock.
*
* \details
*   It is not recommended to call this function directly by the application layer.
*   It is used by initialization to enable the clocks.
*
* \param
*   modClk The divider value for the modulator clock.
*
*******************************************************************************/
void mADC_SetModClkClockDivider(uint32 modClk)
{
    /* Stop modulator clock   */
    CY_SET_REG32(mADC_MODCLK_CMD_PTR,
                 ((uint32)mADC_ModClk__DIV_ID <<
                 mADC_MODCLK_CMD_DIV_SHIFT)|
                 ((uint32)mADC_MODCLK_CMD_DISABLE_MASK));

    /*
     * Set divider value for modClk.
     * 1u is subtracted from modClk because Divider register value 0 corresponds
     * to dividing by 1.
     */
    CY_SET_REG32(mADC_MODCLK_DIV_PTR, ((modClk - 1u) << 8u));

    /* Check whether previous modulator clock start command has finished. */
    while(0u != (CY_GET_REG32(mADC_MODCLK_CMD_PTR) & mADC_MODCLK_CMD_ENABLE_MASK))
    {
        /* Wait until previous modulator clock start command has finished. */
    }

    /* Start modulator clock, aligned to HFCLK */
    CY_SET_REG32(mADC_MODCLK_CMD_PTR,
                 (uint32)(((uint32)mADC_ModClk__DIV_ID << mADC_MODCLK_CMD_DIV_SHIFT) |
                  ((uint32)mADC_ModClk__PA_DIV_ID << mADC_MODCLK_CMD_PA_DIV_SHIFT) |
                  mADC_MODCLK_CMD_ENABLE_MASK));
}

#endif /* #if mADC_ADC_EN */


/* [] END OF FILE */
