/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "timer.h"

#define TIME_MIN_IN_HR      (60u)
#define TIME_SEC_IN_MIN     (60u)
#define TIME_MS_IN_SEC      (1000u)

void SysTickISRCallback(void) {
    static uint32 msCount;

    /* Counts the number of milliseconds in one second */
    if(msCount != 0u) {
        --msCount;
        //pLed_Write(msCount < 500 ? 1 : 0);
        if(msCount < 500) pLed_Write(1);
        else pLed_Write(0);
    }
    else {
        /* Counts from 999 down to 0 */
        msCount = TIME_MS_IN_SEC - 1u;
    }
}

void initTimer() {
    
    CySysTickStart();

    /* Find unused callback slot and assign the callback. */
    for (int i = 0u; i < CY_SYS_SYST_NUM_OF_CALLBACKS; ++i)
    {
        if (CySysTickGetCallback(i) == NULL)
        {
            /* Set callback */
            CySysTickSetCallback(i, SysTickISRCallback);
            break;
        }
    }
}

/* [] END OF FILE */
