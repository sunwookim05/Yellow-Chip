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
#include <stdio.h>
#include "project.h"

#define LEDOFF LEDR_Write(0);LEDG_Write(0);LEDB_Write(0)

int main(void)
{
    int pos = 0, posxy[2];
    CyGlobalIntEnable; /* Enable global interrupts. */
    mUART_Start();
    mADC_Start();
   
    mADC_StartConvert(pos);
    mADC_StartConvert(!pos);
    while(mADC_IsBusy()){}
    LEDOFF;
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    for(;;)
    {    
        if(posxy[1] >= 4000) {LEDOFF;}
        if(posxy[1] <= 1000) LEDR_Write(1);
        if(posxy[0] <= 1000) LEDG_Write(1);
        if(posxy[0] >= 4000) LEDB_Write(1);
        posxy[pos] = mADC_GetResult_mVolts(pos);
        pos = !pos;
        mADC_StartConvert(pos);
        while(mADC_IsBusy()){}
        
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
