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

double kalmanCDS(double inData){
    const double valP = 0.00001;   
    static double r = 0.001, x = 0, p = 1, k;
    p += valP;
    k = p / (p + r);
    x = (k * inData) + (1 - k) * x;
    p = (1 - k) * p;
    return x;
}

double kalmans(double inData){
    const double valP = 0.00001;   
    static double r = 0.001, x = 0, p = 1, k;
    p += valP;
    k = p / (p + r);
    x = (k * inData) + (1 - k) * x;
    p = (1 - k) * p;
    return x;
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    int pos = 0;
    int adcval[2];
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    mUART_Start();
    mADC_Start();
    
    for(;;)
    {
        adcval[pos] = mADC_GetResult_mVolts(pos);
        char str[30];
        sprintf(str, "%04d %04d \n", (int)kalmans(adcval[0]), (int)kalmanCDS(adcval[1]));
        mUART_UartPutString(str);
        pos = !pos;
        CyDelay(50);
        mADC_StartConvert(pos);
        while(mADC_IsBusy()){}
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
