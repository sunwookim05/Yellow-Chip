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
#include <math.h>
#include "project.h"

typedef char *String;

void println(String str){
    mUART_UartPutString(str);
    mUART_UartPutString("\n");
}

typedef struct _System{
    struct _IN{   
    
    }in;
    struct _OUT{
        void (*println)(String);
    }out;
}system;

void publicSetUp(system *System){
    System->out.println = println;
}

void privateSetUp(system *System){
    publicSetUp(System);
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    system System;
    double angleR = 0,angleG = 120, angleB = 240;
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    publicSetUp(&System);
    mUART_Start();
    mPWM0_Start();
    mPWM1_Start();
    mPWM2_Start();
    for(;;)
    {   
        angleR++;
        angleG++;
        angleB++;
        if(angleR >= 360) angleR -= 360;
        if(angleG >= 360) angleG -= 360; 
        if(angleB >= 360) angleB -= 360;
        int valR = (int)(sin(angleR * 3.14 / 180) * 500 + 500);
        int valG = (int)(sin(angleG * 3.14 / 180) * 500 + 500);
        int valB = (int)(sin(angleB * 3.14 / 180) * 500 + 500);
        mPWM0_WriteCompare(valR);
        mPWM1_WriteCompare(valG);
        mPWM2_WriteCompare(valB);
        CyDelay(1);
        char str[30];
        sprintf(str, "%d %d %d", valR, valG, valB);
        System.out.println(str);
    }
}

/* [] END OF FILE */
