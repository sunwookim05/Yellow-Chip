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
        void (*pLed_Writr)(int);
    }out;
}system;

void publicSetUp(system *System){
    System->out.println = println;
    System->out.pLed_Writr = pLed_Write;
}

void privateSetUp(system *System){
    publicSetUp(System);
}

int main(void) {
    system System;
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    publicSetUp(&System);
    mUART_Start();
    for(;;) {
        /* Place your application code here. */
        System.out.pLed_Writr(1);
        CyDelay(500);
        System.out.pLed_Writr(0);
        CyDelay(500);
    }
}

/* [] END OF FILE */
