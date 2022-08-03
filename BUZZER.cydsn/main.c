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
#include "project.h"

#define RXMAX 255

typedef char *String;

char ringBuff[RXMAX];
int ringPptr = 0;
int ringGptr = 0;

void isrUart(){
    if((mUART_GetRxInterruptSource() & mUART_INTR_RX_NOT_EMPTY) != 0){
        ringBuff[ringPptr++] = mUART_UartGetChar();
        ringPptr %= RXMAX;
        mUART_ClearRxInterruptSource(mUART_INTR_RX_NOT_EMPTY);
    }
    mUART_ClearPendingInt();
}

char scan(){
    char c = -1;
    if(ringGptr == ringPptr) return c;
    c = ringBuff[ringGptr++];
    ringGptr %= RXMAX;
    return c;
}

void println(String str){
    mUART_UartPutString(str);
    mUART_UartPutString("\n");
}

void sound(float f){
    float cnt = 1000000 / f;
    mPWM_WriteCompare(cnt / 2);
    mPWM_WritePeriod(cnt);
}

typedef struct _System{
    struct _IN{   
        void (*scan)(char *);
    }in;
    struct _OUT{
        void (*println)(String);
        void (*sound)(float);
    }out;
}system;

void publicSetUp(system *System){
    System->out.println = println;
    System->out.sound = sound;
    //System->in.scan = scan;
}

int main(void)
{   
    system System;
    char c = -1;
    float sheet[] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25};
    CyGlobalIntEnable; /* Enable global interrupts. */
    mUART_Start();
    mUART_EnableInt();
    mUART_SetCustomInterruptHandler(*isrUart);
    mPWM_Start();
    
    publicSetUp(&System);
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {   
        c = scan();
        mUART_UartPutChar(c);
        System.out.sound(sheet[c == '1' ? 0 : c == '2' ? 1 : c == '3' ? 2 : c == '4' ? 3 : c == '5' ? 4 : c == '6' ? 5 : c == '7' ? 6 : c == '8' ? 7 : -1]);
        CyDelay(100);
        /* Place your application code here. */
       
    }
}

/* [] END OF FILE */
