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
#include "ssd1306.h"
#include "GFX_Library.h"
#include "timer.h"

int main(void) {
    CyGlobalIntEnable;

    initTimer();
    mUART_Start();
    mSPI_Start();
    
    SSD1306_begin(SSD1306_SWITCHCAPVCC);
    display_clear();
    display_setTextColor2(WHITE, BLACK);
    display_setTextSize(1);
    display_setCursor(0,0);
    display_string("yellowChip Ver1.0");
    display();
    uint8 value = 0;
    CyDelay(100);
    display_startScrollLeft(0, 0xf);
    for(;;) {
        display_startScrollLeft(0, 0xf);
        CyDelay(1000);
        display_stopScroll();
        CyDelay(1000);
        //mUART_UartPutString("Hello world\n");
        /*display_startScrollLeft(0x00, 0x0f);
        CyDelay(1000);
        display_stopScroll();
        drawPixel(127, value, 1);
        value++;
        if(value > 63) value = 0;
        CyDelay(100);*/
    }
}

/* [] END OF FILE */
