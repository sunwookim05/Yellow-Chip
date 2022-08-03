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
#ifndef __SSD1306__
#define __SSD1306__
    
    #include "project.h"

    #define bool _Bool
    #define true    1
    #define false   0
    #define SSD1306_SPI_MODE
    #define SSD1306_RST
    //------------------------------ SSD1306 Definitions ---------------------------------//

    #ifndef SSD1306_I2C_ADDRESS
        #define SSD1306_I2C_ADDRESS   0x7A  // 011110+SA0+RW - 0x78 or 0x7A (default)
    #endif

    #ifdef SSD1306_SPI_MODE
        #ifndef SSD1306_SPI_Write
            #define SSD1306_SPI_Write(x)  SPI_XFER(SSD1306, x)
        #endif
    #else
        #ifndef SSD1306_I2C_Start
            #define SSD1306_I2C_Start     I2C_Start(SSD1306)
        #endif
        #ifndef SSD1306_I2C_Write
            #define SSD1306_I2C_Write(x)  I2C_Write(SSD1306, x)
        #endif
        #ifndef SSD1306_I2C_Stop
            #define SSD1306_I2C_Stop      I2C_Stop(SSD1306)
        #endif
    #endif

    #if !defined SSD1306_128_32 && !defined SSD1306_96_16
        #define SSD1306_128_64
    #endif
    #if defined SSD1306_128_32 && defined SSD1306_96_16
        #error "Only one SSD1306 display can be specified at once"
    #endif

    #if defined SSD1306_128_64
        #define SSD1306_LCDWIDTH            128
        #define SSD1306_LCDHEIGHT            64
    #endif
    #if defined SSD1306_128_32
        #define SSD1306_LCDWIDTH            128
        #define SSD1306_LCDHEIGHT            32
    #endif
    #if defined SSD1306_96_16
        #define SSD1306_LCDWIDTH             96
        #define SSD1306_LCDHEIGHT            16
    #endif

    #define SSD1306_SETCONTRAST          0x81
    #define SSD1306_DISPLAYALLON_RESUME  0xA4
    #define SSD1306_DISPLAYALLON         0xA5
    #define SSD1306_NORMALDISPLAY        0xA6
    #define SSD1306_INVERTDISPLAY_       0xA7
    #define SSD1306_DISPLAYOFF           0xAE
    #define SSD1306_DISPLAYON            0xAF
    #define SSD1306_SETDISPLAYOFFSET     0xD3
    #define SSD1306_SETCOMPINS           0xDA
    #define SSD1306_SETVCOMDETECT        0xDB
    #define SSD1306_SETDISPLAYCLOCKDIV   0xD5
    #define SSD1306_SETPRECHARGE         0xD9
    #define SSD1306_SETMULTIPLEX         0xA8
    #define SSD1306_SETLOWCOLUMN         0x00
    #define SSD1306_SETHIGHCOLUMN        0x10
    #define SSD1306_SETSTARTLINE         0x40
    #define SSD1306_MEMORYMODE           0x20
    #define SSD1306_COLUMNADDR           0x21
    #define SSD1306_PAGEADDR             0x22
    #define SSD1306_COMSCANINC           0xC0
    #define SSD1306_COMSCANDEC           0xC8
    #define SSD1306_SEGREMAP             0xA0
    #define SSD1306_CHARGEPUMP           0x8D
    #define SSD1306_EXTERNALVCC          0x01
    #define SSD1306_SWITCHCAPVCC         0x02

    // Scrolling #defines
    #define SSD1306_ACTIVATE_SCROLL                      0x2F
    #define SSD1306_DEACTIVATE_SCROLL                    0x2E
    #define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3
    #define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
    #define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
    #define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
    #define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A

    #define BLACK   0
    #define WHITE   1
    #define INVERSE 2

    uint8_t _vccstate, rotation;         ///< Display rotation (0 thru 3) default is 0
    #if !defined SSD1306_SPI_MODE
        uint8_t _i2caddr;
    #endif
    uint8_t _width, _height;
    #ifdef SSD1306_SPI_MODE
        void SSD1306_begin(uint8_t vccstate);  // SPI mode
    #else
        void SSD1306_begin(uint8_t vccstate, uint8_t i2caddr);  // I2C mode
    #endif
    void drawPixel(uint8_t x, uint8_t y, uint8_t color);
    void drawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
    void drawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
    void fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

    void setRotation(uint8_t m);
    void fillScreen(void);
    void display_clear(void);
    void display_dim(bool dim);
    void display(void);
    void display_directSetVertical(uint8 *inData);
    void invertDisplay(bool inv);

    void display_startScrollRight(uint8_t start, uint8_t stop);
    void display_startScrollLeft(uint8_t start, uint8_t stop);
    void display_startScrollDiagRight(uint8_t start, uint8_t stop);
    void display_startScrollDiagLeft(uint8_t start, uint8_t stop);
    void display_stopScroll(void);

    void drawFastHLineInternal(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
    void drawFastVLineInternal(uint8_t x, uint8_t __y, uint8_t __h, uint8_t color);
    
#endif
/* [] END OF FILE */
