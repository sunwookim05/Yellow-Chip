
/******************************************************************************
                                  SSD1306.c                                   *
                   SSD1306 OLED driver for CCS PIC C compiler                 *
                                                                              *
 This driver supports SPI mode and I2C mode (default).                        *
 For the SPI mode and before the include of the driver file, add:             *
   #define SSD1306_SPI_MODE                                                   *
                                                                              *
 https://simple-circuit.com/                                                  *
                                                                              *
*******************************************************************************
*******************************************************************************
 This is a library for our Monochrome OLEDs based on SSD1306 drivers          *
                                                                              *
  Pick one up today in the adafruit shop!                                     *
  ------> http://www.adafruit.com/category/63_98                              *
                                                                              *
 Adafruit invests time and resources providing this open source code,         *
 please support Adafruit and open-source hardware by purchasing               *
 products from Adafruit!                                                      *
                                                                              *
 Written by Limor Fried/Ladyada  for Adafruit Industries.                     *
 BSD license, check license.txt for more information                          *
 All text above, and the splash screen must be included in any redistribution *
*******************************************************************************/

#include "ssd1306.h"

#define HWSPIDELAY  25
#define ssd1306_swap(a, b) { uint8_t t = a; a = b; b = t; }

//*************************** User Functions ***************************//



//--------------------------------------------------------------------------//
void ssd1306_command(uint8_t c);


static uint8_t ssd1306_buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#if (SSD1306_LCDHEIGHT == 64)
,
0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
};

void ssd1306_command(uint8_t c) {
    #ifdef SSD1306_SPI_MODE
    pOledDc_Write(0);
    mSPI_SpiUartPutArray(&c, 1);
    while (0u != (mSPI_SpiUartGetTxBufferSize() + mSPI_GET_TX_FIFO_SR_VALID));
    pOledDc_Write(1);
    #else
    uint8_t control = 0x00;   // Co = 0, D/C = 0
    SSD1306_I2C_Start;
    SSD1306_I2C_Write(_i2caddr);
    SSD1306_I2C_Write(control);
    SSD1306_I2C_Write(c);
    SSD1306_I2C_Stop;
    #endif
}

#ifdef SSD1306_SPI_MODE
void SSD1306_begin(uint8_t vccstate) {
#else
void SSD1306_begin(uint8_t vccstate, uint8_t i2caddr) {
    _i2caddr  = i2caddr;
#endif
    _vccstate = vccstate;
    CyDelay(10);//delay_ms(10);

  #ifdef SSD1306_SPI_MODE
    //pOledDc_Write(HIGH);//output_high(SSD1306_DC);
    //pOledSs_Write(HIGH);//output_high(SSD1306_CS);
    pOledDc_Write(1);
    //output_drive(SSD1306_DC);
    //output_drive(SSD1306_CS);
  #endif

  #ifdef SSD1306_RST
    pOledReset_Write(0);
    //output_drive(SSD1306_RST);
    CyDelay(10);
    pOledReset_Write(1);
  #endif

    // Init sequence
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80

    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(SSD1306_LCDHEIGHT - 1);

    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC)
        { ssd1306_command(0x10); }
    else
        { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);

 #if defined SSD1306_128_32
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0x8F);

#elif defined SSD1306_128_64
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
        { ssd1306_command(0x9F); }
    else
        { ssd1306_command(0xCF); }

#elif defined SSD1306_96_16
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x2);   //ada x12
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
        { ssd1306_command(0x10); }
    else
        { ssd1306_command(0xAF); }

#endif

    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC)
        { ssd1306_command(0x22); }
    else
        { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

    ssd1306_command(SSD1306_DISPLAYON);  //--turn on oled panel

    _height = SSD1306_LCDHEIGHT;
    _width  = SSD1306_LCDWIDTH;
}

// the most basic function, set a single pixel
void drawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if ((x >= _width) || (y >= _height))
        return;

    // check rotation, move pixel around if necessary
    switch (rotation) {
        case 1:
            ssd1306_swap(x, y);
            x = SSD1306_LCDWIDTH - x - 1;
            break;
        case 2:
            x = SSD1306_LCDWIDTH - x - 1;
            y = SSD1306_LCDHEIGHT - y - 1;
            break;
        case 3:
            ssd1306_swap(x, y);
            y = SSD1306_LCDHEIGHT - y - 1;
            break;
    }

  // x is which column
    switch (color) {
      case WHITE:   ssd1306_buffer[x + (uint16_t)(y/8) * SSD1306_LCDWIDTH] |=  (1 << (y&7)); break;
      case BLACK:   ssd1306_buffer[x + (uint16_t)(y/8) * SSD1306_LCDWIDTH] &= ~(1 << (y&7)); break;
      case INVERSE: ssd1306_buffer[x + (uint16_t)(y/8) * SSD1306_LCDWIDTH] ^=  (1 << (y&7)); break;
    }

}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display_scrollright(0x00, 0x0F)
void display_startScrollRight(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display_scrollright(0x00, 0x0F)
void display_startScrollLeft(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);//0x07,0x00
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display_scrollright(0x00, 0x0F)
void display_startScrollDiagRight(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display_scrollright(0x00, 0x0F)
void display_startScrollDiagLeft(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void display_stopScroll(void) {
    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void display_dim(bool dim)
{
  uint8_t contrast;
  if (dim)
    contrast = 0; // Dimmed display
  else {
    if (_vccstate == SSD1306_EXTERNALVCC)
      contrast = 0x9F;
    else
      contrast = 0xCF;
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  ssd1306_command(SSD1306_SETCONTRAST);
  ssd1306_command(contrast);
}

void display(void)
{
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
  #endif

  /*for (uint16_t i = 0; i < (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8); i++) {
      // send a bunch of data in one xmission
      #ifdef SSD1306_SPI_MODE
      //pOledSs_Write(LOW);//output_low(SSD1306_CS);
      //SoftSPI_WriteByte(ssd1306_buffer[i]);//SpiOled_WriteByte(ssd1306_buffer[i]);//SSD1306_SPI_Write( ssd1306_buffer[i] );
      //while((SpiOled_ReadTxStatus() & (SpiOled_STS_SPI_DONE | SpiOled_STS_SPI_IDLE)) == false);
      //SpiOled_WriteTxData(ssd1306_buffer[i]);CyDelayUs(HWSPIDELAY);//SpiOled_WriteByte(ssd1306_buffer[i]);
        pOledDc_Write(1);
        mSPI_SpiUartPutArray(&ssd1306_buffer[i], 1);
        while (0u != (mSPI_SpiUartGetTxBufferSize() + mSPI_GET_TX_FIFO_SR_VALID));
      //pOledSs_Write(HIGH);//output_high(SSD1306_CS);
      #else
      SSD1306_I2C_Start;
      SSD1306_I2C_Write(_i2caddr);
      SSD1306_I2C_Write(0x40);
      for (uint8_t x = 0; x < 16; x++) {
        SSD1306_I2C_Write(ssd1306_buffer[i]);
        i++;
      }
      i--;
      SSD1306_I2C_Stop;
      #endif
    }//*/
    pOledDc_Write(1);
    mSPI_SpiUartPutArray(ssd1306_buffer, SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8);
    while (0u != (mSPI_SpiUartGetTxBufferSize() + mSPI_GET_TX_FIFO_SR_VALID));//*/
}

void display_setColum(uint8 col) {
}

void display_setRow(uint8 row) {
}

void display_directSetVertical(uint8 *inData) {
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);   // Column start address (0 = reset)
    ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0); // Page start address (0 = reset)
    #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
    #endif
    #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
    #endif
    #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
    #endif

    for (uint16_t i = 0; i < 8; i++) {
        pOledDc_Write(1);
        mSPI_SpiUartPutArray(inData, 1);
        while (0u != (mSPI_SpiUartGetTxBufferSize() + mSPI_GET_TX_FIFO_SR_VALID));
    }
}

void display_clear(void)
{
  for (uint16_t i = 0; i < (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8); i++)
    ssd1306_buffer[i] = 0;
}

void fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
  for (int16_t i = x; i < x + w; i++)
    drawVLine(i, y, h, color);
}

void fillScreen(void) {
  fillRect(0, 0, SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, 1);
}

// invert the display
void invertDisplay(bool i)
{
  if (i)
    ssd1306_command(SSD1306_INVERTDISPLAY_);
  else
    ssd1306_command(SSD1306_NORMALDISPLAY);
}

void setRotation(uint8_t m) {
    rotation = (m & 3);
    switch(rotation) {
        case 0:
        case 2:
            _width  = SSD1306_LCDWIDTH;
            _height = SSD1306_LCDHEIGHT;
            break;
        case 1:
        case 3:
            _width  = SSD1306_LCDHEIGHT;
            _height = SSD1306_LCDWIDTH;
            break;
    }
}

void drawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color) {
    bool bSwap = false;
    switch(rotation) {
        case 0:
            // 0 degree rotation, do nothing
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x
            bSwap = true;
            ssd1306_swap(x, y);
            x = SSD1306_LCDWIDTH - x - 1;
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for _height.
            x = SSD1306_LCDWIDTH - x - 1;
            y = SSD1306_LCDHEIGHT - y - 1;
            x -= (w-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
            bSwap = true;
            ssd1306_swap(x, y);
            y = SSD1306_LCDHEIGHT - y - 1;
            y -= (w-1);
            break;
    }

    if(bSwap) {
        drawFastVLineInternal(x, y, w, color);
    }
    else {
        drawFastHLineInternal(x, y, w, color);
    }
}

void drawFastHLineInternal(uint8_t x, uint8_t y, uint8_t w, uint8_t color) {
    // Do bounds/limit checks
    if(y >= SSD1306_LCDHEIGHT) { return; }

    // make sure we don't go off the edge of the display
    if( (x + w) > SSD1306_LCDWIDTH) {
        w = (SSD1306_LCDWIDTH - x);
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = ssd1306_buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((uint16_t)(y/8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y&7);

    switch (color) {
        case WHITE:                 while(w--) { *pBuf++ |= mask; }; break;
        case BLACK: mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
        case INVERSE:               while(w--) { *pBuf++ ^= mask; }; break;
    }
}

void drawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color) {
    bool bSwap = false;
    switch(rotation) {
        case 0:
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
            bSwap = true;
            ssd1306_swap(x, y);
            x = SSD1306_LCDWIDTH - x - 1;
            x -= (h-1);
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for _height.
            x = SSD1306_LCDWIDTH - x - 1;
            y = SSD1306_LCDHEIGHT - y - 1;
            y -= (h-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y
            bSwap = true;
            ssd1306_swap(x, y);
            y = SSD1306_LCDHEIGHT - y - 1;
            break;
    }

    if(bSwap) {
        drawFastHLineInternal(x, y, h, color);
    }
    else {
        drawFastVLineInternal(x, y, h, color);
    }
}

void drawFastVLineInternal(uint8_t x, uint8_t __y, uint8_t __h, uint8_t color) {

    // do nothing if we're off the left or right side of the screen
    if(x >= SSD1306_LCDWIDTH) { return; }

    // make sure we don't go past the _height of the display
    if( (__y + __h) > SSD1306_LCDHEIGHT) {
        __h = (SSD1306_LCDHEIGHT - __y);
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;

    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = ssd1306_buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((uint16_t)(y/8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y&7);
    if(mod) {
        // mask off the high n bits we want to set
        mod = 8-mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if( h < mod) {
            mask &= (0XFF >> (mod-h));
        }

        switch (color) {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }

        // fast exit if we're done here!
        if(h<mod) { return; }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if(h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
                *pBuf=~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while(h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do  {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while(h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if(h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }
    }
}

// end of driver code.