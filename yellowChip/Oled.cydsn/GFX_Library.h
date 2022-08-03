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
#ifndef __GFX_LIBRARY__
#define __GFX_LIBRARY__
    
#include "project.h"
#define bool _Bool
#define true    1
#define false   0
void display_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void display_drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void display_drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void display_drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t color);
void display_fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void display_fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t delta, uint16_t color);
void display_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void display_fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void display_drawRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
void display_fillRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);

void display_setCursor(uint16_t x, uint16_t y);
void display_setTextColor(uint16_t c);
void display_setTextColor2(uint16_t c, uint16_t bg);
void display_setTextSize(uint8_t s);
void display_setTextWrap(bool w);
void display_print(uint8_t c);
void display_string(char *str);
void display_customChar(const uint8_t *c);
void display_drawChar(uint16_t x, uint16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size);

uint8_t  display_getRotation();
uint16_t getCursorX(void);
uint16_t getCursorY(void);
uint16_t display_getWidth();
uint16_t display_getHeight();
uint16_t display_color565(uint8_t red, uint8_t green, uint8_t blue);

void display_drawBitmapV1   (uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color);
void display_drawBitmapV1_bg(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg);
void display_drawBitmapV2   (uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color);
void display_drawBitmapV2_bg(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg);

#endif
/* [] END OF FILE */
