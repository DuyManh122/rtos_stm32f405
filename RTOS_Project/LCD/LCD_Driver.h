/*****************************************************************************
* | File      	:		hx8347d.h
* | Author      :   Waveshare team
* | Function    :   HXD8347D driver
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-04-29
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef __LCD_H__
#define __LCD_H__

#include "stm32f4xx.h"

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

//color define
#define LCD_WIDTH    240
#define LCD_HEIGHT   320

#define FONT_1206    12
#define FONT_1608    16
#define FONT_GB2312  16

#define WHITE          0xFFFF
#define BLACK          0x0000	  
#define BLUE           0x001F  
#define BRED           0XF81F
#define GRED 		   	  0XFFE0
#define GBLUE		   		0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   0XBC40 
#define BRRED 		   0XFC07 
#define GRAY  		   0X8430 


/* PB3( SCK )   PB4( TIRQ )  PB6 ( BKL )  PB7( LCD_CS )  */
/* PB8( DC )    PB9( TCS )   PA6 ( SDO )  PA7( SDI )  	 */
/* PB2( RST ) */
#define LCD_DC_H()		GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define LCD_DC_L()		GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define LCD_CS_H()		GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define LCD_CS_L()		GPIO_ResetBits(GPIOB,GPIO_Pin_7)

#define LCD_BKL_H()    GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define LCD_BKL_L()    GPIO_ResetBits(GPIOB, GPIO_Pin_6)

#define LCD_RST_H()    GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define LCD_RST_L()    GPIO_ResetBits(GPIOB, GPIO_Pin_2)

#define LCD_CMD                0
#define LCD_DATA               1

#define ST7789_DEVICE
//#define HX8347D_DEVICE

void spi1_init(void);
uint8_t spi1_communication(uint8_t send_char);
void lcd_init(void);
void lcd_set_cursor(uint16_t hwXpos, uint16_t hwYpos);
void lcd_display_GBchar(uint16_t hwXpos,uint16_t hwYpos,uint8_t chChr,uint8_t chSize,uint16_t hwColor);
void lcd_display_GB2312(uint8_t gb,uint16_t color_front,uint16_t postion_x,uint16_t postion_y );
void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor);
void lcd_draw_bigdot(uint32_t color_front,uint32_t x,uint32_t y );
void lcd_display_number(uint32_t x,uint32_t y,unsigned long num,uint8_t num_len);
void lcd_clear_screen(uint16_t hwColor);
void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor);
void lcd_display_char(uint16_t hwXpos,uint16_t hwYpos,uint8_t chChr,uint8_t chSize,uint16_t hwColor); 
void lcd_display_num(uint16_t hwXpos,uint16_t hwYpos,uint32_t chNum,uint8_t chLen,uint8_t chSize,uint16_t hwColor); 
void lcd_display_string(uint16_t hwXpos,uint16_t hwYpos,const uint8_t *pchString,uint8_t chSize,uint16_t hwColor); 
void lcd_draw_line(uint16_t hwXpos0,uint16_t hwYpos0,uint16_t hwXpos1,uint16_t hwYpos1,uint16_t hwColor);
void lcd_draw_circle(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwRadius,uint16_t hwColor);
void lcd_fill_rect(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwHeight,uint16_t hwColor); 
void lcd_draw_v_line(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwHeight,uint16_t hwColor);
void lcd_draw_h_line(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwColor);
void lcd_draw_rect(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwHeight,uint16_t hwColor); 
void lcd_clear_Rect(uint32_t color_front,uint32_t x0,uint32_t y0,uint32_t x1,uint32_t y1);			
												
#endif
