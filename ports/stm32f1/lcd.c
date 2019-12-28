/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include "py/mphal.h"
#include "py/runtime.h"

#if MICROPY_HW_HAS_LCD
#include "pin.h"
#include "bufhelper.h"
#include "font_petme128_8x8.h"
#include "lcd.h"

/// \moduleref pyb
/// \class LCD - LCD control for the LCD touch-sensor skin
///
/// The LCD class is used to control the LCD on the LCD touch-sensor skin,
/// 2.8" LCD.  The LCD is a 320x240 color(rgb565) screen.
///
/// you can use:
///
///     lcd.light(True)                 # turn the backlight on
///     lcd.write('Hello world!\n')     # print text to the screen
///
/// No render buffer for STM32F103xx, Because it has very little memory.
/// For example, to make a bouncing dot, try:
///
///     x = y = 0
///     dx = dy = 1
///     while True:
///         # update the dot's position
///         x += dx
///         y += dy
///
///         # make the dot bounce of the edges of the screen
///         if x <= 0 or x >= 239: dx = -dx
///         if y <= 0 or y >= 319: dy = -dy
///
///         lcd.fill(0)                 # clear the buffer with back colour
///         lcd.pixel(x, y, 0xffff)     # draw the dot with white colour
///         pyb.delay(50)               # pause for 50ms

#define LCD_INSTR (0)
#define LCD_DATA (1)

#define LCD_CHAR_BUF_W (16)
#define LCD_CHAR_BUF_H (4)

#define LCD_PIX_BUF_W (240)
#define LCD_PIX_BUF_H (320)
#define LCD_PIX_BUF_BYTE_SIZE (LCD_PIX_BUF_W * LCD_PIX_BUF_H * 2)

#define REG_00 (0x0000U) // Driver Code Read
#define REG_01 (0x0001U) // Driver Output Control 1

#define LCD_CMD(__lcd__,__cmd__) (*(__lcd__->cmd_address) = (__cmd__) )
#define LCD_DAT(__lcd__,__dat__) (*(__lcd__->dat_address) = (__dat__) )

typedef struct _pyb_lcd_obj_t {
    mp_obj_base_t base;

    // hardware control for the LCD
    __IO uint16_t *dat_address;
    __IO uint16_t *cmd_address;
    const pin_obj_t *pin_backlight;
    
    uint16_t fg_colour;
    uint16_t bg_colour;
    uint32_t device_id;

    // character buffer for stdout-like output
    char char_buffer[LCD_CHAR_BUF_W * LCD_CHAR_BUF_H];
    int line;
    int column;
    int next_line;
} pyb_lcd_obj_t;


static pyb_lcd_obj_t pyb_lcd_obj = {
    {&pyb_lcd_type},
    .dat_address = ((__IO uint16_t *)( MICROPY_HW_LCD_BANK | (1<<MICROPY_HW_LCD_FMSC_ADDRn) )),
    .cmd_address = ((__IO uint16_t *) MICROPY_HW_LCD_BANK ),
    .pin_backlight = MICROPY_HW_LCD_EN,
    .fg_colour = 0xffff,
    .bg_colour = 0x0000,
    .device_id = 0xffff,
};

STATIC inline void lcd_write_reg(pyb_lcd_obj_t *self, uint16_t index, uint16_t dat) {
    LCD_CMD(self, index);
    LCD_DAT(self, dat);
}

uint16_t lcd_read_reg(pyb_lcd_obj_t *self, uint16_t index) {
    LCD_CMD(self, index);
    index = *(self->dat_address);
    (void) index;
    return *(self->dat_address);
}

uint16_t lcd_bgr2rgb(uint16_t c) {
    uint16_t r, g, b, rgb;

    b = (c >> 0) & 0x1f;
    g = (c >> 5) & 0x3f;
    r = (c >> 11) & 0x1f;

    rgb = (b << 11) + (g << 5) + (r << 0);

    return (rgb);
}

STATIC inline void lcd_set_cursor(pyb_lcd_obj_t *self, uint16_t x, uint16_t y) {
    lcd_write_reg(self, 0x0020, x);
    lcd_write_reg(self, 0x0021, y);
}

STATIC inline void lcd_set_pixel(pyb_lcd_obj_t *self, uint16_t x, uint16_t y, uint16_t colour) {
    if ((x > LCD_PIX_BUF_W) || (y > LCD_PIX_BUF_H)) {
        return;
    }
    lcd_set_cursor(self, x, y);
    lcd_write_reg(self, 0x0022, colour);
}

STATIC void lcd_set_window(pyb_lcd_obj_t *self, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey) {
    lcd_set_cursor(self, sx, sy);
    lcd_write_reg(self, 0x0050, sx);
    lcd_write_reg(self, 0x0052, sy);
    lcd_write_reg(self, 0x0051, ex);
    lcd_write_reg(self, 0x0053, ey);
}

STATIC void lcd_write_char(pyb_lcd_obj_t *self, uint16_t x, uint16_t y, char c) {
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t tmp_char = 0;
    uint16_t index = (c - 0x20) * 8;

    for (i = 0; i < 8; i++) {
        tmp_char = font_petme128_8x8[index + i];

        // 按X方向绘制, 每次一行
        for (j = 0; j < 8; j++) {
            if ((tmp_char >> j) & 0x01) {
                lcd_set_pixel(self, x + i, y + j, self->fg_colour); // text colour
            } else {
                lcd_set_pixel(self, x + i, y + j, self->bg_colour); // background colour
            }
        }
    }
}

// write a string to the LCD at the current cursor location
// output it straight away (doesn't use the pixel buffer)
STATIC void lcd_write_strn(pyb_lcd_obj_t *lcd, const char *str, unsigned int len) {
}

/// \method init()
///
/// init low level hardware & frame buffer
STATIC mp_obj_t pyb_lcd_init(mp_obj_t self_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // config address pin
    mp_hal_pin_config(MICROPY_HW_LCD_FMSC_ADDR_PIN, MP_HAL_PIN_MODE_ALT, 0, 0);

    #if (MICROPY_HW_LCD_BANK == FSMC_BANK1_4) /* NE4 -- PG12 */
        MODIFY_REG(GPIOG->CRH, 0xF << 16, 0xB << 16);
        FSMC_Bank1->BTCR[6U] = 0x00001011;
        FSMC_Bank1->BTCR[7U] = 0x00000200;
    #endif

    #ifdef MICROPY_HW_LCD_EN
    self->pin_backlight = (MICROPY_HW_LCD_EN);
    mp_hal_pin_output(self->pin_backlight);
    mp_hal_pin_low(self->pin_backlight); // backlight off
    #endif
    
    // init the LCD
    mp_hal_delay_ms(1); // wait a bit
    LCD_DAT(self, 0xffff);

    lcd_write_reg(self, 0x0000, 0x0001); // Start OCS
    mp_hal_delay_ms(1);

    self->device_id = lcd_read_reg(self, REG_00); // read controller type
    if ((self->device_id & 0xff00) != 0x9300) {
        self->device_id = 0x9325;
    }

    if (self->device_id == 0x9320 || self->device_id == 0x9300) {
        lcd_write_reg(self, 0x00, 0x0000);
        lcd_write_reg(self, 0x01, 0x0100); // 驱动器输出控制
        lcd_write_reg(self, 0x02, 0x0700); // LCD驱动波形控制-反转
        lcd_write_reg(self, 0x03, 0x1030); // 进入模式--F-

        lcd_write_reg(self, 0x04, 0x0000); // 重新调整控制寄存器大小---缩放
        lcd_write_reg(self, 0x08, 0x0202); // Display Contral 2.(0x0207)
        lcd_write_reg(self, 0x09, 0x0000); // Display Contral 3.(0x0000)
        lcd_write_reg(self, 0x0a, 0x0000); // Frame信号输出间隔
        lcd_write_reg(self, 0x0c, (1 << 0)); // RGB显示接口控制1--16位
        lcd_write_reg(self, 0x0d, 0x0000);   // 帧标记的位置
        lcd_write_reg(self, 0x0f, 0x0000);   // RGB显示接口控制1
        mp_hal_delay_ms(5);
        lcd_write_reg(self, 0x07, 0x0101); // Display Contral.
        mp_hal_delay_ms(5);
        lcd_write_reg(self, 0x10, (1 << 12) | (0 << 8) | (1 << 7) | (1 << 6) | (0 << 4));//Power Control 1.(0x16b0)
        lcd_write_reg(self, 0x11, 0x0007); //Power Control 2.(0x0001)
        lcd_write_reg(self, 0x12, (1 << 8) | (1 << 4) | (0 << 0));//Power Control 3.(0x0138)
        lcd_write_reg(self, 0x13, 0x0b00);//Power Control 4.
        lcd_write_reg(self, 0x29, 0x0000); //Power Control 7.

        lcd_write_reg(self, 0x2b, (1 << 14) | (1 << 4));//帧速率和色彩控制---70
        lcd_write_reg(self, 0x50, 0);//Set X Star240*320
        lcd_write_reg(self, 0x51, 239); //水平GRAM终止位置Set X End.

        lcd_write_reg(self, 0x52, 0);//Set Y Star
        lcd_write_reg(self, 0x53, 319);//Set Y End.t.

        lcd_write_reg(self, 0x60, 0x2700);//门扫描控制器
        lcd_write_reg(self, 0x61, 0x0001);//Driver Output Control.
        lcd_write_reg(self, 0x6a, 0x0000); //Vertical Srcoll Control.

        lcd_write_reg(self, 0x80, 0x0000); //局部影像控制器1
        lcd_write_reg(self, 0x81, 0x0000); //局部影像控制器1--起始地址
        lcd_write_reg(self, 0x82, 0x0000); //局部影像控制器1--终止地址
        lcd_write_reg(self, 0x83, 0x0000); //Displsy Position? Partial Display 2.
        lcd_write_reg(self, 0x84, 0x0000); //RAM Address Start? Partial Display 2.
        lcd_write_reg(self, 0x85, 0x0000); //RAM Address End? Partial Display 2.

        lcd_write_reg(self, 0x90, (0 << 7) | (16 << 0)); //平板接口控制器1(0x0013)
        lcd_write_reg(self, 0x92, 0x0000);//Panel Interface Contral 2.(0x0000)
        lcd_write_reg(self, 0x93, 0x0001); //Panel Interface Contral 3.
        lcd_write_reg(self, 0x95, 0x0110); //Frame Cycle Contral.(0x0110)
        lcd_write_reg(self, 0x97, (0 << 8)); //
        lcd_write_reg(self, 0x98, 0x0000); //Frame Cycle Contral.
        lcd_write_reg(self, 0x07, 0x0173);//(0x0173)
    }

    /*ILI9325*/
    else if (self->device_id == 0x9325 || self->device_id == 0x9328) {
        lcd_write_reg(self, 0x00e7, 0x0010);
        lcd_write_reg(self, REG_00, 0x0001); // 开启内部时钟
        lcd_write_reg(self, REG_01, 0x0100); // SS=1, SM=0(G320->G1)
        lcd_write_reg(self, 0x0002, 0x0700); // 电源开启

        lcd_write_reg(self, 0x0003, (1 << 12) | (3 << 4) | (0 << 3)); //65K
        lcd_write_reg(self, 0x0004, 0x0000);
        lcd_write_reg(self, 0x0008, 0x0207);
        lcd_write_reg(self, 0x0009, 0x0000);
        lcd_write_reg(self, 0x000a, 0x0000); //display setting
        lcd_write_reg(self, 0x000c, 0x0001); //display setting 16bit-rgb with 1 transfer
        lcd_write_reg(self, 0x000d, 0x0000); //0f3c
        lcd_write_reg(self, 0x000f, 0x0000);
        
        //电源配置
        lcd_write_reg(self, 0x0010, 0x0000);
        lcd_write_reg(self, 0x0011, 0x0007);
        lcd_write_reg(self, 0x0012, 0x0000);
        lcd_write_reg(self, 0x0013, 0x0000);
        mp_hal_delay_ms(1);
        lcd_write_reg(self, 0x0010, 0x1590);
        lcd_write_reg(self, 0x0011, 0x0227);
        mp_hal_delay_ms(1);
        lcd_write_reg(self, 0x0012, 0x009c);
        mp_hal_delay_ms(1);
        lcd_write_reg(self, 0x0013, 0x1900);
        lcd_write_reg(self, 0x0029, 0x0023);
        lcd_write_reg(self, 0x002b, 0x000e);
        mp_hal_delay_ms(1);
        lcd_write_reg(self, 0x0020, 0x0000);
        lcd_write_reg(self, 0x0021, 0x013f);
        mp_hal_delay_ms(1);
        //伽马校正
        lcd_write_reg(self, 0x0030, 0x0007);
        lcd_write_reg(self, 0x0031, 0x0707);
        lcd_write_reg(self, 0x0032, 0x0006);
        lcd_write_reg(self, 0x0035, 0x0704);
        lcd_write_reg(self, 0x0036, 0x1f04);
        lcd_write_reg(self, 0x0037, 0x0004);
        lcd_write_reg(self, 0x0038, 0x0000);
        lcd_write_reg(self, 0x0039, 0x0706);
        lcd_write_reg(self, 0x003c, 0x0701);
        lcd_write_reg(self, 0x003d, 0x000f);
        mp_hal_delay_ms(1);
        lcd_write_reg(self, 0x0050, 0x0000); // 水平GRAM 地址范围 [0,0xef]
        lcd_write_reg(self, 0x0051, 0x00ef); // 结束位置
        lcd_write_reg(self, 0x0052, 0x0000); // 垂直GRAM 地址范围 [0, 0x13f]
        lcd_write_reg(self, 0x0053, 0x013f); // 结束位置

        lcd_write_reg(self, 0x0060, 0xa700);
        lcd_write_reg(self, 0x0061, 0x0001);
        lcd_write_reg(self, 0x006a, 0x0000);
        lcd_write_reg(self, 0x0080, 0x0000);
        lcd_write_reg(self, 0x0081, 0x0000);
        lcd_write_reg(self, 0x0082, 0x0000);
        lcd_write_reg(self, 0x0083, 0x0000);
        lcd_write_reg(self, 0x0084, 0x0000);
        lcd_write_reg(self, 0x0085, 0x0000);

        lcd_write_reg(self, 0x0090, 0x0010);
        lcd_write_reg(self, 0x0092, 0x0000);
        lcd_write_reg(self, 0x0093, 0x0003);
        lcd_write_reg(self, 0x0095, 0x0110);
        lcd_write_reg(self, 0x0097, 0x0000);
        lcd_write_reg(self, 0x0098, 0x0000);

        //开启显示设置
        lcd_write_reg(self, 0x0007, 0x0133);
        lcd_write_reg(self, 0x0020, 0x0000);
        lcd_write_reg(self, 0x0021, 0x013f);
    }

    mp_hal_delay_ms(2);
    mp_hal_pin_high(self->pin_backlight); // 开背光

    // clear local char buffer
    memset(self->char_buffer, ' ', LCD_CHAR_BUF_H * LCD_CHAR_BUF_W);
    self->line = 0;
    self->column = 0;
    self->next_line = 0;
    self->fg_colour = 0xffff;
    self->bg_colour = 0x0000;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_lcd_init_obj, pyb_lcd_init);


STATIC mp_obj_t pyb_lcd_deinit(mp_obj_t self_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    (void) self;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_lcd_deinit_obj, pyb_lcd_deinit);

/// \method reg([index [, value]])
///
/// set or get value from/to ILI9325
STATIC mp_obj_t pyb_lcd_reg(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t reg_index = mp_obj_get_int(args[1]) & 0xff;
    if (n_args == 3) {
        uint16_t reg_value = mp_obj_get_int(args[2]) & 0x00ff;
        lcd_write_reg(self, reg_index, reg_value);
        return mp_const_none;
    } else {
        return mp_obj_new_int( lcd_read_reg(self, reg_index) );
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_reg_obj, 2, 3, pyb_lcd_reg);

/// \method contrast(value)
///
/// Set the contrast of the LCD.  Valid values are between 0 and 47.
STATIC mp_obj_t pyb_lcd_contrast(mp_obj_t self_in, mp_obj_t contrast_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int contrast = mp_obj_get_int(contrast_in);
    (void) self;
    (void) contrast;
    return mp_const_notimplemented;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_lcd_contrast_obj, pyb_lcd_contrast);

/// \method light(value)
///
/// Turn the backlight on/off.  True or 1 turns it on, False or 0 turns it off.
STATIC mp_obj_t pyb_lcd_light(mp_obj_t self_in, mp_obj_t value) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (mp_obj_is_true(value)) {
        mp_hal_pin_high(self->pin_backlight); // set pin high to turn backlight on
    } else {
        mp_hal_pin_low(self->pin_backlight); // set pin low to turn backlight off
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_lcd_light_obj, pyb_lcd_light);

/// \method write(str)
///
/// Write the string `str` to the screen.  It will appear immediately.
STATIC mp_obj_t pyb_lcd_write(mp_obj_t self_in, mp_obj_t str) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    size_t len;
    const char *data = mp_obj_str_get_data(str, &len);
    lcd_write_strn(self, data, len);
    return mp_const_notimplemented;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_lcd_write_obj, pyb_lcd_write);

/// \method fill(colour)
///
/// Fill the screen with the given colour (rgb565 format).
///
/// This method directly writes to LCD GRAM.
STATIC mp_obj_t pyb_lcd_fill(mp_obj_t self_in, mp_obj_t col_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t col = (uint16_t) mp_obj_get_int(col_in);
    uint32_t i;

    lcd_set_cursor(self, 0, 0);
    LCD_CMD(self, 0x0022);
    i = LCD_PIX_BUF_W * LCD_PIX_BUF_H;
    while (i--) {
        LCD_DAT(self, col);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_lcd_fill_obj, pyb_lcd_fill);

/// \method get(x, y)
///
/// Get the color at the position `(x, y)`.  Returns rgb565 value.
STATIC mp_obj_t pyb_lcd_get(mp_obj_t self_in, mp_obj_t x_in, mp_obj_t y_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t x = mp_obj_get_int(x_in);
    uint16_t y = mp_obj_get_int(y_in);

    lcd_set_cursor(self, x, y);

    return mp_obj_new_int( lcd_bgr2rgb(lcd_read_reg(self, 0x0022)) );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_lcd_get_obj, pyb_lcd_get);


/// \method pixel(x, y, colour)
///
/// Set the pixel at `(x, y)` to the given colour (rgb565).
STATIC mp_obj_t pyb_lcd_pixel(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int x = mp_obj_get_int(args[1]);
    int y = mp_obj_get_int(args[2]);
    uint16_t colour = mp_obj_get_int(args[3]);
    lcd_set_pixel(self, x, y, colour);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_pixel_obj, 4, 4, pyb_lcd_pixel);


STATIC mp_obj_t pyb_lcd_cursor(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t x = mp_obj_get_int(args[1]);
    uint16_t y = mp_obj_get_int(args[2]);
    lcd_set_cursor(self, x, y);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_cursor_obj, 3, 3, pyb_lcd_cursor);


/// \method set_window(sx, sy, ex, ey)
///
/// set the window area
STATIC mp_obj_t pyb_lcd_set_window(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t x_start = mp_obj_get_int(args[1]);
    uint16_t y_start = mp_obj_get_int(args[2]);
    uint16_t x_end = mp_obj_get_int(args[3]);
    uint16_t y_end = mp_obj_get_int(args[4]);

    lcd_set_window(self, x_start, y_start, x_end, y_end);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_set_window_obj, 5, 5, pyb_lcd_set_window);


/// \method set_window(sx, sy, ex, ey)
///
/// set the window area
STATIC mp_obj_t pyb_lcd_draw_pic(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t x_start = mp_obj_get_int(args[1]);
    uint16_t y_start = mp_obj_get_int(args[2]);
    uint16_t x_end = mp_obj_get_int(args[3]);
    uint16_t y_end = mp_obj_get_int(args[4]);
    size_t len;
    uint16_t i;
    const uint16_t *data = (const uint16_t *)mp_obj_str_get_data(args[5], &len);
    lcd_set_window(self, x_start, y_start, x_end, y_end);
    lcd_set_cursor(self, x_start, y_start);
    LCD_CMD(self, 0x0022);
    for (i = 0; i < (x_end * y_end); i++) {
        LCD_DAT(self, *data++);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_draw_pic_obj, 6, 6, pyb_lcd_draw_pic);


/// \method draw_h_line(sx, sy, ex, coluor)
///
/// draw horizontal line
STATIC mp_obj_t pyb_lcd_line_h(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t sx = mp_obj_get_int(args[1]);
    uint16_t sy = mp_obj_get_int(args[2]);
    uint16_t ex = mp_obj_get_int(args[3]);
    uint16_t colour = mp_obj_get_int(args[4]);
    uint16_t i;

    for (i = sx; i < sx + ex; i++) {
        lcd_set_pixel(self, i, sy, colour);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_line_h_obj, 5, 5, pyb_lcd_line_h);

STATIC void lcd_line(pyb_lcd_obj_t *self, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t colour) {
    uint16_t x,y, dx, dy;

    // horizontal line
    if (sy == ey) {
        if (sx <= ex) { x = sx; } else { x = ex; ex = sx; }
        
        while(x <= ex) {
            lcd_set_pixel(self, x++, sy, colour);
        }
        return;
    } 
    
    // vertical line
    else if (sx == ex) {
        if (sy <= ey){ y = sy; } else { y = ey; ey = sy; }
        while(y <= ey) {
            lcd_set_pixel(self, sx, y++, colour);
        }
        return;
    }
    
    else if (sx > ex) {
        dx = sx - ex;
        x = ex; ex = sx;
        y = ey; ey = sy;
    }
    
    else {
        dx = ex - sx;
        x = sx;
        y = sy;
    }
    
    if (sy > ey) {
        dy = sy - ey;
    } else {
        dy = ey - sy;
    }
    
    if (dx == dy) {
        while (x <= ex) {
            if (y > ey) { --y; } else { ++y; }
            lcd_set_pixel(self, ++x, y, colour);
        }
    }
    
    // others
    else {
        lcd_set_pixel(self, x, y, colour);

        if (y < ey) {
            if (dx > dy) {
                int16_t p = dy * 2 - dx;
                int16_t twoDy = 2 * dy;
                int16_t twoDyMinusDx = 2 * (dy - dx);
                while (x < ex) {
                    if (p < 0) {
                        p += twoDy;
                    } else {
                        ++y;
                        p += twoDyMinusDx;
                    }
                    lcd_set_pixel(self, ++x, y, colour);
                }
            } else {
                int16_t p = dx * 2 - dy;
                int16_t twoDx = 2 * dx;
                int16_t twoDxMinusDy = 2 * (dx - dy);
                while (y < ey) {
                    if (p < 0) {
                        p += twoDx;
                    } else {
                        ++x;
                        p += twoDxMinusDy;
                    }
                    lcd_set_pixel(self, x, ++y, colour);
                }
            }
        } else {

            if (dx > dy) {
                int16_t p = dy * 2 - dx;
                int16_t twoDy = 2 * dy;
                int16_t twoDyMinusDx = 2 * (dy - dx);
                while (x < ex) {
                    if (p < 0)
                        p += twoDy;
                    else {
                        --y;
                        p += twoDyMinusDx;
                    }
                    lcd_set_pixel(self, ++x, y, colour);
                }
            } else {
                int16_t p = dx * 2 - dy;
                int16_t twoDx = 2 * dx;
                int16_t twoDxMinusDy = 2 * (dx - dy);
                while (ey < y) {
                    if (p < 0)
                        p += twoDx;
                    else {
                        ++x;
                        p += twoDxMinusDy;
                    }
                    lcd_set_pixel(self, x, --y, colour);
                }
            }
        }
    }
}

/// \method line(sx, sy, ex, ey, coluor)
///
/// draw line
STATIC mp_obj_t pyb_lcd_line(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t sx = mp_obj_get_int(args[1]);
    uint16_t sy = mp_obj_get_int(args[2]);
    uint16_t ex = mp_obj_get_int(args[3]);
    uint16_t ey = mp_obj_get_int(args[4]);
    uint16_t colour = mp_obj_get_int(args[5]);
    lcd_line(self, sx, sy, ex, ey, colour);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_line_obj, 6, 6, pyb_lcd_line);

/// \method circle(cx, cy, radius, colour, fill)
///
/// draw circle
STATIC mp_obj_t pyb_lcd_circle(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t cx = mp_obj_get_int(args[1]);
    uint16_t cy = mp_obj_get_int(args[2]);
    uint16_t r = mp_obj_get_int(args[3]);
    uint16_t colour = mp_obj_get_int(args[4]);
    uint16_t x = 0;
    uint16_t y = r;
    int16_t delta = 3 - (r<<1);
    int16_t tmp;

    bool fill = false;
    if (n_args > 5) {
        fill = mp_obj_get_int(args[5]) == 1;
    }

    while (y > x) {
        if (fill) {
            lcd_line(self, cx + x, cy + y, cx - x, cy + y, colour);
            lcd_line(self, cx + x, cy - y, cx - x, cy - y, colour);
            lcd_line(self, cx + y, cy + x, cx - y, cy + x, colour);
            lcd_line(self, cx + y, cy - x, cx - y, cy - x, colour);
        } else {
            lcd_set_pixel(self, cx + x, cy + y, colour);
            lcd_set_pixel(self, cx - x, cy + y, colour);
            lcd_set_pixel(self, cx + x, cy - y, colour);
            lcd_set_pixel(self, cx - x, cy - y, colour);
            lcd_set_pixel(self, cx + y, cy + x, colour);
            lcd_set_pixel(self, cx - y, cy + x, colour);
            lcd_set_pixel(self, cx + y, cy - x, colour);
            lcd_set_pixel(self, cx - y, cy - x, colour);
        }
        ++x;
        if (delta >= 0) {
            --y;
            tmp = (x << 2);
            tmp -= (y << 2);
            delta += (tmp + 10);
        } else {
            delta += ((x << 2) + 6);
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_circle_obj, 5, 6, pyb_lcd_circle);


/// \method line_v(sx, sy, ey, colour)
///
/// draw vertical line
STATIC mp_obj_t  pyb_lcd_line_v(size_t n_args, const mp_obj_t *args) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t sx = mp_obj_get_int(args[1]);
    uint16_t sy = mp_obj_get_int(args[2]);
    uint16_t ey = mp_obj_get_int(args[3]);
    uint16_t colour = mp_obj_get_int(args[4]);
    uint16_t i;

    for (i = sy; i < sy + ey; i++) {
        lcd_set_pixel(self, sx, i, colour);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_line_v_obj, 5, 5, pyb_lcd_line_v);


/// \method rect(sx, sy, colour, ex=, ey=, fill=False)
/// \method rect(sx, sy, colour, width=, fill=False)
/// \method rect(sx, sy, colour, width=, height=, Fill=False)
///
/// Draw the given text to the position `(x, y)` using the given colour.
STATIC mp_obj_t  pyb_lcd_rect(size_t n_args, const mp_obj_t *args_in, mp_map_t *kw_args) {
    enum { ARG_ex, ARG_ey, ARG_width, ARG_height, ARG_fill };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ex,     MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1}},
        { MP_QSTR_ey,     MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1}},
        { MP_QSTR_width,  MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1}},
        { MP_QSTR_height, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1}},
        { MP_QSTR_fill,   MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-4, args_in+4, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args_in[0]);
    uint16_t sx = mp_obj_get_int(args_in[1]);
    uint16_t sy = mp_obj_get_int(args_in[2]);
    uint16_t colour = mp_obj_get_int(args_in[3]);
    bool fill = args[ARG_fill].u_bool;

    int16_t ex = args[ARG_ex].u_int;
    int16_t ey = args[ARG_ey].u_int;
    int16_t width = args[ARG_width].u_int;
    int16_t height = args[ARG_height].u_int;

    if (ex > 0 || ey > 0) {

    } else if (width > 0){
        if (height < 0) {
            height = width;
        }
        ex = sx + width;
        ey = sy + height;
    } else {
        mp_raise_ValueError("key args: ex+ey or width[+height] must be provide");
    }

    if (fill) {
        uint16_t i;
        if (sx > ex) {
            i = ex;
            ex = sx;
        } else {
            i = sx;
        }
        for (; i <= ex; i++) {
            lcd_line(self, i, sy, i, ey, colour);
        }
    } else {
        lcd_line(self, sx, sy, sx, ey, colour);
        lcd_line(self, sx, ey, ex, ey, colour);
        lcd_line(self, ex, ey, ex, sy, colour);
        lcd_line(self, ex, sy, sx, sy, colour);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_lcd_rect_obj, 4, pyb_lcd_rect);


/// \method text(str, x, y[, colour])
///
/// Draw the given text to the position `(x, y)` using the given colour.
STATIC mp_obj_t pyb_lcd_text(size_t n_args, const mp_obj_t *args) {
    bool hasColour = false;
    uint8_t i;
    uint16_t curr_colour;
    size_t len;
    int fg_colour = 0;

    if (n_args > 4 && mp_obj_get_int_maybe(args[4], &fg_colour)) {
        if (fg_colour < 0 || fg_colour > 0xffff) {
            mp_raise_TypeError("must specify colour in range [0, 0xffff] or not");
        }
        hasColour = true;
    }

    // extract arguments
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const char *data = mp_obj_str_get_data(args[1], &len);
    int x = mp_obj_get_int(args[2]);
    int y = mp_obj_get_int(args[3]);

    if (hasColour) {
        curr_colour = self->fg_colour;
        self->fg_colour = fg_colour;
    }

    for (i = 0; i < len; i++) {
        lcd_write_char(self, (x + 8 * i), y, *data++);
    }

    if (hasColour) {
        self->fg_colour = curr_colour;
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_lcd_text_obj, 4, 5, pyb_lcd_text);


/// \method id()
///
/// Get the lcd control chip id
STATIC mp_obj_t pyb_lcd_id(mp_obj_t self_in) {
    pyb_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->device_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_lcd_id_obj, pyb_lcd_id);


STATIC const mp_rom_map_elem_t pyb_lcd_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_init),          MP_ROM_PTR(&pyb_lcd_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),        MP_ROM_PTR(&pyb_lcd_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_id),            MP_ROM_PTR(&pyb_lcd_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg),           MP_ROM_PTR(&pyb_lcd_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_contrast),      MP_ROM_PTR(&pyb_lcd_contrast_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor),        MP_ROM_PTR(&pyb_lcd_cursor_obj) },
    { MP_ROM_QSTR(MP_QSTR_light),         MP_ROM_PTR(&pyb_lcd_light_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),         MP_ROM_PTR(&pyb_lcd_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),          MP_ROM_PTR(&pyb_lcd_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),           MP_ROM_PTR(&pyb_lcd_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixel),         MP_ROM_PTR(&pyb_lcd_pixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_text),          MP_ROM_PTR(&pyb_lcd_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_window),    MP_ROM_PTR(&pyb_lcd_set_window_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_pic),      MP_ROM_PTR(&pyb_lcd_draw_pic_obj) },
    { MP_ROM_QSTR(MP_QSTR_circle),        MP_ROM_PTR(&pyb_lcd_circle_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),          MP_ROM_PTR(&pyb_lcd_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_line_h),        MP_ROM_PTR(&pyb_lcd_line_h_obj) },
    { MP_ROM_QSTR(MP_QSTR_line_v),        MP_ROM_PTR(&pyb_lcd_line_v_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect),          MP_ROM_PTR(&pyb_lcd_rect_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pyb_lcd_locals_dict, pyb_lcd_locals_dict_table);

/// \classmethod \constructor()
///
/// Construct an LCD object.
STATIC mp_obj_t pyb_lcd_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    mp_obj_t self = MP_OBJ_FROM_PTR(&pyb_lcd_obj);
    pyb_lcd_deinit( self );
    pyb_lcd_init( self );
    return self;
}

const mp_obj_type_t pyb_lcd_type = {
    { &mp_type_type },
    .name = MP_QSTR_LCD,
    .make_new = pyb_lcd_make_new,
    .locals_dict = (mp_obj_dict_t*)&pyb_lcd_locals_dict,
};
#endif