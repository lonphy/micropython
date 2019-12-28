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

#include <stdint.h>
#include <stdio.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "lib/utils/pyexec.h"
#include "drivers/dht/dht.h"
#include "stm32_it.h"
#include "irq.h"
#include "lcd.h"
#include "led.h"
#include "usrsw.h"
#include "storage.h"
#include "sdcard.h"
#include "accel.h"
#include "servo.h"
#include "usb.h"
#include "portmodules.h"
#include "modmachine.h"
#include "extmod/vfs.h"
#include "extmod/utime_mphal.h"

char pyb_country_code[2];

STATIC mp_obj_t pyb_fault_debug(mp_obj_t value) {
    pyb_hard_fault_debug = mp_obj_is_true(value);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_fault_debug_obj, pyb_fault_debug);

// Returns the number of milliseconds which have elapsed since `start`.
// This function takes care of counter wrap and always returns a positive number.
STATIC mp_obj_t pyb_elapsed_millis(mp_obj_t start) {
    uint32_t startMillis = mp_obj_get_int(start);
    uint32_t currMillis = mp_hal_ticks_ms();
    return MP_OBJ_NEW_SMALL_INT((currMillis - startMillis) & 0x3fffffff);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_elapsed_millis_obj, pyb_elapsed_millis);

// Returns the number of microseconds which have elapsed since `start`.
// This function takes care of counter wrap and always returns a positive number.
STATIC mp_obj_t pyb_elapsed_micros(mp_obj_t start) {
    uint32_t startMicros = mp_obj_get_int(start);
    uint32_t currMicros = mp_hal_ticks_us();
    return MP_OBJ_NEW_SMALL_INT((currMicros - startMicros) & 0x3fffffff);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_elapsed_micros_obj, pyb_elapsed_micros);

MP_DECLARE_CONST_FUN_OBJ_KW(pyb_main_obj); // defined in main.c

STATIC mp_obj_t pyb_country(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0) {
        return mp_obj_new_str(pyb_country_code, 2);
    } else {
        size_t len;
        const char *str = mp_obj_str_get_data(args[0], &len);
        if (len != 2) {
            mp_raise_ValueError(NULL);
        }
        pyb_country_code[0] = str[0];
        pyb_country_code[1] = str[1];
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_country_obj, 0, 1, pyb_country);

STATIC const mp_rom_map_elem_t pyb_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_pyb) },
    { MP_ROM_QSTR(MP_QSTR_fault_debug), MP_ROM_PTR(&pyb_fault_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_repl_info),   MP_ROM_PTR(&pyb_set_repl_info_obj) },

    #if IRQ_ENABLE_STATS
    { MP_ROM_QSTR(MP_QSTR_irq_stats), MP_ROM_PTR(&pyb_irq_stats_obj) },
    #endif

    { MP_ROM_QSTR(MP_QSTR_main),    MP_ROM_PTR(&pyb_main_obj) },
    { MP_ROM_QSTR(MP_QSTR_country), MP_ROM_PTR(&pyb_country_obj) },

    #if MICROPY_HW_ENABLE_USB
    { MP_ROM_QSTR(MP_QSTR_usb_mode),       MP_ROM_PTR(&pyb_usb_mode_obj) },
    #if MICROPY_HW_USB_HID
    { MP_ROM_QSTR(MP_QSTR_hid_mouse),      MP_ROM_PTR(&pyb_usb_hid_mouse_obj) },
    { MP_ROM_QSTR(MP_QSTR_hid_keyboard),   MP_ROM_PTR(&pyb_usb_hid_keyboard_obj) },
    { MP_ROM_QSTR(MP_QSTR_USB_HID),        MP_ROM_PTR(&pyb_usb_hid_type) },
    #endif
    { MP_ROM_QSTR(MP_QSTR_USB_VCP),        MP_ROM_PTR(&pyb_usb_vcp_type) },
    #endif

    { MP_ROM_QSTR(MP_QSTR_elapsed_millis), MP_ROM_PTR(&pyb_elapsed_millis_obj) },
    { MP_ROM_QSTR(MP_QSTR_elapsed_micros), MP_ROM_PTR(&pyb_elapsed_micros_obj) },

    #if MICROPY_HW_ENABLE_SERVO
    { MP_ROM_QSTR(MP_QSTR_pwm), MP_ROM_PTR(&pyb_pwm_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo), MP_ROM_PTR(&pyb_servo_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_Servo), MP_ROM_PTR(&pyb_servo_type) },
    #endif

    #if MICROPY_HW_HAS_SWITCH
    { MP_ROM_QSTR(MP_QSTR_Switch), MP_ROM_PTR(&pyb_switch_type) },
    #endif

    #if defined(MICROPY_HW_LED1)
    { MP_ROM_QSTR(MP_QSTR_LED), MP_ROM_PTR(&pyb_led_type) },
    #endif

    #if MICROPY_HW_HAS_MMA7660
    { MP_ROM_QSTR(MP_QSTR_Accel), MP_ROM_PTR(&pyb_accel_type) },
    #endif

    #if MICROPY_HW_HAS_LCD
    { MP_ROM_QSTR(MP_QSTR_LCD), MP_ROM_PTR(&pyb_lcd_type) },
    #endif
};

STATIC MP_DEFINE_CONST_DICT(pyb_module_globals, pyb_module_globals_table);

const mp_obj_module_t pyb_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&pyb_module_globals,
};
