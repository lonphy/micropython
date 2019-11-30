/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017-2019 Damien P. George
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
#include "extmod/crypto-algorithms/sha256.c"
#include "usbd_core.h"
#include "storage.h"
#include "mboot.h"
#include "norflash.h"

// Using polling is about 10% faster than not using it (and using IRQ instead)
// This DFU code with polling runs in about 70% of the time of the ST bootloader
#define USE_USB_POLLING (0)

// Using cache probably won't make it faster because we run at a low frequency, and best
// to keep the MCU config as minimal as possible.
#define USE_CACHE (0)

// IRQ priorities (encoded values suitable for NVIC_SetPriority)
#define IRQ_PRI_SYSTICK (NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0))

// Configure PLL to give the desired CPU freq
#undef  MICROPY_HW_FLASH_LATENCY
#define MICROPY_HW_FLASH_LATENCY FLASH_LATENCY_1

#define CORE_PLL_FREQ  (72000000)

// These bits are used to detect valid application firmware at APPLICATION_ADDR
#define APP_VALIDITY_BITS (0x00000003)

#define MP_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void do_reset(void);

uint32_t get_le32(const uint8_t *b) {
    return b[0] | b[1] << 8 | b[2] << 16 | b[3] << 24;
}

void mp_hal_delay_us(mp_uint_t usec) {
    // use a busy loop for the delay
    // sys freq is always a multiple of 2MHz, so division here won't lose precision
    const uint32_t ucount = CORE_PLL_FREQ / 2000000 * usec / 2;
    for (uint32_t count = 0; ++count <= ucount;){}
}

static volatile uint32_t systick_ms;

void mp_hal_delay_ms(mp_uint_t ms) {
    if (__get_PRIMASK() == 0) {
        // IRQs enabled, use systick
        if (ms != 0 && ms != (mp_uint_t)-1) {
            ++ms; // account for the fact that systick_ms may roll over immediately
        }
        uint32_t start = systick_ms;
        while (systick_ms - start < ms) {
            __WFI();
        }
    } else {
        // IRQs disabled, so need to use a busy loop for the delay.
        // To prevent possible overflow of the counter we use a double loop.
        const uint32_t count_1ms = 16000000 / 8000;
        for (uint32_t i = 0; i < ms; i++) {
            for (volatile uint32_t count = 0; ++count <= count_1ms;) {
            }
        }
    }
}

// Needed by parts of the HAL
uint32_t HAL_GetTick(void) {
    return systick_ms;
}

// Needed by parts of the HAL
void HAL_Delay(uint32_t ms) {
    mp_hal_delay_ms(ms);
}

static void __fatal_error(const char *msg) {
    NVIC_SystemReset();
    for (;;) {
    }
}

/******************************************************************************/
// CLOCK
void SystemInit(void) {
    // Set HSION bit
    RCC->CR |= RCC_CR_HSION;

    // Reset CFGR register
    RCC->CFGR = 0x00000000;

    // Reset HSEON, CSSON and PLLON bits
    RCC->CR &= ~(RCC_CR_HSEON || RCC_CR_CSSON || RCC_CR_PLLON);

    // Reset CFGR register
    RCC->CFGR &= 0xFF80FFFFU;

    // Reset HSEBYP bit
    RCC->CR &= 0xFFFBFFFFU;

    // Disable all interrupts
    RCC->CIR = 0x00000000;

    // Set location of vector table
    SCB->VTOR = FLASH_BASE;

    // Enable 8-byte stack alignment for IRQ handlers, in accord with EABI
    SCB->CCR |= SCB_CCR_STKALIGN_Msk;
}

void systick_init(void) {
    // Configure SysTick as 1ms ticker
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, IRQ_PRI_SYSTICK);
}


void SystemClock_Config(void) {
    // This function assumes that HSI is used as the system clock (see RCC->CFGR, SWS bits)

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();

    // Turn HSE on
    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET);

    // Disable PLL
    __HAL_RCC_PLL_DISABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET);

    // Configure PLL factors and source
    RCC->CFGR =
        (1 << RCC_CFGR_PLLSRC_Pos) /* HSE selected as PLL source */
        | (RCC_HSE_PREDIV_DIV1)    /* HSE clock not divided */ 
        | (RCC_PLL_MUL9)           /* PLL input clock x 9 = 72MHz */
        | (RCC_USBCLKSOURCE_PLL_DIV1_5)   /* USB prescaler is divided by 1.5 = 48MHz */
        ;

    // Enable PLL
    __HAL_RCC_PLL_ENABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET);

    // Increase latency before changing clock
    if (MICROPY_HW_FLASH_LATENCY > (FLASH->ACR & FLASH_ACR_LATENCY)) {
        __HAL_FLASH_SET_LATENCY(MICROPY_HW_FLASH_LATENCY);
    }

    // Configure AHB divider
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);

    // Configure SYSCLK source from PLL
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK);

    // Decrease latency after changing clock
    if (MICROPY_HW_FLASH_LATENCY < (FLASH->ACR & FLASH_ACR_LATENCY)) {
        __HAL_FLASH_SET_LATENCY(MICROPY_HW_FLASH_LATENCY);
    }

    // Set APB clock dividers
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_HCLK_DIV1);

    // Update clock value and reconfigure systick now that the frequency changed
    SystemCoreClock = CORE_PLL_FREQ;
    systick_init();
}

// Needed by HAL_PCD_IRQHandler
uint32_t HAL_RCC_GetHCLKFreq(void) {
    return SystemCoreClock;
}

/******************************************************************************/
#define MP_HAL_PIN_MODE_INVALID    (4)
// this table converts from MP_HAL_PIN_MODE_xxx to STM32F1 CRL/CRH value, output use highest speed
const unsigned char mp_hal_pin_mod_cfg[7] = {
    [MP_HAL_PIN_MODE_IN]      = 0x04, // floating input
    [MP_HAL_PIN_MODE_OUT]     = 0x03, // output PP, 50MHz
    [MP_HAL_PIN_MODE_ALT]     = 0x0b, // alternat PP, 50MHz
    [MP_HAL_PIN_MODE_ANALOG]  = 0x00, // analog input
    [MP_HAL_PIN_MODE_INVALID] = 0x00, // not used
    [MP_HAL_PIN_MODE_OUT_OD]  = 0x07, // output OD, 50MHz
    [MP_HAL_PIN_MODE_ALT_OD]  = 0x0f, // alternat OD, 50MHz
};

// GPIO
void mp_hal_pin_config(mp_hal_pin_obj_t port_pin, uint32_t mode, uint32_t pull, uint32_t alt) {
    GPIO_TypeDef *gpio = (GPIO_TypeDef*)(port_pin & ~0xf);
    
    uint32_t gpio_idx = ((uintptr_t)gpio - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE);
    RCC->APB2ENR |= 1 << (RCC_APB2ENR_IOPAEN_Pos + gpio_idx);
    volatile uint32_t tmp = RCC->APB2ENR; // Delay after enabling clock
    (void)tmp;

    uint32_t pin = port_pin & 0xf;
    uint32_t moder = mp_hal_pin_mod_cfg[mode];
    uint32_t pos = (pin % 8) * 4;
    
    // input with pull up/down
    if ( moder == 0x04  && pull ) {
        moder = 0x08;
    }

    // get register CRL/CRH (pin0~pin7 use CRL, pin8~pin15 use CRH)
    register uint32_t *pReg = (uint32_t *)((uint32_t)(&gpio->CRL) + (pin / 8 * 4));
    *pReg = (*pReg & ~(0x0f << pos) ) | ( (moder & 0x0f) << pos );

    // set pull
    if (moder == 0x08) {
        gpio->BSRR= 1 << (pin + (1-pull%2) * 16);
    }
}

void mp_hal_pin_config_speed(uint32_t port_pin, uint32_t speed) {
    GPIO_TypeDef *gpio = (GPIO_TypeDef*)(port_pin & ~0xf);
    uint32_t pin = port_pin & 0xf;
    uint32_t pos = (pin % 8) * 4;

    register uint32_t *pReg = (uint32_t *)((uint32_t)(&gpio->CRL) + (pin / 8 * 4));
    if ( (*pReg >> pos) & 3 ) {
        *pReg = (*pReg & ~(3 << pos)) | (((speed + 1) & 3) << pos);
    }
}

/******************************************************************************/
// LED

#define LED0 MICROPY_HW_LED1
#define LED1 MICROPY_HW_LED2
#ifdef MICROPY_HW_LED3
#define LED2 MICROPY_HW_LED3
#endif
#ifdef MICROPY_HW_LED4
#define LED3 MICROPY_HW_LED4
#endif

void led_init(void) {
    mp_hal_pin_output(LED0);
    mp_hal_pin_output(LED1);
    #ifdef LED2
    mp_hal_pin_output(LED2);
    #endif
    #ifdef LED3
    mp_hal_pin_output(LED3);
    #endif
}

void led_state(int led, int val) {
    if (led == 1) {
        led = LED0;
    }
    if (val) {
        MICROPY_HW_LED_ON(led);
    } else {
        MICROPY_HW_LED_OFF(led);
    }
}

void led_state_all(unsigned int mask) {
    led_state(LED0, mask & 1);
    led_state(LED1, mask & 2);
    #ifdef LED2
    led_state(LED2, mask & 4);
    #endif
    #ifdef LED3
    led_state(LED3, mask & 8);
    #endif
}

/******************************************************************************/
// USR BUTTON

static void usrbtn_init(void) {
    mp_hal_pin_config(MICROPY_HW_USRSW_PIN, MP_HAL_PIN_MODE_IN, MICROPY_HW_USRSW_PULL, 0);
}

static int usrbtn_state(void) {
    return mp_hal_pin_read(MICROPY_HW_USRSW_PIN) == MICROPY_HW_USRSW_PRESSED;
}

/******************************************************************************/
// FLASH

#ifndef MBOOT_NORFLASH_LAYOUT
#define MBOOT_NORFLASH_LAYOUT ""
#endif


typedef struct {
    uint32_t base_address;
    uint32_t page_size;
    uint32_t page_count;
} flash_layout_t;

#if defined(STM32F103xG)
#define FLASH_LAYOUT_STR "@Internal Flash  /0x08000000/128*002Kg,128*002Kg,256*002Kg" MBOOT_NORFLASH_LAYOUT
#endif

static const flash_layout_t flash_layout[] = {
    { 0x08000000U, FLASH_PAGE_SIZE, 128 }, /* 103xC total 128 pages = 256K */
    #if defined(STM32F103xE)
    { 0x08040000U, FLASH_PAGE_SIZE, 128 }, /* 103xE has 512K = 2k * 256 */
    #elif defined(STM32F103xG)
    { 0x08040000U, FLASH_PAGE_SIZE, 128 }, /* 103xF has 768K  = 2k * 384 */
    { 0x08080000U, FLASH_PAGE_SIZE, 256 }, /* 103xG has 1024K = 2k * 512 */
    #endif
};

// 通过布局数组找到内部Flash所在页索引
static uint32_t flash_get_page_index(uint32_t addr, uint32_t *page_size) {
    if (addr >= flash_layout[0].base_address) {
        uint32_t page_index = 0;
        for (int i = 0; i < MP_ARRAY_SIZE(flash_layout); ++i) {
            for (int j = 0; j < flash_layout[i].page_count; ++j) {
                uint32_t page_start_next = flash_layout[i].base_address
                    + (j + 1) * flash_layout[i].page_size;
                if (addr < page_start_next) {
                    *page_size = flash_layout[i].page_size;
                    return page_index;
                }
                ++page_index;
            }
        }
    }
    return 0;
}

static int flash_mass_erase(void) {
    // TODO
    return -1;
}

static int flash_page_erase(uint32_t addr, uint32_t *next_addr) {
    uint32_t page_size = 0;
    uint32_t page = flash_get_page_index(addr, &page_size);
    if (page < 64) {
        // 0x08020000 之前的是bootloader区域, 不允许擦除
        return -1;
    }

    *next_addr = addr + page_size;

    // erase the page(s)
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; /* 按页擦除 */
    EraseInitStruct.Banks = FLASH_BANK_BOTH;
    EraseInitStruct.PageAddress = addr; /* 这里需要填写实际要擦除的地址 */
    EraseInitStruct.NbPages = 1;        /* 这里指定要擦除的总页数 */
    uint32_t PageError = 0;

    HAL_FLASH_Unlock();
    // Clear pending flags (if any)
    __HAL_FLASH_CLEAR_FLAG(SR_FLAG_MASK);

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        // error occurred during page erase
        return -1;
    }

    // Check the erase set bits to 1, at least for the first 256 bytes
    for (int i = 0; i < 64; ++i) {
        if (((volatile uint32_t*)addr)[i] != 0xffffffff) {
            return -2;
        }
    }

    return 0;
}

static int flash_write(uint32_t addr, const uint8_t *src8, size_t len) {
    if (addr >= flash_layout[0].base_address && addr < flash_layout[0].base_address + flash_layout[0].page_size) {
        // Don't allow to write the sector with this bootloader in it
        return -1;
    }

    const uint32_t *src = (const uint32_t*)src8;
    size_t num_word32 = (len + 3) / 4;
    HAL_FLASH_Unlock();

    // program the flash word by word
    for (size_t i = 0; i < num_word32; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *src) != HAL_OK) {
            return -1;
        }
        addr += 4;
        src += 1;
    }

    // TODO verify data

    return 0;
}

/******************************************************************************/
// Writable address space interface

static int do_mass_erase(void) {
    // TODO
    return flash_mass_erase();
}

#if defined(MBOOT_NORFLASH_ADDR)
static int norflash_page_erase(uint32_t addr, uint32_t n_blocks) {
    for (int i = 0; i < n_blocks; ++i) {
        int ret = mp_norflash_erase_block(addr);
        if (ret != 0) {
            return ret;
        }
        addr += MBOOT_NORFLASH_BLOCK_SIZE;
    }
    return 0;
}
#endif

int do_page_erase(uint32_t addr, uint32_t *next_addr) {
    led_state(LED0, 1);

    #if defined(MBOOT_NORFLASH_ADDR)
    if (MBOOT_NORFLASH_ADDR <= addr && addr < MBOOT_NORFLASH_ADDR + MBOOT_NORFLASH_BYTE_SIZE) {
        *next_addr = addr + MBOOT_NORFLASH_BLOCK_SIZE;
        return norflash_page_erase(addr, 1);
    }
    #endif

    return flash_page_erase(addr, next_addr);
}

void do_read(uint32_t addr, int len, uint8_t *buf) {
    #if defined(MBOOT_NORFLASH_ADDR)
    if (MBOOT_NORFLASH_ADDR <= addr && addr < MBOOT_NORFLASH_ADDR + MBOOT_NORFLASH_BYTE_SIZE) {
        // TODO: Just need Read directly from memory
        mp_norflash_read(addr, len, buf);
        return;
    }
    #endif

    // Other addresses, just read directly from memory
    memcpy(buf, (void*)addr, len);
}

int do_write(uint32_t addr, const uint8_t *src8, size_t len) {
    static uint32_t led_tog = 0;
    led_state(LED0, (led_tog++) & 4);

    #if defined(MBOOT_NORFLASH_ADDR)
    if (MBOOT_NORFLASH_ADDR <= addr && addr < MBOOT_NORFLASH_ADDR + MBOOT_NORFLASH_BYTE_SIZE) {
        return mp_norflash_write(addr, len, src8);
    }
    #endif

    return flash_write(addr, src8, len);
}

/******************************************************************************/
// DFU

#define DFU_XFER_SIZE (2048)

enum {
    DFU_DNLOAD = 1,
    DFU_UPLOAD = 2,
    DFU_GETSTATUS = 3,
    DFU_CLRSTATUS = 4,
    DFU_ABORT = 6,
};

enum {
    DFU_STATUS_IDLE = 2,
    DFU_STATUS_BUSY = 4,
    DFU_STATUS_DNLOAD_IDLE = 5,
    DFU_STATUS_MANIFEST = 7,
    DFU_STATUS_UPLOAD_IDLE = 9,
    DFU_STATUS_ERROR = 0xa,
};

enum {
    DFU_CMD_NONE = 0,
    DFU_CMD_EXIT = 1,
    DFU_CMD_UPLOAD = 7,
    DFU_CMD_DNLOAD = 8,
};

typedef struct _dfu_state_t {
    int status;
    int cmd;
    uint16_t wBlockNum;
    uint16_t wLength;
    uint32_t addr;
    uint8_t buf[DFU_XFER_SIZE] __attribute__((aligned(4)));
} dfu_state_t;

static dfu_state_t dfu_state SECTION_NOZERO_BSS;

static void dfu_init(void) {
    dfu_state.status = DFU_STATUS_IDLE;
    dfu_state.cmd = DFU_CMD_NONE;
    dfu_state.addr = 0x08000000;
}

static int dfu_process_dnload(void) {
    int ret = -1;
    if (dfu_state.wBlockNum == 0) {
        // download control commands
        // 0x41 - 擦除Flash, 可选4字节 擦除起始地址, 无则全盘擦除
        if (dfu_state.wLength >= 1 && dfu_state.buf[0] == 0x41) {
            if (dfu_state.wLength == 1) {
                // mass erase
                ret = do_mass_erase();
            } else if (dfu_state.wLength == 5) {
                // erase page
                uint32_t next_addr;
                ret = do_page_erase(get_le32(&dfu_state.buf[1]), &next_addr);
            }
        }

        // 0x21 - 设置地址 4字节(大端)
        else if (dfu_state.wLength >= 1 && dfu_state.buf[0] == 0x21) {
            if (dfu_state.wLength == 5) {
                // set address
                dfu_state.addr = get_le32(&dfu_state.buf[1]);
                ret = 0;
            }
        }
    } else if (dfu_state.wBlockNum > 1) {
        // write data to memory
        uint32_t addr = (dfu_state.wBlockNum - 2) * DFU_XFER_SIZE + dfu_state.addr;
        ret = do_write(addr, dfu_state.buf, dfu_state.wLength);
    }
    if (ret == 0) {
        return DFU_STATUS_DNLOAD_IDLE;
    } else {
        return DFU_STATUS_ERROR;
    }
}

static void dfu_handle_rx(int cmd, int arg, int len, const void *buf) {
    if (cmd == DFU_CLRSTATUS) {
        // clear status
        dfu_state.status = DFU_STATUS_IDLE;
        dfu_state.cmd = DFU_CMD_NONE;
    } else if (cmd == DFU_ABORT) {
        // clear status
        dfu_state.status = DFU_STATUS_IDLE;
        dfu_state.cmd = DFU_CMD_NONE;
    } else if (cmd == DFU_DNLOAD) {
        if (len == 0) {
            // exit DFU
            dfu_state.cmd = DFU_CMD_EXIT;
        } else {
            // download
            dfu_state.cmd = DFU_CMD_DNLOAD;
            dfu_state.wBlockNum = arg;
            dfu_state.wLength = len;
            memcpy(dfu_state.buf, buf, len);
        }
    }
}

static void dfu_process(void) {
    if (dfu_state.status == DFU_STATUS_MANIFEST) {
        do_reset();
    }

    if (dfu_state.status == DFU_STATUS_BUSY) {
        if (dfu_state.cmd == DFU_CMD_DNLOAD) {
            dfu_state.cmd = DFU_CMD_NONE;
            dfu_state.status = dfu_process_dnload();
        }
    }
}

static int dfu_handle_tx(int cmd, int arg, int len, uint8_t *buf, int max_len) {
    if (cmd == DFU_UPLOAD) {
        if (arg >= 2) {
            dfu_state.cmd = DFU_CMD_UPLOAD;
            uint32_t addr = (arg - 2) * max_len + dfu_state.addr;
            do_read(addr, len, buf);
            return len;
        }
    } else if (cmd == DFU_GETSTATUS && len == 6) {
        // execute command and get status
        switch (dfu_state.cmd) {
            case DFU_CMD_NONE:
                break;
            case DFU_CMD_EXIT:
                dfu_state.status = DFU_STATUS_MANIFEST;
                break;
            case DFU_CMD_UPLOAD:
                dfu_state.status = DFU_STATUS_UPLOAD_IDLE;
                break;
            case DFU_CMD_DNLOAD:
                dfu_state.status = DFU_STATUS_BUSY;
                break;
        }
        buf[0] = 0;
        buf[1] = dfu_state.cmd; // TODO is this correct?
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = dfu_state.status;
        buf[5] = 0;
        return 6;
    }
    return -1;
}

/******************************************************************************/
// USB
#define USB_XFER_SIZE (DFU_XFER_SIZE)

typedef struct _pyb_usbdd_obj_t {
    bool started;
    bool tx_pending;
    USBD_HandleTypeDef hUSBDDevice;

    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wLength;
    __ALIGN_BEGIN uint8_t rx_buf[USB_XFER_SIZE] __ALIGN_END;
    __ALIGN_BEGIN uint8_t tx_buf[USB_XFER_SIZE] __ALIGN_END;

    // RAM to hold the current descriptors, which we configure on the fly
    __ALIGN_BEGIN uint8_t usbd_device_desc[USB_LEN_DEV_DESC] __ALIGN_END;
    __ALIGN_BEGIN uint8_t usbd_str_desc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
} pyb_usbdd_obj_t;

#ifndef MBOOT_USBD_LANGID_STRING
#define MBOOT_USBD_LANGID_STRING (0x409)
#endif

#ifndef MBOOT_USBD_MANUFACTURER_STRING
#define MBOOT_USBD_MANUFACTURER_STRING      "MicroPython"
#endif

#ifndef MBOOT_USBD_PRODUCT_STRING
#define MBOOT_USBD_PRODUCT_STRING        "Pyboard DFU"
#endif

#ifndef MBOOT_USB_VID
#define MBOOT_USB_VID 0x0483
#endif

#ifndef MBOOT_USB_PID
#define MBOOT_USB_PID 0xDF11
#endif

static const uint8_t usbd_fifo_size[] = {
    32, 8, 16, 8, 16, 0, 0, // FS: RX, EP0(in), 5x IN endpoints
};

__ALIGN_BEGIN static const uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(MBOOT_USBD_LANGID_STRING),
    HIBYTE(MBOOT_USBD_LANGID_STRING),
};

static const uint8_t dev_descr[0x12] = {
    0x12, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x40,
    LOBYTE(MBOOT_USB_VID), HIBYTE(MBOOT_USB_VID),
    LOBYTE(MBOOT_USB_PID), HIBYTE(MBOOT_USB_PID),
    0x00, 0x22, 0x01, 0x02, 0x03, 0x01
};

// This may be modified by USBD_GetDescriptor
static uint8_t cfg_descr[9 + 9 + 9] =
    "\x09\x02\x1b\x00\x01\x01\x00\xc0\x32"
    "\x09\x04\x00\x00\x00\xfe\x01\x02\x04"
    "\x09\x21\x0b\xff\x00\x00\x08\x1a\x01" // \x00\x08 goes with USB_XFER_SIZE
;

static uint8_t *pyb_usbdd_DeviceDescriptor(USBD_HandleTypeDef *pdev, uint16_t *length) {
    *length = USB_LEN_DEV_DESC;
    return (uint8_t*)dev_descr;
}

static char get_hex_char(int val) {
    val &= 0xf;
    if (val <= 9) {
        return '0' + val;
    } else {
        return 'A' + val - 10;
    }
}

static void format_hex(char *buf, int val) {
    buf[0] = get_hex_char(val >> 4);
    buf[1] = get_hex_char(val);
}

static uint8_t *pyb_usbdd_StrDescriptor(USBD_HandleTypeDef *pdev, uint8_t idx, uint16_t *length) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    uint8_t *str_desc = self->usbd_str_desc;
    switch (idx) {
        case USBD_IDX_LANGID_STR:
            *length = sizeof(USBD_LangIDDesc);
            return (uint8_t*)USBD_LangIDDesc; // the data should only be read from this buf

        case USBD_IDX_MFC_STR:
            USBD_GetString((uint8_t*)MBOOT_USBD_MANUFACTURER_STRING, str_desc, length);
            return str_desc;

        case USBD_IDX_PRODUCT_STR:
            USBD_GetString((uint8_t*)MBOOT_USBD_PRODUCT_STRING, str_desc, length);
            return str_desc;

        case USBD_IDX_SERIAL_STR: {
            // This document: http://www.usb.org/developers/docs/devclass_docs/usbmassbulk_10.pdf
            // says that the serial number has to be at least 12 digits long and that
            // the last 12 digits need to be unique. It also stipulates that the valid
            // character set is that of upper-case hexadecimal digits.
            //
            // The onboard DFU bootloader produces a 12-digit serial number based on
            // the 96-bit unique ID, so for consistency we go with this algorithm.
            // You can see the serial number if you do:
            //
            //     dfu-util -l
            //
            // See: https://my.st.com/52d187b7 for the algorithim used.
            uint8_t *id = (uint8_t*)MP_HAL_UNIQUE_ID_ADDRESS;
            char serial_buf[16];
            format_hex(&serial_buf[0], id[11]);
            format_hex(&serial_buf[2], id[10] + id[2]);
            format_hex(&serial_buf[4], id[9]);
            format_hex(&serial_buf[6], id[8] + id[0]);
            format_hex(&serial_buf[8], id[7]);
            format_hex(&serial_buf[10], id[6]);
            serial_buf[12] = '\0';

            USBD_GetString((uint8_t*)serial_buf, str_desc, length);
            return str_desc;
        }

        case USBD_IDX_CONFIG_STR:
            USBD_GetString((uint8_t*)FLASH_LAYOUT_STR, str_desc, length);
            return str_desc;

        default:
            return NULL;
    }
}

static const USBD_DescriptorsTypeDef pyb_usbdd_descriptors = {
    pyb_usbdd_DeviceDescriptor,
    pyb_usbdd_StrDescriptor,
};

static uint8_t pyb_usbdd_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    (void)self;
    return USBD_OK;
}

static uint8_t pyb_usbdd_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    (void)self;
    return USBD_OK;
}

static uint8_t pyb_usbdd_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    (void)self;
    self->bRequest = req->bRequest;
    self->wValue = req->wValue;
    self->wLength = req->wLength;
    if (req->bmRequest == 0x21) {
        // host-to-device request
        if (req->wLength == 0) {
            // no data, process command straightaway
            dfu_handle_rx(self->bRequest, self->wValue, 0, NULL);
        } else {
            // have data, prepare to receive it
            USBD_CtlPrepareRx(pdev, self->rx_buf, req->wLength);
        }
    } else if (req->bmRequest == 0xa1) {
        // device-to-host request
        int len = dfu_handle_tx(self->bRequest, self->wValue, self->wLength, self->tx_buf, USB_XFER_SIZE);
        if (len >= 0) {
            self->tx_pending = true;
            USBD_CtlSendData(&self->hUSBDDevice, self->tx_buf, len);
        }
    }
    return USBD_OK;
}

static uint8_t pyb_usbdd_EP0_TxSent(USBD_HandleTypeDef *pdev) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    self->tx_pending = false;
    #if !USE_USB_POLLING
    // Process now that we have sent a response
    dfu_process();
    #endif
    return USBD_OK;
}

static uint8_t pyb_usbdd_EP0_RxReady(USBD_HandleTypeDef *pdev) {
    pyb_usbdd_obj_t *self = (pyb_usbdd_obj_t*)pdev->pClassData;
    dfu_handle_rx(self->bRequest, self->wValue, self->wLength, self->rx_buf);
    return USBD_OK;
}

static uint8_t *pyb_usbdd_GetCfgDesc(USBD_HandleTypeDef *pdev, uint16_t *length) {
    *length = sizeof(cfg_descr);
    return (uint8_t*)cfg_descr;
}

static const USBD_ClassTypeDef pyb_usbdd_class = {
    pyb_usbdd_Init,
    pyb_usbdd_DeInit,
    pyb_usbdd_Setup,
    pyb_usbdd_EP0_TxSent,
    pyb_usbdd_EP0_RxReady,
    NULL, // pyb_usbdd_DataIn,
    NULL, // pyb_usbdd_DataOut,
    NULL, // SOF
    NULL, // IsoINIncomplete
    NULL, // IsoOUTIncomplete
    pyb_usbdd_GetCfgDesc,
    pyb_usbdd_GetCfgDesc,
    pyb_usbdd_GetCfgDesc,
    NULL,
};

static pyb_usbdd_obj_t pyb_usbdd SECTION_NOZERO_BSS;

static void pyb_usbdd_init(pyb_usbdd_obj_t *self) {
    self->started = false;
    self->tx_pending = false;
    USBD_HandleTypeDef *usbd = &self->hUSBDDevice;
    usbd->id = 0;
    usbd->dev_state = USBD_STATE_DEFAULT;
    usbd->pDesc = (USBD_DescriptorsTypeDef*)&pyb_usbdd_descriptors;
    usbd->pClass = &pyb_usbdd_class;
    usbd->pClassData = self;
}

static void pyb_usbdd_start(pyb_usbdd_obj_t *self) {
    if (!self->started) {
        USBD_LL_Init(&self->hUSBDDevice, 0, usbd_fifo_size);
        USBD_LL_Start(&self->hUSBDDevice);
        self->started = true;
    }
}

static void pyb_usbdd_stop(pyb_usbdd_obj_t *self) {
    if (self->started) {
        USBD_Stop(&self->hUSBDDevice);
        self->started = false;
    }
}

static int pyb_usbdd_shutdown(void) {
    pyb_usbdd_stop(&pyb_usbdd);
    return 0;
}

/******************************************************************************/
// main

#define RESET_MODE_NUM_STATES (4)
#define RESET_MODE_TIMEOUT_CYCLES (8)
#ifdef LED2
#ifdef LED3
#define RESET_MODE_LED_STATES 0x8421
#else
#define RESET_MODE_LED_STATES 0x7421
#endif
#else
#define RESET_MODE_LED_STATES 0x3210
#endif

static int get_reset_mode(void) {
    usrbtn_init();
    int reset_mode = 1;
    if (usrbtn_state()) {
        // Cycle through reset modes while USR is held
        // Timeout is roughly 20s, where reset_mode=1
        systick_init();
        led_init();
        reset_mode = 0;
        for (int i = 0; i < (RESET_MODE_NUM_STATES * RESET_MODE_TIMEOUT_CYCLES + 1) * 32; i++) {
            if (i % 32 == 0) {
                if (++reset_mode > RESET_MODE_NUM_STATES) {
                    reset_mode = 1;
                }
                uint8_t l = RESET_MODE_LED_STATES >> ((reset_mode - 1) * 4);
                led_state_all(l);
            }
            if (!usrbtn_state()) {
                break;
            }
            mp_hal_delay_ms(19);
        }
        // Flash the selected reset mode
        for (int i = 0; i < 6; i++) {
            led_state_all(0);
            mp_hal_delay_ms(50);
            uint8_t l = RESET_MODE_LED_STATES >> ((reset_mode - 1) * 4);
            led_state_all(l);
            mp_hal_delay_ms(50);
        }
        mp_hal_delay_ms(300);
    }
    return reset_mode;
}

static void do_reset(void) {
    led_state_all(0);
    mp_hal_delay_ms(50);
    pyb_usbdd_shutdown();
    mp_hal_delay_ms(50);
    NVIC_SystemReset();
}

uint32_t SystemCoreClock;

extern PCD_HandleTypeDef pcd_fs_handle;

void stm32_main(int initial_r0) {
    #if PREFETCH_ENABLE
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    #endif

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    #if defined(MBOOT_BOARD_EARLY_INIT)
    MBOOT_BOARD_EARLY_INIT();
    #endif

    #if defined(MBOOT_NORFLASH_ADDR)
    SystemClock_Config();
    mp_norflash_init();
    #endif

    #ifdef MBOOT_BOOTPIN_PIN
    mp_hal_pin_config(MBOOT_BOOTPIN_PIN, MP_HAL_PIN_MODE_IN, MBOOT_BOOTPIN_PULL, 0);
    if (mp_hal_pin_read(MBOOT_BOOTPIN_PIN) == MBOOT_BOOTPIN_ACTIVE) {
        goto enter_bootloader;
    }
    #endif

    if ((initial_r0 & 0xffffff00) == 0x70ad0000) {
        goto enter_bootloader;
    }

    #if !defined(MBOOT_NORFLASH_ADDR)
    // MCU starts up with HSI
    SystemCoreClock = HSI_VALUE;
    #endif
    int reset_mode = get_reset_mode();
    uint32_t msp = *(volatile uint32_t*)(APPLICATION_ADDR);
    if (reset_mode != 4 && (msp & APP_VALIDITY_BITS) == 0) {
        // not DFU mode so jump to application, passing through reset_mode
        // undo our DFU settings
        // TODO probably should disable all IRQ sources first
        __set_MSP(msp);
        // SCB->VTOR = APPLICATION_ADDR;
        ((void (*)(uint32_t)) *((volatile uint32_t*)(APPLICATION_ADDR + 4)))(reset_mode);
    }

enter_bootloader:
    // Init subsystems (get_reset_mode() may call these, calling them again is ok)
    led_init();

    // set the system clock to be HSE
    #if !defined(MBOOT_NORFLASH_ADDR)
    SystemClock_Config();
    #endif

    #if USE_USB_POLLING
    // irqs with a priority value greater or equal to "pri" will be disabled
    // "pri" should be between 1 and 15 inclusive
    uint32_t pri = 2;
    pri <<= (8 - __NVIC_PRIO_BITS);
    __ASM volatile ("msr basepri_max, %0" : : "r" (pri) : "memory");
    #endif

    #if MBOOT_FSLOAD
    if ((initial_r0 & 0xffffff80) == 0x70ad0080) {
        // Application passed through elements, validate then process them
        const uint8_t *elem_end = elem_search(ELEM_DATA_START, ELEM_TYPE_END);
        if (elem_end != NULL && elem_end[-1] == 0) {
            fsload_process();
        }
        // Always reset because the application is expecting to resume
        led_state_all(0);
        NVIC_SystemReset();
    }
    #endif

    dfu_init();
	{// USB detect reset
		mp_hal_pin_config(pin_A12, MP_HAL_PIN_MODE_OUT, MP_HAL_PIN_PULL_DOWN, 0);
		mp_hal_pin_low(pin_A12);
		mp_hal_delay_ms(200); // pin's state keep low with 200ms
	}
    pyb_usbdd_init(&pyb_usbdd);
    pyb_usbdd_start(&pyb_usbdd);

    led_state_all(0);

    #if USE_USB_POLLING
    uint32_t ss = systick_ms;
    int ss2 = -1;
    #endif

    for (;;) {
        #if USE_USB_POLLING
        if ((USB->ISTR & USB->CNTR) >> 8) {
            HAL_PCD_IRQHandler(&pcd_fs_handle);
        }
        if (!pyb_usbdd.tx_pending) {
            dfu_process();
        }
        //__WFI(); // slows it down way too much; might work with 10x faster systick
        if (systick_ms - ss > 50) {
            ss += 50;
            ss2 = (ss2 + 1) % 20;
            switch (ss2) {
                case 0: led_state(LED0, 1); break;
                case 1: led_state(LED0, 0); break;
            }
        }
        #else
        led_state(LED0, 1);
        mp_hal_delay_ms(50);
        led_state(LED0, 0);
        mp_hal_delay_ms(950);
        #endif
    }
}

void NMI_Handler(void) {
}

void MemManage_Handler(void) {
    while (1) {
        __fatal_error("MemManage");
    }
}

void BusFault_Handler(void) {
    while (1) {
        __fatal_error("BusFault");
    }
}

void UsageFault_Handler(void) {
    while (1) {
        __fatal_error("UsageFault");
    }
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    systick_ms += 1;

    // Read the systick control regster. This has the side effect of clearing
    // the COUNTFLAG bit, which makes the logic in mp_hal_ticks_us
    // work properly.
    SysTick->CTRL;
}

#if !USE_USB_POLLING
void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_PCD_IRQHandler(&pcd_fs_handle);
}
#endif
