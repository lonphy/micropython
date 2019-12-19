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
#include <stdbool.h>
#include <string.h>
#include "py/runtime.h"
#include "py/mphal.h"
#include "sram.h"

#ifdef MICROPY_HW_SRAM_BANK

#define MEM_1K         (1024)
#define MEM_SIZE_256K  ((256U << 0U) * MEM_1K)
#define MEM_SIZE_512K  ((256U << 1U) * MEM_1K)
#define MEM_SIZE_1M    ((256U << 2U) * MEM_1K)
#define MEM_SIZE_2M    ((256U << 3U) * MEM_1K)
#define MEM_SIZE_4M    ((256U << 4U) * MEM_1K)
#define MEM_SIZE_8M    ((256U << 5U) * MEM_1K)
#define MEM_SIZE_16M   ((256U << 6U) * MEM_1K)
#define MEM_SIZE_32M   ((256U << 7U) * MEM_1K)
#define MEM_SIZE_64M   ((256U << 8U) * MEM_1K)


#define SRAM_FSMC_BCR  (0x00001091U) /* control register, [ WREN=1, MWID=16bit, MBKEN=1]*/
#define SRAM_FSMC_BTR  (((MICROPY_HW_SRAM_TIMING_DATAST & 0xffU) << 8U) | \
                        ((MICROPY_HW_SRAM_TIMING_ADDHLD & 0x0fU) << 4U) | \
                        ((MICROPY_HW_SRAM_TIMING_ADDSET & 0x0fU) << 0U) ) /* timing register, [DATAST, ADDHLD, ADDSET] */

#define PCFG_OFFSET(n)  (((n)%8U) * 4U )
#define PIN_ALT_CFG(n)  (0x0BU << PCFG_OFFSET(n))  /* Alternate Output with push-pull, max speed */
#define PIN_MASK(n)     (0x0FU << PCFG_OFFSET(n))

bool sram_init(void) {
    RCC->AHBENR = 0x114;

    // FSMC_NE1  -- PD7  FSMC_NE2 -- PG9   FSMC_NE3 -- PG10  FSMC_NE4 -- PG12
    // FSMC_NOE  -- PD4  FSMC_NWE  -- PD5
    // FSMC_NBL1 -- PE1  FSMC_NBL0 -- PE0

    // FSMC_D0  -- PD15  FSMC_D1  -- PD14  FSMC_D2  -- PD0   FSMC_D3  -- PD1
    // FSMC_D4  -- PE7   FSMC_D5  -- PE8   FSMC_D6  -- PE9   FSMC_D7  -- PE10

    // FSMC_D8  -- PE11  FSMC_D9  -- PE12   FSMC_D10 -- PE13  FSMC_D11 -- PE14
    // FSMC_D12 -- PE15  FSMC_D13 -- PD8    FSMC_D14 -- PD9   FSMC_D15 -- PD10

    // FSMC_A0  -- PF0   FSMC_A1  -- PF1    FSMC_A2  -- PF2   FSMC_A3  -- PF3
    // FSMC_A4  -- PF4   FSMC_A5  -- PF5    FSMC_A6  -- PF12  FSMC_A7  -- PF13  
    
    // FSMC_A8  -- PF14  FSMC_A9  -- PF15   FSMC_A10 -- PG0   FSMC_A11 -- PG1
    // FSMC_A12 -- PG2   FSMC_A13 -- PG3    FSMC_A14 -- PG4   FSMC_A15 -- PG5

    // FSMC_A16 -- PD11

    GPIOD->CRL = 0x44BB44BBU; /* fixed: PD0, PD1, PD4, PD5 */
    GPIOD->CRH = 0xBB00BBBBU; /* PD8 ~ PD11, PD14, PD15 */

    GPIOE->CRL = 0xB44444BBU; /* PE0, PE1, PE7 */
    GPIOE->CRH = 0xBBBBBBBBU; /* PE8 ~ PE15 */

    GPIOF->CRL = 0x44BBBBBBU; /* PF0 ~ PF5 */
    GPIOF->CRH = 0xBBBB4444U; /* PF12 ~ PF15 */

    GPIOG->CRL = 0x44BBBBBBU; /* PG0 ~ PG5 */
    

    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_512K) /* A17 -- PD12 */
    MODIFY_REG(GPIOD->CRH, PIN_MASK(12), PIN_ALT_CFG(12));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_1M)   /* A18 -- PD13 */
    MODIFY_REG(GPIOD->CRH, PIN_MASK(13), PIN_ALT_CFG(13));
    #endif

    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_2M)   /* A19 -- PE3 */
    MODIFY_REG(GPIOE->CRL, PIN_MASK(3), PIN_ALT_CFG(3));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_4M)   /* A20 -- PE4 */
    MODIFY_REG(GPIOE->CRL, PIN_MASK(4), PIN_ALT_CFG(4));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_8M)   /* A21 -- PE5 */
    MODIFY_REG(GPIOE->CRL, PIN_MASK(5), PIN_ALT_CFG(5));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_16M)  /* A22 -- PE6 */
    MODIFY_REG(GPIOE->CRL, PIN_MASK(6), PIN_ALT_CFG(6));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_32M)  /* A23 -- PE2 */
    MODIFY_REG(GPIOE->CRL, PIN_MASK(2), PIN_ALT_CFG(2));
    #endif
    
    #if (MICROPY_HW_SRAM_SIZE >= MEM_SIZE_64M)  /* A24 -- PG13 */
    MODIFY_REG(GPIOG->CRH, PIN_MASK(13), PIN_ALT_CFG(13));
    #endif

    /* FSMC Register Config to SRAM */
    #if (MICROPY_HW_SRAM_BANK == FSMC_BANK1_1) /* NE1 -- PD7 */
        MODIFY_REG(GPIOD->CRL, PIN_MASK(7), PIN_ALT_CFG(7));
        FSMC_Bank1->BTCR[1U] = SRAM_FSMC_BTR;
        FSMC_Bank1->BTCR[0U] = SRAM_FSMC_BCR;
    #elif (MICROPY_HW_SRAM_BANK == FSMC_BANK1_2) /* NE2 -- PG9 */
        MODIFY_REG(GPIOG->CRH, PIN_MASK(9), PIN_ALT_CFG(9));
        FSMC_Bank1->BTCR[3U] = SRAM_FSMC_BTR;
        FSMC_Bank1->BTCR[2U] = SRAM_FSMC_BCR;
    #elif (MICROPY_HW_SRAM_BANK == FSMC_BANK1_3) /* NE3 -- PG10 */
        MODIFY_REG(GPIOG->CRH, PIN_MASK(10), PIN_ALT_CFG(10));
        FSMC_Bank1->BTCR[5U] = SRAM_FSMC_BTR;
        FSMC_Bank1->BTCR[4U] = SRAM_FSMC_BCR;
    #elif (MICROPY_HW_SRAM_BANK == FSMC_BANK1_4) /* NE4 -- PG12 */
        MODIFY_REG(GPIOG->CRH, PIN_MASK(12), PIN_ALT_CFG(12));
        FSMC_Bank1->BTCR[7U] = SRAM_FSMC_BTR;
        FSMC_Bank1->BTCR[6U] = SRAM_FSMC_BCR;
    #endif

    return true;
}

void *sram_start(void) {
    return (void*)MICROPY_HW_SRAM_BANK;
}

void *sram_end(void) {
    return (void*)(MICROPY_HW_SRAM_BANK + MICROPY_HW_SRAM_SIZE);
}


bool __attribute__((optimize("O0"))) sram_test(bool fast) {
    uint8_t const pattern = 0xaa;
    uint8_t const antipattern = 0x55;
    uint8_t *const mem_base = (uint8_t*)sram_start();
    uint16_t *const mem16_base = (uint16_t*)sram_start();

    /* test data bus */
    for (uint16_t i = 1; i; i <<= 1) {
        *mem16_base = i;
        if (*mem16_base != i) {
            printf("data bus lines test failed! data (%d)\n", i);
            __asm__ volatile ("BKPT");
        }
    }

    /* test address bus */
    /* Check individual address lines */
    for (uint32_t i = 1; i < MICROPY_HW_SRAM_SIZE; i <<= 1) {
        mem_base[i] = pattern;
        if (mem_base[i] != pattern) {
            printf("address bus lines test failed! address (%p)\n", &mem_base[i]);
            __asm__ volatile ("BKPT");
        }
    }

    /* Check for aliasing (overlaping addresses) */
    mem_base[0] = antipattern;
    for (uint32_t i = 1; i < MICROPY_HW_SRAM_SIZE; i <<= 1) {
        if (mem_base[i] != pattern) {
            printf("address bus overlap %p\n", &mem_base[i]);
            __asm__ volatile ("BKPT");
        }
    }

    /* test all ram cells */
    if (!fast) {
        for (uint32_t i = 0; i < MICROPY_HW_SRAM_SIZE; ++i) {
            mem_base[i] = pattern;
            if (mem_base[i] != pattern) {
                printf("address bus test failed! address (%p)\n", &mem_base[i]);
                __asm__ volatile ("BKPT");
            }
        }
    } else {
        memset(mem_base, pattern, MICROPY_HW_SRAM_SIZE);
    }

    return true;
}
#endif
