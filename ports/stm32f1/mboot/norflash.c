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

/* copied from stm32f1xx_fsmc_nor.h/c */

#include "py/mphal.h"
#include "norflash.h"

#ifdef MBOOT_NORFLASH_ADDR
/* control register, [ASYNCWAIT(b1), WREN(b1), FACCEN(b1) MWID(b01), MTYP(b10), MBKEN(b1)] */
#define NOR_FSMC_BCR (0x000090D9U)

/* timing register, [DATAST=7, ADDHLD=0, ADDSET=5] */
#define NOR_FSMC_BTR (0x10000705U)

/* NOR operations Timeout definitions */
#define NOR_BLOCKERASE_TIMEOUT ((uint32_t)0x00A00000) /* NOR block erase timeout */
#define NOR_CHIPERASE_TIMEOUT  ((uint32_t)0x30000000) /* NOR chip erase timeout  */
#define NOR_PROGRAM_TIMEOUT    ((uint32_t)0x00004400) /* NOR program timeout     */

/* Constants to define address to set to write a command */
#define NOR_CMD_ADDRESS_FIRST     (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FIRST_CFI (uint16_t)0x0055
#define NOR_CMD_ADDRESS_SECOND    (uint16_t)0x02AA
#define NOR_CMD_ADDRESS_THIRD     (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FOURTH    (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FIFTH     (uint16_t)0x02AA
#define NOR_CMD_ADDRESS_SIXTH     (uint16_t)0x0555

/* Constants to define data to program a command */
#define NOR_CMD_DATA_READ_RESET              (uint16_t)0x00F0
#define NOR_CMD_DATA_FIRST                   (uint16_t)0x00AA
#define NOR_CMD_DATA_SECOND                  (uint16_t)0x0055
#define NOR_CMD_DATA_AUTO_SELECT             (uint16_t)0x0090
#define NOR_CMD_DATA_PROGRAM                 (uint16_t)0x00A0
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_THIRD  (uint16_t)0x0080
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_FOURTH (uint16_t)0x00AA
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_FIFTH  (uint16_t)0x0055
#define NOR_CMD_DATA_CHIP_ERASE              (uint16_t)0x0010
#define NOR_CMD_DATA_CFI                     (uint16_t)0x0098

#define NOR_CMD_DATA_BUFFER_AND_PROG         (uint8_t)0x25
#define NOR_CMD_DATA_BUFFER_AND_PROG_CONFIRM (uint8_t)0x29
#define NOR_CMD_DATA_BLOCK_ERASE             (uint8_t)0x30

/* Mask on NOR STATUS REGISTER */
#define NOR_MASK_STATUS_DQ5 (uint16_t)0x0020
#define NOR_MASK_STATUS_DQ6 (uint16_t)0x0040

#define NOR_ADDR_SHIFT(__ADDRESS__)       ((uint32_t)((MBOOT_NORFLASH_ADDR) + (2U * (__ADDRESS__))))
#define NOR_WRITE(__ADDRESS__, __DATA__)  (*(__IO uint16_t *)((uint32_t)(__ADDRESS__)) = (__DATA__))
#define NOR_READ(__ADDRESS__)             (*(__IO uint16_t *)(__ADDRESS__))

// init NorFlash on FSMC
void mp_norflash_init(void) {
    __IO uint32_t tmpreg;

    /* Enable FSMC clock */
    RCC->AHBENR |= RCC_AHBENR_FSMCEN;

    /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);

    /* Enable GPIOD, GPIOE, GPIOF and GPIOG clocks */
    RCC->APB2ENR |= 0x000001E0U;

    /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);
    (void)(tmpreg);

    /**
     * FMSC 地址全开(26bit), NE1~NE4 全开 根据Flash地址配置成NorFlash
     * 
     * GPIOD -> PD0, PD1, PD4, PD5~PD7, PD8~PD15
     * GPIOE -> PE2~PE15
     * GPIOF -> PF0~PF5, PF12~PF15
     * GPIOG -> PG0~PG5, PG9, PG10, PG12~PG14
     */

    GPIOD->CRL = 0xB4BB44BBU; /* PD0, PD1, PD4, PD5~PD7, PD6 is input mode */
    GPIOD->CRH = 0xBBBBBBBBU; /* PD8 ~ PD15 */

    GPIOE->CRL = 0xBBBBBB44U; /* PE2 ~ PE7 */
    GPIOE->CRH = 0xBBBBBBBBU; /* PE8 ~ PE15 */

    GPIOF->CRL = 0x44BBBBBBU; /* PF0 ~ PF5 */
    GPIOF->CRH = 0xBBBB4444U; /* PF12 ~ PF15 */

    GPIOG->CRL = 0x44BBBBBBU; /* PG0 ~ PG5 */
    GPIOG->CRH = 0x4BBB4BB4U; /* PG9, PG10, PG12~PG14 */

/* FSMC Register Config to Nor Flash */
#if (MBOOT_NORFLASH_ADDR == FSMC_BANK1_1)
    FSMC_Bank1->BTCR[0U] = NOR_FSMC_BCR;
    FSMC_Bank1->BTCR[1U] = NOR_FSMC_BTR;
#elif (MBOOT_NORFLASH_ADDR == FSMC_BANK1_2)
    FSMC_Bank1->BTCR[2U] = NOR_FSMC_BCR;
    FSMC_Bank1->BTCR[3U] = NOR_FSMC_BTR;
#elif (MBOOT_NORFLASH_ADDR == FSMC_BANK1_3)
    FSMC_Bank1->BTCR[4U] = NOR_FSMC_BCR;
    FSMC_Bank1->BTCR[5U] = NOR_FSMC_BTR;
#elif (MBOOT_NORFLASH_ADDR == FSMC_BANK1_4)
    FSMC_Bank1->BTCR[6U] = NOR_FSMC_BCR;
    FSMC_Bank1->BTCR[7U] = NOR_FSMC_BTR;
#endif
}

// NorFlash 块擦除(扇区)
// 扇区从0开始, 每次擦除64K(16bit数据宽度)/128K(8bit数据宽度)
int mp_norflash_erase_block(uint32_t addr) {
    /* Send block erase command sequence */
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIRST), NOR_CMD_DATA_FIRST);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_SECOND), NOR_CMD_DATA_SECOND);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_THIRD), NOR_CMD_DATA_CHIP_BLOCK_ERASE_THIRD);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FOURTH), NOR_CMD_DATA_CHIP_BLOCK_ERASE_FOURTH);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIFTH), NOR_CMD_DATA_CHIP_BLOCK_ERASE_FIFTH);
    NOR_WRITE(addr, NOR_CMD_DATA_BLOCK_ERASE);

    /* Return the NOR memory status */
    if (mp_norflash_get_status(MBOOT_NORFLASH_ADDR, NOR_BLOCKERASE_TIMEOUT) != 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief Read data from NorFlash
 * @param addr: absoluet address to read start
 * @param len: size of data to read
 * @param dest: pointer to data to be read
 */
void mp_norflash_read(uint32_t addr, size_t len, uint8_t *dest) {
    uint32_t buf_size = (len + 1U) >> 1;
    uint16_t *data = (uint16_t *)dest;

    /* Send read data command */
    // NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIRST), NOR_CMD_DATA_FIRST);
    // NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_SECOND), NOR_CMD_DATA_SECOND);
    // NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_THIRD), NOR_CMD_DATA_READ_RESET);

    /* Read buffer */
    while (buf_size > 0U) {
        *data = NOR_READ(addr);
        ++data;
        addr += 2U;
        --buf_size;
    }
}

/**
 * @brief Write data to NorFlash
 * @param addr: absoluet address to write start
 * @param len:  size of data to write
 * @param src:  pointer to data to be write
 * @retval NOR memory status
 */
int mp_norflash_write(uint32_t addr, size_t len, const uint8_t *src) {
    uint32_t index = (len+1U) >> 1;
    uint16_t *pData = (uint16_t *)(src);
    
    while (index > 0) {
        /* Write data to NOR */
        /* Send program data command */
        NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIRST), NOR_CMD_DATA_FIRST);
        NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_SECOND), NOR_CMD_DATA_SECOND);
        NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_THIRD), NOR_CMD_DATA_PROGRAM);

        /* Write the data */
        /* 这里注意地址addr 是8bit空间的地址 */
        NOR_WRITE(addr, *pData);

        /* Read NOR device status */
        if (mp_norflash_get_status(MBOOT_NORFLASH_ADDR, NOR_PROGRAM_TIMEOUT) != 0) {
            return -1;
        }

        /* Update the counters */
        index--;
        addr += 2U;
        pData++;
    }

    return 0;
}

/**
 *  @brief Erases the entire NOR chip.
 *  @retval NOR memory status
 */
uint8_t mp_norflash_erase_chip(void) {
    /* Send NOR chip erase command sequence */
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIRST), NOR_CMD_DATA_FIRST);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_SECOND), NOR_CMD_DATA_SECOND);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_THIRD), NOR_CMD_DATA_CHIP_BLOCK_ERASE_THIRD);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FOURTH), NOR_CMD_DATA_CHIP_BLOCK_ERASE_FOURTH);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_FIFTH), NOR_CMD_DATA_CHIP_BLOCK_ERASE_FIFTH);
    NOR_WRITE(NOR_ADDR_SHIFT(NOR_CMD_ADDRESS_SIXTH), NOR_CMD_DATA_CHIP_ERASE);

    /* Return the NOR memory status */
    if (mp_norflash_get_status(MBOOT_NORFLASH_ADDR, NOR_CHIPERASE_TIMEOUT) != 0) {
        return -1;
    } else {
        return 0;
    }
}

/**
  * @brief  NOR Wait for Ready/Busy signal.
  * @param  timeout: Timeout duration
  */
void mp_norflash_mspwait(uint32_t timeout) {
    uint32_t t = timeout;

    /* Polling on Ready/Busy signal */
    while ((mp_hal_pin_read(pin_D6) != 0) && (t > 0U)) {
        t--;
    }

    t = timeout;

    /* Polling on Ready/Busy signal */
    while ((mp_hal_pin_read(pin_D6) != 0U) && (t > 0U)) {
        t--;
    }
}

uint8_t mp_norflash_get_status(uint32_t addr, uint32_t timeout) {
    uint8_t status = 1;
    uint16_t tmpSR1, tmpSR2;
    uint32_t tickstart;

    /* Poll on NOR memory Ready/Busy signal ------------------------------------*/
    mp_norflash_mspwait(timeout);

    /* Get the NOR memory operation status -------------------------------------*/

    /* Get tick */
    tickstart = HAL_GetTick();
    while ((status != 0) && (status != 3)) {
        /* Check for the Timeout */
        if (timeout != HAL_MAX_DELAY) {
            if (((HAL_GetTick() - tickstart) > timeout) || (timeout == 0U)) {
                status = 3;
            }
        }

        /* Read NOR status register (DQ6 and DQ5) */
        tmpSR1 = NOR_READ(addr);
        tmpSR2 = NOR_READ(addr);

        /* If DQ6 did not toggle between the two reads then return success  */
        if ((tmpSR1 & NOR_MASK_STATUS_DQ6) == (tmpSR2 & NOR_MASK_STATUS_DQ6)) {
            return 0;
        }

        if ((tmpSR1 & NOR_MASK_STATUS_DQ5) == NOR_MASK_STATUS_DQ5) {
            status = 1;
        }

        tmpSR1 = NOR_READ(addr);
        tmpSR2 = NOR_READ(addr);

        /* If DQ6 did not toggle between the two reads then return success  */
        if ((tmpSR1 & NOR_MASK_STATUS_DQ6) == (tmpSR2 & NOR_MASK_STATUS_DQ6)) {
            return 0;
        }
        if ((tmpSR1 & NOR_MASK_STATUS_DQ5) == NOR_MASK_STATUS_DQ5) {
            return 2;
        }
    }

    /* Return the operation status */
    return status;
}

#endif