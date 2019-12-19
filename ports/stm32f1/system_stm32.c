/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Taken from ST Cube library and modified.  See below for original header.
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

/**
  ******************************************************************************
  * @file    system_stm32.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "py/mphal.h"
#include "powerctrl.h"

void __fatal_error(const char *msg);

/**
  * @brief  System Clock Configuration
  *
  * Output clocks:
  *
  * CPU             SYSCLK      max 72MHz
  * USB,SDIO        PLL48CK     must be 48MHz for USB
  * AHB             HCLK        max 72MHz
  * APB1            PCLK1       max 36MHz
  * APB2            PCLK2       max 72MHz
  *
  * Timers run from APBx if APBx_PRESC=1, else 2x APBx
  */
void SystemClock_Config(void) {
    /* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	RCC_ClkInitTypeDef RCC_ClkInitStruct = {
		.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2),
		.AHBCLKDivider = RCC_SYSCLK_DIV1,
		.APB1CLKDivider = RCC_HCLK_DIV2,
		.APB2CLKDivider = RCC_HCLK_DIV1,
	};

	RCC_OscInitTypeDef RCC_OscInitStruct = {
		.OscillatorType = MICROPY_HW_RCC_OSCILLATOR_TYPE,
		.HSEState = MICROPY_HW_RCC_HSE_STATE,
		.HSEPredivValue = MICROPY_HW_HSE_PREDIV,
		.HSIState = MICROPY_HW_RCC_HSI_STATE,
		.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT,
		.PLL.PLLSource = MICROPY_HW_RCC_PLL_SRC,
		.PLL.PLLState = RCC_PLL_ON,
		.PLL.PLLMUL = MICROPY_HW_CLK_PLLMUL,
	};

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		__fatal_error("HAL_RCC_OscConfig");
	}

	if (powerctrl_rcc_clock_config_pll(&RCC_ClkInitStruct) != 0) {
		__fatal_error("HAL_RCC_ClockConfig");
	}
}

#ifdef MBOOT_NORFLASH_ADDR
/* control register, [ASYNCWAIT(b1), WREN(b1), FACCEN(b1) MWID(b01), MTYP(b10), MBKEN(b1)] */
#define NOR_FSMC_BCR (0x000090D9U)

/* timing register, [DATAST=7, ADDHLD=0, ADDSET=5] */
#define NOR_FSMC_BTR (0x10000705U)

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
#endif