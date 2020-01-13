#define MPY_ST103

#define MICROPY_HW_BOARD_NAME       "MPY ST103"
#define MICROPY_HW_MCU_NAME         "STM32F103ZG"

#define MICROPY_PY_THREAD           (1)
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (0)
#define MICROPY_HW_HAS_FLASH        (0)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_USB_FS           (1)
#define MICROPY_HW_ENABLE_ADC       (1)
#define MICROPY_HW_ENABLE_DAC       (1)

#define MICROPY_HW_ENABLE_SRAM      (1)
#define MICROPY_HW_ENABLE_SPIFLASH  (0)
#define MICROPY_HW_ENABLE_NORFLASH  (0)
// #define MICROPY_HW_DEBUG            (3)

#define MICROPY_HW_ENABLE_SERVO      (1)
#define MICROPY_HW_HAS_MMA7660       (0)
#define MICROPY_HW_HAS_LCD           (1)

// HSE is 8MHz
#define MICROPY_HW_CLK_USE_HSE      (1)
#define MICROPY_HW_SYSCLK_SRC       (RCC_SYSCLKSOURCE_PLLCLK)
#define MICROPY_HW_HSE_PREDIV       (RCC_HSE_PREDIV_DIV1)
#define MICROPY_HW_CLK_PLLMUL       (RCC_PLL_MUL9)

// The board has a 32.768kHz crystal for the RTC
#define MICROPY_HW_RTC_USE_LSE      (1)
#define MICROPY_HW_RTC_USE_US       (1)

extern void board_early_init(void);
#define MICROPY_BOARD_EARLY_INIT board_early_init

// -------------------- soft spi flash W25Q128 --------------------
#if MICROPY_HW_ENABLE_SPIFLASH
#define MPICRPY_HW_SPIFLASH_SIZE_BITS (128 * 1024 * 1024)
#define MICROPY_HW_SPIFLASH_CS        (pin_G15)
#define MICROPY_HW_SPIFLASH_SCK       (pin_B3)
#define MICROPY_HW_SPIFLASH_MISO      (pin_B4)
#define MICROPY_HW_SPIFLASH_MOSI      (pin_B5)

extern struct _spi_bdev_t spi_bdev;
extern const struct _mp_spiflash_config_t spiflash_config;
#define MICROPY_HW_BDEV_IOCTL(op, arg) ( \
    (op) == BDEV_IOCTL_NUM_BLOCKS ? (MPICRPY_HW_SPIFLASH_SIZE_BITS / 8 / FLASH_BLOCK_SIZE) : \
    (op) == BDEV_IOCTL_INIT ? spi_bdev_ioctl(&spi_bdev, (op), (uint32_t)&spiflash_config)  : \
                              spi_bdev_ioctl(&spi_bdev, (op), (arg))                         \
)
#define MICROPY_HW_BDEV_READBLOCKS(dest, bl, n) spi_bdev_readblocks(&spi_bdev, (dest), (bl), (n))
#define MICROPY_HW_BDEV_WRITEBLOCKS(src, bl, n) spi_bdev_writeblocks(&spi_bdev, (src), (bl), (n))
#endif // MICROPY_HW_ENABLE_SPIFLASH

// --------------------------------------------------------------
// USART1 for firmware download with ch340
#define MICROPY_HW_UART1_TX     (pin_A9)
#define MICROPY_HW_UART1_RX     (pin_A10)

#define MICROPY_HW_UART_REPL        MACHINE_UART_1
#define MICROPY_HW_UART_REPL_BAUD   115200

// USART2
#define MICROPY_HW_UART2_TX     (pin_A2)
#define MICROPY_HW_UART2_RX     (pin_A3)

// USART3 for esp8266
#define MICROPY_HW_UART3_TX     (pin_B10)
#define MICROPY_HW_UART3_RX     (pin_B11)

// I2C busses, need remap, in ./init.c
#define MICROPY_HW_I2C1_SCL (pin_B8)
#define MICROPY_HW_I2C1_SDA (pin_B9)

// SPI1 for NorFlash

// SPI2 for Screen Touch
#define MICROPY_HW_SPI2_NSS  (pin_B12)
#define MICROPY_HW_SPI2_SCK  (pin_B13)
#define MICROPY_HW_SPI2_MISO (pin_B14)
#define MICROPY_HW_SPI2_MOSI (pin_B15)

// 1 x buzzer link to PC7

// 1 x SDIO bus
// PC8~PC12, PD2 used by D0~D3, CK, CMD
#define MICROPY_HW_ENABLE_SDCARD            (1)
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_D3)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (0)

// 2 x DAC channel
// DAC_OUT1 PA4
// DAC_OUT2 PA5

// KEY0 has pullup, and pressing the switch makes the input go low
#define MICROPY_HW_USRSW_PIN        (pin_A0)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)

// LEDs, with PWM support 
#define MICROPY_HW_LED1             (pin_B1)  // Red     LED D1
#define MICROPY_HW_LED2             (pin_B0)  // Green   LED D2
#define MICROPY_HW_LED3             (pin_A7)  // Blue    LED D3
#define MICROPY_HW_LED4             (pin_A6)  // Yellow  LED D4

#define MICROPY_HW_LED_INVERTED     (1)
#define MICROPY_HW_LED1_PWM         { TIM3, 3, TIM_CHANNEL_4, 0 } /* LED1 with TIM3_CH4 PWM */
#define MICROPY_HW_LED2_PWM         { TIM3, 3, TIM_CHANNEL_3, 0 } /* LED2 with TIM3_CH3 PWM */
#define MICROPY_HW_LED3_PWM         { TIM3, 3, TIM_CHANNEL_2, 0 } /* LED3 with TIM3_CH2 PWM */
#define MICROPY_HW_LED4_PWM         { TIM3, 3, TIM_CHANNEL_1, 0 } /* LED4 with TIM3_CH1 PWM */

#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))

// SRAM
#if MICROPY_HW_ENABLE_SRAM
#define MICROPY_HW_SRAM_SIZE                (8 / 8 * 1024 * 1024)  // 8 Mbit
#define MICROPY_HW_SRAM_STARTUP_TEST        (1)
#define MICROPY_HEAP_START                  sram_start()
#define MICROPY_HEAP_END                    sram_end()

#define MICROPY_HW_SRAM_BANK                (FSMC_BANK1_3)
#define MICROPY_HW_SRAM_TIMING_ADDSET       (0)
#define MICROPY_HW_SRAM_TIMING_ADDHLD       (0)
#define MICROPY_HW_SRAM_TIMING_DATAST       (2)
#endif

// LCD
#if MICROPY_HW_HAS_LCD
#define MICROPY_HW_LCD_EN                   (pin_C6)
#define MICROPY_HW_LCD_BANK                 (FSMC_BANK1_4)
#define MICROPY_HW_LCD_DATA_BITS            (16)
#define MICROPY_HW_LCD_FMSC_ADDRn           (23)     // index is 23
#define MICROPY_HW_LCD_FMSC_ADDR_PIN        (pin_E6) // FMSC_A22
#endif

/******************************************************************************/
#if MICROPY_HW_ENABLE_NORFLASH
#define MBOOT_USB_AUTODETECT_PORT               (1)
// NorFlash (S29GL128P10) use 16bit data, all S29GLxxx has the same sector 64K
// 128Mbit = (128/8)MByte = 16MByte
#define MBOOT_NORFLASH_ADDR                     (0x64000000)
#define MBOOT_NORFLASH_BYTE_SIZE                (16 * 1024 * 1024)
#define MBOOT_NORFLASH_BLOCK_SIZE               (128 * 1024)
#define MBOOT_NORFLASH_LAYOUT                   "/0x64000000/128*128Kg"

#define MBOOT_BOOTPIN_PIN                       pin_A0
#define MBOOT_BOOTPIN_ACTIVE                    0
#define MBOOT_BOOTPIN_PULL                      GPIO_PULLUP
#endif // MICROPY_HW_ENABLE_NORFLASH
