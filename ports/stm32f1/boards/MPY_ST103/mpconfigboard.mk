MCU_SERIES = f1
CMSIS_MCU = STM32F103xG
MICROPY_FLOAT_IMPL = signle
AF_FILE = boards/stm32f103_af.csv
LD_FILES = boards/MPY_ST103/mpy_st103.ld boards/common_blifs.ld
TEXT0_ADDR = 0x64000000

# Don't include default frozen modules because MCU is tight on flash space
FROZEN_MPY_DIR ?=
