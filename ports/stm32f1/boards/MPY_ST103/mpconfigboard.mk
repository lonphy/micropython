MCU_SERIES = f1
CMSIS_MCU = STM32F103xG
MICROPY_FLOAT_IMPL = signle
AF_FILE = boards/stm32f103_af.csv
OPENOCD_CONFIG = boards/openocd_stm32f1_xg.cfg

ifeq ($(USE_MBOOT),1)
LD_FILES = boards/MPY_ST103/mpy_st103.ld boards/common_blifs.ld
TEXT0_ADDR = 0x64000000
TEXT0_ADDR = 0x08020000
else
LD_FILES = boards/MPY_ST103/mpy_st103.ld boards/common_basic.ld
endif
