# Select the board to build for: if not given on the command line,
# then default to MPY_ST103.
BOARD ?= MPY_ST103

# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

BOARD_DIR ?= boards/$(BOARD)
ifeq ($(wildcard $(BOARD_DIR)/.),)
$(error Invalid BOARD specified: $(BOARD_DIR))
endif

include ../../py/mkenv.mk
-include mpconfigport.mk
include $(BOARD_DIR)/mpconfigboard.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h $(BUILD)/pins_qstr.h $(BUILD)/modstm_qstr.h
QSTR_GLOBAL_DEPENDENCIES = mpconfigboard_common.h $(BOARD_DIR)/mpconfigboard.h

# File containing description of content to be frozen into firmware.
FROZEN_MANIFEST ?= boards/manifest.py

# include py core make definitions
include $(TOP)/py/py.mk

GIT_SUBMODULES = lib/lwip lib/mbedtls lib/mynewt-nimble lib/stm32lib
CMSIS_MCU_LOWER = $(shell echo $(CMSIS_MCU) | tr '[:upper:]' '[:lower:]')

LD_DIR=boards
CMSIS_DIR=$(TOP)/lib/stm32lib/CMSIS/STM32F1xx/Include
HAL_DIR=lib/stm32lib/STM32F1xx_HAL_Driver
USBDEV_DIR=usbdev
DFU=$(TOP)/tools/dfu.py
# may need to prefix dfu-util with sudo
USE_PYDFU ?= 1
PYDFU ?= $(TOP)/tools/pydfu.py
DFU_UTIL ?= dfu-util
DEVICE=0483:df11
STFLASH ?= st-flash
OPENOCD ?= openocd
OPENOCD_CONFIG ?= boards/openocd_stm32f1.cfg
STARTUP_FILE ?= lib/stm32lib/CMSIS/STM32F1xx/Source/Templates/gcc/startup_$(CMSIS_MCU_LOWER).o
SYSTEM_FILE ?= lib/stm32lib/CMSIS/STM32F1xx/Source/Templates/system_stm32f1xx.o

# Select the cross compile prefix
CROSS_COMPILE ?= arm-none-eabi-

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(TOP)/lib/cmsis/inc
INC += -I$(CMSIS_DIR)/
INC += -I$(TOP)/$(HAL_DIR)/Inc
INC += -I$(USBDEV_DIR)/core/inc -I$(USBDEV_DIR)/class/inc
INC += -Ilwip_inc

# Basic Cortex-M flags
CFLAGS_CORTEX_M = -mthumb

# Select hardware floating-point support
SUPPORTS_HARDWARE_FP_SINGLE = 0
SUPPORTS_HARDWARE_FP_DOUBLE = 0
# CFLAGS_CORTEX_M += -msoft-float

# Options for particular MCU series
CFLAGS_MCU_f1 = $(CFLAGS_CORTEX_M) -mtune=cortex-m3 -mcpu=cortex-m3

CFLAGS = $(INC) -Wall -Wpointer-arith -Werror -std=gnu99 -nostdlib $(CFLAGS_MOD) $(CFLAGS_EXTRA)
CFLAGS += -D$(CMSIS_MCU)
CFLAGS += $(CFLAGS_MCU_f1)
CFLAGS += $(COPT)
CFLAGS += -I$(BOARD_DIR)
CFLAGS += -DMICROPY_HW_VTOR=$(TEXT0_ADDR)
CFLAGS += -DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_FLOAT
CFLAGS += -fsingle-precision-constant -Wdouble-promotion

# Configure for nan-boxing object model if requested
ifeq ($(NANBOX),1)
CFLAGS += -DMP_CONFIGFILE='"mpconfigport_nanbox.h"'
ifneq ($(MICROPY_FLOAT_IMPL),none)
MICROPY_FLOAT_IMPL = double
endif
endif

LDFLAGS = -nostdlib -L $(LD_DIR) $(addprefix -T,$(LD_FILES)) -Map=$(@:.elf=.map) --cref
LDFLAGS += --defsym=_estack_reserve=8
LIBS = $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)

# Remove uncalled code from the final image.
CFLAGS += -fdata-sections -ffunction-sections
LDFLAGS += --gc-sections

# Debugging/Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -g -DPENDSV_DEBUG
COPT = -O0
else
COPT += -Os -DNDEBUG
endif

# Options for mpy-cross
MPY_CROSS_FLAGS += -march=armv7m

SRC_LIB = $(addprefix lib/,\
	libc/string0.c \
	mp-readline/readline.c \
	netutils/netutils.c \
	netutils/trace.c \
	netutils/dhcpserver.c \
	timeutils/timeutils.c \
	utils/pyexec.c \
	utils/interrupt_char.c \
	utils/sys_stdio_mphal.c \
	utils/mpirq.c \
	)

SRC_LIBM = $(addprefix lib/libm/,\
	math.c \
	acoshf.c \
	asinfacosf.c \
	asinhf.c \
	atan2f.c \
	atanf.c \
	atanhf.c \
	ef_rem_pio2.c \
	erf_lgamma.c \
	fmodf.c \
	kf_cos.c \
	kf_rem_pio2.c \
	kf_sin.c \
	kf_tan.c \
	log1pf.c \
	nearbyintf.c \
	sf_cos.c \
	sf_erf.c \
	sf_frexp.c \
	sf_ldexp.c \
	sf_modf.c \
	sf_sin.c \
	sf_tan.c \
	wf_lgamma.c \
	wf_tgamma.c \
	ef_sqrt.c \
	)

EXTMOD_SRC_C = $(addprefix extmod/,\
	modonewire.c \
        )

# ignore bus/softqspi.c
DRIVERS_SRC_C = $(addprefix drivers/,\
	bus/softspi.c \
	memory/spiflash.c \
	dht/dht.c \
	)

SRC_C = \
	main.c \
	stm32_it.c \
	usbd_conf.c \
	usbd_desc.c \
	usbd_cdc_interface.c \
	usbd_hid_interface.c \
	usbd_msc_interface.c \
	mphalport.c \
	mpthreadport.c \
	irq.c \
	pendsv.c \
	systick.c  \
	powerctrl.c \
	pybthread.c \
	factoryreset.c \
	timer.c \
	led.c \
	pin.c \
	pin_defs_stm32.c \
	pin_named_pins.c \
	pin_remap.c \
	bufhelper.c \
	dma.c \
	i2c.c \
	spi.c \
	uart.c \
	can.c \
	usb.c \
	wdt.c \
	gccollect.c \
	help.c \
	machine_i2c.c \
	machine_spi.c \
	machine_uart.c \
	modmachine.c \
	modpyb.c \
	modstm.c \
	moduos.c \
	modutime.c \
	modusocket.c \
	modnetwork.c \
	extint.c \
	usrsw.c \
	rtc.c \
	flash.c \
	flashbdev.c \
	spibdev.c \
	sram.c \
	storage.c \
	sdcard.c \
	fatfs_port.c \
	accel.c \
	lcd.c \
	servo.c \
	dac.c \
	adc.c \
	$(wildcard $(BOARD_DIR)/*.c)

SRC_O = \
	$(STARTUP_FILE) \
	$(SYSTEM_FILE) \
	system_stm32.o \
	resethandler.o \
	lib/utils/gchelper_m3.o

SRC_HAL = $(addprefix $(HAL_DIR)/Src/stm32$(MCU_SERIES)xx_,\
	hal.c \
	hal_adc.c \
	hal_adc_ex.c \
	hal_cortex.c \
	hal_dma.c \
	hal_flash.c \
	hal_flash_ex.c \
	hal_gpio.c \
	hal_i2c.c \
	hal_pcd.c \
	hal_pcd_ex.c \
	hal_pwr.c \
	hal_rcc.c \
	hal_rcc_ex.c \
	hal_rtc.c \
	hal_rtc_ex.c \
	hal_spi.c \
	hal_tim.c \
	hal_tim_ex.c \
	hal_uart.c \
	hal_sd.c \
	ll_sdmmc.c \
	ll_usb.c \
	hal_can.c \
	)

SRC_USBDEV = $(addprefix $(USBDEV_DIR)/,\
	core/src/usbd_core.c \
	core/src/usbd_ctlreq.c \
	core/src/usbd_ioreq.c \
	class/src/usbd_cdc_msc_hid.c \
	class/src/usbd_msc_bot.c \
	class/src/usbd_msc_scsi.c \
)

# for ESP8266-01(S) module
ifeq ($(MICROPY_PY_ESP826601),1)
CFLAGS_MOD += -DMICROPY_PY_ESP826601=$(MICROPY_PY_ESP826601)
CFLAGS_MOD += -DMICROPY_PY_ESP826601_TCP_BUF_SIZE=$(MICROPY_PY_ESP826601_TCP_BUF_SIZE)
SRC_MOD    += modnwesp826601.c
SRC_MOD    += drivers/esp826601/esp826601_ctrl.c
SRC_MOD    += drivers/esp826601/esp826601_socket.c
endif

ifeq ($(MICROPY_SSL_MBEDTLS),1)
CFLAGS_MOD += -DMBEDTLS_CONFIG_FILE='"mbedtls/mbedtls_config.h"'
SRC_MOD += mbedtls/mbedtls_port.c
endif

OBJ =
OBJ += $(PY_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_LIB:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_LIBM:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(EXTMOD_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(DRIVERS_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_O))
OBJ += $(addprefix $(BUILD)/, $(SRC_HAL:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_USBDEV:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_MOD:.c=.o))
OBJ += $(BUILD)/pins_$(BOARD).o

# This file contains performance critical functions so turn up the optimisation
# level.  It doesn't add much to the code size and improves performance a bit.
# Don't use -O3 with this file because gcc tries to optimise memset in terms of itself.
$(BUILD)/lib/libc/string0.o: COPT += -O2

# We put several files into the first 16K section with the ISRs.
# If we compile these using -O0 then it won't fit. So if you really want these
# to be compiled with -O0, then edit boards/common.ld (in the .isr_vector section)
# and comment out the following lines.
$(BUILD)/$(FATFS_DIR)/ff.o: COPT += -Os
$(filter $(PY_BUILD)/../extmod/vfs_fat_%.o, $(PY_O)): COPT += -Os
$(PY_BUILD)/formatfloat.o: COPT += -Os
$(PY_BUILD)/parsenum.o: COPT += -Os
$(PY_BUILD)/mpprint.o: COPT += -Os

all: $(TOP)/lib/stm32lib/README.md $(BUILD)/firmware.dfu $(BUILD)/firmware.hex

# For convenience, automatically fetch required submodules if they don't exist
$(TOP)/lib/stm32lib/README.md:
	$(ECHO) "stm32lib submodule not found, fetching it now..."
	(cd $(TOP) && git submodule update --init lib/stm32lib)

ifneq ($(FROZEN_MANIFEST)$(FROZEN_MPY_DIR),)
# To use frozen bytecode, put your .py files in a subdirectory (eg frozen/) and
# then invoke make with FROZEN_MPY_DIR=frozen (be sure to build from scratch).
CFLAGS += -DMICROPY_QSTR_EXTRA_POOL=mp_qstr_frozen_const_pool
CFLAGS += -DMICROPY_MODULE_FROZEN_MPY
endif

.PHONY: deploy

deploy: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $< to the board"
ifeq ($(USE_PYDFU),1)
	$(Q)$(PYTHON) $(PYDFU) -u $<
else
	$(Q)$(DFU_UTIL) -a 0 -d $(DEVICE) -D $<
endif

# A board should specify TEXT0_ADDR if to use a different location than the
# default for the firmware memory location.  A board can also optionally define
# TEXT1_ADDR to split the firmware into two sections; see below for details.
TEXT0_ADDR ?= 0x08000000

ifeq ($(TEXT1_ADDR),)
# No TEXT1_ADDR given so put all firmware at TEXT0_ADDR location

TEXT0_SECTIONS ?= .isr_vector .text .data

deploy-stlink: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $(BUILD)/firmware.bin to the board via ST-LINK"
	$(Q)$(STFLASH) write $(BUILD)/firmware.bin $(TEXT0_ADDR)

deploy-openocd: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $(BUILD)/firmware.bin to the board via ST-LINK using OpenOCD"
	$(Q)$(OPENOCD) -f $(OPENOCD_CONFIG) -c "stm_flash $(BUILD)/firmware.bin $(TEXT0_ADDR)"

$(BUILD)/firmware.dfu: $(BUILD)/firmware.elf
	$(ECHO) "Create $@"
	$(Q)$(OBJCOPY) -O binary $(addprefix -j ,$(TEXT0_SECTIONS)) $^ $(BUILD)/firmware.bin
	$(Q)$(PYTHON) $(DFU) -b $(TEXT0_ADDR):$(BUILD)/firmware.bin $@

else
# TEXT0_ADDR and TEXT1_ADDR are specified so split firmware between these locations

TEXT0_SECTIONS ?= .isr_vector
TEXT1_SECTIONS ?= .text .data

deploy-stlink: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $(BUILD)/firmware0.bin to the board via ST-LINK"
	$(Q)$(STFLASH) write $(BUILD)/firmware0.bin $(TEXT0_ADDR)
	$(ECHO) "Writing $(BUILD)/firmware1.bin to the board via ST-LINK"
	$(Q)$(STFLASH) --reset write $(BUILD)/firmware1.bin $(TEXT1_ADDR)

deploy-openocd: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $(BUILD)/firmware{0,1}.bin to the board via ST-LINK using OpenOCD"
	$(Q)$(OPENOCD) -f $(OPENOCD_CONFIG) -c "stm_flash $(BUILD)/firmware0.bin $(TEXT0_ADDR) $(BUILD)/firmware1.bin $(TEXT1_ADDR)"
	
$(BUILD)/firmware.dfu: $(BUILD)/firmware.elf
	$(ECHO) "GEN $@"
	$(Q)$(OBJCOPY) -O binary $(addprefix -j ,$(TEXT0_SECTIONS)) $^ $(BUILD)/firmware0.bin
	$(Q)$(OBJCOPY) -O binary $(addprefix -j ,$(TEXT1_SECTIONS)) $^ $(BUILD)/firmware1.bin
	$(Q)$(PYTHON) $(DFU) -b $(TEXT0_ADDR):$(BUILD)/firmware0.bin -b $(TEXT1_ADDR):$(BUILD)/firmware1.bin $@

endif

$(BUILD)/firmware.hex: $(BUILD)/firmware.elf
	$(ECHO) "GEN $@"
	$(Q)$(OBJCOPY) -O ihex $< $@

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LDFLAGS_MOD) $(LIBS)
	$(Q)$(SIZE) $@
ifneq ($(TEXT0_ADDR),0x08000000)
	$(ECHO) "INFO: this build requires mboot to be installed first"
endif
ifeq ($(TEXT1_ADDR),0x64000000)
	$(ECHO) "INFO: this build places firmware in external FMSC flash"
endif

PLLVALUES = boards/pllvalues.py
MAKE_PINS = boards/make-pins.py
BOARD_PINS = $(BOARD_DIR)/pins.csv
PREFIX_FILE = boards/stm32f1xx_prefix.c
GEN_PINS_SRC = $(BUILD)/pins_$(BOARD).c
GEN_PINS_HDR = $(HEADER_BUILD)/pins.h
GEN_PINS_QSTR = $(BUILD)/pins_qstr.h
GEN_PINS_AF_CONST = $(HEADER_BUILD)/pins_af_const.h
GEN_PINS_AF_DEFS = $(HEADER_BUILD)/pins_af_defs.h
GEN_PINS_AF_PY = $(BUILD)/pins_af.py

INSERT_USB_IDS = $(TOP)/tools/insert-usb-ids.py
FILE2H = $(TOP)/tools/file2h.py

USB_IDS_FILE = usb.h
CDCINF_TEMPLATE = pybcdc.inf_template
GEN_CDCINF_FILE = $(HEADER_BUILD)/pybcdc.inf
GEN_CDCINF_HEADER = $(HEADER_BUILD)/pybcdc_inf.h

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C) $(SRC_MOD) $(SRC_LIB) $(EXTMOD_SRC_C)
# Append any auto-generated sources that are needed by sources listed in
# SRC_QSTR
SRC_QSTR_AUTO_DEPS += $(GEN_CDCINF_HEADER)

# Making OBJ use an order-only depenedency on the generated pins.h file
# has the side effect of making the pins.h file before we actually compile
# any of the objects. The normal dependency generation will deal with the
# case when pins.h is modified. But when it doesn't exist, we don't know
# which source files might need it.
$(OBJ): | $(GEN_PINS_HDR)

# With conditional pins, we may need to regenerate qstrdefs.h when config
# options change.
$(HEADER_BUILD)/qstrdefs.generated.h: $(BOARD_DIR)/mpconfigboard.h

# main.c can't be even preprocessed without $(GEN_CDCINF_HEADER)
main.c: $(GEN_CDCINF_HEADER)

# Use a pattern rule here so that make will only call make-pins.py once to make
# both pins_$(BOARD).c and pins.h
$(BUILD)/%_$(BOARD).c $(HEADER_BUILD)/%.h $(HEADER_BUILD)/%_af_const.h $(HEADER_BUILD)/%_af_defs.h $(BUILD)/%_qstr.h: $(BOARD_DIR)/%.csv $(MAKE_PINS) $(AF_FILE) $(PREFIX_FILE) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(MAKE_PINS) --board $(BOARD_PINS) --af $(AF_FILE) \
		--prefix $(PREFIX_FILE) --hdr $(GEN_PINS_HDR) --hdr-obj-decls \
		--qstr $(GEN_PINS_QSTR) --af-const $(GEN_PINS_AF_CONST) \
		--af-defs $(GEN_PINS_AF_DEFS) --af-defs-cmp-strings \
		--af-py $(GEN_PINS_AF_PY) > $(GEN_PINS_SRC)

$(BUILD)/pins_$(BOARD).o: $(BUILD)/pins_$(BOARD).c
	$(call compile_c)

GEN_PLLFREQTABLE_HDR = $(HEADER_BUILD)/pllfreqtable.h
GEN_STMCONST_HDR = $(HEADER_BUILD)/modstm_const.h
GEN_STMCONST_QSTR = $(BUILD)/modstm_qstr.h
GEN_STMCONST_MPZ = $(HEADER_BUILD)/modstm_mpz.h
CMSIS_MCU_HDR = $(CMSIS_DIR)/$(CMSIS_MCU_LOWER).h

modmachine.c: $(GEN_PLLFREQTABLE_HDR)
$(GEN_PLLFREQTABLE_HDR): $(PLLVALUES) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(PLLVALUES) -c file:$(BOARD_DIR)/stm32f1xx_hal_conf.h > $@

$(BUILD)/modstm.o: $(GEN_STMCONST_HDR)
# Use a pattern rule here so that make will only call make-stmconst.py once to
# make both modstm_const.h and modstm_qstr.h
$(HEADER_BUILD)/%_const.h $(BUILD)/%_qstr.h: $(CMSIS_MCU_HDR) make-stmconst.py | $(HEADER_BUILD)
	$(ECHO) "GEN stmconst $@"
	$(Q)$(PYTHON) make-stmconst.py --qstr $(GEN_STMCONST_QSTR) --mpz $(GEN_STMCONST_MPZ) $(CMSIS_MCU_HDR) > $(GEN_STMCONST_HDR)

$(GEN_CDCINF_HEADER): $(GEN_CDCINF_FILE) $(FILE2H) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(FILE2H) $< > $@

$(GEN_CDCINF_FILE): $(CDCINF_TEMPLATE) $(INSERT_USB_IDS) $(USB_IDS_FILE) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(INSERT_USB_IDS) $(USB_IDS_FILE) $< > $@

include $(TOP)/py/mkrules.mk
