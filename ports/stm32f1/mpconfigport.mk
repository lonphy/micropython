# Enable/disable extra modules

# VFS FAT FS support
MICROPY_VFS_FAT ?= 1

MICROPY_PY_ESP826601 ?= 0

# 每个TCP链接的接收缓冲, 单位字节
MICROPY_PY_ESP826601_TCP_BUF_SIZE ?= 2048