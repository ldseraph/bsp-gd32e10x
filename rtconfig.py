import os

ARCH       = 'arm'
CPU        = 'cortex-m4'
CROSS_TOOL = 'gcc'
PLATFORM   = '$(PLATFORM)'
EXEC_PATH  = '$(EXEC_PATH)'
BUILD      = 'release'

PREFIX = 'arm-none-eabi-'
CC = PREFIX + 'gcc'
AS = PREFIX + 'gcc'
AR = PREFIX + 'ar'
LINK = PREFIX + 'gcc'
TARGET_EXT = 'elf'
SIZE = PREFIX + 'size'
OBJDUMP = PREFIX + 'objdump'
OBJCPY = PREFIX + 'objcopy'

DEVICE = '$(CXFLAGS)'
CFLAGS = DEVICE + '$(CFLAGS)' # -D' + PART_TYPE
CPPFLAGS = DEVICE + '$(CXXFLAGS)' # -D' + PART_TYPE
AFLAGS = '$(ASFLAGS)'
LFLAGS = DEVICE + '$(LDFLAGS)'

CPATH = ''
LPATH = ''

POST_ACTION = EXEC_PATH + OBJCPY + ' -O binary $TARGET rtthread.bin && ' + SIZE + ' $TARGET'
