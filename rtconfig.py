import os

ARCH       = 'arm'
CPU        = 'cortex-m4'
CROSS_TOOL = 'gcc'
PLATFORM   = 'gcc'
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

DEVICE = ' -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter -fomit-frame-pointer -ffast-math -ftree-vectorize'
CFLAGS = DEVICE + ' -Dgcc -std=gnu99' # -D' + PART_TYPE
CPPFLAGS = DEVICE + ' -Dgcc -std=c++11' # -D' + PART_TYPE
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '
LFLAGS = DEVICE + ' -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -u Reset_Handler -T vendor/GD32E10x-BSP/linker/gd32_flash.ld -Wl,-cref,-Map=rtthread-gd32.map '

CPATH = ''
LPATH = ''

if BUILD == 'debug':
    CFLAGS += ' -Og -gdwarf-2 -g'
    AFLAGS += ' -gdwarf-2'
else:
    CFLAGS += ' -O2 -Os'

POST_ACTION = EXEC_PATH + OBJCPY + ' -O binary $TARGET rtthread.bin && ' + SIZE + ' $TARGET'
