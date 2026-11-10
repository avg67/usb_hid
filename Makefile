COMX?=/dev/ttyUSB0
BAUDRATE?=921600

SDK_DEMO_PATH ?= .
#BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../../..
BL_SDK_BASE ?= /home/avg/working/bl616/bouffalo_sdk

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-

#echo "BL_SDK_BASE: $(BL_SDK_BASE)"
include $(BL_SDK_BASE)/project.build

term:
	term.sh $(COMX) $(BAUDRATE)
