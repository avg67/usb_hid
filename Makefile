COMX?=/dev/ttyUSB0
BAUDRATE?=921600

SDK_DEMO_PATH ?= .
#BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../../..
#BL_SDK_BASE ?= /home/avg/working/bl616/bouffalo_sdk
BL_SDK_BASE = /home/avg/working/github/bouffalo_sdk

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-

include $(BL_SDK_BASE)/project.build
#echo "BL_SDK_BASE: $(BL_SDK_BASE)"

term:
	term.sh $(COMX) $(BAUDRATE)
