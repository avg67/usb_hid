cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_add_include_directories(.)
sdk_add_include_directories(ascii_keyb)

target_sources(app PRIVATE usb_host.c hidparser.c ascii_keyb/ps2keyboard.c)
sdk_set_main_file(main.c)

project(usbhost)
