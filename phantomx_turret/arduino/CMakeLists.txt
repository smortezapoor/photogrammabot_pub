cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Add ArbotiX Hardware Platform
set(PLATFORM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/arbotix/hardware/arbotix")
set(PLATFORM "ARBOTIX")

set(${PLATFORM}_PLATFORM_PATH ${PLATFORM_PATH} CACHE INTERNAL "The path to ${PLATFORM}")
set(ARDUINO_PLATFORMS ${ARDUINO_PLATFORMS} ${PLATFORM} CACHE INTERNAL "A list of registered platforms")

set(${PLATFORM}_CORES_PATH "${PLATFORM_PATH}/cores")
set(${PLATFORM}_VARIANTS_PATH "${PLATFORM_PATH}/variants")
set(${PLATFORM}_BOOTLOADERS_PATH "${PLATFORM_PATH}/bootloaders")
set(${PLATFORM}_PROGRAMMERS_PATH "${PLATFORM_PATH}/programmers.txt")
set(${PLATFORM}_BOARDS_PATH "${PLATFORM_PATH}/boards.txt")

set(core "arbotix")
set(dir "${${PLATFORM}_CORES_PATH}/${core}")
set(CORES ${CORES} ${core} CACHE INTERNAL "A list of registered cores")
set(${core}.path ${dir} CACHE INTERNAL "The path to the core ${core}")

# Configure Arbotix-M Board
set(arbotixm.name "ArbotiX-M")

set(arbotixm.build.core "arbotix")
set(arbotixm.build.f_cpu "16000000L")
set(arbotixm.build.mcu "atmega644p")
set(arbotixm.build.variant "standard")
set(arbotixm.build.board "AVR_ARBOTIXM")

set(arbotixm.bootloader.tool "avrdude")
set(arbotixm.bootloader.unlock_bits "0x3F")
set(arbotixm.bootloader.lock_bits "0x0F")
set(arbotixm.bootloader.high_fuses "0xDC")
set(arbotixm.bootloader.low_fuses "0xFF")
set(arbotixm.bootloader.extended_fuses "0xFD")
set(arbotixm.bootloader.path "atmega644p")
set(arbotixm.bootloader.file "ATmegaBOOT_644P.hex")

set(arbotixm.upload.tool "avrdude")
set(arbotixm.upload.protocol "arduino")
set(arbotixm.upload.maximum_size "65536")
set(arbotixm.upload.maximum_data_size "4096")
set(arbotixm.upload.speed "38400")

set(arbotixm.SETTINGS  name build bootloader upload)
set(arbotixm.build.SUBSETTINGS  core f_cpu mcu variant board)
set(arbotixm.bootloader.SUBSETTINGS  tool unlock_bits lock_bits high_fuses low_fuses extended_fuses path file)
set(arbotixm.upload.SUBSETTINGS  tool protocol maximum_size maximum_data_size speed)

list(APPEND ARBOTIX_BOARDS "arbotixm")


generate_arduino_library(ax12
  BOARD arbotixm
  NO_AUTOLIBS
  SRCS ax12/ax12.cpp
  HDRS ax12/ax12.h
)

generate_arduino_firmware(phantomx_turret
  SRCS phantomx_turret.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD arbotixm
  LIBS ax12
  PORT /dev/ttyUSB0
)
