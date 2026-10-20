cmake_minimum_required(VERSION 3.20)

# XR-DAP-V307 application: CMSIS-DAP v2 probe on FreeRTOS, linked behind the
# bootloader (bootloader/ is a separate project). Board: -DXRDAP_BOARD=evt|wdap.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ch32v307.cmake")

project(xrdap_v307 C CXX ASM)
xrdap_setup_flags(O2)
xrdap_setup_libraries()

xrdap_add_platform_library()

set(FREERTOS_DIR "${XRDAP_PLATFORM_DIR}/FreeRTOS")
set(FREERTOS_INCLUDES
    "${FREERTOS_DIR}/include"
    "${FREERTOS_DIR}/portable/GCC/RISC-V"
    "${FREERTOS_DIR}/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions")
file(GLOB FREERTOS_SOURCES "${FREERTOS_DIR}/*.c" "${FREERTOS_DIR}/portable/GCC/RISC-V/*.c")
add_library(freertos OBJECT ${FREERTOS_SOURCES}
            "${FREERTOS_DIR}/portable/MemMang/heap_4.c"
            "${FREERTOS_DIR}/portable/GCC/RISC-V/portASM.S")
target_include_directories(freertos PUBLIC ${FREERTOS_INCLUDES} ${XRDAP_PLATFORM_INCLUDES})
target_compile_definitions(freertos PUBLIC ${XRDAP_COMMON_DEFINES})

set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory(libxr)
target_include_directories(xr PUBLIC ${XRDAP_PLATFORM_INCLUDES} ${FREERTOS_INCLUDES})
target_compile_definitions(xr PUBLIC ${XRDAP_COMMON_DEFINES})

add_executable(xrdap.elf app/main.c app/app_main.cpp app/probe_info.cpp app/boot_guard.cpp
                         app/config.cpp)
target_include_directories(xrdap.elf PRIVATE app)
if(XRDAP_HAS_RADIO)
  # Role and modulation are settings in flash (app/config.hpp); one image serves both
  # ends of the link. FLRC coding rate: 0x04 CR 1 (no FEC), 0x02 CR 3/4, 0x00 CR 1/2.
  set(XRDAP_RADIO_CODING_RATE 0x04 CACHE STRING "FLRC coding rate")
  target_sources(xrdap.elf PRIVATE app/sx1281.cpp app/radio_link.cpp app/radio_command.cpp
                                   app/pairing.cpp)
  target_compile_definitions(xrdap.elf PRIVATE XRDAP_HAS_RADIO
                                               XRDAP_RADIO_CODING_RATE=${XRDAP_RADIO_CODING_RATE})
  # Driver switches for experiments (app/sx1281.hpp), comma separated, e.g.
  # -DXRDAP_RADIO_DEFINES=XRDAP_RADIO_AUTOFS=0,XRDAP_RADIO_SPI_DMA_MIN=24
  set(XRDAP_RADIO_DEFINES "" CACHE STRING "Extra radio driver defines")
  string(REPLACE "," ";" _radio_defines "${XRDAP_RADIO_DEFINES}")
  target_compile_definitions(xrdap.elf PRIVATE ${_radio_defines})
endif()
target_link_libraries(xrdap.elf PRIVATE platform freertos xr)
# The image stops in front of the settings and the seal.
math(EXPR XRDAP_APP_IMAGE_SIZE "${XRDAP_APP_SIZE} - ${XRDAP_SEAL_SIZE} - ${XRDAP_CONFIG_SIZE}"
     OUTPUT_FORMAT HEXADECIMAL)
xrdap_add_image(xrdap.elf ${XRDAP_APP_BASE} ${XRDAP_APP_IMAGE_SIZE} 0x1000)
