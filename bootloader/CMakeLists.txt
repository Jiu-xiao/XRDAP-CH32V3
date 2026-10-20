cmake_minimum_required(VERSION 3.20)

# XR-DAP-V307 USB DFU bootloader. Bare-metal LibXR (no RTOS), runs at 96 MHz.
get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
include("${XRDAP_ROOT}/cmake/ch32v307.cmake")

project(xrdap_v307_bootloader C CXX ASM)
xrdap_setup_flags(Os)
xrdap_setup_libraries()
list(APPEND XRDAP_COMMON_DEFINES XRDAP_SYSCLK_96MHZ)

xrdap_add_platform_library()

set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory("${XRDAP_ROOT}/libxr" libxr)
target_include_directories(xr PUBLIC ${XRDAP_PLATFORM_INCLUDES})
target_compile_definitions(xr PUBLIC ${XRDAP_COMMON_DEFINES})

add_executable(bootloader.elf main.c bootloader.cpp)
target_link_libraries(bootloader.elf PRIVATE platform xr)
# LibXR allocates the DFU and USB buffers from the heap at start-up.
xrdap_add_image(bootloader.elf ${XRDAP_BOOTLOADER_BASE} ${XRDAP_BOOTLOADER_SIZE} 0x8000)
