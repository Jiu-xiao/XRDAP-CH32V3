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

add_executable(xrdap.elf app/main.c app/app_main.cpp app/probe_info.cpp)
target_include_directories(xrdap.elf PRIVATE app)
target_link_libraries(xrdap.elf PRIVATE platform freertos xr)
xrdap_add_image(xrdap.elf ${XRDAP_APP_BASE} ${XRDAP_APP_SIZE} 0x1000)
