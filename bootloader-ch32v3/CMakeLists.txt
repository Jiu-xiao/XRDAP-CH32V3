cmake_minimum_required(VERSION 3.20)

# Three-stage bootflow of the CH32V305 board (v305): the stage-1 trampoline at 0, which
# only jumps to the stage-3 USB DFU bootloader at 0x20000. The bootloader stays in DFU on
# the IAP key (PC7), a boot mailbox request or an application without a valid entry and
# seal; otherwise it starts the application at 0x2000 when it gets RUN_APP (vendor
# request 0x5A), or by itself with XRDAP_V3_BOOT_AUTORUN. It builds against its own libxr
# (the libxr submodule here, the version it has run with), bare metal, with the flags it
# was released with.
#
#   cmake -S bootloader-ch32v3 -B build/bootloader-ch32v3
#   cmake --build build/bootloader-ch32v3
#
# Output: stage1.bin (written at 0x08000000) and bootloader.bin (at 0x08020000), with a
# WCH-Link; the application then goes in over DFU (README).
#
# CH32V305 板（v305）的三段式启动：地址 0 的 stage1 只跳到 0x20000 的 stage3 USB DFU
# bootloader。bootloader 在 IAP 键（PC7）按下、启动信箱有请求或应用入口和 seal 无效时停在
# DFU；否则收到 RUN_APP（厂商请求 0x5A）时启动 0x2000 的应用，设了 XRDAP_V3_BOOT_AUTORUN
# 时自动启动。它用自己的 libxr（本目录的 libxr 子模块，即它运行时的版本）、裸机、发布时的
# 编译选项编译。

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(COMPILER_PREFIX "riscv32-wch-elf-" CACHE STRING "Cross compiler prefix")
set(CMAKE_C_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_OBJCOPY ${COMPILER_PREFIX}objcopy)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(xrdap_ch32v3_bootloader C CXX ASM)

get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(PLATFORM "${XRDAP_ROOT}/platform")

# Layout, USB identity and behaviour of the bootloader on the board.
set(XRDAP_V3_APP_BASE 0x00002000 CACHE STRING "Application window base")
set(XRDAP_V3_APP_LIMIT 0x00020000 CACHE STRING "Application window end")
set(XRDAP_V3_BL_BASE 0x00020000 CACHE STRING "Stage-3 bootloader base")
set(XRDAP_V3_BL_LENGTH 0x00030000 CACHE STRING "Stage-3 bootloader length")
set(XRDAP_V3_BOOT_USB_PID 0x3070 CACHE STRING "Bootloader USB PID (VID 0x1209)")
option(XRDAP_V3_BOOT_AUTORUN "Start a valid application without RUN_APP" OFF)

set(CPU_FLAGS "-march=rv32imafc_xw -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore")
set(COMMON_FLAGS "-Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -fsigned-char -fsingle-precision-constant -Wunused -Wuninitialized -fno-unwind-tables -fno-asynchronous-unwind-tables")
set(OPT_FLAGS "-O3 -flto -fomit-frame-pointer -fno-math-errno -fno-trapping-math -g0")
set(CMAKE_C_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${OPT_FLAGS} -fmessage-length=0" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${OPT_FLAGS} -fno-use-cxa-atexit -fno-exceptions -fno-rtti -fno-threadsafe-statics" CACHE INTERNAL "" FORCE)
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp -g0" CACHE INTERNAL "" FORCE)
set(CMAKE_BUILD_TYPE "" CACHE STRING "" FORCE)
add_compile_definitions(HSE_VALUE=12000000)
add_link_options(-nostartfiles -Xlinker --gc-sections -Wl,--print-memory-usage
                 --specs=nano.specs --specs=nosys.specs -march=rv32imafc_xw -mabi=ilp32f
                 -flto -lm -lc)

# The board's Core files and one peripheral header (2- vs 4-level interrupt nesting) of
# this bootloader; startup, syscalls and the peripheral library are shared with the
# application.
set(INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/core"
             "${CMAKE_CURRENT_SOURCE_DIR}" "${PLATFORM}/Peripheral/inc")
file(GLOB PERIPHERAL_SOURCES "${PLATFORM}/Peripheral/src/*.c")
add_library(peripheral OBJECT ${PERIPHERAL_SOURCES})
target_compile_definitions(peripheral PUBLIC CH32V30x_D8C)
target_include_directories(peripheral PUBLIC ${INCLUDES})

set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory(libxr)
target_include_directories(xr PUBLIC ${INCLUDES})
target_compile_definitions(xr PRIVATE CH32V30x_D8C)

function(xrdap_v3_image target)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.bin
    VERBATIM)
endfunction()

add_executable(stage1 stage1_jump.S)
set_target_properties(stage1 PROPERTIES LINKER_LANGUAGE C)
target_link_options(stage1 PRIVATE -T "${CMAKE_CURRENT_SOURCE_DIR}/Link.stage1.ld")
target_compile_definitions(stage1 PRIVATE CH32_STAGE3_BL_BASE=${XRDAP_V3_BL_BASE})
xrdap_v3_image(stage1)

file(GLOB CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/core/*.c")
add_executable(bootloader ${CORE_SOURCES} boot_main.c bootloader_main.cpp
               "${PLATFORM}/syscalls.c" "${PLATFORM}/startup_ch32v30x_D8C.S")
target_include_directories(bootloader PRIVATE ${INCLUDES})
target_link_options(bootloader PRIVATE
  -T "${CMAKE_CURRENT_SOURCE_DIR}/Link.stage3.ld"
  -Wl,--defsym=APP_FLASH_BASE=${XRDAP_V3_BL_BASE}
  -Wl,--defsym=APP_FLASH_LENGTH=${XRDAP_V3_BL_LENGTH})
target_compile_definitions(bootloader PRIVATE
  CH32V30x_D8C
  CH32_BOOT_APP_BASE=${XRDAP_V3_APP_BASE}
  CH32_BOOT_APP_LIMIT=${XRDAP_V3_APP_LIMIT}
  CH32_BOOT_USB_VID=0x1209
  CH32_BOOT_USB_PID=${XRDAP_V3_BOOT_USB_PID}
  CH32_BOOT_USB_BCD=0x0001
  CH32_BOOT_ENABLE_AUX_CDC=0
  CH32_BOOT_RECLOCK_48M=1
  CH32_BOOT_AUTORUN=$<IF:$<BOOL:${XRDAP_V3_BOOT_AUTORUN}>,1,0>
  SYSCLK_FREQ_144MHz_HSE=144000000)
target_link_libraries(bootloader PRIVATE peripheral xr)
xrdap_v3_image(bootloader)
