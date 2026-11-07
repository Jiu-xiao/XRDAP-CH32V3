cmake_minimum_required(VERSION 3.20)

# XR-DAP CH32 application: CMSIS-DAP v2 probe in a single loop without an RTOS, linked behind the
# bootloader. A preset selects the firmware (cmake --preset wdap; cmake --list-presets), or
# -DXRDAP_BOARD=wdap|v305|linkw the board. The same build makes the board's boot chain
# (bootloader/, a project of its own) in <build>/bootloader/.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/xrdap.cmake")

project(xrdap_ch32v3 VERSION ${XRDAP_VERSION} LANGUAGES C CXX ASM)
message(STATUS "XR-DAP firmware version ${XRDAP_FIRMWARE_VERSION}")
xrdap_setup_flags(O2)
xrdap_setup_libraries()

xrdap_add_chip_library()

set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ${XRDAP_CHIP_CONFIG_FILE})
add_subdirectory(libxr)
target_include_directories(xr PUBLIC ${XRDAP_INCLUDES})
target_compile_definitions(xr PUBLIC ${XRDAP_COMMON_DEFINES})

# The common probe (app/probe.hpp) and the board's composition (boards/<board>/board.cpp) with
# the parts it lists in XRDAP_BOARD_SOURCES.
add_executable(xrdap.elf app/main.c app/probe.cpp app/boot_guard.cpp app/sampler_tick.cpp
                         app/boot_slot_info.cpp common/sysclk.c
                         boards/${XRDAP_BOARD}/board.cpp ${XRDAP_BOARD_SOURCES})
target_compile_definitions(xrdap.elf PRIVATE XRDAP_SYSCLK_HZ=${XRDAP_APP_SYSCLK_HZ}
                           XRDAP_DAP_PACKET_COUNT=${XRDAP_DAP_PACKET_COUNT}
                           XRDAP_RING_BYTES=${XRDAP_RING_BYTES}
                           XRDAP_FIRMWARE_VERSION="${XRDAP_FIRMWARE_VERSION}")
# XRDAP-Core (app/xrdap_port.hpp is its port header) and the SX1281 Module, header only.
include(xrdap-core/xrdap_core.cmake)
target_sources(xrdap.elf PRIVATE ${XRDAP_CORE_SOURCES})
target_include_directories(xrdap.elf PRIVATE app common modules/SX1281
                                             ${XRDAP_CORE_INCLUDE_DIRS})
target_link_libraries(xrdap.elf PRIVATE chip xr)
# The image stops in front of the settings and the seal.
math(EXPR XRDAP_APP_IMAGE_SIZE "${XRDAP_APP_SIZE} - ${XRDAP_SEAL_SIZE} - ${XRDAP_CONFIG_SIZE}"
     OUTPUT_FORMAT HEXADECIMAL)
xrdap_add_image(xrdap.elf ${XRDAP_APP_BASE} ${XRDAP_APP_IMAGE_SIZE} ${XRDAP_C_HEAP_SIZE})

# The boot chain of the same board: a project of its own, since it builds LibXR again for
# 96 MHz and -Os.
include(ExternalProject)
ExternalProject_Add(bootloader
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/bootloader"
  BINARY_DIR "${CMAKE_BINARY_DIR}/bootloader"
  CMAKE_ARGS -DXRDAP_BOARD=${XRDAP_BOARD} -DXRDAP_APP_SYSCLK_HZ=${XRDAP_APP_SYSCLK_HZ}
             -DCOMPILER_PREFIX=${COMPILER_PREFIX} -DXRDAP_GIT_REV=${XRDAP_GIT_REV}
  INSTALL_COMMAND ""
  BUILD_ALWAYS ON)
