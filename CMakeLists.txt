cmake_minimum_required(VERSION 3.20)

# XR-DAP CH32 application: CMSIS-DAP v2 probe in a single loop without an RTOS, linked behind the
# bootloader (bootloader/ is a separate project). Board: -DXRDAP_BOARD=evt|wdap|v305.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ch32v3.cmake")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/version.cmake")
project(xrdap_ch32v3 VERSION ${XRDAP_VERSION} LANGUAGES C CXX ASM)
message(STATUS "XR-DAP firmware version ${XRDAP_FIRMWARE_VERSION}")
xrdap_setup_flags(O2)
xrdap_setup_libraries()

xrdap_add_platform_library()

set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory(libxr)
target_include_directories(xr PUBLIC ${XRDAP_PLATFORM_INCLUDES})
target_compile_definitions(xr PUBLIC ${XRDAP_COMMON_DEFINES})

add_executable(xrdap.elf app/main.c app/app_main.cpp app/boot_guard.cpp app/config.cpp
                         app/settings_log.cpp app/sampler_tick.cpp common/sysclk.c)
target_compile_definitions(xrdap.elf PRIVATE XRDAP_SYSCLK_HZ=${XRDAP_APP_SYSCLK_HZ}
                           XRDAP_DAP_PACKET_COUNT=${XRDAP_DAP_PACKET_COUNT}
                           XRDAP_RING_BYTES=${XRDAP_RING_BYTES})
# XRDAP-Core: session owner and command executor, XR vendor extension, status light
# rule, probe identity and family constants (app/xrdap_port.hpp is its port header).
include(xrdap-core/xrdap_core.cmake)
target_sources(xrdap.elf PRIVATE ${XRDAP_CORE_SOURCES})
target_include_directories(xrdap.elf PRIVATE app common ${XRDAP_CORE_INCLUDE_DIRS})
target_compile_definitions(xrdap.elf PRIVATE XRDAP_FIRMWARE_VERSION="${XRDAP_FIRMWARE_VERSION}")
if(XRDAP_HAS_RADIO)
  # Role and modulation are settings in flash (app/config.hpp); one image serves both
  # ends of the link.
  target_sources(xrdap.elf PRIVATE app/sx1281.cpp app/link_protocol.cpp app/link_session.cpp
                                   app/link_transport.cpp app/dap_forwarding.cpp
                                   app/dap_session.cpp app/pair_identity.cpp
                                   app/radio_command.cpp app/pairing.cpp)
  target_compile_definitions(xrdap.elf PRIVATE XRDAP_HAS_RADIO)
endif()
if(NOT XRDAP_EXTERNAL_BOOTLOADER)
  # Stage 1 and the A/B bootloader slots (bootloader/): BootInfo reports them.
  target_sources(xrdap.elf PRIVATE app/boot_info.cpp)
  target_compile_definitions(xrdap.elf PRIVATE XRDAP_HAS_AB_BOOT)
endif()
if(XRDAP_HAS_STATUS_LIGHTS)
  target_sources(xrdap.elf PRIVATE app/status_lights.cpp app/ws2812.cpp)
  target_compile_definitions(xrdap.elf PRIVATE XRDAP_HAS_STATUS_LIGHTS)
endif()
if(XRDAP_HAS_STATUS_LED)
  target_sources(xrdap.elf PRIVATE app/status_led.cpp)
  target_compile_definitions(xrdap.elf PRIVATE XRDAP_HAS_STATUS_LED)
endif()
target_link_libraries(xrdap.elf PRIVATE platform xr)
# The image stops in front of the settings and the seal.
math(EXPR XRDAP_APP_IMAGE_SIZE "${XRDAP_APP_SIZE} - ${XRDAP_SEAL_SIZE} - ${XRDAP_CONFIG_SIZE}"
     OUTPUT_FORMAT HEXADECIMAL)
xrdap_add_image(xrdap.elf ${XRDAP_APP_BASE} ${XRDAP_APP_IMAGE_SIZE} ${XRDAP_C_HEAP_SIZE})
