cmake_minimum_required(VERSION 3.20)

# XR-DAP CH32 application: CMSIS-DAP v2 probe in a single loop without an RTOS, linked behind the
# bootloader (bootloader/ is a separate project). Board: -DXRDAP_BOARD=evt|wdap|v305.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ch32v3.cmake")

# Firmware version, reported in DAP_Info: the version below plus the commit it was built
# from, e.g. 0.2.0+g9c0860c (".dirty" for uncommitted changes). A build from a copy
# without .git passes the commit with -DXRDAP_GIT_REV=<short hash>[.dirty].
project(xrdap_ch32v3 VERSION 0.2.0 LANGUAGES C CXX ASM)
if(NOT DEFINED XRDAP_GIT_REV)
  execute_process(COMMAND git rev-parse --short=7 HEAD
                  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                  OUTPUT_VARIABLE XRDAP_GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE
                  ERROR_QUIET RESULT_VARIABLE _git_result)
  if(_git_result EQUAL 0)
    execute_process(COMMAND git status --porcelain --untracked-files=no
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                    OUTPUT_VARIABLE _git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
    if(_git_dirty)
      string(APPEND XRDAP_GIT_REV ".dirty")
    endif()
  else()
    set(XRDAP_GIT_REV "")
  endif()
endif()
set(XRDAP_FIRMWARE_VERSION "${PROJECT_VERSION}")
if(XRDAP_GIT_REV)
  string(APPEND XRDAP_FIRMWARE_VERSION "+g${XRDAP_GIT_REV}")
endif()
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
target_include_directories(xrdap.elf PRIVATE app ${XRDAP_CORE_INCLUDE_DIRS})
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
