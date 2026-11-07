cmake_minimum_required(VERSION 3.20)

# XR-DAP boot chain (evt, wdap, wdap-144, v305, v305-144, linkw): stage 1 at 0 and the USB DFU bootloader, linked once
# for each bootloader slot (common/boot_slots.hpp). Bare-metal LibXR (no RTOS), runs at
# 96 MHz. With Python 3 it also builds boot-chain.bin/.hex (stage 1 + the board's first
# slot with its record, tools/boot_chain.py) for the first installation and the
# migration image.
get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
include("${XRDAP_ROOT}/cmake/ch32v3.cmake")
include("${XRDAP_ROOT}/cmake/version.cmake")

project(xrdap_ch32v3_bootloader C CXX ASM)
xrdap_setup_flags(Os)
xrdap_setup_libraries()

# The boot key stage 1 reads (board cmake: XRDAP_BOOT_KEY_PORT A..D, XRDAP_BOOT_KEY_PIN).
set(_gpio_A 0x40010800)
set(_gpio_B 0x40010C00)
set(_gpio_C 0x40011000)
set(_gpio_D 0x40011400)
set(_rcc_A 0x04)
set(_rcc_B 0x08)
set(_rcc_C 0x10)
set(_rcc_D 0x20)
if(NOT DEFINED XRDAP_BOOT_KEY_PORT OR NOT DEFINED XRDAP_BOOT_KEY_PIN)
  message(FATAL_ERROR "Board '${XRDAP_BOARD}' sets no XRDAP_BOOT_KEY_PORT/XRDAP_BOOT_KEY_PIN")
endif()
set(XRDAP_BOOT_KEY_DEFINES XRDAP_BOOT_KEY_GPIO=${_gpio_${XRDAP_BOOT_KEY_PORT}}
    XRDAP_BOOT_KEY_PIN=${XRDAP_BOOT_KEY_PIN} XRDAP_BOOT_KEY_RCC_BIT=${_rcc_${XRDAP_BOOT_KEY_PORT}})

xrdap_add_platform_library()

set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ${XRDAP_CHIP_CONFIG_FILE})
add_subdirectory("${XRDAP_ROOT}/libxr" libxr)
target_include_directories(xr PUBLIC ${XRDAP_PLATFORM_INCLUDES})
target_compile_definitions(xr PUBLIC ${XRDAP_COMMON_DEFINES})

# Stage 1: assembly only, linked by itself at 0.
add_executable(stage1.elf stage1.S)
# Linked with the C driver: CMAKE_ASM_FLAGS carries -x assembler-with-cpp, which must not
# reach the link line.
set_target_properties(stage1.elf PROPERTIES LINKER_LANGUAGE C)
target_compile_definitions(stage1.elf PRIVATE ${XRDAP_COMMON_DEFINES} ${XRDAP_BOOT_KEY_DEFINES})
target_link_options(stage1.elf PRIVATE -T "${CMAKE_CURRENT_LIST_DIR}/stage1.ld" -nostartfiles
                    -nostdlib -Wl,--print-memory-usage
                    ${XRDAP_ARCH_FLAGS})
add_custom_command(TARGET stage1.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:stage1.elf> ${PROJECT_BINARY_DIR}/stage1.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:stage1.elf> ${PROJECT_BINARY_DIR}/stage1.bin
  VERBATIM)

# The bootloader, once per slot, at 96 MHz (common/sysclk.h). Each image ends before its
# slot's record.
math(EXPR _bl_image_size "${XRDAP_BL_SLOT_SIZE} - ${XRDAP_BL_RECORD_SIZE}" OUTPUT_FORMAT HEXADECIMAL)
foreach(_slot a b)
  if(_slot STREQUAL "a")
    set(_index 0)
    set(_base ${XRDAP_BL_SLOT_A_BASE})
  else()
    set(_index 1)
    set(_base ${XRDAP_BL_SLOT_B_BASE})
  endif()
  set(_target bootloader-${_slot}.elf)
  add_executable(${_target} main.c bootloader.cpp "${XRDAP_ROOT}/common/sysclk.c")
  target_compile_definitions(${_target} PRIVATE XRDAP_SYSCLK_HZ=96000000 XRDAP_BL_SLOT=${_index}
                             XRDAP_FIRMWARE_VERSION="${XRDAP_FIRMWARE_VERSION}"
                             XRDAP_VERSION_BCD=${XRDAP_VERSION_BCD} ${XRDAP_BOOT_KEY_DEFINES}
                             XRDAP_BL_PRODUCT="${XRDAP_BL_PRODUCT}")
  # Family names (vendor, serial prefix) from XRDAP-Core, header only; the chip UID is this
  # repository's common/chip_uid.hpp.
  target_include_directories(${_target} PRIVATE "${XRDAP_ROOT}/xrdap-core" "${XRDAP_ROOT}/common")
  target_link_libraries(${_target} PRIVATE platform xr)
  # LibXR allocates the DFU and USB buffers from the heap at start-up.
  xrdap_add_image(${_target} ${_base} ${_bl_image_size} ${XRDAP_BL_HEAP_SIZE})
endforeach()

if(XRDAP_BL_FIRST_SLOT STREQUAL "b")
  set(_first_base ${XRDAP_BL_SLOT_B_BASE})
else()
  set(_first_base ${XRDAP_BL_SLOT_A_BASE})
endif()

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/boot-chain.bin ${PROJECT_BINARY_DIR}/boot-chain.hex
           ${PROJECT_BINARY_DIR}/boot-chain-parts.bin
    COMMAND ${Python3_EXECUTABLE} "${XRDAP_ROOT}/tools/boot_chain.py"
            --stage1 ${PROJECT_BINARY_DIR}/stage1.bin
            --bootloader ${PROJECT_BINARY_DIR}/bootloader-${XRDAP_BL_FIRST_SLOT}.elf.bin
            --out ${PROJECT_BINARY_DIR}/boot-chain --stage1-size ${XRDAP_STAGE1_SIZE}
            --slot-base ${_first_base} --slot-size ${XRDAP_BL_SLOT_SIZE}
            --record-size ${XRDAP_BL_RECORD_SIZE}
    DEPENDS stage1.elf bootloader-${XRDAP_BL_FIRST_SLOT}.elf "${XRDAP_ROOT}/tools/boot_chain.py"
    VERBATIM)
  add_custom_target(boot_chain ALL DEPENDS ${PROJECT_BINARY_DIR}/boot-chain.bin)

  if(XRDAP_BOOT_MIGRATION)
  # Migration of a board with the old bootloader (migrate.c): an application for the old
  # layout (XRDAP_MIGRATE_BASE) that carries boot-chain-parts.bin (stage 1's sector and
  # the first slot).
  set(_migrate_defines MIGRATE_PAYLOAD=${PROJECT_BINARY_DIR}/boot-chain-parts.bin
      MIGRATE_SLOT_BASE=${_first_base})
  add_executable(migrate-to-ab.elf migrate.c migrate_payload.S)
  set_source_files_properties(migrate_payload.S PROPERTIES OBJECT_DEPENDS
                              ${PROJECT_BINARY_DIR}/boot-chain-parts.bin)
  target_compile_definitions(migrate-to-ab.elf PRIVATE ${_migrate_defines})
  target_link_libraries(migrate-to-ab.elf PRIVATE platform)
  add_dependencies(migrate-to-ab.elf boot_chain)
  xrdap_add_image(migrate-to-ab.elf ${XRDAP_MIGRATE_BASE} ${XRDAP_MIGRATE_SIZE} 0x400)

  # The same program linked at the application address of the A/B layout: a bootloader of
  # this layout takes it as an application, and it writes the boot chain again (repair
  # and bring-up tool; not part of the released images).
  add_executable(rewrite-boot-chain.elf migrate.c migrate_payload.S)
  target_compile_definitions(rewrite-boot-chain.elf PRIVATE ${_migrate_defines})
  target_link_libraries(rewrite-boot-chain.elf PRIVATE platform)
  add_dependencies(rewrite-boot-chain.elf boot_chain)
  math(EXPR _rewrite_size "${XRDAP_APP_SIZE} - ${XRDAP_SEAL_SIZE} - ${XRDAP_CONFIG_SIZE}" OUTPUT_FORMAT HEXADECIMAL)
  xrdap_add_image(rewrite-boot-chain.elf ${XRDAP_APP_BASE} ${_rewrite_size} 0x400)
  endif()
else()
  message(STATUS "No Python 3: boot-chain and migrate-to-ab are not built (tools/boot_chain.py)")
endif()
