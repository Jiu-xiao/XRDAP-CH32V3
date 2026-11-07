# Shared CH32 build setup for the application (CMakeLists.txt) and the boot chain
# (bootloader/CMakeLists.txt): toolchain, board and chip selection, defaults, flash layout,
# version, compiler flags, the WCH SDK library and the image helper. Each project includes
# this file before project().
#
# Board: XRDAP_BOARD names a directory in boards/: board.cmake (crystal, core clock, chip
# family, memory, flash layout, boot key, the bootloader's LED (XRDAP_HAS_LED) and the
# board's sources XRDAP_BOARD_SOURCES), board.hpp (pins and timing used by the firmware) and
# board.cpp (the application's composition). The presets in CMakePresets.json name the
# firmwares built from them; wdap-144 and v305-144 set XRDAP_APP_SYSCLK_HZ to 144 MHz.
# Chip family: XRDAP_CHIP_FAMILY in board.cmake selects chips/<family>/ (WCH SDK, startup
# file, and chip.cmake with the compiler flags); the default is ch32v30x.

if(NOT DEFINED XRDAP_ROOT)
  get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# WCH RISC-V GCC 15.2 or newer (riscv32-wch-elf-gcc) on PATH, or a prefix that
# includes the directory, e.g. "/opt/wch/bin/riscv32-wch-elf-".
set(COMPILER_PREFIX "riscv32-wch-elf-" CACHE STRING "Cross compiler prefix")
set(CMAKE_C_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_OBJCOPY ${COMPILER_PREFIX}objcopy)
set(CMAKE_SIZE ${COMPILER_PREFIX}size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Board: a preset (cmake --preset wdap; cmake --list-presets) or -DXRDAP_BOARD=<directory>.
set(XRDAP_BOARD "" CACHE STRING "Board directory in boards/")
set_property(CACHE XRDAP_BOARD PROPERTY STRINGS wdap v305 linkw)
if(NOT XRDAP_BOARD)
  message(FATAL_ERROR "No board: cmake --preset <name> (cmake --list-presets), or -DXRDAP_BOARD=wdap|v305|linkw")
endif()
set(XRDAP_BOARD_DIR "${XRDAP_ROOT}/boards/${XRDAP_BOARD}")
if(NOT EXISTS "${XRDAP_BOARD_DIR}/board.cmake")
  message(FATAL_ERROR "Unknown XRDAP_BOARD '${XRDAP_BOARD}'")
endif()
include("${XRDAP_BOARD_DIR}/board.cmake")
if(NOT DEFINED XRDAP_CHIP_FAMILY)
  set(XRDAP_CHIP_FAMILY ch32v30x)
endif()
set(XRDAP_CHIP_DIR "${XRDAP_ROOT}/chips/${XRDAP_CHIP_FAMILY}")
include("${XRDAP_CHIP_DIR}/chip.cmake")
# Application core clock (common/sysclk.h); the bootloader runs at 96 MHz.
if(NOT DEFINED XRDAP_APP_SYSCLK_HZ)
  set(XRDAP_APP_SYSCLK_HZ 144000000)
endif()
# Chip memory and the application's share of it. The defaults are the CH32V307's 256K
# FLASH + 64K SRAM split; a board on a smaller chip sets its own values.
#   XRDAP_RAM_SIZE        SRAM linked
#   XRDAP_MAIN_STACK_SIZE stack of the single loop (main() runs the application and the
#                         interrupts run on the same stack)
#   XRDAP_C_HEAP_SIZE     newlib heap of the application (operator new takes the LibXR
#                         objects built at start-up from it)
#   XRDAP_DAP_PACKET_COUNT CMSIS-DAP packet count (1024-byte packets)
#   XRDAP_RING_BYTES      sampling ring buffer (reported by vendor INFO)
if(NOT DEFINED XRDAP_BOOT_REQUEST_MAGIC0)
  set(XRDAP_BOOT_REQUEST_MAGIC0 0x50414458)  # "XDAP"
  set(XRDAP_BOOT_REQUEST_MAGIC1 0x55464442)  # "BDFU"
endif()
if(NOT DEFINED XRDAP_RAM_SIZE)
  set(XRDAP_RAM_SIZE 0x10000)
endif()
if(NOT DEFINED XRDAP_MAIN_STACK_SIZE)
  set(XRDAP_MAIN_STACK_SIZE 4096)
endif()
if(NOT DEFINED XRDAP_C_HEAP_SIZE)
  set(XRDAP_C_HEAP_SIZE 0x1000)
endif()
if(NOT DEFINED XRDAP_DAP_PACKET_COUNT)
  set(XRDAP_DAP_PACKET_COUNT 4)
endif()
if(NOT DEFINED XRDAP_RING_BYTES)
  set(XRDAP_RING_BYTES 4096)
endif()

# Flash layout (CPU addresses; the flash is also mapped at 0x08000000). Stage 1 at 0
# picks one of two bootloader slots (common/boot_slots.hpp). The last
# XRDAP_BL_RECORD_SIZE bytes of each slot hold its slot record, the last XRDAP_SEAL_SIZE
# bytes of the application window the DFU seal record and the XRDAP_CONFIG_SIZE bytes in
# front of it the application's settings (app/settings_store.hpp). The defaults are the
# CH32V307's: the application follows the slots,
#   0x00000-0x00FFF stage 1, 0x01000-0x0DFFF bootloader slot A, 0x0E000-0x1AFFF slot B,
#   0x1B000-0x3FFFF application window (settings at 0x3F700, seal at 0x3FF00).
# A board may set its own slot and application addresses (v305) and:
#   XRDAP_BL_FIRST_SLOT   slot (a or b) that boot-chain and migrate-to-ab write
#   XRDAP_MIGRATE_BASE/SIZE  application window and image limit of the old bootloader,
#                         where migrate-to-ab is linked
#   XRDAP_BOOT_MIGRATION  OFF on a board that never ran the old bootloader: no
#                         migrate-to-ab and no rewrite-boot-chain
#   XRDAP_BL_HEAP_SIZE    heap of the bootloader
#   XRDAP_BL_PRODUCT      USB product string of the bootloader, without the version
set(XRDAP_STAGE1_BASE 0x00000000)
set(XRDAP_STAGE1_SIZE 0x00001000)
if(NOT DEFINED XRDAP_BL_SLOT_A_BASE)
  set(XRDAP_BL_SLOT_A_BASE 0x00001000)
  set(XRDAP_BL_SLOT_B_BASE 0x0000E000)
endif()
set(XRDAP_BL_SLOT_SIZE 0x0000D000)
set(XRDAP_BL_RECORD_SIZE 256)
if(NOT DEFINED XRDAP_BL_FIRST_SLOT)
  set(XRDAP_BL_FIRST_SLOT a)
endif()
if(NOT DEFINED XRDAP_BOOT_MIGRATION)
  set(XRDAP_BOOT_MIGRATION ON)
endif()
if(NOT DEFINED XRDAP_MIGRATE_BASE)
  set(XRDAP_MIGRATE_BASE 0x00010000)
  set(XRDAP_MIGRATE_SIZE 0x0002F700)
endif()
if(NOT DEFINED XRDAP_BL_HEAP_SIZE)
  set(XRDAP_BL_HEAP_SIZE 0x8000)
endif()
if(NOT DEFINED XRDAP_BL_PRODUCT)
  set(XRDAP_BL_PRODUCT "XR-DAP-V307 DFU")
endif()
if(NOT DEFINED XRDAP_APP_BASE)
  set(XRDAP_APP_BASE 0x0001B000)
  set(XRDAP_APP_SIZE 0x00025000)
endif()
set(XRDAP_SEAL_SIZE 256)
set(XRDAP_CONFIG_SIZE 2048)

list(JOIN XRDAP_ARCH_FLAGS " " _arch_flags)
set(XRDAP_CPU_FLAGS "${_arch_flags} -msmall-data-limit=8 -mno-save-restore")

macro(xrdap_setup_flags opt_level)
  set(_common "-Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -fsigned-char")
  set(_common "${_common} -fsingle-precision-constant -fno-unwind-tables")
  set(_common "${_common} -fno-asynchronous-unwind-tables -${opt_level} -g")
  set(CMAKE_C_FLAGS "${XRDAP_CPU_FLAGS} ${_common}" CACHE INTERNAL "" FORCE)
  set(CMAKE_CXX_FLAGS
      "${XRDAP_CPU_FLAGS} ${_common} -fno-use-cxa-atexit -fno-exceptions -fno-rtti -fno-threadsafe-statics"
      CACHE INTERNAL "" FORCE)
  set(CMAKE_ASM_FLAGS "${XRDAP_CPU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "" FORCE)
  set(CMAKE_BUILD_TYPE "" CACHE STRING "" FORCE)
endmacro()

# WCH GCC 15 has no Zaamo-only multilib; link against the compatible library directories
# (XRDAP_MULTILIB_FLAGS) without changing the ISA used for compilation.
macro(xrdap_setup_libraries)
  foreach(_lib libgcc.a libc_nano.a)
    execute_process(
      COMMAND ${CMAKE_C_COMPILER} ${XRDAP_MULTILIB_FLAGS} -print-file-name=${_lib}
      OUTPUT_VARIABLE _path OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
    get_filename_component(_dir "${_path}" DIRECTORY)
    link_directories(BEFORE "${_dir}")
  endforeach()
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS 15.2)
    message(FATAL_ERROR "XRDAP-CH32V3 needs WCH RISC-V GCC 15.2 or newer")
  endif()
endmacro()

# Defines shared by every target: chip family, crystal and the layout.
set(XRDAP_COMMON_DEFINES
    ${XRDAP_CHIP_DEFINE}
    HSE_VALUE=${XRDAP_HSE_HZ}
    XRDAP_APP_SYSCLK_HZ=${XRDAP_APP_SYSCLK_HZ}
    XRDAP_APP_BASE=${XRDAP_APP_BASE}
    XRDAP_APP_SIZE=${XRDAP_APP_SIZE}
    XRDAP_SEAL_SIZE=${XRDAP_SEAL_SIZE}
    XRDAP_CONFIG_SIZE=${XRDAP_CONFIG_SIZE}
    XRDAP_STAGE1_SIZE=${XRDAP_STAGE1_SIZE}
    XRDAP_BL_SLOT_A_BASE=${XRDAP_BL_SLOT_A_BASE}
    XRDAP_BL_SLOT_B_BASE=${XRDAP_BL_SLOT_B_BASE}
    XRDAP_BL_SLOT_SIZE=${XRDAP_BL_SLOT_SIZE}
    XRDAP_BL_RECORD_SIZE=${XRDAP_BL_RECORD_SIZE}
    XRDAP_RAM_SIZE=${XRDAP_RAM_SIZE}
    XRDAP_BOARD_HEADER="board.hpp")
if(XRDAP_HAS_LED)
  list(APPEND XRDAP_COMMON_DEFINES XRDAP_HAS_LED)
endif()
list(APPEND XRDAP_COMMON_DEFINES XRDAP_BOOT_REQUEST_MAGIC0=${XRDAP_BOOT_REQUEST_MAGIC0}u
     XRDAP_BOOT_REQUEST_MAGIC1=${XRDAP_BOOT_REQUEST_MAGIC1}u)

# Include directories of every target: the chip family's SDK, the board, common/.
set(XRDAP_INCLUDES
    "${XRDAP_CHIP_DIR}"
    "${XRDAP_CHIP_DIR}/Core"
    "${XRDAP_CHIP_DIR}/Peripheral/inc"
    "${XRDAP_BOARD_DIR}"
    "${XRDAP_ROOT}/common")

# The chip family's WCH SDK (core files, peripheral library, startup file) and the newlib
# stubs, built once per project.
macro(xrdap_add_chip_library)
  file(GLOB _peripheral_sources "${XRDAP_CHIP_DIR}/Peripheral/src/*.c")
  file(GLOB _core_sources "${XRDAP_CHIP_DIR}/Core/*.c")
  add_library(chip OBJECT ${_peripheral_sources} ${_core_sources}
              "${XRDAP_CHIP_DIR}/${XRDAP_STARTUP_FILE}"
              "${XRDAP_ROOT}/common/syscalls.c")
  target_include_directories(chip PUBLIC ${XRDAP_INCLUDES})
  target_compile_definitions(chip PUBLIC ${XRDAP_COMMON_DEFINES})
endmacro()

# Links `target` into the flash window [base, base + size) with a C heap of
# heap_size bytes (operator new on bare metal, newlib malloc) and emits .hex/.bin.
function(xrdap_add_image target base size heap_size)
  target_link_options(${target} PRIVATE
    -T "${XRDAP_ROOT}/chips/Link.ld"
    -Wl,--defsym=IMAGE_FLASH_BASE=${base}
    -Wl,--defsym=IMAGE_FLASH_LENGTH=${size}
    -Wl,--defsym=HEAP_SIZE=${heap_size}
    -Wl,--defsym=RAM_SIZE=${XRDAP_RAM_SIZE}
    -Wl,--defsym=STACK_SIZE=${XRDAP_MAIN_STACK_SIZE}
    -nostartfiles -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_BINARY_DIR}/${target}.map
    --specs=nano.specs --specs=nosys.specs
    ${XRDAP_ARCH_FLAGS})
  target_link_libraries(${target} PRIVATE m c)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
    VERBATIM)
endfunction()

# Firmware version shared by the application and the bootloader: XRDAP_VERSION plus the
# commit it was built from, e.g. 0.6.0+g9c0860c (".dirty" for uncommitted changes). A
# build from a copy without .git passes the commit with -DXRDAP_GIT_REV=<short hash>[.dirty].
# The application reports it in DAP_Info, the bootloader in its USB strings and slot record.
set(XRDAP_VERSION 0.6.0)
if(NOT DEFINED XRDAP_GIT_REV)
  execute_process(COMMAND git rev-parse --short=7 HEAD
                  WORKING_DIRECTORY "${XRDAP_ROOT}"
                  OUTPUT_VARIABLE XRDAP_GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE
                  ERROR_QUIET RESULT_VARIABLE _git_result)
  if(_git_result EQUAL 0)
    execute_process(COMMAND git status --porcelain --untracked-files=no
                    WORKING_DIRECTORY "${XRDAP_ROOT}"
                    OUTPUT_VARIABLE _git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
    if(_git_dirty)
      string(APPEND XRDAP_GIT_REV ".dirty")
    endif()
  else()
    set(XRDAP_GIT_REV "")
  endif()
endif()
set(XRDAP_FIRMWARE_VERSION "${XRDAP_VERSION}")
if(XRDAP_GIT_REV)
  string(APPEND XRDAP_FIRMWARE_VERSION "+g${XRDAP_GIT_REV}")
endif()
# Version in BCD for the USB bcdDevice field: 0.6.0 -> 0x0060.
string(REPLACE "." ";" _version_parts "${XRDAP_VERSION}")
list(GET _version_parts 0 _major)
list(GET _version_parts 1 _minor)
list(GET _version_parts 2 _patch)
math(EXPR XRDAP_VERSION_BCD "(${_major} << 8) | (${_minor} << 4) | ${_patch}" OUTPUT_FORMAT HEXADECIMAL)
