# Shared CH32V307 build setup for the bootloader and the application projects:
# toolchain, CPU flags, board selection, the WCH peripheral library and the image
# helpers. Each project includes this file before project().
#
# Board: -DXRDAP_BOARD=<name> selects boards/<name>.cmake (HSE crystal, core clock,
# which parts the board has: XRDAP_HAS_LED, XRDAP_HAS_RADIO, XRDAP_HAS_STATUS_LIGHTS,
# XRDAP_HAS_STATUS_LED, XRDAP_EXTERNAL_BOOTLOADER, and the memory of its chip) and
# boards/<name>.hpp (pins used by the firmware).

if(NOT DEFINED XRDAP_ROOT)
  get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()
set(XRDAP_PLATFORM_DIR "${XRDAP_ROOT}/platform")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_CXX_STANDARD 17)
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

set(XRDAP_BOARD "evt" CACHE STRING "Board configuration in boards/")
set_property(CACHE XRDAP_BOARD PROPERTY STRINGS evt wdap v305)
if(NOT EXISTS "${XRDAP_ROOT}/boards/${XRDAP_BOARD}.cmake")
  message(FATAL_ERROR "Unknown XRDAP_BOARD '${XRDAP_BOARD}'")
endif()
include("${XRDAP_ROOT}/boards/${XRDAP_BOARD}.cmake")
# Application core clock (common/sysclk.h); the bootloader runs at 96 MHz.
if(NOT DEFINED XRDAP_APP_SYSCLK_HZ)
  set(XRDAP_APP_SYSCLK_HZ 144000000)
endif()
# Chip memory and the application's share of it. The defaults are the CH32V307's 256K
# FLASH + 64K SRAM split; a board on a smaller chip sets its own values.
#   XRDAP_EXTERNAL_BOOTLOADER ON: the board keeps the bootloader of another project; the
#                         board gives the application window (XRDAP_APP_BASE,
#                         XRDAP_APP_SIZE) and the boot mailbox magics
#                         (XRDAP_BOOT_REQUEST_MAGIC0/1), and bootloader/ is not built
#   XRDAP_RAM_SIZE        SRAM linked
#   XRDAP_RTOS_HEAP_SIZE  FreeRTOS heap (task stacks and the LibXR objects)
#   XRDAP_APP_STACK_WORDS stack of the application task (app/main.c)
#   XRDAP_MAIN_STACK_SIZE stack of main() before the scheduler starts (interrupts then
#                         run on the FreeRTOS ISR stack)
#   XRDAP_C_HEAP_SIZE     newlib heap of the application (operator new uses the FreeRTOS
#                         heap, so nothing in the application allocates from it)
#   XRDAP_DAP_PACKET_COUNT CMSIS-DAP packet count (1024-byte packets)
#   XRDAP_RING_BYTES      sampling ring buffer (reported by vendor INFO)
if(NOT DEFINED XRDAP_EXTERNAL_BOOTLOADER)
  set(XRDAP_EXTERNAL_BOOTLOADER OFF)
endif()
if(NOT DEFINED XRDAP_BOOT_REQUEST_MAGIC0)
  set(XRDAP_BOOT_REQUEST_MAGIC0 0x50414458)  # "XDAP"
  set(XRDAP_BOOT_REQUEST_MAGIC1 0x55464442)  # "BDFU"
endif()
if(NOT DEFINED XRDAP_RAM_SIZE)
  set(XRDAP_RAM_SIZE 0x10000)
endif()
if(NOT DEFINED XRDAP_RTOS_HEAP_SIZE)
  set(XRDAP_RTOS_HEAP_SIZE 34816)
endif()
if(NOT DEFINED XRDAP_APP_STACK_WORDS)
  set(XRDAP_APP_STACK_WORDS 1536)
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

# Flash layout (CPU addresses; the flash is also mapped at 0x08000000). The
# bootloader starts at 0 and the application right after it; the last
# XRDAP_SEAL_SIZE bytes of the application window hold the DFU seal record and the
# XRDAP_CONFIG_SIZE bytes in front of it the application's settings (app/config.hpp).
set(XRDAP_BOOTLOADER_BASE 0x00000000)
set(XRDAP_BOOTLOADER_SIZE 0x00010000)
if(NOT XRDAP_EXTERNAL_BOOTLOADER)
  set(XRDAP_APP_BASE 0x00010000)
  set(XRDAP_APP_SIZE 0x00030000)
endif()
set(XRDAP_SEAL_SIZE 256)
set(XRDAP_CONFIG_SIZE 2048)

set(XRDAP_CPU_FLAGS
    "-march=rv32imcf_zicsr_zifencei_zaamo -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore")

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

# WCH GCC 15 has no Zaamo-only ilp32f multilib; link against the compatible
# hard-float library directories without changing the ISA used for compilation.
macro(xrdap_setup_libraries)
  foreach(_lib libgcc.a libc_nano.a)
    execute_process(
      COMMAND ${CMAKE_C_COMPILER} -march=rv32imafc_zaamo_zalrsc -mabi=ilp32f
              -print-file-name=${_lib}
      OUTPUT_VARIABLE _path OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
    get_filename_component(_dir "${_path}" DIRECTORY)
    link_directories(BEFORE "${_dir}")
  endforeach()
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS 15.2)
    message(FATAL_ERROR "XR-DAP-V307 needs WCH RISC-V GCC 15.2 or newer")
  endif()
endmacro()

# Defines shared by every target: chip family, crystal and the layout.
set(XRDAP_COMMON_DEFINES
    CH32V30x_D8C
    HSE_VALUE=${XRDAP_HSE_HZ}
    XRDAP_APP_SYSCLK_HZ=${XRDAP_APP_SYSCLK_HZ}
    XRDAP_APP_BASE=${XRDAP_APP_BASE}
    XRDAP_APP_SIZE=${XRDAP_APP_SIZE}
    XRDAP_SEAL_SIZE=${XRDAP_SEAL_SIZE}
    XRDAP_CONFIG_SIZE=${XRDAP_CONFIG_SIZE}
    XRDAP_RTOS_HEAP_SIZE=${XRDAP_RTOS_HEAP_SIZE}
    XRDAP_RAM_SIZE=${XRDAP_RAM_SIZE}
    XRDAP_BOARD_HEADER="${XRDAP_BOARD}.hpp")
if(XRDAP_HAS_LED)
  list(APPEND XRDAP_COMMON_DEFINES XRDAP_HAS_LED)
endif()
list(APPEND XRDAP_COMMON_DEFINES XRDAP_BOOT_REQUEST_MAGIC0=${XRDAP_BOOT_REQUEST_MAGIC0}u
     XRDAP_BOOT_REQUEST_MAGIC1=${XRDAP_BOOT_REQUEST_MAGIC1}u)

set(XRDAP_PLATFORM_INCLUDES
    "${XRDAP_PLATFORM_DIR}/Core"
    "${XRDAP_PLATFORM_DIR}/Peripheral/inc"
    "${XRDAP_ROOT}/boards"
    "${XRDAP_ROOT}/common")

# WCH core files and peripheral library, built once per project.
macro(xrdap_add_platform_library)
  file(GLOB _peripheral_sources "${XRDAP_PLATFORM_DIR}/Peripheral/src/*.c")
  file(GLOB _core_sources "${XRDAP_PLATFORM_DIR}/Core/*.c")
  add_library(platform OBJECT ${_peripheral_sources} ${_core_sources}
              "${XRDAP_PLATFORM_DIR}/startup_ch32v30x_D8C.S"
              "${XRDAP_PLATFORM_DIR}/syscalls.c")
  target_include_directories(platform PUBLIC ${XRDAP_PLATFORM_INCLUDES})
  target_compile_definitions(platform PUBLIC ${XRDAP_COMMON_DEFINES})
endmacro()

# Links `target` into the flash window [base, base + size) with a C heap of
# heap_size bytes (operator new on bare metal, newlib malloc) and emits .hex/.bin.
function(xrdap_add_image target base size heap_size)
  target_link_options(${target} PRIVATE
    -T "${XRDAP_PLATFORM_DIR}/Link.ld"
    -Wl,--defsym=IMAGE_FLASH_BASE=${base}
    -Wl,--defsym=IMAGE_FLASH_LENGTH=${size}
    -Wl,--defsym=HEAP_SIZE=${heap_size}
    -Wl,--defsym=RAM_SIZE=${XRDAP_RAM_SIZE}
    -Wl,--defsym=STACK_SIZE=${XRDAP_MAIN_STACK_SIZE}
    -nostartfiles -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_BINARY_DIR}/${target}.map
    --specs=nano.specs --specs=nosys.specs
    -march=rv32imcf_zicsr_zifencei_zaamo -mabi=ilp32f)
  target_link_libraries(${target} PRIVATE m c)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
    VERBATIM)
endfunction()
