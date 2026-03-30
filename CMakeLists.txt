cmake_minimum_required(VERSION 3.19)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# Standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Toolchain
set(CH32_TOOLCHAIN_BIN_DIR "/home/xiao/toolchains/wch-gcc15-v240/Toolchain/RISC-V Embedded GCC15/bin" CACHE PATH
    "Optional toolchain bin directory")
set(COMPILER_PREFIX "riscv32-wch-elf-" CACHE STRING "Cross compiler prefix")

if(CH32_TOOLCHAIN_BIN_DIR)
  file(TO_CMAKE_PATH "${CH32_TOOLCHAIN_BIN_DIR}" CH32_TOOLCHAIN_BIN_DIR)
  set(_CH32_TOOLCHAIN_PREFIX "${CH32_TOOLCHAIN_BIN_DIR}/${COMPILER_PREFIX}")
else()
  set(_CH32_TOOLCHAIN_PREFIX "${COMPILER_PREFIX}")
endif()

set(CMAKE_C_COMPILER ${_CH32_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${_CH32_TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${_CH32_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_LINKER ${_CH32_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_AR ${_CH32_TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${_CH32_TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${_CH32_TOOLCHAIN_PREFIX}objdump)
set(SIZE ${_CH32_TOOLCHAIN_PREFIX}size)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

project(CH32V305FBP6)

# Enable languages

# Board-level HSE crystal on WCH-LinkE is 12MHz.
add_compile_definitions(HSE_VALUE=12000000)

enable_language(C CXX ASM)

# Flags
set(CH32_CPU_ARCH "rv32imafc_xw" CACHE STRING
    "Target ISA passed to -march")
set(CH32_CPU_ABI "ilp32f" CACHE STRING
    "Target ABI passed to -mabi")
option(CH32_USE_SAVE_RESTORE "Enable -msave-restore for WCH toolchain" OFF)
if(CH32_USE_SAVE_RESTORE)
  set(CH32_SAVE_RESTORE_FLAG "-msave-restore")
else()
  set(CH32_SAVE_RESTORE_FLAG "-mno-save-restore")
endif()
set(CPU_FLAGS "-march=${CH32_CPU_ARCH} -mabi=${CH32_CPU_ABI} -msmall-data-limit=8 ${CH32_SAVE_RESTORE_FLAG}")
set(COMMON_FLAGS "-Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -fsigned-char -fsingle-precision-constant -Wunused -Wuninitialized -fno-unwind-tables -fno-asynchronous-unwind-tables")

option(CH32_AGGRESSIVE_OPTIMIZE "Enable aggressive compile optimization" ON)
set(CH32_OPT_LEVEL "O3" CACHE STRING "Optimization level: Og/O2/O3/Ofast/Os")
set_property(CACHE CH32_OPT_LEVEL PROPERTY STRINGS Og O2 O3 Ofast Os)
option(CH32_LIBXR_DEBUG_ASSERT "Enable libxr runtime asserts (LIBXR_DEBUG_BUILD)" OFF)

if(CH32_AGGRESSIVE_OPTIMIZE)
  set(CH32_OPT_FLAGS "-${CH32_OPT_LEVEL} -flto -fomit-frame-pointer -fno-math-errno -fno-trapping-math -g0")
else()
  set(CH32_OPT_FLAGS "-Og -g")
endif()

set(CMAKE_C_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${CH32_OPT_FLAGS} -fmessage-length=0" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${CH32_OPT_FLAGS} -fno-use-cxa-atexit -fno-exceptions -fno-rtti -fno-threadsafe-statics" CACHE INTERNAL "" FORCE)
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp -g0" CACHE INTERNAL "" FORCE)

# Keep CMake's default build type from injecting host flags in cross builds.
set(CMAKE_BUILD_TYPE "" CACHE STRING "" FORCE)

# Link options
add_link_options(
  -nostartfiles
  -Xlinker --gc-sections
  -Wl,--print-memory-usage
  -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
  --specs=nano.specs
  --specs=nosys.specs
  -march=${CH32_CPU_ARCH}
  -mabi=${CH32_CPU_ABI}
  -flto
  -lm -lc
)

# Linker scripts and image layout
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Link.ld)
set(STAGE1_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Link.stage1.ld)

# Current project default targets the verified app slot behind the BL-first flow.
# Override these cache entries explicitly if a pure single-image layout is needed.
set(CH32_APP_FLASH_BASE 0x00002000 CACHE STRING
    "Application flash base address")
set(CH32_APP_FLASH_LENGTH 0x0001E000 CACHE STRING
    "Application flash length")

# Optional legacy 3-stage switches (disabled by default).
option(CH32_BUILD_STAGE1_TRAMPOLINE "Build stage-1 trampoline target" OFF)
option(CH32_BUILD_STAGE3_BOOTLOADER "Build stage-3 custom USB bootloader target" OFF)
set(CH32_STAGE3_BL_BASE 0x00020000 CACHE STRING
    "Stage-3 bootloader base address (extra flash)")
set(CH32_STAGE3_BL_LENGTH 0x00030000 CACHE STRING
    "Stage-3 bootloader length (extra flash)")
set(CH32_BOOT_USB_VID 0x1209 CACHE STRING
    "Stage-3 bootloader USB VID")
set(CH32_BOOT_USB_PID 0x3070 CACHE STRING
    "Stage-3 bootloader USB PID")
set(CH32_BOOT_USB_BCD 0x0001 CACHE STRING
    "Stage-3 bootloader USB bcdDevice")
option(CH32_BOOT_ENABLE_AUX_CDC
       "Enable temporary dual-CDC composite mode in stage-3 bootloader"
       OFF)
option(CH32_BOOT_RECLOCK_48M
       "Reclock stage-3 bootloader runtime SYSCLK to 48MHz"
       ON)
option(CH32_BOOT_AUTORUN
       "Automatically jump to app when the app image is valid"
       OFF)
set(CH32_APP_USE_CUSTOM_BOOTLOADER_DEFAULT ON)
if(CH32_BUILD_STAGE3_BOOTLOADER)
  set(CH32_APP_USE_CUSTOM_BOOTLOADER_DEFAULT ON)
endif()
option(CH32_APP_USE_CUSTOM_BOOTLOADER
       "Route app 'iap/bl' command to custom stage-3 bootloader"
       ${CH32_APP_USE_CUSTOM_BOOTLOADER_DEFAULT})

math(EXPR CH32_APP_FLASH_LIMIT "${CH32_APP_FLASH_BASE} + ${CH32_APP_FLASH_LENGTH}")

# Source files
file(GLOB CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/*.c")
set(APP_USER_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/User/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/app_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/User/syscalls.c
)
set(BOOT_USER_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/User/boot_main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/bootloader_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/User/syscalls.c
)
set(STAGE1_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/User/stage1_jump.S
)

# Peripheral object library
file(GLOB PERIPHERAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/src/*.c")
add_library(Peripheral OBJECT ${PERIPHERAL_SOURCES})
target_compile_definitions(Peripheral PUBLIC CH32V30x_D8C)

target_include_directories(Peripheral PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
)

# LibXR
set(LIBXR_SYSTEM None)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory(libxr)
if(CH32_LIBXR_DEBUG_ASSERT)
  target_compile_definitions(xr PUBLIC LIBXR_DEBUG_BUILD)
endif()

target_include_directories(xr PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/User
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
)
target_compile_definitions(xr PRIVATE CH32V30x_D8C)

function(add_image_outputs target_name output_stem)
  set(HEX_FILE ${PROJECT_BINARY_DIR}/${output_stem}.hex)
  set(BIN_FILE ${PROJECT_BINARY_DIR}/${output_stem}.bin)
  add_custom_command(TARGET ${target_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${BIN_FILE}
    COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
  )
endfunction()

if(CH32_BUILD_STAGE1_TRAMPOLINE)
  # Stage-1: minimal trampoline at 0x00000000, only jumps to stage-3 BL.
  add_executable(${PROJECT_NAME}_stage1.elf)
  target_sources(${PROJECT_NAME}_stage1.elf PRIVATE ${STAGE1_SOURCES})
  set_target_properties(${PROJECT_NAME}_stage1.elf PROPERTIES LINKER_LANGUAGE C)
  target_link_options(${PROJECT_NAME}_stage1.elf PRIVATE
    -T ${STAGE1_LINKER_SCRIPT}
  )
  target_compile_definitions(${PROJECT_NAME}_stage1.elf PRIVATE
    CH32_STAGE3_BL_BASE=${CH32_STAGE3_BL_BASE}
  )
  add_image_outputs(${PROJECT_NAME}_stage1.elf ${PROJECT_NAME}_stage1)
endif()

# Single application image.
add_executable(${PROJECT_NAME}.elf)
target_sources(${PROJECT_NAME}.elf
  PRIVATE
    ${CORE_SOURCES}
    ${APP_USER_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_ch32v30x_D8C.S
)
target_include_directories(${PROJECT_NAME}.elf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/User
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
)
target_link_options(${PROJECT_NAME}.elf PRIVATE
  -T ${LINKER_SCRIPT}
  -Wl,--defsym=APP_FLASH_BASE=${CH32_APP_FLASH_BASE}
  -Wl,--defsym=APP_FLASH_LENGTH=${CH32_APP_FLASH_LENGTH}
)
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
  CH32V30x_D8C
  CH32_APP_FLASH_BASE=${CH32_APP_FLASH_BASE}
)
if(CH32_APP_USE_CUSTOM_BOOTLOADER)
  target_compile_definitions(${PROJECT_NAME}.elf PRIVATE CH32_APP_USE_CUSTOM_BOOTLOADER=1)
else()
  target_compile_definitions(${PROJECT_NAME}.elf PRIVATE CH32_APP_USE_CUSTOM_BOOTLOADER=0)
endif()
target_link_libraries(${PROJECT_NAME}.elf PRIVATE Peripheral xr)
add_image_outputs(${PROJECT_NAME}.elf ${PROJECT_NAME})

if(CH32_BUILD_STAGE3_BOOTLOADER)
  # Stage-3: optional USB bootloader in extra flash.
  add_executable(${PROJECT_NAME}_bl.elf)
  target_sources(${PROJECT_NAME}_bl.elf
    PRIVATE
      ${CORE_SOURCES}
      ${BOOT_USER_SOURCES}
      ${CMAKE_CURRENT_SOURCE_DIR}/startup_ch32v30x_D8C.S
  )
  target_include_directories(${PROJECT_NAME}_bl.elf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
  )
  target_link_options(${PROJECT_NAME}_bl.elf PRIVATE
    -T ${LINKER_SCRIPT}
    -Wl,--defsym=APP_FLASH_BASE=${CH32_STAGE3_BL_BASE}
    -Wl,--defsym=APP_FLASH_LENGTH=${CH32_STAGE3_BL_LENGTH}
  )
  target_compile_definitions(${PROJECT_NAME}_bl.elf PRIVATE
    CH32V30x_D8C
    CH32_BOOT_APP_BASE=${CH32_APP_FLASH_BASE}
    CH32_BOOT_APP_LIMIT=${CH32_APP_FLASH_LIMIT}
    CH32_BOOT_USB_VID=${CH32_BOOT_USB_VID}
    CH32_BOOT_USB_PID=${CH32_BOOT_USB_PID}
    CH32_BOOT_USB_BCD=${CH32_BOOT_USB_BCD}
    CH32_BOOT_ENABLE_AUX_CDC=$<IF:$<BOOL:${CH32_BOOT_ENABLE_AUX_CDC}>,1,0>
    CH32_BOOT_RECLOCK_48M=$<IF:$<BOOL:${CH32_BOOT_RECLOCK_48M}>,1,0>
    CH32_BOOT_AUTORUN=$<IF:$<BOOL:${CH32_BOOT_AUTORUN}>,1,0>
    SYSCLK_FREQ_144MHz_HSE=144000000
  )
  target_link_libraries(${PROJECT_NAME}_bl.elf PRIVATE Peripheral xr)
  add_image_outputs(${PROJECT_NAME}_bl.elf ${PROJECT_NAME}_bl)
endif()
