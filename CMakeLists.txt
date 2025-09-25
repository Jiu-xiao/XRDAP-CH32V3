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
set(COMPILER_PREFIX "riscv32-unknown-elf-")
set(CMAKE_C_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_LINKER ${COMPILER_PREFIX}gcc)
set(CMAKE_AR ${COMPILER_PREFIX}ar)
set(CMAKE_OBJCOPY ${COMPILER_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${COMPILER_PREFIX}objdump)
set(SIZE ${COMPILER_PREFIX}size)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

project(CH32V307VC)

# Enable languages
enable_language(C CXX ASM)

# Flags
set(CPU_FLAGS "-march=rv32imacf_zicsr_zifencei -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore")
set(COMMON_FLAGS "-Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -fsigned-char -fsingle-precision-constant -Wunused -Wuninitialized")

set(CMAKE_C_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} -fmessage-length=0" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} -fno-use-cxa-atexit -fno-exceptions -fno-rtti -fno-threadsafe-statics" CACHE INTERNAL "" FORCE)
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "" FORCE)

add_compile_options(-g)

# Link options
add_link_options(
  -nostartfiles
  -Xlinker --gc-sections
  -Wl,--print-memory-usage
  -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
  --specs=nano.specs
  --specs=nosys.specs
  -march=rv32imacf_zicsr_zifencei
  -mabi=ilp32f
  -flto
  -lm -lc
)

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Link.ld)

# Main executable
add_executable(${PROJECT_NAME}.elf)

target_link_options(${PROJECT_NAME}.elf PRIVATE -T ${LINKER_SCRIPT})
target_compile_options(${PROJECT_NAME}.elf PRIVATE -O3)

# Source files
file(GLOB CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/*.c")
file(GLOB USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/User/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp")

target_sources(${PROJECT_NAME}.elf
  PRIVATE
    ${CORE_SOURCES}
    ${USER_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_ch32v30x_D8C.S
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V/portASM.S
)

# Includes
target_include_directories(${PROJECT_NAME}.elf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/User
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
)

# Peripheral + FreeRTOS object library
file(GLOB PERIPHERAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/src/*.c")
file(GLOB FREERTOS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/MemMang/heap_4.c"
)

add_library(Peripheral OBJECT ${PERIPHERAL_SOURCES} ${FREERTOS_SOURCES})

target_include_directories(Peripheral PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/include
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/MemMang
)

target_compile_options(Peripheral PUBLIC -O3)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE Peripheral)

# LibXR
set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER ch)
set(LIBXR_CH32_CONFIG_FILE ch32v30x_conf.h)
add_subdirectory(libxr)

target_link_libraries(${PROJECT_NAME}.elf PUBLIC xr)

target_include_directories(xr PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Core
  ${CMAKE_CURRENT_SOURCE_DIR}/User
  ${CMAKE_CURRENT_SOURCE_DIR}/Peripheral/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/include
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/MemMang
)

target_compile_options(xr PRIVATE -O3)
# Output binary and hex
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
  COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)
