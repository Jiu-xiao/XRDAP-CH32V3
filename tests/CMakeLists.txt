cmake_minimum_required(VERSION 3.20)

# Tests of the boot layout (flash layout, boot slots) and of the parts of app/ that write
# registers, on the build machine with its own compiler. The radio link's logic is tested
# in XRDAP-Core.
#   cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests
project(xrdap_ch32v3_tests CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
add_executable(xrdap_tests boot_test.cpp)
target_include_directories(xrdap_tests PRIVATE "${XRDAP_ROOT}/common")
# The layout of cmake/ch32v3.cmake.
target_compile_definitions(xrdap_tests PRIVATE XRDAP_APP_BASE=0x0001B000
                           XRDAP_APP_SIZE=0x00025000 XRDAP_SEAL_SIZE=256
                           XRDAP_CONFIG_SIZE=2048 XRDAP_BL_SLOT_A_BASE=0x00001000
                           XRDAP_BL_SLOT_B_BASE=0x0000E000 XRDAP_BL_SLOT_SIZE=0x0000D000
                           XRDAP_BL_RECORD_SIZE=256)
target_compile_options(xrdap_tests PRIVATE -Wall -Wextra)

# tests/fake stands in for the WCH headers, LibXR's GPIO driver and XRDAP-Core's vendor
# extension.
add_executable(xrdap_app_tests app_test.cpp "${XRDAP_ROOT}/app/sampler_tick.cpp")
target_include_directories(xrdap_app_tests PRIVATE fake "${XRDAP_ROOT}/common"
                           "${XRDAP_ROOT}/app")
target_compile_options(xrdap_app_tests PRIVATE -Wall -Wextra)

enable_testing()
add_test(NAME xrdap_tests COMMAND xrdap_tests)
add_test(NAME xrdap_app_tests COMMAND xrdap_app_tests)
