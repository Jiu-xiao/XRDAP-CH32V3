cmake_minimum_required(VERSION 3.20)

# Tests of the boot layout (flash layout, boot slots), of the parts of app/ that write
# registers and of V208Radio, on the build machine with its own compiler. The radio link's
# logic is tested in XRDAP-Core.
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
                           "${XRDAP_ROOT}/app" "${XRDAP_ROOT}/platform")
target_compile_options(xrdap_app_tests PRIVATE -Wall -Wextra)

# V208Radio against the transceiver contract, with its registers as memory at their real
# addresses (mmap, Linux only).
find_package(Threads REQUIRED)
add_executable(xrdap_v208_radio_tests v208_radio_test.cpp)
target_include_directories(xrdap_v208_radio_tests PRIVATE fake/v20x
                           "${XRDAP_ROOT}/platform/ch32v20x")
target_compile_options(xrdap_v208_radio_tests PRIVATE -Wall -Wextra
                       -Wno-int-to-pointer-cast)
target_link_libraries(xrdap_v208_radio_tests PRIVATE Threads::Threads)

enable_testing()
add_test(NAME xrdap_tests COMMAND xrdap_tests)
add_test(NAME xrdap_app_tests COMMAND xrdap_app_tests)
add_test(NAME xrdap_v208_radio_tests COMMAND xrdap_v208_radio_tests)
