cmake_minimum_required(VERSION 3.20)

# Tests of the pure logic (link protocol, session rules, pair identity, DAP forwarding,
# settings log, flash layout) on the build machine, with its own compiler:
#   cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests
project(xrdap_ch32v3_tests CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(XRDAP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
add_executable(xrdap_tests
               link_test.cpp
               "${XRDAP_ROOT}/app/dap_forwarding.cpp"
               "${XRDAP_ROOT}/app/link_protocol.cpp"
               "${XRDAP_ROOT}/app/link_session.cpp"
               "${XRDAP_ROOT}/app/pair_identity.cpp"
               "${XRDAP_ROOT}/app/settings_log.cpp")
# The DAP command ids come from libxr's CMSIS-DAP definitions through XRDAP-Core
# (xrdap/protocol/dap_cmd.hpp), so the tests need libxr's headers.
set(LIBXR_SYSTEM linux)
set(LIBXR_DRIVER linux)
add_subdirectory("${XRDAP_ROOT}/libxr" libxr EXCLUDE_FROM_ALL)
file(GLOB_RECURSE LIBXR_DAP_HEADERS "${XRDAP_ROOT}/libxr/src/driver/usb/device/dap/*.hpp")
foreach(header ${LIBXR_DAP_HEADERS})
  get_filename_component(dir "${header}" DIRECTORY)
  list(APPEND LIBXR_DAP_DIRS "${dir}")
endforeach()
list(REMOVE_DUPLICATES LIBXR_DAP_DIRS)
target_include_directories(xrdap_tests PRIVATE "${XRDAP_ROOT}/app" "${XRDAP_ROOT}/common"
                                               "${XRDAP_ROOT}/xrdap-core" ${LIBXR_DAP_DIRS})
target_link_libraries(xrdap_tests PRIVATE xr)
# The layout of cmake/ch32v3.cmake.
target_compile_definitions(xrdap_tests PRIVATE XRDAP_APP_BASE=0x00010000
                           XRDAP_APP_SIZE=0x00030000 XRDAP_SEAL_SIZE=256
                           XRDAP_CONFIG_SIZE=2048)
target_compile_options(xrdap_tests PRIVATE -Wall -Wextra)
enable_testing()
add_test(NAME xrdap_tests COMMAND xrdap_tests)
