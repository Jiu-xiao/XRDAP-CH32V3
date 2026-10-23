cmake_minimum_required(VERSION 3.20)

# Tests of the pure logic (link protocol, session rules, pair identity, DAP forwarding,
# settings log, flash layout) on the build machine, with its own compiler:
#   cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests
project(xrdap_v307_tests CXX)
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
# The DAP command ids come from XRDAP-Core (xrdap/protocol/dap_ids.hpp).
target_include_directories(xrdap_tests PRIVATE "${XRDAP_ROOT}/app" "${XRDAP_ROOT}/common"
                                               "${XRDAP_ROOT}/xrdap-core")
# The layout of cmake/ch32v307.cmake.
target_compile_definitions(xrdap_tests PRIVATE XRDAP_APP_BASE=0x00010000
                           XRDAP_APP_SIZE=0x00030000 XRDAP_SEAL_SIZE=256
                           XRDAP_CONFIG_SIZE=2048)
target_compile_options(xrdap_tests PRIVATE -Wall -Wextra)
enable_testing()
add_test(NAME xrdap_tests COMMAND xrdap_tests)
