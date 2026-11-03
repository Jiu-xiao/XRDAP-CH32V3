# Firmware version shared by the application and the bootloader: XRDAP_VERSION plus the
# commit it was built from, e.g. 0.2.0+g9c0860c (".dirty" for uncommitted changes). A
# build from a copy without .git passes the commit with -DXRDAP_GIT_REV=<short hash>[.dirty].
# The application reports it in DAP_Info, the bootloader in its USB strings and slot record.
set(XRDAP_VERSION 0.2.0)
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
# Version in BCD for the USB bcdDevice field: 0.2.0 -> 0x0020.
string(REPLACE "." ";" _version_parts "${XRDAP_VERSION}")
list(GET _version_parts 0 _major)
list(GET _version_parts 1 _minor)
list(GET _version_parts 2 _patch)
math(EXPR XRDAP_VERSION_BCD "(${_major} << 8) | (${_minor} << 4) | ${_patch}" OUTPUT_FORMAT HEXADECIMAL)
