#!/usr/bin/env bash
set -euo pipefail
CH32_WLINK_BIN=${CH32_WLINK_BIN:-/home/xiao/toolchains/wlink-v0.1.1/wlink-linux-x64/wlink}
CH32_WLINK_CHIP=${CH32_WLINK_CHIP:-CH32V30X}
ch32_wlink_retry() {
  local log_file=$1
  shift
  local attempt
  : > "$log_file"
  for attempt in 1 2 3; do
    if sudo -n "$CH32_WLINK_BIN" --chip "$CH32_WLINK_CHIP" "$@" >"$log_file" 2>&1; then
      return 0
    fi
    if ! grep -q "Resource busy" "$log_file"; then
      return 1
    fi
    sleep 1
  done
  return 1
}
