#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_restore_app_and_seal_$(date -u +%Y%m%dT%H%M%SZ)}
APP_BIN=${2:?need app bin}
SEAL_BIN=${3:?need seal bin}
mkdir -p "$RUN_DIR"
ch32_wlink_retry "$RUN_DIR/10_halt.log" halt
ch32_wlink_retry "$RUN_DIR/11_flash_app.log" flash -a 0x08002000 "$APP_BIN"
ch32_wlink_retry "$RUN_DIR/12_flash_seal.log" flash -a 0x0801ff00 "$SEAL_BIN"
ch32_wlink_retry "$RUN_DIR/13_reset.log" reset || true
sudo -n python3 "$SCRIPT_DIR/ch32_usb_wait.py" --vid 0x1209 --pid 0x3072 --timeout 20 > "$RUN_DIR/14_wait_bl.log" || true
{
  echo '== wait_bl =='
  cat "$RUN_DIR/14_wait_bl.log"
} | tee "$RUN_DIR/99_summary.txt"
echo "$RUN_DIR"
