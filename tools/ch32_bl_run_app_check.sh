#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_run_app_check_$(date -u +%Y%m%dT%H%M%SZ)}
APP_VID=${2:-0x0D28}
APP_PID=${3:-0x2041}
mkdir -p "$RUN_DIR"
sudo -n python3 "$SCRIPT_DIR/ch32_usb_run_app.py" > "$RUN_DIR/10_run_app.log" 2>&1 || true
sudo -n python3 "$SCRIPT_DIR/ch32_usb_wait.py" --vid "$APP_VID" --pid "$APP_PID" --timeout 20 > "$RUN_DIR/11_wait_app.log" || true
ch32_wlink_retry "$RUN_DIR/12_reset.log" reset || true
sudo -n python3 "$SCRIPT_DIR/ch32_usb_wait.py" --vid 0x1209 --pid 0x3072 --timeout 20 > "$RUN_DIR/13_wait_bl_again.log" || true
{
  echo '== run_app =='
  cat "$RUN_DIR/10_run_app.log"
  echo '== wait_app =='
  cat "$RUN_DIR/11_wait_app.log"
  echo '== wait_bl_again =='
  cat "$RUN_DIR/13_wait_bl_again.log"
} | tee "$RUN_DIR/99_summary.txt"
echo "$RUN_DIR"
