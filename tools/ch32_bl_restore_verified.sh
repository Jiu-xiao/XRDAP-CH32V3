#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_restore_verified_$(date -u +%Y%m%dT%H%M%SZ)}
mkdir -p "$RUN_DIR"
STAGE1=/home/xiao/runs/xrdap_cdcuart_hw_verify_20260317T212400Z/stage1_pad256.bin
BL=/home/xiao/runs/xrdap_cdcuart_hw_verify_20260317T212400Z/build/CH32V305FBP6_bl.bin
ch32_wlink_retry "$RUN_DIR/10_halt.log" halt
ch32_wlink_retry "$RUN_DIR/11_flash_stage1.log" flash -a 0x08000000 "$STAGE1"
ch32_wlink_retry "$RUN_DIR/12_flash_bl.log" flash -a 0x08020000 "$BL"
ch32_wlink_retry "$RUN_DIR/13_reset.log" reset || true
sudo -n python3 "$SCRIPT_DIR/ch32_usb_wait.py" --vid 0x1209 --pid 0x3072 --timeout 20 > "$RUN_DIR/14_wait_bl.log" || true
sudo -n "$WL" --chip CH32V30X dump -o "$RUN_DIR/stage1_read.bin" 0x08000000 256 > "$RUN_DIR/15_dump_stage1.log" 2>&1
sudo -n "$WL" --chip CH32V30X dump -o "$RUN_DIR/bl_read.bin" 0x08020000 87080 > "$RUN_DIR/16_dump_bl.log" 2>&1
sha256sum "$STAGE1" "$RUN_DIR/stage1_read.bin" > "$RUN_DIR/20_stage1_sha.txt"
sha256sum "$BL" "$RUN_DIR/bl_read.bin" > "$RUN_DIR/21_bl_sha.txt"
{
  echo '== wait_bl =='
  cat "$RUN_DIR/14_wait_bl.log"
  echo '== stage1 sha =='
  cat "$RUN_DIR/20_stage1_sha.txt"
  echo '== bl sha =='
  cat "$RUN_DIR/21_bl_sha.txt"
} | tee "$RUN_DIR/99_summary.txt"
echo "$RUN_DIR"
