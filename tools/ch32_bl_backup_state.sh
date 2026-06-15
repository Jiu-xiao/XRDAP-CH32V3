#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_backup_state_$(date -u +%Y%m%dT%H%M%SZ)}
mkdir -p "$RUN_DIR"
ch32_wlink_retry "$RUN_DIR/10_halt.log" halt
ch32_wlink_retry "$RUN_DIR/11_dump_stage1.log" dump -o "$RUN_DIR/stage1_08000000.bin" 0x08000000 256
ch32_wlink_retry "$RUN_DIR/12_dump_appslot.log" dump -o "$RUN_DIR/app_08002000.bin" 0x08002000 0x1df00
ch32_wlink_retry "$RUN_DIR/13_dump_bl.log" dump -o "$RUN_DIR/bl_08020000.bin" 0x08020000 87080
ch32_wlink_retry "$RUN_DIR/14_dump_seal.log" dump -o "$RUN_DIR/seal_0801ff00.bin" 0x0801ff00 256
sha256sum "$RUN_DIR"/*.bin > "$RUN_DIR/20_sha256.txt"
cat "$RUN_DIR/20_sha256.txt"
echo "$RUN_DIR"
