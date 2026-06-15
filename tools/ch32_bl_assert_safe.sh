#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_assert_safe_$(date -u +%Y%m%dT%H%M%SZ)}
mkdir -p "$RUN_DIR"
EXPECT_STAGE1=71d82955d8e0953113d6a687d316dd0e3a3f0231959c5b7c80c04c3e467e849f
EXPECT_BL=47e723530704f14771b2128b09a91cdcc522632f8a6bfd7a908f54efec6e6e1f
ch32_wlink_retry "$RUN_DIR/10_halt.log" halt
ch32_wlink_retry "$RUN_DIR/11_dump_stage1.log" dump -o "$RUN_DIR/stage1_read.bin" 0x08000000 256
ch32_wlink_retry "$RUN_DIR/12_dump_bl.log" dump -o "$RUN_DIR/bl_read.bin" 0x08020000 87080
sha_stage1=$(sha256sum "$RUN_DIR/stage1_read.bin" | awk '{print $1}')
sha_bl=$(sha256sum "$RUN_DIR/bl_read.bin" | awk '{print $1}')
ch32_wlink_retry "$RUN_DIR/13_reset.log" reset || true
sudo -n python3 "$SCRIPT_DIR/ch32_usb_wait.py" --vid 0x1209 --pid 0x3072 --timeout 15 > "$RUN_DIR/14_wait_bl.log" || true
ok=1
[[ "$sha_stage1" == "$EXPECT_STAGE1" ]] || ok=0
[[ "$sha_bl" == "$EXPECT_BL" ]] || ok=0
grep -q 'FOUND 1209:3072' "$RUN_DIR/14_wait_bl.log" || ok=0
{
  echo "stage1_sha=$sha_stage1"
  echo "bl_sha=$sha_bl"
  echo '== wait_bl =='
  cat "$RUN_DIR/14_wait_bl.log"
  echo "SAFE_BASELINE=$ok"
} | tee "$RUN_DIR/99_summary.txt"
if [[ "$ok" == 1 ]]; then
  echo "$RUN_DIR"
else
  exit 1
fi
