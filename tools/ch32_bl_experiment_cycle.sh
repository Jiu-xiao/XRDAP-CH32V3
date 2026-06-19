#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ch32_wlink_common.sh"
RUN_DIR=${1:-/home/xiao/runs/ch32_bl_experiment_cycle_$(date -u +%Y%m%dT%H%M%SZ)}
APP_BIN=${2:?need candidate app bin}
APP_VID=${3:-0x0D28}
APP_PID=${4:-0x2041}
RESTORE_BACKUP=${RESTORE_BACKUP:-0}
mkdir -p "$RUN_DIR"
ASSERT_DIR="$RUN_DIR/00_assert_safe"
BACKUP_DIR="$RUN_DIR/00_backup"
RESTORE_DIR="$RUN_DIR/30_restore_backup"
cleanup_restore() {
  if [[ "$RESTORE_BACKUP" != "0" && -d "$BACKUP_DIR" && ! -f "$RUN_DIR/30_restore_path.txt" ]]; then
    "$SCRIPT_DIR/ch32_bl_restore_app_and_seal.sh" \
      "$RESTORE_DIR" \
      "$BACKUP_DIR/app_08002000.bin" \
      "$BACKUP_DIR/seal_0801ff00.bin" > "$RUN_DIR/30_restore_path.txt" 2>&1 || true
  fi
}
trap 'rc=$?; if [[ $rc -ne 0 ]]; then cleanup_restore; fi; exit $rc' EXIT
"$SCRIPT_DIR/ch32_bl_assert_safe.sh" "$ASSERT_DIR" > "$RUN_DIR/00_assert_path.txt"
# 1. Backup current stable state before touching app slot.
"$SCRIPT_DIR/ch32_bl_backup_state.sh" "$BACKUP_DIR" > "$RUN_DIR/00_backup_path.txt"
# 2. Flash candidate app while keeping board in BL-first policy.
FLASH_DIR="$RUN_DIR/10_flash_candidate"
"$SCRIPT_DIR/ch32_bl_flash_app_only.sh" "$FLASH_DIR" "$APP_BIN" > "$RUN_DIR/10_flash_path.txt"
# 3. Verify manual RUN_APP works and reset returns to BL.
RUNAPP_DIR="$RUN_DIR/20_run_app_check"
"$SCRIPT_DIR/ch32_bl_run_app_check.sh" "$RUNAPP_DIR" "$APP_VID" "$APP_PID" > "$RUN_DIR/20_runapp_path.txt"
# 4. Optional rollback to the exact pre-experiment app+seal contents.
if [[ "$RESTORE_BACKUP" != "0" ]]; then
  RESTORE_DIR="$RUN_DIR/30_restore_backup"
  "$SCRIPT_DIR/ch32_bl_restore_app_and_seal.sh" \
    "$RESTORE_DIR" \
    "$BACKUP_DIR/app_08002000.bin" \
    "$BACKUP_DIR/seal_0801ff00.bin" > "$RUN_DIR/30_restore_path.txt"
fi
{
  echo '== assert safe =='
  cat "$ASSERT_DIR/99_summary.txt"
  echo '== backup sha =='
  cat "$BACKUP_DIR/20_sha256.txt"
  echo '== flash candidate =='
  cat "$FLASH_DIR/99_summary.txt"
  echo '== run app check =='
  cat "$RUNAPP_DIR/99_summary.txt"
  if [[ -f "$RUN_DIR/30_restore_path.txt" ]]; then
    echo '== restore backup =='
    cat "$RESTORE_DIR/99_summary.txt"
  fi
} | tee "$RUN_DIR/99_summary.txt"
trap - EXIT
echo "$RUN_DIR"
