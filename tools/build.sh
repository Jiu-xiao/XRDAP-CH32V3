#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the bootloader and the application for one or more boards and collect the images
# in build/images/<board>/. A board that keeps another project's bootloader
# (XRDAP_EXTERNAL_BOOTLOADER ON, e.g. v305) gets the application only. Needs CMake and
# riscv32-wch-elf-gcc (WCH GCC 15) in PATH. Usage: tools/build.sh [board...]   (default:
# evt wdap)
#
# 为一块或几块板子编译 bootloader 和应用，镜像放在 build/images/<板子>/。沿用别的工程
# bootloader 的板子（XRDAP_EXTERNAL_BOOTLOADER ON，如 v305）只编译应用。需要 CMake 和
# PATH 里的 riscv32-wch-elf-gcc（WCH GCC 15）。
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
BOARDS=("$@")
[ ${#BOARDS[@]} -eq 0 ] && BOARDS=(evt wdap)
JOBS=$(nproc 2>/dev/null || echo 4)

for board in "${BOARDS[@]}"; do
  images=(app-$board/xrdap.elf.bin app-$board/xrdap.elf.hex)
  if ! grep -q "XRDAP_EXTERNAL_BOOTLOADER ON" "$ROOT/boards/$board.cmake"; then
    cmake -S "$ROOT/bootloader" -B "$ROOT/build/bootloader-$board" -DXRDAP_BOARD="$board"
    cmake --build "$ROOT/build/bootloader-$board" -j"$JOBS"
    images+=(bootloader-$board/bootloader.elf.bin bootloader-$board/bootloader.elf.hex)
  fi
  cmake -S "$ROOT" -B "$ROOT/build/app-$board" -DXRDAP_BOARD="$board"
  cmake --build "$ROOT/build/app-$board" -j"$JOBS"
  out="$ROOT/build/images/$board"
  mkdir -p "$out"
  for f in "${images[@]}"; do
    cp "$ROOT/build/$f" "$out/"
  done
done
ls -l "$ROOT"/build/images/*
