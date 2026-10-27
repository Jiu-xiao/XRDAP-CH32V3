#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the bootloader and the application for one or more boards and collect the images
# in build/images/<board>/. A board with the three-stage bootflow (XRDAP_EXTERNAL_BOOTLOADER
# ON, v305) gets bootloader-ch32v3 (stage1 + stage-3 bootloader) instead of bootloader/.
# Needs CMake and riscv32-wch-elf-gcc (WCH GCC 15) in PATH. Usage: tools/build.sh
# [board...]   (default: evt wdap)
#
# 为一块或几块板子编译 bootloader 和应用，镜像放在 build/images/<板子>/。三段式启动的
# 板子（XRDAP_EXTERNAL_BOOTLOADER ON，v305）编译 bootloader-ch32v3（stage1 + stage3
# bootloader），不编译 bootloader/。需要 CMake 和 PATH 里的 riscv32-wch-elf-gcc（WCH GCC
# 15）。
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
BOARDS=("$@")
[ ${#BOARDS[@]} -eq 0 ] && BOARDS=(evt wdap)
JOBS=$(nproc 2>/dev/null || echo 4)

for board in "${BOARDS[@]}"; do
  images=(app-$board/xrdap.elf.bin app-$board/xrdap.elf.hex)
  if grep -q "XRDAP_EXTERNAL_BOOTLOADER ON" "$ROOT/boards/$board.cmake"; then
    cmake -S "$ROOT/bootloader-ch32v3" -B "$ROOT/build/bootloader-ch32v3"
    cmake --build "$ROOT/build/bootloader-ch32v3" -j"$JOBS"
    images+=(bootloader-ch32v3/stage1.bin bootloader-ch32v3/stage1.hex
             bootloader-ch32v3/bootloader.bin bootloader-ch32v3/bootloader.hex)
  else
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
