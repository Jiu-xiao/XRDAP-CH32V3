#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the boot chain (stage 1, the bootloader for slots A and B, boot-chain and the
# migration image; bootloader/) and the application for one or more boards and collect
# the images in build/images/<board>/. Needs CMake and riscv32-wch-elf-gcc (WCH GCC 15)
# in PATH.
# Usage: tools/build.sh [board...]   (default: wdap wdap-144 v305 v305-144 linkw)
#
# 为一块或几块板子编译启动链（stage1、A/B 两个槽的 bootloader、boot-chain 和迁移镜像；
# bootloader/）和应用，镜像放在 build/images/<板子>/。需要 CMake 和 PATH 里的
# riscv32-wch-elf-gcc（WCH GCC 15）。
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
BOARDS=("$@")
[ ${#BOARDS[@]} -eq 0 ] && BOARDS=(wdap wdap-144 v305 v305-144 linkw)
JOBS=$(nproc 2>/dev/null || echo 4)

for board in "${BOARDS[@]}"; do
  images=(app-$board/xrdap.elf.bin app-$board/xrdap.elf.hex)
  cmake -S "$ROOT/bootloader" -B "$ROOT/build/bootloader-$board" -DXRDAP_BOARD="$board"
  cmake --build "$ROOT/build/bootloader-$board" -j"$JOBS"
  # boot-chain and the migration image need Python 3 (bootloader/CMakeLists.txt).
  for f in stage1.bin stage1.hex bootloader-a.elf.bin bootloader-a.elf.hex \
           bootloader-b.elf.bin bootloader-b.elf.hex boot-chain.bin boot-chain.hex \
           migrate-to-ab.elf.bin; do
    if [ -f "$ROOT/build/bootloader-$board/$f" ]; then
      images+=("bootloader-$board/$f")
    fi
  done
  cmake -S "$ROOT" -B "$ROOT/build/app-$board" -DXRDAP_BOARD="$board"
  cmake --build "$ROOT/build/app-$board" -j"$JOBS"
  out="$ROOT/build/images/$board"
  mkdir -p "$out"
  for f in "${images[@]}"; do
    cp "$ROOT/build/$f" "$out/"
  done
done
ls -l "$ROOT"/build/images/*
