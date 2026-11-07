#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build the firmwares of CMakePresets.json (all of them, or the presets named) and collect
# the images in build/images/<preset>/: the application and the board's boot chain (stage 1,
# the bootloader for slots A and B, boot-chain and, where the board has one, the migration
# image). Needs CMake 3.21 or newer, riscv32-wch-elf-gcc (WCH GCC 15) in PATH and, for
# boot-chain and the migration image, Python 3. XRDAP_GIT_REV in the environment replaces
# the commit in the version string (a build from a copy without .git).
# Usage: tools/build.sh [preset...]
#
# 编译 CMakePresets.json 里的固件（全部，或命令行列出的 preset），镜像放在
# build/images/<preset>/：应用和这块板的启动链（stage1、A/B 两个槽的 bootloader、
# boot-chain，板子有迁移镜像时还有它）。需要 CMake 3.21 以上、PATH 里的
# riscv32-wch-elf-gcc（WCH GCC 15），boot-chain 和迁移镜像还要 Python 3。环境变量
# XRDAP_GIT_REV 代替版本号里的提交号（从没有 .git 的副本编译时用）。
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
PRESETS=("$@")
if [ ${#PRESETS[@]} -eq 0 ]; then
  mapfile -t PRESETS < <(cmake --list-presets=configure | sed -n 's/^ *"\([^"]*\)".*/\1/p')
fi
JOBS=$(nproc 2>/dev/null || echo 4)
REV=()
[ -n "${XRDAP_GIT_REV+set}" ] && REV=(-DXRDAP_GIT_REV="$XRDAP_GIT_REV")

for preset in "${PRESETS[@]}"; do
  cmake --preset "$preset" "${REV[@]}"
  cmake --build --preset "$preset" -j"$JOBS"
  out="build/images/$preset"
  rm -rf "$out"
  mkdir -p "$out"
  cp "build/$preset/xrdap.elf.bin" "build/$preset/xrdap.elf.hex" "$out/"
  for f in stage1.bin stage1.hex bootloader-a.elf.bin bootloader-a.elf.hex \
           bootloader-b.elf.bin bootloader-b.elf.hex boot-chain.bin boot-chain.hex \
           migrate-to-ab.elf.bin; do
    if [ -f "build/$preset/bootloader/$f" ]; then
      cp "build/$preset/bootloader/$f" "$out/"
    fi
  done
done
ls -l build/images/*
