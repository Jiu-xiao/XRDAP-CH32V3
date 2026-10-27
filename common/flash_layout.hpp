// SPDX-License-Identifier: Apache-2.0
#pragma once

// Flash layout shared by the bootloader and the application (cmake/ch32v3.cmake sets
// the numbers). CPU addresses start at 0; the flash controller sees the same flash at
// kFlashBase. Inside the application window, from its start:
//   [0, kConfigOffset)                  application image (DFU payload)
//   [kConfigOffset, kSealOffset)        settings (app/config.hpp), kept across updates
//   [kSealOffset, kAppSize)             DFU seal record, written by the bootloader
//
// Bootloader 与应用共用的 Flash 布局（数值由 cmake/ch32v3.cmake 给出）。CPU 地址从 0 开
// 始；Flash 控制器在 kFlashBase 处看到同一块 Flash。应用窗口内从头起依次为：应用镜像（DFU
// 下载内容）、设置区（app/config.hpp，升级时保留）、DFU seal 记录（bootloader 写）。

#include <cstdint>

namespace XrDap::Layout
{
/// Flash 控制器看到的 Flash 起始地址 / Flash address as seen by the flash controller.
inline constexpr uint32_t kFlashBase = 0x08000000U;
/// 应用窗口的 CPU 地址与大小 / CPU address and size of the application window.
inline constexpr uint32_t kAppBase = XRDAP_APP_BASE;
inline constexpr uint32_t kAppSize = XRDAP_APP_SIZE;
/// 应用窗口内 seal 记录的偏移 / Offset of the seal record in the application window.
inline constexpr uint32_t kSealOffset = XRDAP_APP_SIZE - XRDAP_SEAL_SIZE;
/// 应用窗口内设置区的偏移与大小 / Offset and size of the settings in the window.
inline constexpr uint32_t kConfigOffset = kSealOffset - XRDAP_CONFIG_SIZE;
inline constexpr uint32_t kConfigSize = XRDAP_CONFIG_SIZE;

/// @brief [offset, offset + size)（应用窗口内偏移）是否碰到设置区 / Whether
///        [offset, offset + size) (offsets in the application window) touches the
///        settings.
constexpr bool OverlapsConfig(uint32_t offset, uint32_t size)
{
  const uint64_t end = static_cast<uint64_t>(offset) + size;
  return size != 0U && offset < kConfigOffset + kConfigSize && end > kConfigOffset;
}
}  // namespace XrDap::Layout
