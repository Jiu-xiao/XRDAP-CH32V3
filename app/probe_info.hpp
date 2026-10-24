// SPDX-License-Identifier: Apache-2.0
#pragma once

// Identity of this probe: product (per board, boards/<name>.hpp), firmware version and
// chip UID. The serial rule, the
// DAP_Info strings, the chip UID and the family USB ids come from XRDAP-Core
// (xrdap/probe): prefix + the 12-byte UID in upper-case hex, the same string the USB
// stack builds for the device serial.
//
// 本探针的身份：产品名（按板子，boards/<名字>.hpp）、固件版本和芯片 UID。序列号规则、
// DAP_Info 字符串、芯片 UID 和全系列的 USB 标识来自 XRDAP-Core（xrdap/probe）：前缀 +
// 12 字节 UID 的大写十六进制，与 USB 协议栈生成的设备序列号相同。

#include <cstdint>
#include <string_view>

#include XRDAP_BOARD_HEADER
#include "xrdap/probe/chip_uid.hpp"
#include "xrdap/probe/family.hpp"
#include "xrdap/probe/probe_identity.hpp"

namespace XrDap
{
/// 固件版本，由 CMake 给出（工程版本加提交号）/ Firmware version from CMake (project
/// version plus the commit).
inline constexpr char kFirmwareVersion[] = XRDAP_FIRMWARE_VERSION;
using Board::kProductName;
using Board::kUsbProductName;

/// @brief usb 是否为 product 加 " CMSIS-DAP"（protocol.md 第 4 节）/ Whether usb is
///        product plus " CMSIS-DAP" (protocol.md section 4).
constexpr bool IsUsbProductName(std::string_view usb, std::string_view product)
{
  constexpr std::string_view kSuffix = " CMSIS-DAP";
  return usb.size() == product.size() + kSuffix.size() &&
         usb.substr(0, product.size()) == product &&
         usb.substr(product.size()) == kSuffix && product.substr(0, 7) == "XR-DAP-";
}
static_assert(IsUsbProductName(kUsbProductName, kProductName),
              "the board's USB product string must be its product name + \" CMSIS-DAP\"");

/// 芯片 UID / The chip UID.
inline const uint8_t* ChipUid() { return F1ChipUid(); }
inline constexpr uint32_t kChipUidSize = kF1ChipUidBytes;
}  // namespace XrDap
