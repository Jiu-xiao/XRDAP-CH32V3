// SPDX-License-Identifier: Apache-2.0
#pragma once

// Identity of this probe: product, firmware version and chip UID. The serial rule, the
// DAP_Info strings, the chip UID and the family USB ids come from XRDAP-Core
// (xrdap/probe): prefix + the 12-byte UID in upper-case hex, the same string the USB
// stack builds for the device serial.
//
// 本探针的身份：产品名、固件版本和芯片 UID。序列号规则、DAP_Info 字符串、芯片 UID 和全系
// 列的 USB 标识来自 XRDAP-Core（xrdap/probe）：前缀 + 12 字节 UID 的大写十六进制，与 USB
// 协议栈生成的设备序列号相同。

#include <cstdint>

#include "xrdap/probe/chip_uid.hpp"
#include "xrdap/probe/family.hpp"
#include "xrdap/probe/probe_identity.hpp"

namespace XrDap
{
/// 固件版本，由 CMake 给出（工程版本加提交号）/ Firmware version from CMake (project
/// version plus the commit).
inline constexpr char kFirmwareVersion[] = XRDAP_FIRMWARE_VERSION;
inline constexpr char kProductName[] = "XR-DAP-V307";
inline constexpr char kUsbProductName[] = "XR-DAP-V307 CMSIS-DAP";

/// 芯片 UID / The chip UID.
inline const uint8_t* ChipUid() { return F1ChipUid(); }
inline constexpr uint32_t kChipUidSize = kF1ChipUidBytes;
}  // namespace XrDap
