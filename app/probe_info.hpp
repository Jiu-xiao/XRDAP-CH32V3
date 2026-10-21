// SPDX-License-Identifier: Apache-2.0
#pragma once

// Identity of this probe: product, firmware version and chip UID. The serial rule and the
// DAP_Info strings come from XRDAP-Core's probe_identity.hpp: prefix + the 12-byte UID
// in upper-case hex, the same string the USB stack builds for the device serial.
//
// 本探针的身份：产品名、固件版本和芯片 UID。序列号规则和 DAP_Info 字符串来自
// XRDAP-Core 的 probe_identity.hpp：前缀 + 12 字节 UID 的大写十六进制，与 USB 协议栈
// 生成的设备序列号相同。

#include <cstdint>

#include "chip_uid.h"
#include "probe_identity.hpp"

namespace XrDap
{

/// 固件版本，由 CMake 给出（工程版本加提交号）/ Firmware version from CMake (project
/// version plus the commit).
inline constexpr char kFirmwareVersion[] = XRDAP_FIRMWARE_VERSION;
inline constexpr char kProductName[] = "XR-DAP-V307";
inline constexpr char kUsbProductName[] = "XR-DAP-V307 CMSIS-DAP";

/// 芯片 UID 的地址与长度 / Address and length of the chip UID.
inline const uint8_t* ChipUid()
{
  return reinterpret_cast<const uint8_t*>(XRDAP_CHIP_UID_ADDR);
}
inline constexpr uint32_t kChipUidSize = XRDAP_CHIP_UID_SIZE;

}  // namespace XrDap
