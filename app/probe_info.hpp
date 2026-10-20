// SPDX-License-Identifier: Apache-2.0
#pragma once

// Probe identity for DAP_Info and USB: serial = prefix + the 12-byte chip UID in
// upper-case hex, the same string the USB stack builds for the device serial, so host
// tools see one probe whichever way they reach it.
//
// 探针身份（DAP_Info 与 USB）：序列号 = 前缀 + 12 字节芯片 UID 的大写十六进制，与 USB
// 协议栈生成的设备序列号相同，上位机无论经哪条路都认作同一块探针。

#include <cstdint>

namespace XrDap
{

inline constexpr char kFirmwareVersion[] = "0.1.0";
inline constexpr char kVendorName[] = "XRobot";
inline constexpr char kProductName[] = "XR-DAP-V307";
inline constexpr char kUsbProductName[] = "XR-DAP-V307 CMSIS-DAP";
inline constexpr char kSerialPrefix[] = "XRobot-XRDAP-";

/// 芯片 UID 的地址与长度 / Address and length of the chip UID.
inline const uint8_t* ChipUid() { return reinterpret_cast<const uint8_t*>(0x1FFFF7E8U); }
inline constexpr uint32_t kChipUidSize = 12;

/// 序列号，例如 XRobot-XRDAP-68E3BE7B54507AAF39E339E3 / Serial, e.g.
/// XRobot-XRDAP-68E3BE7B54507AAF39E339E3.
const char* ProbeSerial();

/// @brief 把探针身份写入 DAP_Info / Write the probe identity into DAP_Info.
template <typename DapT>
void SetProbeInfo(DapT& dap, const char* board_name)
{
  dap.SetInfoStrings({
      kVendorName,       // vendor
      kProductName,      // product
      ProbeSerial(),     // serial
      "2.0.0",           // CMSIS-DAP protocol version
      nullptr,           // target device vendor (not fixed)
      nullptr,           // target device name
      kVendorName,       // board vendor
      board_name,        // board name
      kFirmwareVersion,  // probe firmware version
  });
}

}  // namespace XrDap
