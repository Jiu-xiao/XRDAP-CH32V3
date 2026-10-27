// SPDX-License-Identifier: Apache-2.0
#pragma once

// The 96-bit unique id of the CH32V3 (CH32V3 reference manual, "electronic signature").
// It is the id part of the probe's USB serial and the pairing hello.
//
// CH32V3 的 96 位芯片唯一标识（CH32V3 参考手册“电子签名”），是探针 USB 序列号和配对
// 握手里的标识部分。

#include <cstdint>

namespace XrDap
{

/// 唯一标识的字节数 / Bytes of the unique id.
inline constexpr uint32_t kChipUidSize = 12U;

/// 唯一标识（只读，可在任何上下文读）/ The unique id (read-only, readable from any
/// context).
inline const uint8_t* ChipUid() { return reinterpret_cast<const uint8_t*>(0x1FFFF7E8U); }

}  // namespace XrDap
