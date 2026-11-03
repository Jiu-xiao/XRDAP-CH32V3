// SPDX-License-Identifier: Apache-2.0
#pragma once

// Platform part of XRDAP-Core on the CH32V3 boards: the size of the sample frame buffer.
//
// XRDAP-Core 在 CH32V3 板子上的平台部分：采样帧缓冲的大小。

#include <cstdint>

namespace XrDap::Port
{
/// 采样帧缓冲字节数，2 的幂，INFO 会报告 / Sample frame buffer size, a power of two,
/// reported by INFO.
inline constexpr uint32_t kRingBytes = XRDAP_RING_BYTES;  // per board (cmake)
}  // namespace XrDap::Port
