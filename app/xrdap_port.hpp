// SPDX-License-Identifier: Apache-2.0
#pragma once

// Platform part of XRDAP-Core on the CH32V3 boards: the size of the sample frame buffer, the
// sampling policy and the lock. The probe runs in a single loop without an RTOS: the main
// loop samples between two commands, so nothing is shared between contexts and the lock is
// empty.
//
// XRDAP-Core 在 CH32V3 板子上的平台部分：采样帧缓冲的大小、采样策略和锁。探针在单个循环里
// 运行，没有 RTOS：主循环在两条命令之间采样，没有跨上下文共享的状态，锁是空的。

#include <cstdint>

namespace XrDap::Vendor
{
class LoopSampling;
}  // namespace XrDap::Vendor

namespace XrDap::Port
{
/// 单循环采样：主循环在两条命令之间调用 Poll / Single-loop sampling: the main loop calls
/// Poll between two commands.
using SamplerPolicy = Vendor::LoopSampling;

/// 采样帧缓冲字节数，2 的幂，INFO 会报告 / Sample frame buffer size, a power of two,
/// reported by INFO.
inline constexpr uint32_t kRingBytes = XRDAP_RING_BYTES;  // per board (cmake)

/// 空锁 / No-op lock.
class SpinLock
{
 public:
  void Lock() {}
  void Unlock() {}
};
}  // namespace XrDap::Port
