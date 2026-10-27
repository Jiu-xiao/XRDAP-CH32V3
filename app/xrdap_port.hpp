// SPDX-License-Identifier: Apache-2.0
#pragma once

// Platform part of XRDAP-Core: the size of the sample frame buffer and its spinlock. The
// CH32V307 has 64 KB of RAM and one core, so the buffer is 4 KB and a FreeRTOS critical
// section is the lock.
//
// XRDAP-Core 的平台部分：采样帧缓冲的大小和它的自旋锁。CH32V307 只有 64 KB RAM、一个核，
// 缓冲取 4 KB，锁用 FreeRTOS 临界区。

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

namespace XrDap::Vendor
{
class TaskSampling;
}  // namespace XrDap::Vendor

namespace XrDap::Port
{
/// 采样在 RTOS 任务里 / Sampling runs in an RTOS task.
using SamplerPolicy = Vendor::TaskSampling;

/// 采样帧缓冲字节数，2 的幂，INFO 会报告 / Sample frame buffer size, a power of two,
/// reported by INFO.
inline constexpr uint32_t kRingBytes = XRDAP_RING_BYTES;  // per board (cmake)

/// Short critical section, held for at most a 1 KB copy / 短临界区，持有时间不超过拷贝
/// 1 KB。
class SpinLock
{
 public:
  void Lock() { taskENTER_CRITICAL(); }
  void Unlock() { taskEXIT_CRITICAL(); }
};
}  // namespace XrDap::Port
