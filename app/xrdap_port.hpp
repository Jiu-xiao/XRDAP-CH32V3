// SPDX-License-Identifier: Apache-2.0
#pragma once

// Platform part of XRDAP-Core: the spinlock of the sample frame buffer. The CH32V307 has
// one core, so a FreeRTOS critical section is enough.
//
// XRDAP-Core 的平台部分：采样帧缓冲用的自旋锁。CH32V307 只有一个核，用 FreeRTOS 临界区
// 即可。

#include "FreeRTOS.h"
#include "task.h"

namespace XrDap::Port
{
/// Short critical section, held for at most a 1 KB copy / 短临界区，持有时间不超过拷贝
/// 1 KB。
class SpinLock
{
 public:
  void Lock() { taskENTER_CRITICAL(); }
  void Unlock() { taskEXIT_CRITICAL(); }
};
}  // namespace XrDap::Port
