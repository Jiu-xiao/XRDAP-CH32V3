// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 LibXR 的时基：每读一次过去 100 µs，V208Radio 的等待都能结束。 / Stands
// in for LibXR's time base in the host tests: every read takes 100 µs, so that
// V208Radio's waits end.

#include <cstdint>

namespace LibXR
{

class Timebase
{
 public:
  static uint64_t GetMicroseconds()
  {
    now_us += 100U;
    return now_us;
  }

  static inline uint64_t now_us = 0U;
};

}  // namespace LibXR
