// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 XRDAP-Core 的厂商扩展，只有节拍定时器接口，并数节拍。
//
// Stands in for XRDAP-Core's vendor extension in the host tests: the tick timer interface
// only, and a count of the ticks.

#include <cstdint>

namespace XrDap
{

namespace Vendor
{
class TickTimer
{
 public:
  virtual ~TickTimer() = default;
  virtual void Start(uint32_t period_us) = 0;
  virtual void Stop() = 0;
};
}  // namespace Vendor

class DapVendor
{
 public:
  void OnTickIsr() { ++ticks; }

  uint32_t ticks = 0U;
};

}  // namespace XrDap
