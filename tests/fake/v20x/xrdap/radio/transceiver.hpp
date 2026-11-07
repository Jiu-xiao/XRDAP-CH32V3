// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 XRDAP-Core 的 xrdap/radio/transceiver.hpp，只有 V208Radio 用到的类型：
// 测试不取子模块。
//
// Stands in for XRDAP-Core's xrdap/radio/transceiver.hpp in the host tests, with only the
// types V208Radio uses: the tests do not fetch the submodule.

#include <cstdint>

namespace XrDap::Radio
{

enum class Rate : uint8_t
{
  kHigh = 0,
  kLow = 1,
};

struct LinkParams
{
  Rate rate = Rate::kHigh;
  uint32_t frequency_hz = 0;
  uint16_t crc_seed = 0;
  uint8_t address[4] = {};
};

}  // namespace XrDap::Radio
