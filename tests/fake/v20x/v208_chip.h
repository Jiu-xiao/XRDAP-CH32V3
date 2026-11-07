// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 platform/ch32v20x/xrdap_chip.h，只有 V208Radio 用到的部分：HSE
// 校准寄存器 和 BLE 时钟的开关。v208_radio.hpp 和真的头文件在同一目录，#include
// 总是先找到它，所以这里 占住它的头文件卫士，并且要先于 v208_radio.hpp 包含。
//
// Stands in for platform/ch32v20x/xrdap_chip.h in the host tests, with only what
// V208Radio uses: the HSE calibration register and the switch of the BLE clock.
// v208_radio.hpp sits next to the real header and its #include always finds that one
// first, so this file holds the real header's guard and comes before v208_radio.hpp.

#define XRDAP_CHIP_H_

#include <cstdint>

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;

#define RCC_AHBPeriph_BLE_CRC 0x00030040U

struct OSC_TypeDef
{
  volatile uint32_t HSE_CAL_CTRL;
};

namespace Fake
{
inline OSC_TypeDef osc{};
inline uint32_t ahb_clocks = 0U;
}  // namespace Fake

#define OSC (&Fake::osc)

inline void RCC_AHBPeriphClockCmd(uint32_t periph, FunctionalState state)
{
  if (state != DISABLE)
  {
    Fake::ahb_clocks |= periph;
  }
}
