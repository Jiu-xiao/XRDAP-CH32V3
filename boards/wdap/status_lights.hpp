// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP status on the WS2812 chain of the wdap board, drawn by XRDAP-Core's status light
// rule (xrdap/light/status_light.hpp). Only the inputs are decided here: wireless while a
// debug session runs over the radio, wired while USB is configured, otherwise no link;
// pairing is a setting being changed. The whole chain shows the rule's colour, and each
// activity pulse moves a white comet one LED on. A frame goes out every 20 ms.
//
// wdap 板 WS2812 灯串上的 DAP 状态，由 XRDAP-Core 的状态灯规则
// （xrdap/light/status_light.hpp）算出。这里只决定输入：
// 调试会话经无线运行时为无线，USB 已配置时为有线，否则没有连接；配对算正在改设置。
// 整串灯显示规则的颜色，每个活动脉冲让一颗白色彗星前进一颗灯。每 20 ms 发一帧。

#include <cstdint>

#include "ch32_spi.hpp"
#include "part.hpp"
#include "probe.hpp"
#include "radio_part.hpp"
#include "sx1281_radio.hpp"
#include "ws2812.hpp"
#include "xrdap/light/status_light.hpp"

namespace XrDap
{

class StatusLights final : public Part
{
 public:
  StatusLights(const Probe& probe, const RadioPart<RadioDriver>& radio);

  bool Poll() override;

 private:
  static constexpr uint8_t kCount = Board::kStatusLightCount;
  static_assert(kCount <= Ws2812::kMaxLeds, "the chain is longer than the driver frame");

  void Draw(uint32_t now);

  const Probe& probe_;
  const RadioPart<RadioDriver>& radio_;
  uint8_t spi_rx_[Ws2812::kFrameBytes] = {};
  uint8_t spi_tx_[Ws2812::kFrameBytes] = {};
  LibXR::CH32SPI spi_;
  Ws2812 chain_;
  Light::StatusLight rule_;
  Light::Rgb leds_[kCount] = {};
  uint8_t comet_ = 0;
  uint16_t last_pulse_ = 0;
  uint32_t next_ms_;
};

}  // namespace XrDap
