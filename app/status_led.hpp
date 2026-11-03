// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP status on a single plain LED, from XRDAP-Core's status light rule
// (xrdap/light/status_light.hpp) as far as one LED of one colour can show it: lit while
// USB is configured, dark without a link, and dark for the first half of each activity
// pulse, so it flickers under DAP traffic. The rule's colours and host states have no
// equivalent on it.
//
// 单个普通 LED 上的 DAP 状态，取自 XRDAP-Core 的状态灯规则
// （xrdap/light/status_light.hpp），一个单色灯能表示多少就表示多少：
// USB 已配置时亮，没有连接时灭，每个活动脉冲的前一半灭，所以有 DAP 流量时闪烁。
// 规则里的颜色和上位机状态在它上面没有对应。

#include "ch32_gpio.hpp"
#include "part.hpp"
#include "probe.hpp"
#include "xrdap/light/status_light.hpp"

namespace XrDap
{

class StatusLed final : public Part
{
 public:
  explicit StatusLed(const Probe& probe);

  bool Poll() override;

 private:
  void Write(bool lit);

  const Probe& probe_;
  LibXR::CH32GPIO led_;
  Light::StatusLight rule_;
};

}  // namespace XrDap
