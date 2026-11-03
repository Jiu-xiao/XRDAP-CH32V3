// SPDX-License-Identifier: Apache-2.0
#pragma once

// Tick timer of the vendor extension's sampling: TIM7 (basic timer, RM chapter 16) is
// XRDAP-Core's Vendor::TickTimer on the CH32V3. It counts at 1 MHz; a period longer than
// the 16-bit counter is split into `repeat` equal timer periods and every repeat-th
// interrupt is a tick, which rounds the period down by less than `repeat` µs. The
// interrupt only passes the tick on (DapVendor::OnTickIsr); the main loop samples. LibXR
// has no hardware timer abstraction, so the registers are written here.
//
// 厂商扩展采样的节拍定时器：CH32V3 上用 TIM7（基本定时器，RM 第 16 章）
// 实现 XRDAP-Core 的 Vendor::TickTimer。它以 1 MHz 计数；
// 比 16 位计数器长的周期分成 repeat 个相等的定时周期，每 repeat 次中断算一拍，
// 周期因此少于 repeat µs。中断只转交节拍（DapVendor::OnTickIsr），
// 采样在主循环里做。LibXR 没有硬件定时器抽象，所以这里直接写寄存器。

#include <cstdint>

#include "xrdap/vendor/dap_vendor.hpp"

namespace XrDap
{

class SamplerTick final : public Vendor::TickTimer
{
 public:
  /// @note 只能有一个实例：TIM7 的中断转给它 / One instance only: the TIM7 interrupt
  ///       goes to it.
  explicit SamplerTick(DapVendor& vendor);

  void Start(uint32_t period_us) override;
  void Stop() override;

  /// TIM7 中断里调用 / Called from the TIM7 interrupt.
  void OnInterrupt();

  static SamplerTick* instance;

 private:
  DapVendor& vendor_;
  volatile uint32_t repeat_ = 1U;
  volatile uint32_t count_ = 0U;
};

}  // namespace XrDap
