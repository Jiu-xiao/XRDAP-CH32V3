// SPDX-License-Identifier: Apache-2.0
#pragma once

// Tick timer of the sampling of the vendor extension: TIM7 (basic timer, RM chapter 16) is
// XRDAP-Core's Vendor::TickTimer on the CH32V3. Its interrupt only passes the tick on
// (DapVendor::OnTickIsr); the main loop samples (DapVendor::Poll).
//
// 厂商扩展采样的节拍定时器：TIM7（基本定时器，RM 第 16 章）是 CH32V3 上 XRDAP-Core 的
// Vendor::TickTimer。中断只把节拍交出去（DapVendor::OnTickIsr），由主循环采样
// （DapVendor::Poll）。

#include <cstdint>

#include "xrdap/vendor/dap_vendor.hpp"

namespace XrDap
{
/// @brief 节拍定时器，绑定到 vendor / The tick timer, bound to the vendor.
/// @note 启动时调用一次；之后主循环里 vendor.Poll(SamplerTimer(vendor)) / Call once at
///       start-up; then the main loop runs vendor.Poll on it.
Vendor::TickTimer& SamplerTimer(DapVendor& vendor);
}  // namespace XrDap
