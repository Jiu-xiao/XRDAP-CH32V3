// SPDX-License-Identifier: Apache-2.0
#pragma once

// Sampling task of the vendor extension, woken on every tick by TIM7 (the CH32
// implementation of XRDAP-Core's Vendor::TickTimer).
//
// 厂商扩展的采样任务，由 TIM7 每拍唤醒（XRDAP-Core 的 Vendor::TickTimer 的 CH32 实现）。

#include <cstdint>

#include "dap_vendor.hpp"

namespace XrDap
{
/// @brief Create the sampling task / 创建采样任务。
/// @param priority Should be the highest application task priority / 应为最高的应用任务
///        优先级。
void StartSamplerTask(DapVendor& vendor, uint32_t priority);
}  // namespace XrDap
