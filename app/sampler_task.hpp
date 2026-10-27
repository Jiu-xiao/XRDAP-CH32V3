// SPDX-License-Identifier: Apache-2.0
#pragma once

// Sampling task of the vendor extension and its tick timer: TIM7 (basic timer, RM chapter
// 16) is XRDAP-Core's Vendor::TickTimer on the CH32V307. Its interrupt only passes the
// tick on (DapVendor::OnTickIsr); the task samples.
//
// 厂商扩展的采样任务和它的节拍定时器：TIM7（基本定时器，RM 第 16 章）是 CH32V307 上
// XRDAP-Core 的 Vendor::TickTimer。中断只把节拍交出去（DapVendor::OnTickIsr），由任务采
// 样。

#include <cstdint>

#include "xrdap/vendor/dap_vendor.hpp"

namespace XrDap
{
/// @brief 创建采样任务 / Create the sampling task.
/// @param priority 应为最高的应用任务优先级 / Should be the highest application task
///        priority.
/// @note 启动时由主任务调用一次 / Call once from the main task at start-up.
void StartSamplerTask(DapVendor& vendor, uint32_t priority);
}  // namespace XrDap
