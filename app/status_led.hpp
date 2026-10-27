// SPDX-License-Identifier: Apache-2.0
#pragma once
// DAP status on a single plain LED (boards with XRDAP_HAS_STATUS_LED), drawn from
// XRDAP-Core's status light rule (xrdap/light/status_light.hpp) as far as one LED of one
// colour can show it: lit while USB is configured (wired), dark without a link, and
// dark for the moment of each activity pulse, so it flickers under DAP traffic. The
// colours of the rule (red/blue/purple) and the host states have no equivalent on it.
// The main loop calls StatusLedUpdate() at least every 10 ms; no task of its own, so it
// costs no stack.
//
// 单个普通 LED 上的 DAP 状态（设了 XRDAP_HAS_STATUS_LED 的板子），按 XRDAP-Core 的状态灯
// 规则（xrdap/light/status_light.hpp）显示，一颗单色灯能表达多少就表达多少：USB 已配置
// （有线）时亮，没有连接时灭，每个活动脉冲的瞬间灭一下，所以 DAP 有通信时闪烁。规则里的
// 颜色（红/蓝/紫）和上位机状态在它上面没有对应。主循环至少每 10 ms 调一次
// StatusLedUpdate()；不单开任务，不占栈。

#include <cstdint>

#include "xrdap/activity/dap_activity.hpp"

namespace XrDap
{
/// @brief Set up the LED pin, dark / 初始化 LED 引脚，熄灭。
/// @param usb_configured Whether USB is configured by the host / 查询 USB 是否已被主机
///        配置。
void StatusLedBegin(const DapActivity& activity, bool (*usb_configured)(void*),
                    void* usb_ctx);

/// @brief Draw the current state; from the main loop / 画出当前状态，由主循环调用。
void StatusLedUpdate();
}  // namespace XrDap
