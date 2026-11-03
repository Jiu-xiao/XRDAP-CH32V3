// SPDX-License-Identifier: Apache-2.0
#pragma once
// DAP status on the WS2812 chain (boards with XRDAP_HAS_STATUS_LIGHTS), drawn by
// XRDAP-Core's status light rule (xrdap/light/status_light.hpp). Only the link and the
// setting state are decided here: wireless while a debug session runs over the radio (on
// the probe that forwards the commands, and while the remote session is open on the one
// that executes them), wired while USB is configured, otherwise no link; pairing is a
// setting being changed. The whole chain shows the rule's colour; each activity pulse
// also moves a white comet one LED on. A frame is computed and sent every 20 ms from the main loop.
//
// WS2812 灯链上的 DAP 状态（设了 XRDAP_HAS_STATUS_LIGHTS 的板子），按 XRDAP-Core 的状态灯
// 规则（xrdap/light/status_light.hpp）显示。这里只判断连接和设置状态：调试会话走无线时为
// 无线（转发命令的探针上，以及远端会话打开期间执行命令的探针上），USB 已配置为有线，否则
// 为没有连接；配对算改设置。整条灯链显示规则给的颜色；每个活动脉冲还让一颗白色彗星前进一
// 格。每 20 ms 由主循环算一帧并发出。

#include <cstdint>

#include "xrdap/activity/dap_activity.hpp"

namespace XrDap
{
/// @brief Set up the WS2812 chain / 初始化 WS2812 灯链。
/// @note Call once at start-up, before the main loop / 启动时调用一次，在主循环之前。
/// @param usb_configured Whether USB is configured by the host / 查询 USB 是否已被主机
///        配置。
void StatusLightsBegin(const DapActivity& activity, bool (*usb_configured)(void*),
                       void* usb_ctx);

/// @brief Draw a frame when 20 ms have passed since the last one / 距上一帧满 20 ms 时画一帧。
/// @note Call from the main loop, often: a frame takes about a quarter of a millisecond /
///       在主循环里频繁调用：一帧约四分之一毫秒。
void StatusLightsPoll();
}  // namespace XrDap
