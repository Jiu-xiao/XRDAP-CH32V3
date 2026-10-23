// SPDX-License-Identifier: Apache-2.0
#pragma once
// DAP status on the WS2812 chain (boards with XRDAP_HAS_STATUS_LIGHTS), drawn by
// XRDAP-Core's status light rule (xrdap/light/status_light.hpp). Only the link and the
// setting state are decided here: wireless while a debug session runs over the radio (on
// the probe that forwards the commands, and while the remote session is open on the one
// that executes them), wired while USB is configured, otherwise no link; pairing is a
// setting being changed. The whole chain shows the rule's colour; each activity pulse
// also moves a white comet one LED on. A frame is computed and sent every 20 ms by a task
// beside the DAP loop.
//
// WS2812 灯链上的 DAP 状态（设了 XRDAP_HAS_STATUS_LIGHTS 的板子），按 XRDAP-Core 的状态灯
// 规则（xrdap/light/status_light.hpp）显示。这里只判断连接和设置状态：调试会话走无线时为
// 无线（转发命令的探针上，以及远端会话打开期间执行命令的探针上），USB 已配置为有线，否则
// 为没有连接；配对算改设置。整条灯链显示规则给的颜色；每个活动脉冲还让一颗白色彗星前进一
// 格。每 20 ms 由 DAP 循环旁的任务算一帧并发出。

#include <cstdint>

#include "xrdap/activity/dap_activity.hpp"

namespace XrDap
{
/// @brief Set up the WS2812 chain and start the light task / 初始化 WS2812 灯链并启动灯
///        任务。
/// @note Call once from the main task during start-up, before other tasks configure
///       pins / 启动时由主任务调用一次，早于其他任务配置引脚。
/// @param task_prio At least the DAP loop's priority: with the radio in use the DAP loop
///        does not block / 不低于 DAP 循环：无线角色下 DAP 循环不阻塞。
/// @param usb_configured Whether USB is configured by the host, called from the light
///        task / 查询 USB 是否已被主机配置，从灯任务里调用。
void StatusLightsBegin(uint32_t task_prio, const DapActivity& activity,
                       bool (*usb_configured)(void*), void* usb_ctx);
}  // namespace XrDap
