// SPDX-License-Identifier: Apache-2.0
#pragma once

// What the host probe needs to know about DAP commands to forward them over the radio.
// Pure logic; tests/link_test.cpp checks it.
//
// ExecutionUs(): how long the remote may take to execute a command (its waits, or its
// SWD transactions at the forwarded SWD clock, twice over for margin); the host adds it
// to the time it waits for the answer.
// SettingsReplay: SWJ_Clock, TransferConfigure and SWD_Configure that a tool sends before
// DAP_Connect run locally, as the session is not decided yet; the last of each is kept
// and sent to the remote when the session goes over the radio.
//
// 主机端经无线转发 DAP 命令时需要知道的命令语义。纯逻辑；tests/link_test.cpp 检查它。
// ExecutionUs()：远端执行一条命令可能用的时间（命令自带的等待，或按转发的 SWD 时钟算出的
// SWD 事务时间，再乘 2 留余量），主机端等应答时加上它。SettingsReplay：工具在 DAP_Connect
// 之前发的 SWJ_Clock、TransferConfigure、SWD_Configure 先在本地执行（会话走哪条路还没
// 定），每种保留最后一条，会话走无线时补发给远端。

#include <cstdint>

#include "xrdap/protocol/dap_cmd.hpp"

namespace XrDap
{
/// @brief req[0] 是不是这条 DAP 命令（XRDAP-Core 的 DapCmd 命令号）/ Whether req[0] is
///        this DAP command (XRDAP-Core's DapCmd ids).
inline bool IsCommand(const uint8_t* req, uint16_t len, uint8_t id)
{
  return len != 0U && req[0] == id;
}

/// libxr 在 SWJ_Clock 之前用的 SWD 时钟 / libxr's SWD clock until SWJ_Clock.
inline constexpr uint32_t kDefaultSwdHz = 500'000;

/// @brief 远端执行这条命令可能用的时间 / Time the remote may spend on this command.
/// @param swd_hz 最近转发的 SWJ_Clock / The last forwarded SWJ_Clock.
uint32_t ExecutionUs(const uint8_t* req, uint16_t len, uint32_t swd_hz);

/// @brief SWJ_Clock 命令里的频率；不是该命令或为 0 时返回 0 / The clock of an SWJ_Clock
///        command; 0 for other commands or a zero clock.
uint32_t SwjClockHz(const uint8_t* req, uint16_t len);

/// 会话开始前的设置命令，会话走无线时补发 / Setting commands from before the session,
/// replayed when it goes over the radio.
class SettingsReplay
{
 public:
  /// 记下一条命令（不是设置命令时忽略）/ Note a command (ignored unless a setting).
  void Note(const uint8_t* req, uint16_t len);

  /// 记下的设置命令数，最多 kSettings / Number of settings kept, at most kSettings.
  static constexpr uint8_t kSettings = 3;
  /// @brief 第 i 条（按 SWJ_Clock、TransferConfigure、SWD_Configure 顺序）；没记下时长度
  ///        为 0 / Setting `i` (SWJ_Clock, TransferConfigure, SWD_Configure); length 0
  ///        when none was noted.
  const uint8_t* Data(uint8_t i) const { return items_[i].data; }
  uint8_t Size(uint8_t i) const { return items_[i].size; }

 private:
  struct Item
  {
    uint8_t id;
    uint8_t size;
    uint8_t data[8];
  };
  Item items_[kSettings] = {{Byte(DapCmd::SWJ_CLOCK), 0, {}},
                            {Byte(DapCmd::TRANSFER_CONFIGURE), 0, {}},
                            {Byte(DapCmd::SWD_CONFIGURE), 0, {}}};
};
}  // namespace XrDap
