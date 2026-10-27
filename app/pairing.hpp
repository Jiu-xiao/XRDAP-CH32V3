// SPDX-License-Identifier: Apache-2.0
#pragma once

// Pairing. Holding KEY2 for 3 s (or DAP vendor command 0x9F 0x20) puts a probe into
// pairing for up to 10 s. Two probes in pairing both use the default radio identity
// (GFSK) and send HELLO packets every 20 ms or so:
//   [0] 5 (link_protocol.hpp)  [1..12] own chip UID  [13..24] the UID of the peer
//   heard so far, or zeros
// A probe that hears a HELLO naming itself is done; it keeps answering for another second
// so that the other probe hears its own UID as well. Both probes derive the same link id
// from their two UIDs and store it with the peer's UID; the identity derived from it
// (pair_identity.hpp) takes effect when the caller applies the settings again. Pairing
// the same two probes again gives the same identity.
//
// 配对。按住 KEY2 3 s（或 DAP 厂商命令 0x9F 0x20）进入配对，最长 10 s。两块处于配对的探针
// 都用默认射频身份（GFSK），大约每 20 ms 发一个 HELLO 包（格式见上）。听到对方在 HELLO 里
// 报出自己 UID 的探针即完成，再继续应答 1 s，让对方也听到自己的 UID。两块探针由两者的 UID
// 得出同一个链路 id，与对端 UID 一起保存；由它推导的身份（pair_identity.hpp）在调用者重新
// 应用设置后生效。同一对探针重新配对得到同一个身份。

#include <cstdint>

namespace XrDap
{
enum class PairState : uint8_t
{
  IDLE = 0,
  RUNNING = 1,
  DONE = 2,
  FAILED = 3,
};

/// @brief 请求配对，从下一次 PairingPoll() 开始 / Ask for pairing; it runs from the next
///        PairingPoll().
void PairingRequest();

/// @brief 检查 KEY2 并执行已请求的配对 / Watch KEY2 and run a requested pairing.
/// @return 配对用过射频（无论成败）、调用者应重新应用设置（DapSessionApplyConfig）时为
///         true / true when pairing used the radio (success or not) and the caller must
///         apply the settings again (DapSessionApplyConfig).
/// @note 主任务调用；配对时最长阻塞 11 s，期间喂看门狗 / Main task; blocks up to 11 s
///       while pairing and feeds the watchdog meanwhile.
bool PairingPoll();

/// 最近一次配对的状态 / State of the latest pairing.
PairState GetPairState();
}  // namespace XrDap
