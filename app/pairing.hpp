// SPDX-License-Identifier: Apache-2.0
#pragma once

// Pairing. Holding KEY2 for 3 s (or DAP vendor command 0x9F 0x20) puts a probe into
// pairing for up to 10 s. Two probes in pairing both use the default radio identity
// (GFSK) and send HELLO packets every 20 ms or so:
//   [0] 5  [1..12] own chip UID  [13..24] the UID of the peer heard so far, or zeros
// A probe that hears a HELLO naming itself is done; it keeps answering for another second
// so that the other probe hears its own UID as well. Both probes derive the same link id
// from their two UIDs, store it with the peer's UID, and switch to the identity that
// PairIdentity() derives from it: frequency, CRC seed and GFSK sync word. Pairing the
// same two probes again gives the same identity.
//
// 配对。按住 KEY2 3 s（或 DAP 厂商命令 0x9F 0x20）进入配对，最长 10 s。两块处于配对的
// 探针都用默认射频身份（GFSK），大约每 20 ms 发一个 HELLO 包（格式见上）。听到对方在
// HELLO 里报出自己 UID 的探针即完成，再继续应答 1 s，让对方也听到自己的 UID。两块探针
// 由两者的 UID 得出同一个链路 id，与对端 UID 一起保存，并切到 PairIdentity() 由它推导
// 的身份：频点、CRC 种子和 GFSK 同步字。同一对探针重新配对得到同一个身份。

#include <cstdint>

#include "sx1281.hpp"

namespace XrDap
{
enum class PairState : uint8_t
{
  IDLE = 0,
  RUNNING = 1,
  DONE = 2,
  FAILED = 3,
};

/// Radio identity of a pair / 一对探针的射频身份。
Sx1281::Identity PairIdentity(uint32_t link_id);

/// Ask for pairing; it runs from the next PairingPoll() / 请求配对。
void PairingRequest();
/// Watch KEY2 and run a requested pairing (blocks for up to 11 s); call from the main
/// loop / 检查 KEY2 并执行已请求的配对（最长阻塞 11 s），由主循环调用。
void PairingPoll();
PairState GetPairState();
}  // namespace XrDap
