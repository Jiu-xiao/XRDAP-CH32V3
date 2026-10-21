// SPDX-License-Identifier: Apache-2.0
#pragma once

// Session rules of the radio link (see link_protocol.hpp for the packets). Pure logic
// with the time passed in; tests/link_test.cpp checks it.
//
// A session is one host session over the radio, named by a 16-bit session id (0 = no
// session). The host probe picks a new non-zero id for every new session.
//
// Host (HostSession):
//   H1 The first command of a session gets a new id; a PING with it goes first.
//   H2 While a session is open and no command went out for kHeartbeatUs, a PING with the
//      id goes out (the heartbeat).
//   H3 The session ends on DAP_Disconnect or after kHostIdleUs without a command from
//      the PC; a PING with id 0 then tells the remote.
// Remote (RemoteLease):
//   R1 A PING with id 0 ends the session.
//   R2 A PING with another id ends the session and opens one with the new id.
//   R3 Every packet from the host (PING, COMMAND) renews the lease; kLeaseUs without one
//      ends the session.
//   R4 Ending a session clears the cached answer and the command being reassembled, and
//      stops what the session started (sampling).
//   R5 A COMMAND outside a session is not executed and not answered.
// Duplicates (AnswerCache): inside a session the remote keeps its last answer; a command
// with the same sequence is answered from the cache instead of being executed again.
//
// 无线链路的会话规则（包格式见 link_protocol.hpp）。纯逻辑，时间由调用者传入；
// tests/link_test.cpp 按此检查。会话即一次经无线的主机会话，用 16 位会话号标识（0 = 无会
// 话），主机端每开一次新会话换一个非零号。主机端规则 H1–H3、远端规则 R1–R5、重复命令的处
// 理见上。

#include <cstdint>

namespace XrDap::Link
{
/// 主机端空闲时发心跳 PING 的间隔 / Heartbeat PING interval of an idle host.
inline constexpr uint32_t kHeartbeatUs = 1'000'000;
/// 远端多久收不到主机的包就结束会话 / Remote lease: no host packet for this long ends
/// the session.
inline constexpr uint32_t kLeaseUs = 3'000'000;
/// 主机端多久收不到 PC 的命令就结束会话 / Host: no command from the PC for this long
/// ends the session.
inline constexpr uint32_t kHostIdleUs = 5'000'000;

/// 远端的会话租期 / The remote's session lease.
class RemoteLease
{
 public:
  /// 结束会话时做什么 / What a session end means for the caller.
  enum class Event : uint8_t
  {
    NONE,     ///< 无变化 / nothing changed
    ENDED,    ///< 会话结束（R1、R3）/ the session ended (R1, R3)
    REPLACED  ///< 旧会话结束，新会话开始（R2，或无会话时开始）/ the old session ended and
              ///< a new one began (R2, or a first one)
  };

  /// @brief 收到 PING / A PING arrived.
  Event OnPing(uint16_t session, uint32_t now_us);
  /// @brief 收到主机的 COMMAND 分片 / A COMMAND fragment from the host arrived.
  /// @return 会话打开时为 true，即应处理该分片（R5）/ true while a session is open, i.e.
  ///         the fragment is to be processed (R5).
  bool OnCommand(uint32_t now_us);
  /// @brief 检查租期（R3），定期调用 / Check the lease (R3); call regularly.
  Event Tick(uint32_t now_us);

  bool Active() const { return session_ != 0U; }
  uint16_t session() const { return session_; }

 private:
  uint16_t session_ = 0;
  uint32_t heard_us_ = 0;
};

/// 主机端的会话 / The host side of a session.
class HostSession
{
 public:
  /// 现在该发什么 / What to send now.
  enum class Action : uint8_t
  {
    NONE,       ///< 什么都不发 / nothing
    HEARTBEAT,  ///< 发 PING(id)（H2）/ send PING(id) (H2)
    END,        ///< 会话结束，发 PING(0)（H3）/ the session ended, send PING(0) (H3)
  };

  /// @param seed 第一个会话号的来源，例如启动后的微秒数 / Source of the first session id,
  ///        e.g. microseconds since start.
  explicit HostSession(uint32_t seed = 0) : next_(static_cast<uint16_t>(seed)) {}

  /// @brief 一条命令要转发（H1）/ A command is about to be forwarded (H1).
  /// @return 新会话开始，应先发 PING(id) 时为 true / true when a new session begins and
  ///         a PING(id) must go first.
  bool OnCommand(uint32_t now_us);
  /// @brief 转发命令发出去了，推迟心跳 / A forwarded command went out; defers the
  ///        heartbeat.
  void OnSent(uint32_t now_us) { sent_us_ = now_us; }
  /// @brief PC 结束了会话（DAP_Disconnect）/ The PC ended the session (DAP_Disconnect).
  /// @return 有会话要结束、应发 PING(0) 时为 true / true when a session ends and PING(0)
  ///         must go out.
  bool OnDisconnect();
  /// @brief 定期调用：心跳或空闲结束（H2、H3）/ Call regularly: heartbeat or idle end
  ///        (H2, H3).
  Action Tick(uint32_t now_us);

  bool Active() const { return session_ != 0U; }
  uint16_t session() const { return session_; }

 private:
  uint16_t session_ = 0;
  uint16_t next_;
  bool seeded_ = false;
  uint32_t command_us_ = 0;  // last command from the PC
  uint32_t sent_us_ = 0;     // last packet sent in the session
};

/// 远端的应答缓存（重复命令）/ The remote's answer cache (duplicate commands).
class AnswerCache
{
 public:
  /// @brief 这个序号的应答是否已缓存 / Whether the answer of this sequence is cached.
  bool Has(uint8_t sequence) const { return valid_ && sequence == sequence_; }
  void Store(uint8_t sequence)
  {
    sequence_ = sequence;
    valid_ = true;
  }
  /// 会话结束时清空（R4）/ Cleared when a session ends (R4).
  void Clear() { valid_ = false; }

 private:
  uint8_t sequence_ = 0;
  bool valid_ = false;
};
}  // namespace XrDap::Link
