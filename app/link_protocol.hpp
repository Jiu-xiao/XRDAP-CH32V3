// SPDX-License-Identifier: Apache-2.0
#pragma once

// Packet format of the radio link between two probes. This header is the link
// specification; tests/link_test.cpp checks the code against it. Pure logic, no
// hardware.
//
// Packet: a 6-byte header, then up to `fragment` payload bytes, where fragment =
// largest radio packet - 6 (249 bytes in GFSK, 121 in FLRC).
//   [0] kind  [1] sequence  [2] fragment index  [3] fragment count
//   [4..5] message size (total), little endian
// Kinds:
//   1 COMMAND  a DAP or 0x9F command, host -> remote
//   2 ANSWER   its answer, remote -> host, same sequence as the command
//   3 PING     host -> remote; payload: session id, u16 little endian
//   4 PONG     remote -> host; same sequence, payload: the session id it now holds
//   5 HELLO    pairing (pairing.hpp; its own layout, no header)
// A message of `total` bytes (at most kMaxMessage) travels as count = max(1,
// ceil(total / fragment)) fragments; fragment i carries bytes [i * fragment,
// min(total, (i + 1) * fragment)). A receiver accepts a message only when every fragment
// of one (sequence, count, total) arrived and each fragment has exactly its length;
// anything else is dropped.
//
// 两块探针之间无线链路的包格式。本文件即链路规范，tests/link_test.cpp 按它检查代码。纯逻
// 辑，不涉及硬件。
//
// 包：6 字节包头，其后最多 fragment 字节载荷，fragment = 最大无线包长 - 6（GFSK 249 字
// 节，FLRC 121 字节）。包头与包类型见上。total 字节（最多 kMaxMessage）的消息分成 count =
// max(1, ceil(total / fragment)) 片；第 i 片承载 [i * fragment, min(total, (i + 1) *
// fragment)) 字节。接收方只在同一（序号、片数、总长）的所有分片都已到达、且每片长度都恰好
// 符合时才接受这条消息，否则丢弃。

#include <cstdint>

namespace XrDap::Link
{
/// 包类型 / Packet kinds.
enum Kind : uint8_t
{
  kCommand = 1,
  kAnswer = 2,
  kPing = 3,
  kPong = 4,
  kHello = 5,
};

/// 包头长度 / Header size.
inline constexpr uint8_t kHeader = 6;
/// 一条消息的最大长度（一条 DAP 命令或应答）/ Largest message (one DAP command or
/// answer).
inline constexpr uint16_t kMaxMessage = 1024;
/// 最小分片（FLRC 127 字节包）下的最多分片数 / Most fragments, at the smallest fragment
/// (127-byte FLRC packets).
inline constexpr uint8_t kMaxFragments =
    (kMaxMessage + (127 - kHeader) - 1) / (127 - kHeader);
static_assert(kMaxFragments <= 16, "the fragment mask has 16 bits");
/// PING/PONG 载荷：会话号 / PING/PONG payload: the session id.
inline constexpr uint8_t kPingPayload = 2;

/// @brief total 字节的消息分几片 / Fragments of a message of `total` bytes.
uint8_t FragmentCount(uint16_t total, uint8_t fragment);

/// @brief 写出消息的第 index 片 / Write fragment `index` of a message.
/// @param out 至少 kHeader + fragment 字节 / At least kHeader + fragment bytes.
/// @return 包长 / The packet size.
uint8_t BuildFragment(uint8_t kind, uint8_t sequence, const uint8_t* message,
                      uint16_t total, uint8_t fragment, uint8_t index, uint8_t* out);

/// @brief 写出一个 PING 或 PONG 包 / Write a PING or PONG packet.
/// @return 包长 / The packet size.
uint8_t BuildPing(uint8_t kind, uint8_t sequence, uint16_t session, uint8_t* out);

/// @brief 读 PING/PONG 包里的会话号 / Read the session id of a PING/PONG packet.
/// @return 包长不对时为 false / false when the packet has the wrong size.
bool ParsePing(const uint8_t* packet, int size, uint16_t& session);

/// 把分片拼回消息 / Reassembles fragments into a message.
class Reassembler
{
 public:
  enum class Result : uint8_t
  {
    INCOMPLETE,  ///< 还缺分片 / fragments still missing
    COMPLETE,    ///< 消息齐了，size() 为长度 / the message is complete, see size()
    REJECTED,  ///< 分片与规范不符，已丢弃 / the fragment breaks the spec and was dropped
  };

  /// @param buffer 消息缓冲，capacity 字节 / Message buffer of `capacity` bytes.
  Reassembler(uint8_t* buffer, uint16_t capacity) : buffer_(buffer), capacity_(capacity)
  {
  }

  /// 丢掉拼了一半的消息 / Drop a partly assembled message.
  void Reset() { mask_ = 0; }

  /// @brief 收下一片 / Take one fragment.
  /// @param packet 完整的包（含包头）/ The whole packet including the header.
  /// @param fragment 当前调制下的分片长度 / Fragment size of the current modulation.
  /// @note 序号、片数或总长与在拼的消息不同时，先丢掉在拼的消息 / A different sequence,
  ///       count or total first drops the message being assembled.
  Result Add(const uint8_t* packet, int size, uint8_t fragment);

  uint16_t size() const { return total_; }
  uint8_t sequence() const { return sequence_; }

 private:
  uint8_t* buffer_;
  uint16_t capacity_;
  uint16_t mask_ = 0;
  uint16_t total_ = 0;
  uint8_t sequence_ = 0;
  uint8_t count_ = 0;
};
}  // namespace XrDap::Link
