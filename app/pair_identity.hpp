// SPDX-License-Identifier: Apache-2.0
#pragma once

// Radio identity of a probe pair. Pure logic; tests/link_test.cpp checks it.
//
// Unpaired probes use the default identity: 2.44 GHz, CRC seed 0xFFFF, GFSK sync word
// "XRDP". Pairing (pairing.hpp) gives both probes the same link id, LinkId() of their two
// chip UIDs; PairIdentity() derives from it a frequency (2403..2479 MHz, which keeps the
// 2.4 MHz GFSK bandwidth inside 2400..2483.5 MHz), a CRC seed and a GFSK sync word with
// 10..22 bit transitions, so that it stands out from the preamble and from an idle
// channel. The same two probes always get the same identity. The identity separates
// pairs; it does not authenticate them.
//
// 一对探针的射频身份。纯逻辑；tests/link_test.cpp 检查它。未配对的探针用默认身份：
// 2.44 GHz、CRC 种子 0xFFFF、GFSK 同步字 "XRDP"。配对（pairing.hpp）让两块探针得到同一个
// 链路 id，即两者芯片 UID 的 LinkId()；PairIdentity() 由它推导频点（2403..2479 MHz，
// GFSK 2.4 MHz 带宽不出 2400..2483.5 MHz）、CRC 种子和有 10..22 次跳变的 GFSK 同步字，
// 使其区别于前导码和空闲信道。同一对探针总得到同一个身份。身份只区分不同的对，不做认证。

#include <cstddef>
#include <cstdint>

namespace XrDap
{
/// 区分不同探针对的射频参数 / What tells one pair of probes from another.
struct RadioIdentity
{
  uint32_t frequency_hz;  ///< 载波频率 / Carrier frequency.
  uint16_t crc_seed;      ///< CRC 初值 / CRC seed.
  /// 仅 GFSK；FLRC 固定 "XRDP"（勘误 16.4）/ GFSK only; FLRC keeps "XRDP" (errata
  /// 16.4).
  uint8_t sync_word[4];
};

/// 未配对探针的身份 / The identity of unpaired probes.
RadioIdentity DefaultIdentity();

/// @brief FNV-1a 32 位散列 / 32-bit FNV-1a hash.
uint32_t Fnv1a(const uint8_t* data, size_t size, uint32_t hash = 2166136261U);

/// @brief 两个 UID 的链路 id，与顺序无关，不为 0 / The link id of two UIDs: independent
///        of their order, never 0.
uint32_t LinkId(const uint8_t* a, const uint8_t* b, size_t uid_size);

/// @brief 链路 id 对应的射频身份 / The radio identity of a link id.
RadioIdentity PairIdentity(uint32_t link_id);
}  // namespace XrDap
