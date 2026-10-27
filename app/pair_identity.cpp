// SPDX-License-Identifier: Apache-2.0
#include "pair_identity.hpp"

#include <cstring>

namespace XrDap
{
namespace
{
/// Bit changes along a 32-bit word.
int Transitions(uint32_t word)
{
  return __builtin_popcount(word ^ (word >> 1)) - static_cast<int>((word >> 31) & 1U);
}
}  // namespace

RadioIdentity DefaultIdentity() { return {2'440'000'000U, 0xFFFF, {'X', 'R', 'D', 'P'}}; }

uint32_t Fnv1a(const uint8_t* data, size_t size, uint32_t hash)
{
  for (size_t i = 0; i < size; ++i) hash = (hash ^ data[i]) * 16777619U;
  return hash;
}

uint32_t LinkId(const uint8_t* a, const uint8_t* b, size_t uid_size)
{
  // Smaller UID first, so that both probes compute the same id.
  const bool a_first = std::memcmp(a, b, uid_size) < 0;
  uint32_t hash = Fnv1a(a_first ? a : b, uid_size);
  hash = Fnv1a(a_first ? b : a, uid_size, hash);
  return hash != 0U ? hash : 1U;
}

RadioIdentity PairIdentity(uint32_t link_id)
{
  RadioIdentity identity{};
  identity.frequency_hz = (2403U + link_id % 77U) * 1'000'000U;
  identity.crc_seed = static_cast<uint16_t>(link_id >> 16 ^ link_id);
  uint8_t bytes[4];
  std::memcpy(bytes, &link_id, sizeof(bytes));
  uint32_t word = Fnv1a(bytes, sizeof(bytes));
  while (Transitions(word) < 10 || Transitions(word) > 22)
  {
    std::memcpy(bytes, &word, sizeof(bytes));
    word = Fnv1a(bytes, sizeof(bytes));
  }
  for (int i = 0; i < 4; ++i)
    identity.sync_word[i] = static_cast<uint8_t>(word >> (24 - 8 * i));
  return identity;
}
}  // namespace XrDap
