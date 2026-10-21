// SPDX-License-Identifier: Apache-2.0
#include "link_protocol.hpp"

#include <cstring>

namespace XrDap::Link
{
uint8_t FragmentCount(uint16_t total, uint8_t fragment)
{
  if (total == 0U) return 1U;
  return static_cast<uint8_t>((total + fragment - 1U) / fragment);
}

namespace
{
void PutHeader(uint8_t* out, uint8_t kind, uint8_t sequence, uint8_t index, uint8_t count,
               uint16_t total)
{
  out[0] = kind;
  out[1] = sequence;
  out[2] = index;
  out[3] = count;
  out[4] = static_cast<uint8_t>(total);
  out[5] = static_cast<uint8_t>(total >> 8);
}

/// Length of fragment `index` of a message of `total` bytes.
uint16_t FragmentLength(uint16_t total, uint8_t fragment, uint8_t index)
{
  const uint32_t offset = static_cast<uint32_t>(index) * fragment;
  if (offset >= total) return 0U;
  const uint32_t rest = total - offset;
  return static_cast<uint16_t>(rest < fragment ? rest : fragment);
}
}  // namespace

uint8_t BuildFragment(uint8_t kind, uint8_t sequence, const uint8_t* message,
                      uint16_t total, uint8_t fragment, uint8_t index, uint8_t* out)
{
  const uint16_t length = FragmentLength(total, fragment, index);
  PutHeader(out, kind, sequence, index, FragmentCount(total, fragment), total);
  if (length != 0U) std::memcpy(out + kHeader, message + index * fragment, length);
  return static_cast<uint8_t>(kHeader + length);
}

uint8_t BuildPing(uint8_t kind, uint8_t sequence, uint16_t session, uint8_t* out)
{
  PutHeader(out, kind, sequence, 0, 1, kPingPayload);
  out[kHeader] = static_cast<uint8_t>(session);
  out[kHeader + 1] = static_cast<uint8_t>(session >> 8);
  return kHeader + kPingPayload;
}

bool ParsePing(const uint8_t* packet, int size, uint16_t& session)
{
  if (size != kHeader + kPingPayload || packet[3] != 1U) return false;
  session = static_cast<uint16_t>(packet[kHeader] | packet[kHeader + 1] << 8);
  return true;
}

Reassembler::Result Reassembler::Add(const uint8_t* packet, int size, uint8_t fragment)
{
  if (size < kHeader || fragment == 0U) return Result::REJECTED;
  const uint8_t sequence = packet[1], index = packet[2], count = packet[3];
  const uint16_t total = static_cast<uint16_t>(packet[4] | packet[5] << 8);
  const uint16_t length = static_cast<uint16_t>(size - kHeader);
  // Each fragment must agree with the spec on its own: a count that matches the total,
  // an index inside it, and exactly the length the index implies.
  if (total > capacity_ || count == 0U || count > kMaxFragments ||
      count != FragmentCount(total, fragment) || index >= count ||
      length != FragmentLength(total, fragment, index))
  {
    return Result::REJECTED;
  }
  if (mask_ == 0U || sequence != sequence_ || count != count_ || total != total_)
  {
    mask_ = 0;
    sequence_ = sequence;
    count_ = count;
    total_ = total;
  }
  if (length != 0U) std::memcpy(buffer_ + index * fragment, packet + kHeader, length);
  mask_ = static_cast<uint16_t>(mask_ | 1U << index);
  if (mask_ != static_cast<uint16_t>((1U << count) - 1U)) return Result::INCOMPLETE;
  mask_ = 0;
  return Result::COMPLETE;
}
}  // namespace XrDap::Link
